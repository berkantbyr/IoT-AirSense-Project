/**
 * @file iot_monitor_main.c
 * @brief Smart Multi-Sensor & Hazard Warning System
 * 
 * This project is an IoT application that reads temperature, humidity,
 * flammable gas, carbon monoxide, and flame status from the environment;
 * generates a loud square-wave alarm via I2S during hazards; and displays
 * real-time data on an OLED screen and a local Web Server.
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// FreeRTOS Task and Timing Libraries
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Hardware Driver Libraries (I2S, ADC, GPIO, I2C)
#include "driver/i2s.h"
#include "driver/adc.h"
#include "driver/gpio.h"
#include "driver/i2c.h"

// System and Network Libraries
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_http_server.h"
#include "mdns.h"

// Peripheral Libraries
#include "ssd1306.h"        // OLED Display Driver
#include "dashboard_html.h" // Embedded HTML for dashboard interface


/*NETWORK & USER SETTINGS*/
#define WIFI_SSID "-"       // Enter your Wi-Fi network name to connect
#define WIFI_PASS "-"  //Enter your Wi-Fi password
#define HOSTNAME "airsense"       // mDNS hostname (http://airsense.local)


/*HARDWARE & PIN SETTINGS*/
#define I2S_PORT    I2S_NUM_0       // I2S port for speaker audio output
#define FLAME_PIN   5               // Flame detection sensor digital pin
#define MQ2_CH      ADC1_CHANNEL_0  // Flammable gas sensor (MQ-2) ADC channel (GPIO 1)
#define MQ7_CH      ADC1_CHANNEL_1  // Carbon monoxide sensor (MQ-7) ADC channel (GPIO 2)
#define DHT_PIN     4               // Temperature & humidity sensor (DHT22) data pin

// Gas sensor threshold value
#define GAS_DANGER_THRESHOLD 3500 

static const char *TAG = "SMART_SYSTEM";

// OLED display controller
ssd1306_handle_t oled_handle = NULL;

// Global variables holding the latest sensor readings
static float temperature = 0;
static float humidity    = 0;
static int   mq2_val     = 0;
static int   mq7_val     = 0;
static int   flame_val   = 1; // Initially 1 (Safe). 0: Fire detected

// Hardware mutex to prevent DHT sensor read from being interrupted
static portMUX_TYPE dht_mux = portMUX_INITIALIZER_UNLOCKED;



/*ALARM FUNCTION*/
/**
 * @brief Sends a square-wave signal to the speaker via I2S for 5 seconds during hazard. The period may be extended.
 */
void play_alarm_beep_5_sec()
{
    ESP_LOGW(TAG, "DANGER: Alarm Activated!");
    size_t bw;
    int16_t beep_data[100];

    TickType_t start_tick = xTaskGetTickCount();
    while (xTaskGetTickCount() - start_tick < pdMS_TO_TICKS(5000)) {
        for (int i = 0; i < 100; i++) {
            beep_data[i] = (i % 50 < 25) ? 15000 : -15000;
        }
        i2s_write(I2S_PORT, beep_data, sizeof(beep_data), &bw, portMAX_DELAY);
    }
    
    i2s_zero_dma_buffer(I2S_PORT);
}



/*DHT22 SENSOR READING ALGORITHM*/

/**
 * @brief Communicates with the DHT22 sensor over a single data pin using timed protocol.
 * @param temp Pointer where the calculated temperature value will be stored.
 * @param hum Pointer where the calculated humidity value will be stored.
 * @return esp_err_t Returns ESP_OK on success, ESP_FAIL on failure.
 */
void dht_init(void)
{
    gpio_set_direction(DHT_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(DHT_PIN, GPIO_PULLUP_ENABLE);
}

int dht_read(float *temp, float *hum)
{
    uint8_t data[5] = {0};
    int timeout;

    // Idle check
    gpio_set_direction(DHT_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(DHT_PIN, GPIO_PULLUP_ENABLE);
    esp_rom_delay_us(50);
    int idle = gpio_get_level(DHT_PIN);
    if (idle == 0) {
        ESP_LOGE(TAG, "DHT: Pin %d stuck LOW (no pull-up or sensor fault)", DHT_PIN);
        return -1;
    }

    // 1. Start signal: push-pull OUTPUT drives strong LOW for 20ms
    gpio_set_direction(DHT_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(DHT_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(20));

    // 2. Critical section: drive HIGH briefly, then switch to INPUT to read sensor
    portENTER_CRITICAL(&dht_mux);
    gpio_set_level(DHT_PIN, 1);
    esp_rom_delay_us(30);
    gpio_set_direction(DHT_PIN, GPIO_MODE_INPUT);

    // 3. Wait for sensor to pull LOW (response start, ~20-40us after release, then ~80us LOW)
    timeout = 500;
    while (gpio_get_level(DHT_PIN) == 1 && --timeout) esp_rom_delay_us(1);
    if (timeout <= 0) { portEXIT_CRITICAL(&dht_mux); ESP_LOGE(TAG, "DHT: No response (stage 1)"); return -2; }

    // 4. Wait for sensor to go HIGH (~80us)
    timeout = 500;
    while (gpio_get_level(DHT_PIN) == 0 && --timeout) esp_rom_delay_us(1);
    if (timeout <= 0) { portEXIT_CRITICAL(&dht_mux); ESP_LOGE(TAG, "DHT: Stuck LOW (stage 2)"); return -3; }

    // 5. Wait for sensor to go LOW (data start)
    timeout = 500;
    while (gpio_get_level(DHT_PIN) == 1 && --timeout) esp_rom_delay_us(1);
    if (timeout <= 0) { portEXIT_CRITICAL(&dht_mux); ESP_LOGE(TAG, "DHT: Stuck HIGH (stage 3)"); return -4; }

    // 6. Read 40-bit data
    for (int i = 0; i < 40; i++)
    {
        timeout = 500;
        while (gpio_get_level(DHT_PIN) == 0 && --timeout) esp_rom_delay_us(1);
        if (timeout <= 0) { portEXIT_CRITICAL(&dht_mux); ESP_LOGE(TAG, "DHT: Bit %d timeout (stage 4)", i); return -5; }

        esp_rom_delay_us(30);

        if (gpio_get_level(DHT_PIN) == 1)
        {
            data[i / 8] |= (1 << (7 - (i % 8)));
            timeout = 500;
            while (gpio_get_level(DHT_PIN) == 1 && --timeout) esp_rom_delay_us(1);
        }
    }

    portEXIT_CRITICAL(&dht_mux);

    // Checksum verification
    if (data[4] != ((data[0] + data[1] + data[2] + data[3]) & 0xFF)) {
        ESP_LOGE(TAG, "DHT: Checksum FAIL [%02X %02X %02X %02X %02X]", data[0], data[1], data[2], data[3], data[4]);
        return -6;
    }

    // DHT22: 16-bit values divided by 10
    uint16_t raw_hum  = (data[0] << 8) | data[1];
    uint16_t raw_temp = (data[2] << 8) | data[3];

    *hum  = raw_hum * 0.1f;
    *temp = (raw_temp & 0x8000) ? -(raw_temp & 0x7FFF) * 0.1f : raw_temp * 0.1f;

    return 0;
}



/*HTTP SERVER (WEB INTERFACE)*/


/**
 * @brief Serves the HTML dashboard when the client navigates to root (/).
 */
static esp_err_t root_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, dashboard_html, HTTPD_RESP_USE_STRLEN);
}

/**
 * @brief Returns real-time sensor values as JSON when the client requests /api/data.
 */
static esp_err_t api_data_handler(httpd_req_t *req)
{
    char json_resp[128];
    snprintf(json_resp, sizeof(json_resp), 
             "{\"temp\":%.1f, \"hum\":%.1f, \"mq2\":%d, \"mq7\":%d, \"flame\":%d}", 
             temperature, humidity, mq2_val, mq7_val, flame_val);
             
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, json_resp, HTTPD_RESP_USE_STRLEN);
}

// Root HTML page endpoint
static const httpd_uri_t uri_root = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = root_get_handler,
    .user_ctx  = NULL
};

// Sensor data API endpoint
static const httpd_uri_t uri_api_data = {
    .uri       = "/api/data",
    .method    = HTTP_GET,
    .handler   = api_data_handler,
    .user_ctx  = NULL
};

/**
 * @brief Starts the HTTP web server and registers URI handlers.
 */
static httpd_handle_t start_webserver(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;
    
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_register_uri_handler(server, &uri_root);
        httpd_register_uri_handler(server, &uri_api_data);
        ESP_LOGI(TAG, "Web server started successfully.");
    }
    return server;
}



/*WI-FI CONNECTION*/

/**
 * @brief Asynchronous event handler for Wi-Fi connection stages.
 */
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } 
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
        ESP_LOGI(TAG, "Failed to connect to WiFi, retrying...");
    } 
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "CONNECTION SUCCESSFUL! Device IP Address: " IPSTR, IP2STR(&event->ip_info.ip));
        start_webserver();
    }
}

/**
 * @brief Configures the Wi-Fi hardware and initiates network connection.
 */
void wifi_init_sta(void)
{
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL);

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_wifi_start();

    // Start mDNS service (access device by name instead of IP)
    mdns_init();
    mdns_hostname_set(HOSTNAME);
    mdns_instance_name_set("AirSense V4 IoT Monitor");
    ESP_LOGI(TAG, "mDNS service running. Access device at http://%s.local", HOSTNAME);
}



/*HARDWARE INITIALIZATION*/


/**
 * @brief Initializes all connected peripheral modules (I2S, ADC, I2C, etc.).
 */
void init_hardware()
{
    // 1. I2C & OLED Display Init
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER, 
        .sda_io_num = 8, 
        .scl_io_num = 9, 
        .sda_pullup_en = 1, 
        .scl_pullup_en = 1, 
        .master.clk_speed = 400000 
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0));
    vTaskDelay(pdMS_TO_TICKS(100));
    oled_handle = ssd1306_create(I2C_NUM_0, 0x3C);
    if (oled_handle == NULL) {
        ESP_LOGE(TAG, "OLED: ssd1306_create FAILED! Check I2C wiring (SDA=GPIO8, SCL=GPIO9).");
    } else {
        ESP_LOGI(TAG, "OLED: Initialized successfully on I2C address 0x3C.");
    }

    // 2. I2S Audio Output Init
    i2s_config_t i2s_config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_TX,
        .sample_rate = 22050,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .dma_buf_count = 8,
        .dma_buf_len = 512,
        .use_apll = false
    };
    i2s_pin_config_t pin_config = {
        .bck_io_num = 16,
        .ws_io_num = 15,
        .data_out_num = 17,
        .data_in_num = I2S_PIN_NO_CHANGE 
    };
    i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_PORT, &pin_config);

    // 3. Analog Sensor (ADC) Pin Configuration
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(MQ2_CH, ADC_ATTEN_DB_12);
    adc1_config_channel_atten(MQ7_CH, ADC_ATTEN_DB_12);
    
    // 4. Digital Sensor Pin Configuration
    gpio_set_direction(FLAME_PIN, GPIO_MODE_INPUT);
    dht_init();
}



/*MAIN SENSOR LOOP*/


/**
 * @brief FreeRTOS task that continuously reads sensors and makes decisions in real-time.
 */
void sensor_task(void *pvParameters)
{
    char oled_txt[32];
    int loops = 0;

    while (1) {
        // 1. Read new values from sensors
        mq2_val = adc1_get_raw(MQ2_CH);
        mq7_val = adc1_get_raw(MQ7_CH);
        flame_val = gpio_get_level(FLAME_PIN);

        // Read temperature/humidity every 2 seconds, with retry
        if (loops % 20 == 0) {
            int dht_ret = -1;
            for (int retry = 0; retry < 3 && dht_ret != 0; retry++) {
                dht_ret = dht_read(&temperature, &humidity);
                if (dht_ret != 0) vTaskDelay(pdMS_TO_TICKS(250));
            }
            if (dht_ret == 0) {
                ESP_LOGI(TAG, "DHT OK: Temp=%.1f C, Hum=%.1f %%", temperature, humidity);
            }
        }
        loops++;

        // 2. Clear OLED screen for fresh frame
        ssd1306_clear_screen(oled_handle, 0x00);
        
        // 3. Hazard Decision: Flame triggered (0) OR gas above thresholds?
        bool is_dangerous = (flame_val == 0 || mq2_val > GAS_DANGER_THRESHOLD || mq7_val > GAS_DANGER_THRESHOLD);

        if (is_dangerous) {
            ssd1306_draw_string(oled_handle, 20, 20, (const uint8_t *)"!! DANGER !!", 16, 1);
            
            if (flame_val == 0) {
                ssd1306_draw_string(oled_handle, 10, 40, (const uint8_t *)"FLAME DETECTED", 12, 1);
            } else {
                ssd1306_draw_string(oled_handle, 10, 40, (const uint8_t *)"HIGH GAS LEVEL", 12, 1);
            }
            ssd1306_refresh_gram(oled_handle);

            play_alarm_beep_5_sec();
            
            // Cool-down period to prevent false re-triggers
            vTaskDelay(pdMS_TO_TICKS(1000));
        } else {
            // -- STATUS: SAFE --
            snprintf(oled_txt, sizeof(oled_txt), "Temp: %d C", (int)temperature);
            ssd1306_draw_string(oled_handle, 0, 0, (const uint8_t *)oled_txt, 12, 1);
            
            snprintf(oled_txt, sizeof(oled_txt), "Humidity: %%%d", (int)humidity);
            ssd1306_draw_string(oled_handle, 0, 13, (const uint8_t *)oled_txt, 12, 1);
            
            snprintf(oled_txt, sizeof(oled_txt), "MQ2 (LPG): %d", mq2_val);
            ssd1306_draw_string(oled_handle, 0, 26, (const uint8_t *)oled_txt, 12, 1);
            
            snprintf(oled_txt, sizeof(oled_txt), "MQ7 (CO):  %d", mq7_val);
            ssd1306_draw_string(oled_handle, 0, 39, (const uint8_t *)oled_txt, 12, 1);
            
            ssd1306_draw_string(oled_handle, 0, 52, (const uint8_t *)"No Fire (Safe)", 12, 1);
            
            ssd1306_refresh_gram(oled_handle);
            
            vTaskDelay(pdMS_TO_TICKS(100)); 
        }
    }
}



/*APPLICATION ENTRY*/


/**
 * @brief Main entry point executed when ESP32 powers on.
 */
void app_main(void)
{
    // 1. Initialize NVS Flash (required for Wi-Fi credentials storage)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 2. Power up all GPIO and I2C lines
    init_hardware();

    // 3. Show startup message on OLED
    if (oled_handle != NULL) {
        ssd1306_clear_screen(oled_handle, 0x00);
        ssd1306_draw_string(oled_handle, 0, 20, (const uint8_t *)"System Starting...", 12, 1);
        ssd1306_draw_string(oled_handle, 0, 35, (const uint8_t *)"Connecting to WiFi", 12, 1);
        esp_err_t oled_ret = ssd1306_refresh_gram(oled_handle);
        if (oled_ret != ESP_OK) {
            ESP_LOGE(TAG, "OLED: refresh_gram failed (err=%d). Check I2C bus.", oled_ret);
        } else {
            ESP_LOGI(TAG, "OLED: Startup message displayed OK.");
        }
    }

    // 4. Start Wi-Fi connection and activate Web Server
    wifi_init_sta();

    // 5. Launch sensor_task pinned to Core 1 (Core 0 is used by WiFi)
    xTaskCreatePinnedToCore(sensor_task, "sensor_task", 8192, NULL, 5, NULL, 1);
}
