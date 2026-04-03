idf_component_register(SRCS "iot_monitor_main.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_http_server nvs_flash esp_wifi)