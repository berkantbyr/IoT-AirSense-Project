# Air Sense — ESP32-S3 Multi-Sensor IoT Monitor

**TR** [Türkçe sürüm için aşağıya kaydırın](#türkçe-sürüm--turkish-version)

Air Sense is an **ESP32-S3** IoT project that reads **temperature**, **humidity**, **flammable gas (MQ-2)**, **carbon monoxide (MQ-7)**, and **flame** status. On a hazard it plays a **5-second I2S square-wave alarm** through a speaker, shows status on an **SSD1306 OLED**, and serves a **local web dashboard** with JSON API. Firmware is written in **C** with **ESP-IDF**; there is no separate PC server—the HTTP server runs on the device.

---

## Features

- **Environmental sensing:** **DHT22** on a single GPIO for temperature and relative humidity.
- **Gas & flame detection:** **MQ-2** (LPG / combustible gases) and **MQ-7** (CO) on **ADC1**; **digital flame sensor** on GPIO.
- **Alarm:** When flame is detected or raw ADC values exceed **`GAS_DANGER_THRESHOLD`**, a loud **I2S** beep runs for about five seconds.
- **Display:** **SSD1306** OLED (I2C) shows live readings or a danger screen.
- **Connectivity:** **Wi-Fi** station mode; **mDNS** hostname (default **`http://airsense.local`**, configurable via **`HOSTNAME`**).
- **Web UI:** Embedded HTML dashboard at **`/`**; machine-readable data at **`GET /api/data`** (`temp`, `hum`, `mq2`, `mq7`, `flame`).

---

## Hardware Requirements

- ESP32-S3 development board
- DHT22 temperature & humidity sensor
- MQ-2 flammable gas sensor module
- MQ-7 carbon monoxide sensor module
- Digital flame sensor (output tied to **GPIO 5** in firmware)
- SSD1306 OLED (128×64, I2C address **0x3C**)
- I2S audio amplifier + speaker (BCK **GPIO 16**, WS **GPIO 15**, DATA **GPIO 17**)
- Jumper wires and stable **3.3 V** / **5 V** supplies as required by your modules

**Pin summary (see `main/iot_monitor_main.c`):** I2C **SDA GPIO 8**, **SCL GPIO 9**; DHT **GPIO 4**; MQ-2 **ADC1_CH0 (GPIO 1)**; MQ-7 **ADC1_CH1 (GPIO 2)**; flame **GPIO 5**.

---

## Software Setup & Usage

### 1 — PC: ESP-IDF toolchain

1. Install **Espressif ESP-IDF** (Windows: official **ESP-IDF Windows Installer**, or follow the [ESP-IDF setup guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/get-started/)). Use **ESP-IDF PowerShell**, **ESP-IDF CMD**, or the ESP-IDF VS Code extension so that **`idf.py`** is on your `PATH`.
2. Open a terminal where **`idf.py`** works, then change to this project directory.

### 2 — ESP32: configuration

1. Open **`main/iot_monitor_main.c`**.
2. Set your Wi-Fi credentials:
   - **`WIFI_SSID`** — your network name  
   - **`WIFI_PASS`** — your network password  
3. Optional: change **`HOSTNAME`** (mDNS name, e.g. `airsense` → `http://airsense.local`).
4. Calibrate **`GAS_DANGER_THRESHOLD`** (default **3500**) for your room and sensor placement so normal air does not trigger false alarms.

### 3 — Build, flash, monitor

```bash
idf.py set-target esp32s3
idf.py build
idf.py -p COM3 flash monitor
```

Replace **`COM3`** with your board’s serial port (Device Manager on Windows).

---

## 3 — How It Works

After boot, the ESP32 connects to Wi-Fi, starts **mDNS** with the configured hostname, and starts the **HTTP server** when an IP address is obtained. The **`sensor_task`** loop reads ADC values, the flame GPIO, and periodically the DHT22; it updates the OLED and evaluates danger (flame **LOW** or gas raw values above threshold). In danger mode it shows a warning on the OLED and plays the I2S alarm. The browser dashboard at **`/`** shows the embedded UI; clients can poll **`/api/data`** for JSON snapshots.

**Security note:** Do not commit real **Wi-Fi passwords** or other secrets in **`iot_monitor_main.c`** or push them to public repositories. Use placeholders locally and rotate credentials if they were ever exposed.

---

---

## [Türkçe Sürüm / Turkish Version]

Air Sense, **ESP32-S3** üzerinde çalışan bir IoT projesidir: **sıcaklık**, **nem**, **yanıcı gaz (MQ-2)**, **karbon monoksit (MQ-7)** ve **alev** bilgisini okur; tehlike durumunda hoparlörden **I2S** ile yaklaşık **5 saniye** alarm verir; verileri **SSD1306 OLED** ekranda ve cihaz üzerindeki **yerel web sunucusunda** gösterir. Yazılım **ESP-IDF** ile **C** dilindedir; ayrı bir bilgisayar sunucusu yoktur—HTTP sunucusu doğrudan ESP32 üzerindedir.

### Özellikler

- **Çevre ölçümü:** Tek pin üzerinden **DHT22** ile sıcaklık ve bağıl nem.
- **Gaz ve alev:** **MQ-2** ve **MQ-7** analog okuma; **dijital alev sensörü** GPIO ile.
- **Alarm:** Alev algılandığında veya ham ADC değerleri **`GAS_DANGER_THRESHOLD`** üzerindeyken **I2S** kare dalga alarmı.
- **Görüntü:** **SSD1306** OLED canlı değer veya tehlike ekranı.
- **Ağ:** **Wi-Fi** istasyon modu; **mDNS** ile varsayılan **`http://airsense.local`** ( **`HOSTNAME`** ile değiştirilebilir).
- **Web:** Kök dizinde gömülü pano (**`/`**); JSON için **`GET /api/data`** (`temp`, `hum`, `mq2`, `mq7`, `flame`).

### Donanım Gereksinimleri

- ESP32-S3 geliştirme kartı
- DHT22 sıcaklık ve nem sensörü
- MQ-2 yanıcı gaz sensör modülü
- MQ-7 karbon monoksit sensör modülü
- Dijital alev sensörü (firmware’de **GPIO 5**)
- SSD1306 OLED (128×64, I2C **0x3C**)
- I2S uyumlu amfi + hoparlör (BCK **GPIO 16**, WS **GPIO 15**, DATA **GPIO 17**)
- Gerekli bağlantı kabloları ve modülleriniz için uygun güç kaynağı

**Pin özeti (`main/iot_monitor_main.c`):** I2C **SDA GPIO 8**, **SCL GPIO 9**; DHT **GPIO 4**; MQ-2 **ADC1_CH0 (GPIO 1)**; MQ-7 **ADC1_CH1 (GPIO 2)**; alev **GPIO 5**.

### Yazılım Kurulumu ve Kullanımı

#### 1 — Bilgisayar: ESP-IDF

1. **Espressif ESP-IDF** kurun (Windows: resmi **ESP-IDF Windows Installer** veya [ESP-IDF kurulum kılavuzu](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/get-started/)). **`idf.py`** komutunun çalıştığı **ESP-IDF PowerShell**, **ESP-IDF CMD** veya VS Code ESP-IDF uzantısı ortamını kullanın.
2. Proje klasöründe terminal açın.

#### 2 — ESP32 yapılandırması

1. **`main/iot_monitor_main.c`** dosyasını açın.
2. **`WIFI_SSID`** ve **`WIFI_PASS`** değerlerini kendi ağınıza göre girin.
3. İsteğe bağlı: **`HOSTNAME`** (mDNS adı, örn. `airsense` → `http://airsense.local`).
4. Ortamınıza göre **`GAS_DANGER_THRESHOLD`** değerini kalibre edin (varsayılan **3500**).

#### 3 — Derleme, yükleme ve izleme

```bash
idf.py set-target esp32s3
idf.py build
idf.py -p COM3 flash monitor
```

**`COM3`** kısmını kartınızın seri portuna göre değiştirin (Windows’ta Aygıt Yöneticisi).

### Nasıl Çalışır?

Cihaz açıldığında Wi-Fi’ye bağlanır, **`HOSTNAME`** ile **mDNS** yayını yapar ve IP alındığında **HTTP sunucusunu** başlatır. **`sensor_task`** döngüsü ADC, alev pini ve periyodik DHT22 okumalarını yapar; OLED’i günceller ve tehlike durumunu değerlendirir (alev **LOW** veya gaz eşiği aşımı). Tehlikede OLED’de uyarı ve I2S alarmı çalınır. Tarayıcıda **`/`** adresinde pano; **`/api/data`** ile JSON verisi alınabilir.

**Güvenlik notu:** **`iot_monitor_main.c`** içine gerçek **Wi-Fi şifrelerini** veya gizli anahtarları yazmayın ve bunları herkese açık depolara yüklemeyin. Yerelde yer tutucu kullanın; sızan kimlik bilgilerini değiştirin.
