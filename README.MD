Air Sense —
An IoT application running on ESP32-S3. It reads temperature, humidity, flammable gas, carbon monoxide, and flame status from the environment; triggers an alarm via speaker in case of danger; displays data on an OLED screen and through a local web server.
Features

Sensors: DHT22 (temperature/humidity), MQ-2 (flammable gas), MQ-7 (CO), digital flame sensor
Display: SSD1306 OLED
Audio: I2S output with ~5-second alarm
Network: Wi-Fi station mode, HTTP server, mDNS (http://airsense.local — configured via HOSTNAME in main/iot_monitor_main.c)
Web: Embedded HTML dashboard at root, JSON sensor data via /api/data



[ESP-IDF]
Target: ESP32-S3

Build and flash
In the project root (in a terminal with ESP-IDF environment loaded):
bashidf.py set-target esp32s3
idf.py build
idf.py -p PORT flash monitor
Replace PORT with typically COM7 on Windows
Wi-Fi and security
Network name and password are defined in main/iot_monitor_main.c as WIFI_SSID and WIFI_PASS.# Air Sense
An IoT application running on ESP32-S3. It reads temperature, humidity, flammable gas, carbon monoxide, and flame status from the environment; triggers an alarm via the speaker in case of danger; displays data on an OLED screen and through a local web server.
## How to run the project 
### 1. Install ESP-IDF
### 2. Configure Wi-Fi 
1. Open main/iot_monitor_main.c.
2. Set your network credentials:
   - WIFI_SSID — your Wi-Fi name  
   - WIFI_PASS — your Wi-Fi password  
3. Optionally change HOSTNAME if you want a different mDNS name (default airsense → http://airsense.local).
### 3. Connect the board
1. Wire sensors and OLED according to the table above.
2. Connect the ESP32-S3 to the PC.
### 4. Open the project in the ESP-IDF terminal
bash
cd "path\......"

Use the real folder path on your machine (spaces in the path are fine if quoted).
### 5. Set the chip target (once per clone / clean tree)
bash
idf.py set-target esp32s3

This selects ESP32-S3 and may regenerate build config.
### 6. Build the firmware
bash
idf.py build

Wait until you see **Project build complete** with no errors.
### 7. Flash firmware and open the serial monitor
Replace PORT with your port, e.g. on Windows often COM7:
bash
idf.py -p COM7 flash monitor

### 8. Verify that it is running
1. **Serial log:** After Wi-Fi connects, look for the device IP and mDNS message, e.g. http://airsense.local.
2. **Browser:** On the **same Wi-Fi network** as the ESP32, open:
   - http://airsense.local — dashboard 
3. **API:** Open http://airsense.local/api/data  — you should see JSON with temp, hum, mq2, mq7, flame.
The HTTP server starts **after** the board gets an IP address (IP_EVENT_STA_GOT_IP).

How to run the project
1. Install ESP-IDF extension in VS Code

Open Visual Studio Code.
Go to Extensions (Ctrl+Shift+X).
Search for "ESP-IDF" and install the official extension by Espressif.
After installation, press Ctrl+Shift+P and run "ESP-IDF: Configure ESP-IDF Extension".
Choose Express installation or select an existing ESP-IDF installation path.

2. Configure Wi-Fi

Open main/iot_monitor_main.c.
Set your network credentials:

WIFI_SSID — your Wi-Fi name
WIFI_PASS — your Wi-Fi password


Optionally change HOSTNAME if you want a different mDNS name (default airsense → http://airsense.local).

3. Connect the board

Wire sensors and OLED according to the table above.
Connect the ESP32-S3 to the PC via USB.

4. Open the project in VS Code

In VS Code, go to File → Open Folder.
Navigate to and select the project folder:

   path\......

VS Code will detect the ESP-IDF project structure.

5. Set the chip target

Press Ctrl+Shift+P and run "ESP-IDF: Set Espressif Device Target".
Select ESP32-S3 from the list.

Alternatively, open the ESP-IDF Terminal in VS Code and run:
bashidf.py set-target esp32s3
6. Build the firmware
Option A — Using the GUI:

Click the ESP-IDF Build Project button in the bottom status bar (🔨 icon).

Option B — Using the terminal:

Open the ESP-IDF Terminal in VS Code.
Run:

bash   idf.py build
Wait until you see Project build complete with no errors.
7. Select the serial port

Press Ctrl+Shift+P and run "ESP-IDF: Select Port to Use".
Choose your COM port (e.g., COM7 on Windows).

8. Flash firmware and open the serial monitor
Option A — Using the GUI:

Click the ESP-IDF Flash Device button (⚡ icon) in the bottom status bar.
After flashing completes, click the ESP-IDF Monitor Device button (📟 icon) to open the serial monitor.

Option B — Using the terminal:
bashidf.py -p COM7 flash monitor
Replace COM7 with your actual port.
9. Verify that it is running

Serial log: After Wi-Fi connects, look for the device IP and mDNS message, e.g. http://airsense.local.
Browser: On the same Wi-Fi network as the ESP32, open:

http://airsense.local 


API: Open http://airsense.local/api/data 

