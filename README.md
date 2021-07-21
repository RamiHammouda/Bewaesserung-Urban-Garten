# Bewaesserung Urban Garden

**HTW Smart Garden Watering System** is **_InternetOfThing_** **(IoT)**  **LoraWan** based Project from a wonderful suggestion of Mr. Prof. Dr.-Ing. Mohammad Abuosba. The target of the project is to build a smart watering system for an urban garden in HTW Berlin. 
<br/> <br/>
# Motivation
The ca.800m<sup>2</sup> Urban garden in HTW Berlin locates in the back of the university, next to the famous Spree River. It is a really beautiful garden which provides everyone fresh air along with really wonderful green feeling due to many trees and plants. And to have more time to enjoy it or to keep this feeling lasts forever, a smart watering system comes up as an answer :). 
<br/> <br/>
# Screenshot

<img src="https://i.ibb.co/7z0fqQ0/FULL-Image-2.png" width="550" title = "screenshot"> <img src="https://i.ibb.co/82QXB2x/FULL-Image-3.png" width="550" title = "UI Program">


  
# Usage


# Technology stack:
#### Operation System:
- Windows 10/(Linux)
#### IDE:
- Visual Studio Code
- Platform IO
#### Frameworks:
- Arduino
#### Dependencies:
- MCCI LoraWAN LMIC Library v4.4
- Adafruit Unified Sensor v1.1.4
- Adafruit BME280 v2.1.4
- Thingpulse/ESP8266 and ESP32 OLED driver for SSD1306 displays v4.2.0 (platformio lib install 2978)
- ~~NewPing v1.9.1~~ (replaced with our own implemented class)
#### Protocol:
- MQTT, HTTP
#### Connectivity Protocol:
- LoRaWAN (Long Range Wide Area Network)
#### Tools:
- MQTT Mosquito
- MQTT Explorer
- MQTT Fx
#### Server:
- TTN (thethingsnetwork)
#### UI Programming: 
- JavaScript
- Node-Red Framework
#### UI Dependencies:
- node-red-dashboard (base)
- node-red-contrib-ui-led (led only)
- node-red-contrib-telegrambot (connect to telegram bot API)
- node-red-contrib-remote (remote system on Mobile)
#### Hardware Programming: 
- C++
#### Hardware:
- Lora32 (TTGO-Lora v2.1.6)
+ BME280 Sensor
+ YF-S201 Waterflow sensor
+ Water pressure sensor 
+ HC-SR04 Ultrasonic sensor
+ Capacitive soil moisture sensor v1.2
+ Water pump
+ Magnetic Ventil
+ SRD-05VDC-SL-C 4 CH Relay
<br/> <br/>
# Feature List

##### Controlling Sensors, Actors:
- [x] Collect current ambient parameters (Temperatur sensor, Humidity sensor, Air pressure sensor.. )
- [x] Collect current water level in water tank (Ultrasound sensor)
- [x] Collect current water hose parameters (Water pressure sensor, Water flow sensor)
- [x] Control Actors (Water pump, Magnetic valve)
##### On LoraWan Server:
- [x] Send collected sensor data to TTN Network
- [x] Operate with ABP Mode or OTAA Mode (prefer OTAA)
- [x] Support float decode payload
- [x] Support int decode payload
- [x] Binding Server with opensensemap.org
##### Operate Actors:
- [x] Watering System can run independently of controlling UI
- [x] Actors can be manually remote-controlled 
- [x] Actors can operate itself automatically based on pre-configuration
     - [x] Watering will turn on if Moisture (Dryness) Value over a definied value (default is 95%)
     - [x] Watering will turn off if Moisture (Dryness) Value under a definied value (default is 90%)
     - [x] Watering will turn off if Water Level under a definied value (default is 10%)
     - [x] Watering will turn off if Waterflow Volume over a definied value (default is 10L for testing)(experiment feature)
- [x] Auto Mode and Manual Mode should be selectable (by Master Switch)
- [x] Remote control actors manually 
- [x] Updating-sensor Tempo is changeable (through Debug Mode on/off for quick/slow)
##### Sensor Data Visualization:
- [x] Simple sensor data are visualizated by opensensemap
- [x] Complex sensor data will be visualizated by a self development UI
##### UI Development:
- [x] Create an UI to get all current information of system ( Temperatur, Humidity, Moisture Value, Air Pressure, Water Pressure, Waterflow Speed, Waterflow Volume, current status of Ventil, current status of Pump, current Update Tempo, current operation status of system)
- [x] Assisten Management Information (Streng of Signal, Noise Ratio, relative Water Pressure, Curent Operation Mode )
- [x] Control actors through simple user-friendly UI like button, not on JSON UI 
      (comply to requirement from Mr. Prof. Dr.-Ing. Abuosba )
- [x] Pre-configuration parameters should be changeable (e.g through input fields) 
      (comply to suggestion from Mr. Holger Martin )
##### Options:
- [x] An UI on mobile for convenient monitoring
- [x] Send Notification to user for alert and notify current status automatically through Telegram with Bot "HTWGarden". (Only Admin received alert)
- [x] Everyone can query current important status of system through Telegram bot "HTWGarden"
- [x] Create wrapper classes for multiple sensors/actors (for an easy scalability)
 <br/> <br/>
## For testing purpose
#### THETHINGSNETWORK.ORG _(Legacy V2 Console)_
- ##### User name: _htwgardenproject_
- ##### Email address: _htwgardenss21@gmail.com_
- ##### Password: _htwgarden2021_

## UI Rebuild: (build from Flow.json file)
**Default:**
- **Logic base: _localhost:1880_**
- **UI board: _localhost:1880/ui_**
### MQTT Topic Setting:
- **MQTT-In Topic:**  **_mygardenproject/devices/sensortest02/up_**
- **MQTT-Out Topic:**  **_mygardenproject/devices/sensortest02/down_**
- **Server:**  **_eu.thethings.network_**
- **Port:**  **_1883_**
- **Protocol:**  **_MQTT V3.1.1_**
- **Username:**  **_mygardenproject_**
- **Password:**  **_ttn-account-v2.60jnFj-pF6rapK8BtiWsr2CQXM8TufQspWzjreeI2Zc_**
### Telegrambot:
- **Bot-Name: _HTWGarden_**
- **Token: _1565148953:AAHQ8Jx3c4r1mIf3uV5PS_4BW0aKDrDjfuM_**
 <br/> <br/>
# License
**MIT** © **HTW Berlin - IngenieurInformatik SS21** - **Team 4 - Khac Hoa Le, Rami Hammouda, Jaro Machnow**
