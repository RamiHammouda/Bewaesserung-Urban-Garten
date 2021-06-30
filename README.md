# Bewaesserung Urban Garden

**HTW Smart Garden Watering System** is **_InternetOfThing_** **(IoT)**  **LoraWan** based Project from a wonderful suggestion of Mr. Prof. Dr.-Ing. Mohammad Abuosba. The target of the project is to build a smart watering system for an urban garden in HTW Berlin. 
<br/> <br/>
# Motivation
The ca.800m<sup>2</sup> Urban garden in HTW Berlin locates in the back of the university, next to the famous Spree River. It is a really beautiful garden which provides everyone fresh air along with really wonderful green feeling due to many trees and plants. And to have more time to enjoy it or to keep this feeling lasts forever, a smart watering system comes up as an answer :). 
<br/> <br/>
# Screenshot

  
# Usage


# Technology stack:
#### Operation System:
- Windows 10/(Linux)
#### IDE:
- Visual Studio Code
- Platform IO
#### Framework:
- Arduino
#### Dependency:
- MCCI LoraWAN LMIC Library v3.3
- Adafruit Unified Sensor v1.1.4
- DHT sensor library v1.4.2
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
- NodeJS
#### Hardware Programming: 
- C++
#### Hardware:
- Lora32 (TTGO)
<br/> <br/>
# Feature List

##### Controlling Sensors, Actors:
- [x] Collect current ambient parameters (Temperatur sensor, Humidity sensor, Air pressure sensor.. )
- [ ] Collect current water level in water tank (Ultrasound sensor)
- [ ] Collect current water hose parameters (Water pressure sensor, Water flow sensor)
- [ ] Control Actors (Water pump, Magnetic valve)
##### On LoraWan Server:
- [x] Send collected sensor data to TTN Network
- [x] Operate with ABP Mode or OTAA Mode (prefer OTAA)
- [x] Support float decode payload
- [x] Support int decode payload
- [x] Binding Server with opensensemap.org
##### Operate Actors:
- [x] Remote control actors manually
- [ ] Actors operates itself automatically based on pre-configuration
- [x] Auto Mode and Manual Mode should be selectable
##### Sensor Data Visualization:
- [x] Simple sensor data are visualizated by opensensemap
- [ ] Complex sensor data will be visualizated by a self development UI
##### UI Development:
- [ ] Create a UI to get all current information of system
- [x] Control actors through simple user-friendly UI like button, not on JSON UI 
      (comply to requirement from Mr. Prof. Dr.-Ing. Abuosba )
- [ ] Pre-configuration parameters should be changeable (e.g through input fields) 
      (comply to suggestion from Mr. Holger Martin )
##### Options:
- [ ] An UI on mobile for convenient monitoring
- [ ] Send Notification to user for alert 
 <br/> <br/>
## For testing purpose
#### THETHINGSNETWORK.ORG _(Legacy V2 Console)_
- ##### User name: _htwgardenproject_
- ##### Email address: _htwgardenss21@gmail.com_
- ##### Password: _htwgarden2021_
 <br/> <br/>
# License
**MIT** © **HTW Berlin - IngenieurInformatik SS21** - ***Team 4:*** **Khac Hoa Le, Rami Hammouda, Jaro Machnow**
