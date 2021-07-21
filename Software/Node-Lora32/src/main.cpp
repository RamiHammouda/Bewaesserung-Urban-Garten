//Use Arduino framework on PlatformIO
#include <Arduino.h>
//For Lorawan 
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
//For using ESP32 framework
#include <ESP.h>
//For Oled in Lora32 TTGO v2.1.6
#include <SSD1306.h>
//#include "SSD1306Wire.h"
#define OLED_I2C_ADDR 0x3C
#define OLED_RESET 16
#define OLED_SDA 21 //4  //21,22 for pin on Lora32 Oled v2.1.6
#define OLED_SCL 22 //15


//Created Class for wrapping sensor/actor
#include "UltrasonicSensor.h"
#include "PressureSensor.h"
#include "MoistureSensor.h"
#include "Actor.h"
#include "ActorGroup.h"

UltrasonicSensor ultrasonicSS;
PressureSensor pressureSS;
MoistureSensor moistureSS;
//boolean option ist for led-demonstration only, when we connect to Inverted relay
//default true: inverted relay is connected
//false: inverted relay ist not connected
Actor myPump(false),myVentil(false);
ActorGroup myActorSet;

//Define to check error of pump and ventil while operating
#define OPERATION_MODE
int notifyError = 100;

//Obsolete
//only for relay multiple mode- led notification only
//#define RELAY_INVERT
#ifdef RELAY_INVERT
bool mode = LOW;
#else
bool mode = HIGH;
#endif


//For BMESensor
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme; // I2C
SSD1306 display(OLED_I2C_ADDR, OLED_SDA, OLED_SCL);

//For Ultrasonic Sensor
//#include <NewPing.h>

#define TRIGGER_PIN 13   //34 (Important to pay attention, that pins from 34 to 39 are not able to be output )
#define ECHO_PIN 35      //35 (only be able to input)
#define MAX_DISTANCE 400 // Maximum sensor distance is rated at 400-500cm.
//NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);


//For Waterpressure Sensor
#define WATER_PRESSURE_PIN 36
//For WaterFlow Sensor
#define WATER_FLOW_PIN 34

//For Soil Moisture Sensor
#define MOISTURE_PIN 39

#define LEDPIN 3        //17
#define ACTOR_PUMPE 15  //23 //Led Red
#define ACTOR_VENTIL 12 //25  //Led Blue

float humi = 0, temC = 0, airPress = 0, altitude = 0, adcValue = 0, waterPress = 0;
int distanceToWaterSurface = 0, waterLeftInTank = 0; //(%) we will send waterleftIntank in % for better understanding
const int distanceToWaterAtBottomTank = 80;          //placeholder, accuray parameter is better;
int nextStop= 0;
//internal controlling
bool debugModeRq = true, manualModeRq = false, openVentilRq = false, openPumpRq = false, activeWateringRq = false;
//moisture level to turn on watering by Pertage = 95% = 0.95*1023 = 971; WaterVolume to Off by Litre (l)
int moistureOnPertg = 95, moistureOffPertg = 90, waterLvlOffPertg = 10, waterVolOffLit = 10 ; //200L, 10L only for testing

//variable for Waterflowsensor
int soilMoistureValue = 0, waterFlowSpeedValue = 0;
static unsigned int waterFlowVolValue = 0;
unsigned int flowMilliLitres;
unsigned long totalMilliLitres;
long currentMillis = 0, previousMillis = 0;
int intervalCheck = 1000;
//calibration factor
float calibrationFactor = 4.5;

volatile byte pulseCount;
byte pulse1Sec = 0;
float flowRate;
//Taskhandler in FreeRTOS for separate Core 0, to handle WaterflowSensor
TaskHandle_t Task1;

void IRAM_ATTR pulseCounter()
{
  pulseCount++;
}

//Structur for custom Lorawan Signal
typedef struct
{
  float temperature;
  float humidity;
  float airPressure;
  float waterPressure;
  int16_t waterLeft;
  int16_t soilMoisture;
  int16_t waterFlowSpeed;
  int16_t waterFlowVol;
  int16_t notifyCode;
  boolean openVentil;
  boolean openPump;
  boolean manualMode;
  
} sensorData;

#define PACKET_SIZE sizeof(sensorData)

typedef union
{
  sensorData sensor;
  byte LoRa_PacketBytes[PACKET_SIZE];
} LoRa_Packet;

LoRa_Packet mySensors;
char TTN_response[30];
int requestCodeFromServer;

//Device 2 on Server
// LoRaWAN NwkSKey, network session key
static const PROGMEM u1_t NWKSKEY[16] = {0xBF, 0x44, 0x04, 0x36, 0xB4, 0x9C, 0x5A, 0x85, 0x07, 0x5B, 0xEE, 0x33, 0xB6, 0x0A, 0x52, 0xCC};
// LoRaWAN AppSKey, application session key
static const u1_t PROGMEM APPSKEY[16] = {0xD4, 0xD6, 0x5D, 0xB5, 0x3C, 0xC0, 0x12, 0x75, 0xE3, 0x8E, 0x72, 0xD2, 0x90, 0x69, 0x3B, 0xD5};
static const u4_t DEVADDR = 0x260139A5;

void os_getArtEui(u1_t *buf) {}
void os_getDevEui(u1_t *buf) {}
void os_getDevKey(u1_t *buf) {}

// payload to send to TTN gateway
//static uint8_t payload[7];
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
static unsigned TX_INTERVAL = 5;
int short_interval = 5, long_interval = 58; //60s
// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 18,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 14,
    .dio = {26, 33, 32} // Pins for the Heltec ESP32 Lora board/ TTGO Lora32 with 3D metal antenna
};

//Re-implement map with float, incase we need to use float as input with small range
float mapWithFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  const float dividend = out_max - out_min;
  const float divisor = in_max - in_min;
  const float delta = x - in_min;
  if (divisor == 0)
  {
    log_e("Invalid map input range, min == max");
    return -1;
  }
  return (delta * dividend + (divisor / 2)) / divisor + out_min;
}

/// Main function to calculate waterflow sensor
void processWaterFlowSensor(void *pvParameters)
{
  for (;;)
  {
    currentMillis = millis();
    if (currentMillis - previousMillis > intervalCheck)
    {

      pulse1Sec = pulseCount;
      pulseCount = 0;
      flowRate = ((1000.0 / (millis() - previousMillis)) * pulse1Sec) / calibrationFactor;
      previousMillis = millis();
      flowMilliLitres = (flowRate / 60) * 1000;
      totalMilliLitres += flowMilliLitres;

      waterFlowSpeedValue = (int)flowRate;
      waterFlowVolValue = totalMilliLitres/1000;
    }

    vTaskDelay(5);
  }
}

void turnOnVentil()
{ 
  myActorSet.turnOnPassiveActors();
  openVentilRq = true;
};
void turnOffVentil()
{
  myActorSet.turnOffPassiveActors();
  openVentilRq = false;
}
void turnOnPump()
{
  myActorSet.turnOnActiveActors();
  openPumpRq = true;
};
void turnOffPump()
{
  myActorSet.turnOffActiveActors();
  openPumpRq = false;
}
//delay 2s to avoid dynamic water pressure detroying system
void turnOnWatering()
{
  myActorSet.turnOnAllActors();
  openVentilRq = true;
  openPumpRq = true;
}
void turnOffWatering()
{
  myActorSet.turnOffAllActors();
  openVentilRq = false;
  openPumpRq = false;
}
void turnOnDebugMode()
{
  debugModeRq = true;
  TX_INTERVAL = short_interval;
  Serial.println("Set update interval time = " + String(short_interval) + "s");
}
void turnOffDebugMode()
{
  debugModeRq = false;
  TX_INTERVAL = long_interval;
  Serial.println("Set update interval time = " + String((float)long_interval / 60) + "min");
}
String boolToString(bool value) { return value ? "TRUE" : "FALSE"; }
void HandlerDownlinksFromServer(lmic_t &lmic)
{
  int i = 0;
  // data received in rx slot after tx
  Serial.print(F("Data Received: "));
  Serial.write(lmic.frame + lmic.dataBeg, lmic.dataLen);
  Serial.println();

  //display.drawString (0, 20, "Received DATA.");
  for (i = 0; i < lmic.dataLen; i++)
    TTN_response[i] = lmic.frame[lmic.dataBeg + i];

  TTN_response[i] = 0;
  char *pNext;
  //requestCodeFromServer = (int) (TTN_response);
  requestCodeFromServer = strtol(TTN_response, &pNext, 10);
  display.drawString(0, 20, "Request Code " + String(requestCodeFromServer));
  Serial.println(String(requestCodeFromServer));

  //Request Code:
  //11001: TurnOnWatering on Manual Mode
  //10001: TurnOffWatering on Manual Mode
  //210xx: Moisture level to active watering from above xx% on Auto Mode
  //200xx: Moisture level to turn off watering from below xx% and below on Auto Mode
  //300xx: Water level to turn off watering from below xx% on Auto Mode
  //40xxx: Water Volume to turn off watering from above xxx% automatically
  //71001: Turn on Pumpe (for testing only)
  //70001: Turn off Pumpe (for testing only)
  //71002: Turn on Ventils (for testing only)
  //70002: Turn off Ventils (for testing only)
  //81001: Turn on Manual Mode
  //80001: Turn off Manual Mode (change to AutoMode)
  //91001: Turn on Debug Mode (TX_INTERVAL will be shorter): actually 9s update change on Server
  //90001: Turn off Debug Mode (TX_INTERVAL will be longer): 30min to update to Server

  switch (requestCodeFromServer)
  {
  case 11001:
    if (manualModeRq)
      turnOnWatering();
    break; //31 31 30 30 31 in Hexa
  case 10001:
    if (manualModeRq)
      turnOffWatering();
    break; //31 30 30 30 31
  case 71001:
    turnOnPump();
    break; //37 31 30 30 31
  case 70001:
    turnOffPump();
    break; //37 30 30 30 31
  case 71002:
    turnOnVentil();
    break; //37 31 30 30 32
  case 70002:
    turnOffVentil();
    break; //37 30 30 30 32
  case 81001:
    manualModeRq = true;
    break; //38 31 30 30 31
  case 80001:
    manualModeRq = false;
    break;//38 30 30 30 31
  case 91001:
    turnOnDebugMode();
    break;
  case 90001:
    turnOffDebugMode();
    break;
  //case 99999: Serial.println("System will reboot in 10s"); delay(10000);ESP.restart();break;
  default:
    if (requestCodeFromServer >= 21000 && requestCodeFromServer <= 21100)
    {
      moistureOnPertg = requestCodeFromServer == 21100 ? 100 : requestCodeFromServer % 100;
      Serial.println("Set Moisture On Pertage: " + String(moistureOnPertg) + "%");
    }
    if (requestCodeFromServer >= 20000 && requestCodeFromServer <= 20100)
    {
      moistureOffPertg = requestCodeFromServer == 20100 ? 100 : requestCodeFromServer % 100;
      Serial.println("Set Moisture Off Pertage: " + String(moistureOffPertg) + "%");
    }
    if (requestCodeFromServer >= 30000 && requestCodeFromServer <= 30100)
    {
      waterLvlOffPertg = requestCodeFromServer == 30100 ? 100 : requestCodeFromServer % 100;
      Serial.println("Set Water level Off Pertage: " + String(waterLvlOffPertg) + "%");
    }
    if (requestCodeFromServer >= 40000 && requestCodeFromServer <= 40200)
    {
      waterVolOffLit = requestCodeFromServer == 40200 ? 200 : requestCodeFromServer % 200;
      Serial.println("Set Water flow Volume Off Listre: " + String(waterVolOffLit) + "L");
    }
  }
}

int errorChecking(){
  if (openVentilRq && openPumpRq && waterFlowSpeedValue< 1){
    //This is only simulation testing.
    //Because these below values need to be suitable to pratical operation
    //These values might need to be changed
      if (waterPress>5.5){
        Serial.println("Got stuck!!! May be ventil defect or stuck in Hose");
        return 101;
      }
      if (waterPress< 1.3){
        Serial.println("Pump defect");
        return 102;
      }
      return 100;}
  return 100; 
}
//Error Handler function
//It checks error Code, current status and give a corresponding handle
void errorHandler(){
  Serial.println("System Error checking");
  if (errorChecking()==100)
    return;
  Serial.println("Defect detected, All Actors must be shutdowned now");
  myActorSet.turnOffAllActors();
  manualModeRq = true;
}
//With old version of LMIC Library we can only add inline function into provided function of LMIC
//With new version of LMIC now we can add normal functions
//And now functions in main loop can also work well (but need to pay attention to avoid conflict between function in main loop vs scheduled transmission or block transsmisstion)
inline void GetAndUpdateSensorData()
{
  temC = bme.readTemperature() - 1.5; //calibration
  humi = bme.readHumidity();

  if (isnan(humi) || isnan(temC))
  {
    Serial.println(F("Failed to read from bme sensor!"));
    return;
  }
  airPress = bme.readPressure() / 100.0F;
  altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);

  //Ultrasonic sensor:
  distanceToWaterSurface = (int)ultrasonicSS.getDistance_cm();
  Serial.println("actually mesuared distance " + String(distanceToWaterSurface));
  //Remove outliner of Value
  distanceToWaterSurface = distanceToWaterSurface > distanceToWaterAtBottomTank ? distanceToWaterAtBottomTank : distanceToWaterSurface;
  Serial.println("distance after fix " + String(distanceToWaterSurface));
  //water lef in tank wird be calculated in percentage
  waterLeftInTank = (float)(distanceToWaterAtBottomTank - distanceToWaterSurface) / distanceToWaterAtBottomTank * 100;
  Serial.printf("Temp: %.2f °C, Humidity: %.2f %%, Air Pressure: %.2f hPa, Altitude: %.2f m\n", temC, humi, airPress, altitude);
  Serial.println("Water Level: " + String(waterLeftInTank) + " %");

  //Get Waterpressure Data:
  waterPress = pressureSS.getPressure_bar();
  Serial.println("Water Pressure: " + String(waterPress) + " Bar");

  //Get Waterflow data

  //Get soil Moisture data
  soilMoistureValue = (float) moistureSS.getMoisture_percent();
  if (isnan(soilMoistureValue))
  {
    Serial.println(F("Failed to read soil Moisture value from Sensor!"));
    return;
  }

  Serial.println("Moisture Value: " + String(soilMoistureValue) + "% - Flow rate: " + String(int(waterFlowSpeedValue)) + "L/min \t Total Liquid Quantity: " + String(waterFlowVolValue) + "L\n");

  #ifdef OPERATION_MODE
  notifyError = errorChecking();
  #endif

  //Packing data to send
  mySensors.sensor.temperature = temC;
  mySensors.sensor.humidity = humi;
  mySensors.sensor.airPressure = airPress / 1000;
  mySensors.sensor.waterPressure = waterPress;
  mySensors.sensor.waterLeft = waterLeftInTank;
  mySensors.sensor.soilMoisture = soilMoistureValue;
  mySensors.sensor.waterFlowSpeed = waterFlowSpeedValue;
  mySensors.sensor.waterFlowVol = waterFlowVolValue;
  mySensors.sensor.notifyCode = (int16_t)notifyError;
  mySensors.sensor.openVentil = openVentilRq;
  mySensors.sensor.openPump = openPumpRq;
  mySensors.sensor.manualMode = manualModeRq;


  digitalWrite(LEDPIN, HIGH);
}

/**
 * Main function to evaluate various parameter to give final decision for controlling watering system
 */
void operateSystemAutomatically()
{
  if (manualModeRq || (errorChecking()!=100))
    return;

  bool moistureOn, moistureOff, waterLvlOff, waterVolOff, finalDecision;
  moistureOn = soilMoistureValue > moistureOnPertg;
  moistureOff = soilMoistureValue < moistureOffPertg;
  waterLvlOff = waterLeftInTank < waterLvlOffPertg;
  waterVolOff = waterFlowVolValue > waterVolOffLit;

  Serial.println("Condition check:\nPass Check Moisture On: " + boolToString(moistureOn) + "\nPass Check Moisture Off: " + boolToString(!moistureOff) +
                 "\nPass Check Waterlevel Off: " + boolToString(!waterLvlOff) + "\nPass Check WaterVolume Off: " + boolToString(!waterVolOff));
  finalDecision = moistureOn && !moistureOff && !waterLvlOff && !waterVolOff;

  Serial.println("Final Decision - Turn On Watering: " + boolToString(finalDecision) + "\n\n");
  if (finalDecision)
  {
    delay(500);
    turnOnWatering();
  }
  else
  {
    turnOffWatering();
      //if (waterFlowVolValue > 0)
      //ESP.restart();
  }
}

void displayAllInforOnOled()
{
  //oled:
  display.drawString(0, 10, String(millis()));
  display.drawString(0, 30, "Te " + String(temC) + "°C");
  display.drawString(70, 30, "Hu " + String(humi) + "%");
  display.drawString(0, 40, "A.P " + String(airPress / 1000) + "Bar");
  display.drawString(70, 40, "W.P " + String(waterPress) + "Bar");
  display.drawString(0, 50, "W.L " + String(waterLeftInTank) + "%");
  display.drawString(45, 50, "M " + String(soilMoistureValue) + "%");
  display.drawString(75, 50, " W.Vl  " + String(waterFlowVolValue) + "L");
  display.display();
}

void do_send(osjob_t *j)
{
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND)
  {
    Serial.println(F("OP_TXRXPEND, not sending"));
  }
  else
  {
    // read the sensorvalue
    GetAndUpdateSensorData();
    operateSystemAutomatically();
    errorHandler();

    LMIC_setTxData2(16, mySensors.LoRa_PacketBytes, sizeof(mySensors.LoRa_PacketBytes), 0);
  }
  // Next TX is scheduled after TX_COMPLETE event.
}

void onEvent(ev_t ev)
{
  Serial.print(os_getTime());
  Serial.print(": ");
  switch (ev)
  {
  case EV_SCAN_TIMEOUT:
    Serial.println(F("EV_SCAN_TIMEOUT"));
    break;
  case EV_BEACON_FOUND:
    Serial.println(F("EV_BEACON_FOUND"));
    break;
  case EV_BEACON_MISSED:
    Serial.println(F("EV_BEACON_MISSED"));
    break;
  case EV_BEACON_TRACKED:
    Serial.println(F("EV_BEACON_TRACKED"));
    break;
  case EV_JOINING:
    Serial.println(F("EV_JOINING"));
    break;
  case EV_JOINED:
    Serial.println(F("EV_JOINED"));
    break;
  case EV_JOIN_FAILED:
    Serial.println(F("EV_JOIN_FAILED"));
    break;
  case EV_REJOIN_FAILED:
    Serial.println(F("EV_REJOIN_FAILED"));
    break;
  case EV_TXCOMPLETE:
    //Oled
    display.clear();
    display.drawString(0, 0, "EV_TXCOMPLETE event!");

    Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
    if (LMIC.txrxFlags & TXRX_ACK)
      Serial.println(F("Received ack"));
    if (LMIC.dataLen)
    {
      Serial.println("Received " + String(LMIC.dataLen) + " bytes of payload");
      HandlerDownlinksFromServer(LMIC);
    }
    // Schedule next transmission
    os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
    //Turn led:
    digitalWrite(LEDPIN, LOW);
    displayAllInforOnOled();
    break;
  case EV_LOST_TSYNC:
    Serial.println(F("EV_LOST_TSYNC"));
    break;
  case EV_RESET:
    Serial.println(F("EV_RESET"));
    break;
  case EV_RXCOMPLETE:
    // data received in ping slot
    Serial.println(F("EV_RXCOMPLETE"));
    break;
  case EV_LINK_DEAD:
    Serial.println(F("EV_LINK_DEAD"));
    break;
  case EV_LINK_ALIVE:
    Serial.println(F("EV_LINK_ALIVE"));
    break;
  case EV_TXSTART:
    Serial.println(F("EV_TXSTART"));
    break;
  case EV_TXCANCELED:
    Serial.println(F("EV_TXCANCELED"));
    break;
  case EV_RXSTART:
    /* do not print anything -- it wrecks timing */
    break;
  case EV_JOIN_TXCOMPLETE:
    Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
    break;
  default:
    Serial.print(F("Unknown event: "));
    Serial.println((unsigned)ev);
    break;
  }
}

void setup()
{
  delay(1500);
  while (!Serial)
    ;
  Serial.begin(9600);
  delay(100);
  Serial.println(F("Starting"));

  //start bme sensor:
  if (!bme.begin(0x76))
  {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1)
      delay(10);
  }
  pinMode(LEDPIN, OUTPUT);
  //pinMode(ACTOR_VENTIL, OUTPUT);
  //pinMode(ACTOR_PUMPE, OUTPUT);
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(WATER_FLOW_PIN, INPUT_PULLUP);
  pinMode(MOISTURE_PIN, INPUT);
  //use default same SDA,SCL of I2C Pins for BME Sensor

  //Custom classes:
  ultrasonicSS.set_trig_echo(TRIGGER_PIN,ECHO_PIN);
  pressureSS.setInputPin(WATER_PRESSURE_PIN);
  moistureSS.setInputPin(MOISTURE_PIN);

  myPump.setOutputPin(ACTOR_PUMPE);
  myVentil.setOutputPin(ACTOR_VENTIL);

  myActorSet.addActiveActor(myPump);
  myActorSet.addPassiveActor(myVentil);


  pulseCount = 0;
  flowRate = 0.0;
  flowMilliLitres = 0;
  totalMilliLitres = 0;
  previousMillis = 0;

  attachInterrupt(digitalPinToInterrupt(WATER_FLOW_PIN), pulseCounter, FALLING);

  xTaskCreatePinnedToCore(
      processWaterFlowSensor, /* Task function. */
      "Task1",                /* name of task. */
      5000,                   /* Stack size of task */
      NULL,                   /* parameter of the task */
      1,                      /* priority of the task */
      &Task1,                 /* Task handle to keep track of created task */
      0);                     /* pin task to core 0 */
  delay(500);

  // reset the OLED
  pinMode(OLED_RESET, OUTPUT);
  digitalWrite(OLED_RESET, LOW);
  delay(50);
  digitalWrite(OLED_RESET, HIGH);

  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);

  display.setTextAlignment(TEXT_ALIGN_LEFT);

  display.drawString(0, 0, "Init!");
  display.display();

  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  // Copy static session parameters to a temporary buffer here
  uint8_t appskey[sizeof(APPSKEY)];
  uint8_t nwkskey[sizeof(NWKSKEY)];
  memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
  memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
  LMIC_setSession(0x13, DEVADDR, nwkskey, appskey);

  // We'll disable all 72 channels used by TTN
  for (int c = 0; c < 72; c++)
  {
    LMIC_disableChannel(c);
  }

  LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
  LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI); // g-band
  LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
  LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
  LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
  LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
  LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
  LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
  LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK, DR_FSK), BAND_MILLI);   // g2-band

  // Disable link check validation
  LMIC_setLinkCheckMode(0);

  // TTN uses SF9 for its RX2 window.
  LMIC.dn2Dr = DR_SF7;

  // Set data rate and transmit power for uplink
  LMIC_setDrTxpow(DR_SF7, 14);

  // Start job
  do_send(&sendjob);
}

void loop()
{
  os_runloop_once();
  
}
