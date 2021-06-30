#include <Arduino.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

#include <Wire.h>
//#include "SSD1306Wire.h"
#include <SSD1306.h> //For Oled
//#include "SSD1306Wire.h"
#define OLED_I2C_ADDR 0x3C
#define OLED_RESET 16
#define OLED_SDA 21 //4  //21,22 for pin on Lora32 Oled v2.1.6
#define OLED_SCL 22 //15
SSD1306 display (OLED_I2C_ADDR, OLED_SDA, OLED_SCL);

#include "DHT.h"

// DHT digital pin and sensor type
#define LEDPIN 12
#define LEDACTOR 15
#define DHTPIN 25

#define DHTTYPE DHT22
// init. DHT
DHT dht(DHTPIN, DHTTYPE);
float humi = 0, temC = 0, heatC = 0;

typedef struct
{
  float temperature;
  float humidity;
  int16_t feellike;
} sensorData;

#define PACKET_SIZE sizeof(sensorData)

typedef union
{
  sensorData sensor;
  byte LoRa_PacketBytes[PACKET_SIZE];
} LoRa_Packet;

LoRa_Packet myTempSensor;
char TTN_response[30];
int commandCodeFromServer;

void TurnOnActor() { digitalWrite(LEDACTOR, HIGH); };
inline void TurnOffActor() { digitalWrite(LEDACTOR, LOW); }


void GetAndUpdateSensorData()
{
  temC = dht.readTemperature();
  humi = dht.readHumidity();

  if (isnan(humi) || isnan(temC))
  {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }
  heatC = dht.computeHeatIndex(false);

  //Serial.println(F("Reading sensor data"));
  Serial.printf("Temp: %.2f °C, Humidity: %.2f %%, Feel like: %.2f °C\n", temC, humi, heatC);

  myTempSensor.sensor.temperature = temC;
  myTempSensor.sensor.humidity = humi;
  myTempSensor.sensor.feellike = (int16_t)heatC;

  digitalWrite(LEDPIN, HIGH);
}

void displayAllInforOnOled(){
  //oled:
  //String text4Display = "Temp: "+ String(temC)+ "°C, Humidity: "+String(humi)+"%, Feel like: "+String(heatC)+ "°C";
  display.drawString (0, 10, String(millis()));
  display.drawString (0, 30, "Tempetaur    "+ String(temC)+"°C");
  display.drawString (0, 40, "Humidity        "+ String(humi)+"%");
  display.drawString (0, 50, "Feel like         "+ String(heatC)+"°C");
  display.display ();
  
}

// LoRaWAN NwkSKey, network session key
static const PROGMEM u1_t NWKSKEY[16] = {0xA5, 0xEE, 0x9B, 0xC0, 0xC5, 0x2D, 0xC1, 0xC7, 0xFF, 0xF8, 0x75, 0x42, 0x4F, 0xFF, 0x73, 0x0E};
// LoRaWAN AppSKey, application session key
static const u1_t PROGMEM APPSKEY[16] = {0xC1, 0x15, 0x8F, 0xCC, 0x5C, 0xEB, 0xE0, 0xB3, 0x36, 0x19, 0xD8, 0x28, 0x31, 0x7B, 0x86, 0xC7};
static const u4_t DEVADDR = 0x260114DA;

void os_getArtEui(u1_t *buf) {}
void os_getDevEui(u1_t *buf) {}
void os_getDevKey(u1_t *buf) {}

// payload to send to TTN gateway
//static uint8_t payload[7];
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
//const unsigned TX_INTERVAL = 5;
unsigned TX_INTERVAL = 5;
// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 18,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 14,
    .dio = {26, 33, 32} // Pins for the Heltec ESP32 Lora board/ TTGO Lora32 with 3D metal antenna
};

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
  //display.drawString (0, 32, String(TTN_response));
  char *pNext;
  commandCodeFromServer = (int) (TTN_response);
  commandCodeFromServer = strtol(TTN_response,&pNext,10);
  display.drawString (0, 20,"Request code:"+String(commandCodeFromServer));
  Serial.println(String(TTN_response));
  // if (String(TTN_response) == "1")
  //   TurnOnActor(); //But we can use inline fuction
  // else
  //   //turnOffActor();
  //   digitalWrite(LEDACTOR, LOW);
  switch (commandCodeFromServer){
    case 1: TurnOnActor();break;
    case 0: TurnOffActor();break;
  }

  if ((int)(commandCodeFromServer/1e5)==9){
    Serial.println("Try to set Interval"+String(commandCodeFromServer-9e5));
    TX_INTERVAL = commandCodeFromServer-9e5;
  }

  Serial.println("TX_INTERVAL = "+String(TX_INTERVAL));
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
    // read the temperature from the DHT22
    GetAndUpdateSensorData();
    

    //sendata //Port 3 for actual binding, Port 4 for showing only
    //Connect to UI(Opensensemap and Node-red)
    LMIC_setTxData2(3, myTempSensor.LoRa_PacketBytes, sizeof(myTempSensor.LoRa_PacketBytes) - 3, 0);
    //Connect only to TTN
    //LMIC_setTxData2(4, myTempSensor.LoRa_PacketBytes, sizeof(myTempSensor.LoRa_PacketBytes) - 3, 0);
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
    //oled
    display.clear();
    display.drawString (0, 0, "EV_TXCOMPLETE event!");
    Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
    if (LMIC.txrxFlags & TXRX_ACK)
      Serial.println(F("Received ack"));
    if (LMIC.dataLen)
    {
      Serial.println("Received "+String(LMIC.dataLen)+" bytes of payload");
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
  delay(1000);
  while (!Serial)
    ;
  Serial.begin(9600);
  delay(100);
  Serial.println(F("Starting"));

  //set Led and start dht sensor:
  dht.begin();
  pinMode(LEDPIN, OUTPUT);
  pinMode(LEDACTOR, OUTPUT);
  
  pinMode(22,OUTPUT);
  digitalWrite(22,LOW);

  display.init ();
  display.flipScreenVertically ();
  display.setFont (ArialMT_Plain_10);

  display.setTextAlignment (TEXT_ALIGN_LEFT);

  display.drawString (0, 0, "Init!");
  display.display ();  


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

