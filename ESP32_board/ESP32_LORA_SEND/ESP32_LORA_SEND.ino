// If the SPI is not connected to the standard pins, change it in 
// the lmic driver in line 80 in the static void hal_spi_init () function
// of hal.cpp

#include <SPI.h>
#include <Wire.h>
#include <lmic.h>
#include <hal/hal.h>
#include <Preferences.h>
#include <SDS011-select-serial.h>

//config Dust Sensor
HardwareSerial Serial2(2);
SDS011 sds(Serial2);
float pm25, pm10; 

// LEDs
#define led_red 4
#define led_green 2

// config lines Mics Sensor
#define enable 14
#define preheat 27
#define pin_red 35
#define pin_nox 34
int a_co;
int a_no;
int read_index = 0;              // the index of the current reading
int total = 0;                  // the running total
int average = 0;                // the average


// SI7006-A20 I2C address is 0x40(64)
#define Addr 0x40
float ctemp;
float humidity;
// temp calibration
#define correction 7


//LoRaWAN and TTN settings
SPIClass loraConnect = SPIClass(HSPI);
// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t PROGMEM APPEUI[8] = { 0x48, 0x8B, 0x00, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8] = { 0xC1, 0x57, 0xD2, 0xE3, 0xCA, 0xF1, 0x14, 0x00 };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
// The key shown here is the semtech default key.
static const u1_t PROGMEM APPKEY[16] = { 0xE0, 0x3F, 0x4D, 0x39, 0x07, 0x3B, 0xA5, 0xD0, 0x41, 0x5D, 0xAD, 0x0C, 0x59, 0xF7, 0x9E, 0x88 };
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

static osjob_t sendjob;

// This is that data that is going to be send.
// It is important, that the right values are at the right position.
// Every Value that is send is 12 bits, so 6 bytes is 4 values: RNOx, RCO, Temperature, and Humidity
// 
uint8_t txData[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations). Send every fifteen minutes.
const unsigned TX_INTERVAL = 15 * 60;

// Pin mapping for RFM95
const lmic_pinmap lmic_pins = {
    .nss = 33,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 32,
    .dio = {21, 22, 23}, //workaround to use 1 pin for all 3 radio dio pins
};

//settings
long lastMsg = 0;
long lastRead = 0;
long lastRestart = 0;
int heaterState = LOW;
unsigned long previousMillis = 0;        // will store last time LED was updated
long OnTime = 15000;           // milliseconds of on-time
long OffTime = 885000;          // milliseconds of off-time
int rssi;

void onEvent (ev_t ev) {
  Serial.print(os_getTime());
  Serial.print(": ");
  switch(ev) {
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
    
      // Disable link check validation (automatically enabled
      // during join, but not supported by TTN at this time).
      LMIC_setLinkCheckMode(0);
      break;
    case EV_RFU1:
      Serial.println(F("EV_RFU1"));
      break;
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      break;
      break;
    case EV_TXCOMPLETE:
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      if (LMIC.txrxFlags & TXRX_ACK)
        Serial.println(F("Received ack"));
      if (LMIC.dataLen) {
        Serial.println(F("Received "));
        Serial.println(LMIC.dataLen);
        Serial.println(F(" bytes of payload"));
      }
      // Schedule next transmission
      os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
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
    default:
      Serial.println(F("Unknown event"));
      break;
  }
}

void do_send(osjob_t* j){
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
    // Prepare upstream data transmission at the next possible time.
    LMIC_setTxData2(1, txData, sizeof(txData), 0);
    Serial.println(F("Packet queued"));
  }
  // Next TX is scheduled after TX_COMPLETE event.
}

void readCo() {
  total += analogRead(pin_red);
  read_index += 1;
}

void turnSensorsOn() {
  Serial2.begin(9600);
  Serial.println("preheat on");
  sds.wakeup();
  digitalWrite(preheat, HIGH);
  digitalWrite(led_red, HIGH);
}

void turnSensorsOffAndReadOnce() {
  unsigned int humData[2];
  unsigned int tempData[2];

  // Start I2C transmission
  Wire.beginTransmission(Addr);
  // Send humidity measurement command, NO HOLD MASTER
  Wire.write(0xF5);
  // Stop I2C transmission
  Wire.endTransmission();
  delay(500);

  // Request 2 bytes of data
  Wire.requestFrom(Addr, 2);

  // Read 2 bytes of data
  // humidity msb, humidity lsb
  if (Wire.available() == 2)
  {
    humData[0] = Wire.read();
    humData[1] = Wire.read();
  }
  
  // Convert the data
  humidity  = ((humData[0] * 256.0) + tempData[1]);
  humidity = ((125 * humidity) / 65536.0) - 6;

  // Start I2C transmission
  Wire.beginTransmission(Addr);
  // Send temperature measurement command, NO HOLD MASTER
  Wire.write(0xF3);
  // Stop I2C transmission
  Wire.endTransmission();
  delay(500);

  // Request 2 bytes of data
  Wire.requestFrom(Addr, 2);

  // Read 2 bytes of data
  // temp msb, temp lsb
  if (Wire.available() == 2)
  {
    tempData[0] = Wire.read();
    tempData[1] = Wire.read();
  }
  
  // Convert the data
  float temp  = ((tempData[0] * 256.0) + tempData[1]);
  ctemp = (((175.72 * temp) / 65536.0) - 46.85) - correction;

  // Output data to serial monitor
  Serial.print("Relative humidity : ");
  Serial.print(humidity);
  Serial.println(" % RH");
  Serial.print("Temperature in Celsius : ");
  Serial.print(ctemp);
  Serial.println(" C");
  Serial.println("Preheat off and read");

  // disregard accuracy, save bytes.
  // temp and hum changed from 16 to 12 bit to save one byte in sending data.
  // only affects second position after decimal point, thats an acceptable loss
  txData[3] = tempData[0];
  txData[4] = humData[0];
  txData[5] = (tempData[1] & 0xF0) | ((humData[1] >> 4) & 0x0F) ;
  
  

  // get nox and co value
  a_no = analogRead(pin_nox);
  a_co = total/read_index;
  total = 0;
  read_index = 0;
  Serial.println(a_no);
  //cram the nox and co values into 3 bytes. safe payloadspace yo
  int rawNoxHighByte = (highByte(a_no) << 4) & 0xF0;
  int rawCoHighByte = highByte(a_co) & 0x0F;
  txData[0] = rawNoxHighByte | rawCoHighByte;
  txData[1] = lowByte(a_no);
  txData[2] = lowByte(a_co);
  digitalWrite(preheat, LOW);
  digitalWrite(led_red, LOW);
  int error = sds.read(&pm25,&pm10);
  if (! error) {
    Serial.println(pm25);
  } else {
    Serial.println("Could not read air data: "+String(error));
  }
  sds.sleep();
}

void setup()   {
  Serial.begin(115200);
  delay(2000);
  pinMode(led_red, OUTPUT);
  pinMode(led_green, OUTPUT);

  digitalWrite(led_green, LOW);
  digitalWrite(led_red, LOW);

  // LMIC init
  digitalWrite(led_green, HIGH);
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  digitalWrite(led_red, HIGH);
  LMIC_reset();
  
  digitalWrite(led_green, LOW);
  digitalWrite(led_red, LOW);

  // Start job (sending automatically starts OTAA too)
  do_send(&sendjob);
  
  // Initialise I2C communication as MASTER
  Wire.begin(18, 19);
  // Start I2C transmission
  Wire.beginTransmission(Addr);
  // Stop I2C transmission
  Wire.endTransmission();

  //mics sensor
  pinMode(pin_nox, INPUT);
  pinMode(pin_red, INPUT);
  pinMode(preheat, OUTPUT);
  pinMode(enable, OUTPUT);
  digitalWrite(enable, HIGH);

  //dust sensor
  Serial2.begin(9600);
}


void loop(){
  long currentMillis = millis();
  if (currentMillis - lastRead > 5000) {
    lastRead = currentMillis;
    readCo();
    digitalWrite(led_green, HIGH);
    delay(1);
    digitalWrite(led_green, LOW);
  }
  if((heaterState == HIGH) && (currentMillis - previousMillis >= OnTime)){
    heaterState = LOW;
    previousMillis = currentMillis;
    turnSensorsOffAndReadOnce();
  }
  else if ((heaterState == LOW) && (currentMillis - previousMillis >= OffTime)){
    heaterState = HIGH;
    previousMillis = currentMillis;
    turnSensorsOn();
  }
  if (currentMillis - lastRestart > 20*60*60000) {
    ESP.restart(); // remove if openairnode works longer...
  }
  //Do LoRa magic. I mean check if TX is ready and send.
  os_runloop_once();
}
