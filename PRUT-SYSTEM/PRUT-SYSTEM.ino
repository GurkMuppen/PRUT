#include <Wire.h>
#include <SPI.h>
#include <Servo.h>
#include <Adafruit_BMP280.h>
#include <string>
#include <SD.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <iostream>
#include <RH_RF95.h>
using namespace std; 

#define RFM95_CS    4
#define RFM95_INT   5
#define RFM95_RST   6

// radio frequenzy
#define RF95_FREQ 420.0

// PRESSURE SENSOR PINOUTS
#define BMP_SCK (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS (10)

const int chipSelect = SDCARD_SS_PIN;


Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

RH_RF95 rf95(RFM95_CS, RFM95_INT);

Adafruit_BMP280 bmp;  // I2C
//Adafruit_BMP280 bmp(BMP_CS); // hardware SPI
//Adafruit_BMP280 bmp(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK);

float lokaltlufttryck = 1013.25;
float startAltitude = 0;
float maxAltitude = 0;
float upperReleaseThreshold = 2;


Servo servo1;
Servo servo2;

int armPin = 0;
int armLed = 1;
int standbyLed = 2;
bool armed;
bool useSdCard = true;


void setup() {

  pinMode(standbyLed, OUTPUT);
  digitalWrite(standbyLed, HIGH);
  //radio pin
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  //radio saker
  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  Serial.begin(9600);
  delay(1000);
  unsigned status;
  //status = bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);

  //kollar då radio fungerar
  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
    while (1);
  }
  //sätter frequens
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);

  //sätter signalstyrka för radio
  rf95.setTxPower(23, false);

  if (useSdCard) {
    Serial.print("Initializing SD card...");
    if (!SD.begin(chipSelect)) {
      Serial.println("Card failed, or not present");
      // Don't do anything more:
      while (1)
        ;
    }
    Serial.println("card initialized.");
  }
  status = bmp.begin();

  if (!accel.begin()) {
    Serial.println("Ingen ADXL345 kunde detekteras... Kontrollera dina kopplingar!");
    while (1);
  }

  accel.setRange(ADXL345_RANGE_4_G);

  if (!status) {
    logMessage(F("Could not find a valid BMP280 sensor, check wiring or "
                     "try a different address!"),true);
    logMessage("SensorID was: 0x",false);
    Serial.println(bmp.sensorID(), 16);
    while (1) delay(10);
  }

  /* SET PRESSURE SAMPLING*/
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X8,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X4,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_63); /* Standby time. */


  calibrateAltitude();

  // INIT SERVOS & ARMING SYSTEMS
  armed = false;
  servo1.attach(6);
  servo2.attach(7);
  servo1.write(0);
  servo2.write(0);

  pinMode(armPin, INPUT);
  pinMode(armLed, OUTPUT);

  // ARM SYSTEM 
  calibrateAltitude();
  armed = true;
  digitalWrite(standbyLed, LOW);
}

void loop() {

  listenForSafteyMessage();


  /*Serial.print(F("Temperature = "));
  Serial.print(bmp.readTemperature());
  Serial.println(" *C");

  Serial.print(F("Pressure = "));
  Serial.print(bmp.readPressure());
  Serial.println(" Pa");*/

  float altitude = bmp.readAltitude(lokaltlufttryck) - startAltitude;


  //altitude (meterd)
  logMessage(String(altitude),false);
  //temprature (degres celcius?)
  logMessage(String(bmp.readTemperature()),false);
  //pressure (kpa?)
  logMessage(String(bmp.readPressure()),false);

  sensors_event_t event; 
  accel.getEvent(&event);

  float accx = event.acceleration.x;
  float accy = event.acceleration.y;
  float accz = event.acceleration.z;
  //logging accelerationdata for x,y and z
  logMessage(String(accx), false);
  logMessage(String(accy), false);
  logMessage(String(accz), false);


  


  logMessage("",true);

  //FLASH ARMING LED
  if (armed == true && millis() % 1000 > 500) {
    digitalWrite(armLed, HIGH);
  } else {
    digitalWrite(armLed, LOW);
  }

  // ALTITUDE RELEASE SYSTEM
  if (maxAltitude < altitude) {
    maxAltitude = altitude;
  }

  if (maxAltitude - altitude > upperReleaseThreshold && armed == true) {
    logMessage("RELEASE CHUTE",true);
    parachuteRelease();
  }
  delay(100);
}

void parachuteRelease() {
  servo1.write(180);
  servo2.write(180);
  armed = false;
  digitalWrite(armLed, HIGH);
}

void calibrateAltitude() {
  //calibrate
  float calibrationHeight = 0;

  for (int i = 0; i < 10; i++) {
    calibrationHeight += bmp.readAltitude(lokaltlufttryck);
    delay(300);
  }

  startAltitude = calibrationHeight / 10;
}

void listenForSafteyMessage() {
  if (rf95.available()) {

    // Should be a message for us now
    int custombufsize = 128;
    //uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t buf[custombufsize];
    uint8_t len = sizeof(buf);

    if (rf95.recv(buf, &len)) {
      digitalWrite(LED_BUILTIN, HIGH);
      RH_RF95::printBuffer("Received: ", buf, len);
      Serial.print("Got: ");
      Serial.println((char*)buf);
      if(strcmp((char*)buf,"Saftey Parachute Overide")==0)
        {
          parachuteRelease();
        }
       Serial.print("RSSI: ");
      Serial.println(rf95.lastRssi(), DEC);


    
    } else {
      Serial.println("Receive failed");
    }
  }
  else {

  }

}


void logMessage(String logText, bool newLine) {
  if (useSdCard) {
    File dataFile = SD.open("datalog.csv", FILE_WRITE);
    if (!dataFile){
      Serial.println("something very wrong with sd card");
    }
    if (newLine) {
      Serial.println(logText);
      dataFile.println(logText);
      dataFile.close();
    } else {
      Serial.print(logText);
      dataFile.print(logText);
      dataFile.close();
    }
  } else {
    if (newLine) {
      Serial.println(logText);
    } else {
      Serial.print(logText);
    }
  }
}
