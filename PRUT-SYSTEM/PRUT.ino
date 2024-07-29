/***************************************************************************
  This is a library for the BMP280 humidity, temperature & pressure sensor

  Designed specifically to work with the Adafruit BMP280 Breakout
  ----> http://www.adafruit.com/products/2651

  These sensors use I2C or SPI to communicate, 2 or 4 pins are required
  to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Limor Fried & Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/

#include <Wire.h>
#include <SPI.h>
#include <Servo.h>
#include <Adafruit_BMP280.h>

#define BMP_SCK  (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS   (10)

Adafruit_BMP280 bmp; // I2C
//Adafruit_BMP280 bmp(BMP_CS); // hardware SPI
//Adafruit_BMP280 bmp(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK);

float lokaltlufttryck = 1013.25;
float startAltitude = 0;
float maxAltitude = 0;
float upperReleaseThreshold = 1;


Servo servo1;
Servo servo2;

int armPin = 0;
int armLed = 1;
bool armed;


void setup() {
  Serial.begin(9600);
  while ( !Serial ) delay(100);   // wait for native usb
  Serial.println(F("BMP280 test"));
  unsigned status;
  //status = bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);

  status = bmp.begin();

  if (!status) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
    Serial.print("SensorID was: 0x"); Serial.println(bmp.sensorID(),16);
    while (1) delay(10);
  }

  /* SET PRESSURE SAMPLING*/
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X1,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X2,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_125); /* Standby time. */


  //calibrate 
  float calibrationHeight = 0;

  for (int i = 0; i < 10; i++){
    calibrationHeight += bmp.readAltitude(lokaltlufttryck);
    delay(150);
  }

  startAltitude = calibrationHeight / 10;

  // INIT SERVOS & ARMING SYSTEMS 
  armed = false;
  servo1.attach(6);
  servo2.attach(7);
  servo1.write(0);
  servo2.write(0);

  pinMode(armPin, INPUT);
  pinMode(armLed, OUTPUT);
}

void loop() {
  Serial.print(F("Temperature = "));
  Serial.print(bmp.readTemperature());
  Serial.println(" *C");

  Serial.print(F("Pressure = "));
  Serial.print(bmp.readPressure());
  Serial.println(" Pa");

  float altitude = bmp.readAltitude(lokaltlufttryck) - startAltitude;

  Serial.print(F("Approx altitude = "));
  Serial.print(altitude); /* Adjusted to local forecast! */
  Serial.println(" m");


  Serial.println();

  // GROUND ARMING
  if (digitalRead(armPin) == LOW)
  {
    armed = true;
  }

  //FLASH ARMING LED
  if (armed == true && millis() % 1000 > 500)
  {
    digitalWrite(armLed, HIGH);
  }
  else
  {
    digitalWrite(armLed, LOW);
  }

  // ALTITUDE RELEASE SYSTEM
  if (maxAltitude < altitude)
  {
    maxAltitude = altitude;
  }

  if (maxAltitude - altitude > upperReleaseThreshold && armed == true)
  {
    Serial.println("RELEASE CHUTE");
    parachuteRelease();
  }
}

void parachuteRelease()
{
  servo1.write(180);
  servo2.write(180);
  armed = false;
  digitalWrite(armLed, HIGH);
}
