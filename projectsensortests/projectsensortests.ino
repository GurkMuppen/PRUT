#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
// Chip select pin for the SD card reader
const int chipSelect = SDCARD_SS_PIN;

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

// Interval between data logging (in milliseconds)
const unsigned long loggingInterval = 100;
unsigned long previousMillis = 0;

bool useSdCard = true;

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ;  // wait for serial port to connect. Needed for native USB port only
  }
  if (!accel.begin()) {
    Serial.println("Ingen ADXL345 kunde detekteras... Kontrollera dina kopplingar!");
    while (1)
      ;
  }

  // Sätt maxvärde till vad du tror passar projektet
  //accel.setRange(ADXL345_RANGE_16_G);
  // accel.setRange(ADXL345_RANGE_8_G);
  // accel.setRange(ADXL345_RANGE_4_G);
  accel.setRange(ADXL345_RANGE_16_G);
  Serial.println("");


  // Check if the card is present and can be initialized:
  if (useSdCard) {
    Serial.print("Initializing SD card...");
    if (!SD.begin(chipSelect)) {
      Serial.println("Card failed, or not present");
      // Don't do anything more:
      while (1)
        ;
    }
    Serial.println("card initialized.");


    // Create or open the file. Note that only one file can be open at a time,
    // so you have to close this one before opening another.
    File dataFile = SD.open("datalog.csv", FILE_WRITE);

    // If the file opened okay, write the header:
    if (dataFile) {
      dataFile.println("Timestamp,Temperature,Humidity");
      dataFile.close();
      Serial.println("CSV file created with header.");
    } else {
      // If the file didn't open, print an error:
      Serial.println("error opening datalog.csv");
    }
  }
}

void loop() {
  // Check if it's time to log data
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= loggingInterval) {
    previousMillis = currentMillis;

    // Simulate sensor data
    sensors_event_t event;
    accel.getEvent(&event);

    // Visa resultaten (acceleration mäts i m/s^2)

    float accx = event.acceleration.x;
    float accy = event.acceleration.y;
    float accz = event.acceleration.z;


    // Create a timestamp
    String timestamp = String(currentMillis);

    // Open the file. Note that only one file can be open at a time,
    // so you have to close this one before opening another.
    if (useSdCard) {
      File dataFile = SD.open("datalog.csv", FILE_WRITE);

      // If the file is available, write to it:
      if (dataFile) {
        dataFile.print(timestamp);
        dataFile.print(",");
        dataFile.print(accx);
        dataFile.print(",");
        dataFile.print(accy);
        dataFile.print(",");
        dataFile.println(accz);
        dataFile.close();

      } else {
        // If the file didn't open, print an error:
        Serial.println("error opening datalog.csv");
      }
    }

    Serial.print(timestamp);
    Serial.print(",");
    Serial.print(accx);
    Serial.print(",");
    Serial.print(accy);
    Serial.print(",");
    Serial.println(accz);
  }
}
