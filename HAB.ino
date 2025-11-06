#include "DFRobot_OzoneSensor.h"
#include "Wire.h"

// Create an instance for I2C mode
DFRobot_OzoneSensor ozoneSensor;

void setup() {
  Serial.begin(9600);
  while (!Serial);

  // Initialize sensor in I2C mode
  if (!ozoneSensor.begin(OZONE_ADDRESS_3)) {  // Default I2C address = 0x73
    Serial.println("Ozone sensor not detected!");
    while (1);
  }
  
  ozoneSensor.setModes(MEASURE_MODE_PASSIVE);
  Serial.println("Ozone sensor initialized successfully!");
}

void loop() {
  readOzone();
}

void readOzone(){
  float ozoneConcentration = ozoneSensor.readOzoneData(20);  // Averaging 20 samples

  Serial.print("Ozone concentration: ");
  Serial.print(ozoneConcentration);
  Serial.println(" ppm");

  delay(1000);
}



//code with the SD card readings
#include <DFRobot_OzoneSensor.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>

#define CS_PIN 10  // SD card chip select pin

DFRobot_OzoneSensor ozoneSensor;
File dataFile;
bool sdAvailable = false; // Tracks if SD card is working

void setup() {
  Serial.begin(9600);
  while (!Serial);

  initOzoneSensor();
  initSD();
}

void loop() {
  readOzone();
  delay(1000);
}


// Initialize Ozone Sensor
void initOzoneSensor() {
  if (!ozoneSensor.begin(OZONE_ADDRESS_3)) {
    Serial.println("Ozone sensor not detected!");
    while (1);
  }
  ozoneSensor.setModes(MEASURE_MODE_PASSIVE);
  Serial.println("Ozone sensor initialized successfully!");
}

// Initialize SD card
void initSD() {
  Serial.print("Initializing SD card...");
  if (!SD.begin(CS_PIN)) {
    Serial.println("SD card not detected! Logging to Serial only.");
    sdAvailable = false;
  } else {
    Serial.println("SD card initialized.");
    sdAvailable = true;

    // Create CSV header if file is new
    dataFile = SD.open("ozone_log.csv", FILE_WRITE);
    if (dataFile) {
      if (dataFile.size() == 0) {
        dataFile.println("Time(ms),Ozone(ppm)");
      }
      dataFile.close();
    } else {
      Serial.println("Error creating ozone_log.csv");
      sdAvailable = false;
    }
  }
}

// Read ozone data
void readOzone() {
  float ozoneConcentration = ozoneSensor.readOzoneData(20);  // Average 20 samples

  Serial.print("Ozone concentration: ");
  Serial.print(ozoneConcentration);
  Serial.println(" ppm");

  // Log to SD or Serial depending on availability
  if (sdAvailable) {
    logToSD(ozoneConcentration);
  } else {
    logToSerial(ozoneConcentration);
  }
}

// Write a line to SD card
void logToSD(float ozoneValue) {
  dataFile = SD.open("ozone_log.csv", FILE_WRITE);
  if (dataFile) {
    dataFile.print(millis());
    dataFile.print(",");
    dataFile.println(ozoneValue);
    dataFile.close();
  } else {
    Serial.println("Error writing to SD card!");
  }
}




