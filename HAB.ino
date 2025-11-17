#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include "DFRobot_OzoneSensor.h"
#include <SPI.h>
#include <SD.h>

#define COLLECT_NUMBER   20              // collect number, the collection range is 1-100
#define Ozone_IICAddress OZONE_ADDRESS_3

// Create BMP280 object
Adafruit_BMP280 bmp; // I2C interface
DFRobot_OzoneSensor Ozone;

const int chipSelect = 10; // Set to the CS pin connected to your microSD module
volatile unsigned long pulseCount = 0;
unsigned long lastMillis = 0;
const float CONVERSION_FACTOR = 0.0057; // 1 CPM ≈ 0.0057 µSv/hr
unsigned long startTime;

//Method prototypes
String getElapsedTime();
void countPulse();

void setup() {
  Serial.begin(9600);
  startTime = millis();

  while (!Serial); // wait for serial port to connect

  if (!bmp.begin(0x76)) {  // 0x76 or 0x77 depending on your module
    Serial.println("Could not find BMP280 sensor, check wiring!");
    while (1);
  }

   while(!Ozone.begin(Ozone_IICAddress)) {
    Serial.println("Ozone: I2c device number error !");
    delay(1000);
  }  

  // Initialize SD card
  if (!SD.begin(chipSelect)) {
    Serial.println("SD card initialization failed!");
    return;
  }
 
  Serial.println("BMP280 Sensor Initialized");
  Serial.println("Ozone: I2c connect success !");
  Serial.println("SD card initialized.");

  Ozone.setModes(MEASURE_MODE_PASSIVE);
  // Optional: You can configure the sensor
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */  

  pinMode(2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), countPulse, FALLING);

  lastMillis = millis();

  Serial.println("SBM-20 Geiger Counter Starting...");
  Serial.println("");
}

void loop() {
  unsigned long now = millis();

  String timestamp = getElapsedTime();
  int16_t ozoneConcentration = Ozone.readOzoneData(COLLECT_NUMBER);

  lastMillis = now;

  static unsigned long lastPulseCount = 0;
  unsigned long CPM = pulseCount - lastPulseCount;
  lastPulseCount = pulseCount;

  float radiation = CPM * CONVERSION_FACTOR;
  

  //Write to SD card
  File dataFile = SD.open("data.txt", FILE_WRITE);
  if (dataFile) {
    dataFile.println(timestamp);
    dataFile.print("Temperature = ");
    dataFile.print(bmp.readTemperature());
    dataFile.println(" °C");

    dataFile.print("Pressure = ");
    dataFile.print(bmp.readPressure()); // hPa
    dataFile.println(" Pa");

    dataFile.print("Approx Altitude = ");
    dataFile.print(bmp.readAltitude(1013.25)); // Adjust for local sea level pressure
    dataFile.println(" m");

    dataFile.print("Ozone concentration = ");
    dataFile.print(ozoneConcentration);
    dataFile.println(" PPB.");

    dataFile.print("Clicks per Minute = ");
    dataFile.print(ozoneConcentration);
    dataFile.println(" Click(S)");

    dataFile.print("Radiation = ");
    dataFile.print(radiation);
    dataFile.println(" uSv/h");

    dataFile.close();
    Serial.println("Data written to test.txt.");
  } else {
    Serial.println("Error opening test.txt for writing.");
  }

  delay(60000); // Read every 1 minute
}

String getElapsedTime() {
  unsigned long elapsed = millis() - startTime; // safe across overflow
  unsigned long seconds = (elapsed / 1000UL) % 60UL;
  unsigned long minutes = (elapsed / 60000UL) % 60UL;
  unsigned long hours   = (elapsed / 3600000UL); // total hours

  char buffer[20];
  snprintf(buffer, sizeof(buffer), "%02lu:%02lu:%02lu", hours, minutes, seconds);
  return String(buffer);
}

void countPulse() {
  pulseCount++;
}
