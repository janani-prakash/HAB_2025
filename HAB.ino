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
  delay(1000); // Give serial time to initialize
  startTime = millis();
  
  Serial.println("\n\n=== SYSTEM STARTING ===");
  
  // Initialize SD card BEFORE other sensors
  Serial.println("Initializing SD card...");
  Serial.print("CS Pin: ");
  Serial.println(chipSelect);
  
  if (!SD.begin(chipSelect)) {
    Serial.println("❌ SD CARD INITIALIZATION FAILED!");
    Serial.println("\nTroubleshooting:");
    Serial.println("1. Check SD card is inserted properly");
    Serial.println("2. Check wiring (MOSI=11, MISO=12, SCK=13, CS=10)");
    Serial.println("3. SD card must be FAT32 format");
    Serial.println("4. Try different CS pin or SD card");
    while (1); // Halt if SD fails
  }
  Serial.println("✓ SD card initialized successfully!");
  
  // Test write immediately
  Serial.println("Testing SD card write...");
  File testFile = SD.open("test.txt", FILE_WRITE);
  if (testFile) {
    testFile.println("Arduino SD Test - System Started");
    testFile.close();
    Serial.println("✓ Test write successful!");
  } else {
    Serial.println("❌ Test write FAILED!");
    while(1);
  }
  
  // Initialize BMP280
  if (!bmp.begin(0x76)) {  // 0x76 or 0x77 depending on your module
    Serial.println("Could not find BMP280 sensor, check wiring!");
    while (1);
  }
  Serial.println("BMP280 Sensor Initialized");
  
  // Initialize Ozone sensor
  while(!Ozone.begin(Ozone_IICAddress)) {
    Serial.println("Ozone: I2c device number error !");
    delay(1000);
  }  
  Serial.println("Ozone: I2c connect success !");
  
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
  
  // Read sensor data
  int16_t ozoneConcentration = Ozone.readOzoneData(COLLECT_NUMBER);
  float temperature = bmp.readTemperature();
  float pressure = bmp.readPressure();
  float altitude = bmp.readAltitude(1013.25);
  
  // Calculate radiation
  static unsigned long lastPulseCount = 0;
  unsigned long CPM = pulseCount - lastPulseCount;
  lastPulseCount = pulseCount;
  float radiation = CPM * CONVERSION_FACTOR;
  
  // Print to Serial first
  Serial.println("======================");
  Serial.println(timestamp);
  Serial.print("Temperature = ");
  Serial.print(temperature);
  Serial.println(" °C");
  Serial.print("Pressure = ");
  Serial.print(pressure);
  Serial.println(" Pa");
  Serial.print("Approx Altitude = ");
  Serial.print(altitude);
  Serial.println(" m");
  Serial.print("Ozone concentration = ");
  Serial.print(ozoneConcentration);
  Serial.println(" PPB");
  Serial.print("Clicks per Minute = ");
  Serial.print(CPM);  // FIXED: Was printing ozoneConcentration instead of CPM
  Serial.println(" Click(s)");
  Serial.print("Radiation = ");
  Serial.print(radiation);
  Serial.println(" µSv/h");
  
  //Write to SD card
  Serial.println("\nWriting to SD card...");
  File dataFile = SD.open("data.txt", FILE_WRITE);
  
  if (!dataFile) {
    Serial.println("❌ ERROR: Could not open data.txt!");
    Serial.println("Checking if SD card is still responding...");
    
    // Try to recover
    SD.begin(chipSelect);
    dataFile = SD.open("data.txt", FILE_WRITE);
  }
  
  if (dataFile) {
    Serial.print("File size before write: ");
    Serial.print(dataFile.size());
    Serial.println(" bytes");
    
    dataFile.println("======================");
    dataFile.println(timestamp);
    dataFile.print("Temperature = ");
    dataFile.print(temperature);
    dataFile.println(" °C");
    dataFile.print("Pressure = ");
    dataFile.print(pressure);
    dataFile.println(" Pa");
    dataFile.print("Approx Altitude = ");
    dataFile.print(altitude);
    dataFile.println(" m");
    dataFile.print("Ozone concentration = ");
    dataFile.print(ozoneConcentration);
    dataFile.println(" PPB");
    dataFile.print("Clicks per Minute = ");
    dataFile.print(CPM);  // FIXED: Was printing ozoneConcentration instead of CPM
    dataFile.println(" Click(s)");
    dataFile.print("Radiation = ");
    dataFile.print(radiation);
    dataFile.println(" µSv/h");
    
    dataFile.flush(); // Force write to SD card
    
    Serial.print("File size after write: ");
    Serial.print(dataFile.size());
    Serial.println(" bytes");
    
    dataFile.close();
    Serial.println("✓ Data successfully written to data.txt");
  } else {
    Serial.println("❌ CRITICAL ERROR: Cannot write to SD card!");
  }
  
  lastMillis = now;
  
  delay(60000); // Read every 1 minute
}

String getElapsedTime() {
  unsigned long elapsed = millis() - startTime;
  unsigned long seconds = (elapsed / 1000UL) % 60UL;
  unsigned long minutes = (elapsed / 60000UL) % 60UL;
  unsigned long hours   = (elapsed / 3600000UL);
  
  char buffer[20];
  snprintf(buffer, sizeof(buffer), "%02lu:%02lu:%02lu", hours, minutes, seconds);
  return String(buffer);
}

void countPulse() {
  pulseCount++;
}
