#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include "DFRobot_OzoneSensor.h"
#include <SPI.h>
#include <SD.h>

#define COLLECT_NUMBER   20
#define Ozone_IICAddress OZONE_ADDRESS_3

Adafruit_BMP280 bmp;
DFRobot_OzoneSensor Ozone;

const int chipSelect = 10;
volatile unsigned long pulseCount = 0;
const float CONVERSION_FACTOR = 0.0057;
unsigned long startTime;

String getElapsedTime();
void countPulse();

void setup() {
  Serial.begin(9600);
  delay(1000);
  startTime = millis();
  
  Serial.println(F("Starting..."));
  
  if (!SD.begin(chipSelect)) {
    Serial.println(F("SD fail!"));
    while (1);
  }
  Serial.println(F("SD OK"));
  
  if (!bmp.begin(0x76)) {
    Serial.println(F("BMP fail!"));
    while (1);
  }
  
  while(!Ozone.begin(Ozone_IICAddress)) {
    Serial.println(F("O3 fail!"));
    delay(1000);
  }
  
  Ozone.setModes(MEASURE_MODE_PASSIVE);
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_500);
  
  pinMode(2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), countPulse, FALLING);
  
  Serial.println(F("Ready"));
}

void loop() {
  String timestamp = getElapsedTime();
  int16_t ozoneConc = Ozone.readOzoneData(COLLECT_NUMBER);
  float temp = bmp.readTemperature();
  float press = bmp.readPressure();
  float alt = bmp.readAltitude(1013.25);
  
  static unsigned long lastPulse = 0;
  unsigned long CPM = pulseCount - lastPulse;
  lastPulse = pulseCount;
  float rad = CPM * CONVERSION_FACTOR;
  
  Serial.println(timestamp);
  Serial.print(temp); Serial.println(F(" C"));
  Serial.print(press); Serial.println(F(" Pa"));
  Serial.print(alt); Serial.println(F(" m"));
  Serial.print(ozoneConc); Serial.println(F(" PPB"));
  Serial.print(CPM); Serial.println(F(" CPM"));
  Serial.print(rad); Serial.println(F(" uSv/h"));
  
  File f = SD.open("data.txt", FILE_WRITE);
  if (f) {
    f.println(timestamp);
    f.print(F("Temp=")); f.print(temp); f.println(F("C"));
    f.print(F("Press=")); f.print(press); f.println(F("Pa"));
    f.print(F("Alt=")); f.print(alt); f.println(F("m"));
    f.print(F("O3=")); f.print(ozoneConc); f.println(F("PPB"));
    f.print(F("CPM=")); f.print(CPM); f.println();
    f.print(F("Rad=")); f.print(rad); f.println(F("uSv/h"));
    f.println();
    f.flush();
    f.close();
    Serial.println(F("Saved"));
  } else {
    Serial.println(F("Write fail"));
  }
  
  delay(60000);
}

String getElapsedTime() {
  unsigned long e = millis() - startTime;
  unsigned long s = (e / 1000UL) % 60UL;
  unsigned long m = (e / 60000UL) % 60UL;
  unsigned long h = (e / 3600000UL);
  
  char buf[12];
  snprintf(buf, sizeof(buf), "%02lu:%02lu:%02lu", h, m, s);
  return String(buf);
}

void countPulse() {
  pulseCount++;
}
