#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <OneWire.h>
#include <Adafruit_BMP280.h>
#include "DFRobot_OzoneSensor.h"
#include "SSD1306Ascii.h"
#include "SSD1306AsciiSoftSpi.h"

const unsigned long LOG_INTERVAL    = 60000; // sd 
const unsigned long SERIAL_INTERVAL = 1000;  // serial
const unsigned long SCREEN_INTERVAL = 3000;  //oled

#define HEADER_FONT Verdana12_italic   
#define DATA_FONT   lcdnums14x24       

//pins
#define SDCARD_CS_PIN 10
#define GEIGER_PIN    2
#define ONE_WIRE_BUS  4  //dsb18 gray wire
#define BMP_ADDRESS   0x76
#define OZONE_ADDRESS OZONE_ADDRESS_3

#define OLED_CLK  8//D pins VVVVV
#define OLED_MOSI 7
#define OLED_RES  6
#define OLED_DC   5
#define OLED_CS   3


#define SEA_LEVEL_PRESSURE_HPA 1023.1
const float RAD_CONVERSION = 0.0057; 



Adafruit_BMP280 bmp;
DFRobot_OzoneSensor Ozone;
OneWire oneWire(ONE_WIRE_BUS);
SSD1306AsciiSoftSpi oled; 

unsigned long lastLogTime = 0;
unsigned long lastSerialTime = 0;
unsigned long lastScreenChange = 0;
unsigned long lastCPMUpdate = 0; 


volatile int irqPulseCount = 0;  
int cpmHistory[60];              
int historyIdx = 0;              
long rollingCPM = 0;             

int currentScreen = 0; 
bool sdErrorVisible = false;
unsigned long sdMsgTime = 0;

void countPulse() {
  irqPulseCount++;
}


float getBackupTemp() {
  byte data[12]; byte addr[8];
  bool found = false;

  for (int i = 0; i < 3; i++) {
    if (oneWire.search(addr)) { found = true; break; }
    oneWire.reset_search(); delay(10); 
  }
  if (!found) return -999.0; 
  
  oneWire.reset(); oneWire.select(addr); oneWire.write(0x44, 1); 
  oneWire.reset(); oneWire.select(addr); oneWire.write(0xBE); 
  for (int i = 0; i < 9; i++) data[i] = oneWire.read();
  
  int16_t raw = (data[1] << 8) | data[0];
  float temp = (float)raw / 16.0;
  if (temp == 85.0) return -999.0;
  return temp;
}

void setup() {
  Serial.begin(9600);
  for(int i=0; i<60; i++) cpmHistory[i] = 0;

  oled.begin(&SH1106_128x64, OLED_CS, OLED_DC, OLED_CLK, OLED_MOSI, OLED_RES);
  oled.setFont(HEADER_FONT);
  oled.clear();

  

  if (bmp.begin(BMP_ADDRESS)) {
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL, Adafruit_BMP280::SAMPLING_X2, Adafruit_BMP280::SAMPLING_X16, Adafruit_BMP280::FILTER_X16, Adafruit_BMP280::STANDBY_MS_500);
  }
  Ozone.begin(OZONE_ADDRESS);
  Ozone.setModes(MEASURE_MODE_PASSIVE);
  

  if (SD.begin(SDCARD_CS_PIN)) {
    bool fileExists = SD.exists("datalog.csv");
    File dataFile = SD.open("datalog.csv", FILE_WRITE);
    if (dataFile) {
      if (!fileExists) {

        dataFile.println(F("Timestamp,BMP_Temp_C,DS18B20_Temp_C,Pressure_Pa,Calculated_Alt_m,Ozone_PPB,CPM,uSv_h"));
      } 
      dataFile.close();
      oled.println(F("SD CARD OK"));
    } else {
      oled.println(F("SD CARD ERROR"));
    }
  } else {
    oled.println(F("NO SD CARD"));
  }
  
  delay(2000); 


  pinMode(GEIGER_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(GEIGER_PIN), countPulse, FALLING);
  

  Serial.println(F("Timestamp,BMP_Temp_C,DS18B20_Temp_C,Pressure_Pa,Calculated_Alt_m,Ozone_PPB,CPM,uSv_h"));
}

void loop() {
  unsigned long currentMillis = millis();


  if (currentMillis - lastLogTime >= LOG_INTERVAL) {
    lastLogTime = currentMillis;
    performLogging();
  }


  if (currentMillis - lastCPMUpdate >= 1000) {
    lastCPMUpdate = currentMillis;
    noInterrupts();
    int countsThisSecond = irqPulseCount;
    irqPulseCount = 0; 
    interrupts();
    rollingCPM -= cpmHistory[historyIdx]; 
    rollingCPM += countsThisSecond;
    cpmHistory[historyIdx] = countsThisSecond;
    historyIdx++;
    if (historyIdx >= 60) historyIdx = 0;
  }


  if (sdErrorVisible) {
    if (currentMillis - sdMsgTime > 2000) {
      sdErrorVisible = false;
      oled.clear();
    } else { return; } 
  }


  if (currentMillis - lastSerialTime >= SERIAL_INTERVAL) {
    lastSerialTime = currentMillis;
    printSerialDebug();
  }


  if (currentMillis - lastScreenChange >= SCREEN_INTERVAL) {
    lastScreenChange = currentMillis;
    currentScreen++;
    if (currentScreen > 4) currentScreen = 0; 
    updateOLED();
  }
}


void performLogging() {
  float t1 = bmp.readTemperature();
  float t2 = getBackupTemp();
  float p = bmp.readPressure();
  float alt = bmp.readAltitude(SEA_LEVEL_PRESSURE_HPA);
  int16_t oz = Ozone.readOzoneData(20); 
  float uSv = rollingCPM * RAD_CONVERSION;

  File dataFile = SD.open("datalog.csv", FILE_WRITE);
  if (dataFile) {
    dataFile.print(millis()/1000); dataFile.print(",");
    dataFile.print(t1); dataFile.print(",");
    dataFile.print(t2); dataFile.print(",");
    dataFile.print(p); dataFile.print(",");
    dataFile.print(alt); dataFile.print(",");
    dataFile.print(oz); dataFile.print(",");
    dataFile.print(rollingCPM); dataFile.print(",");
    dataFile.println(uSv); 

    dataFile.close();
  } else {
    sdErrorVisible = true;
    sdMsgTime = millis();
    oled.clear();
    oled.setFont(HEADER_FONT);
    oled.set2X();
    oled.println(F("SD ERROR"));
  }
}

void printSerialDebug() {
  float t1 = bmp.readTemperature();
  float t2 = getBackupTemp();
  float p = bmp.readPressure(); 
  float alt = bmp.readAltitude(SEA_LEVEL_PRESSURE_HPA); 
  int16_t oz = Ozone.readOzoneData(5); 
  float uSv = rollingCPM * RAD_CONVERSION;

  Serial.print(millis()/1000); Serial.print(",");
  Serial.print(t1); Serial.print(",");
  Serial.print(t2); Serial.print(",");
  Serial.print(p); Serial.print(",");
  Serial.print(alt); Serial.print(",");
  Serial.print(oz); Serial.print(",");
  Serial.print(rollingCPM); Serial.print(",");
  Serial.println(uSv); 
}

void updateOLED() {
  oled.clear();
  oled.set1X();           
  oled.setFont(HEADER_FONT);
  
  float valFloat = 0;
  long valInt = 0;
  bool useInt = false; 
  
  switch (currentScreen) {
    case 0: 
      oled.println(F("BMP280 TEMP(C)")); 
      valFloat = bmp.readTemperature(); 
      useInt = false; 
      break;
    case 1: 
      oled.println(F("DSB18 TEMP(C)")); 
      valFloat = getBackupTemp(); 
      useInt = false;
      break;
    case 2: 
      oled.println(F("ALTITUDE (kFT)")); 

      valFloat = (bmp.readAltitude(SEA_LEVEL_PRESSURE_HPA) * 3.28084) / 1000.0; 
      useInt = false;
      break;
    case 3: 
      oled.println(F("OZONE(PPB)")); 
      valInt = (long)Ozone.readOzoneData(5); 
      useInt = true; 
      break;
    case 4: 
      oled.println(F("CPM")); 
      valInt = (long)rollingCPM; 
      useInt = true; 
      break;
  }
  
  oled.setFont(DATA_FONT); 
  oled.set2X();            
  
  if (useInt) {
    oled.print(valInt); 
  } else {
 
    if (currentScreen == 2) {
      oled.print(valFloat, 2); 
    } else {
      oled.print(valFloat, 1); 
    }
  }
}
