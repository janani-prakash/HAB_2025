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
