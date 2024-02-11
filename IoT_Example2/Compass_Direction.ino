#include <QMC5883LCompass.h>

QMC5883LCompass compass;

void setup() {
  Serial.begin(9600);
  compass.init();
  delay(500);
  compass.setCalibrationOffsets(189.00, 302.00, 59.00);
  compass.setCalibrationScales(0.96, 1.04, 1.00);
}

void loop() {
  compass.read();
  

  byte a = compass.getAzimuth();

  char myArray[3];
  compass.getDirection(myArray, a);
  
  Serial.print(myArray[0]);
  Serial.print(myArray[1]);
  Serial.print(myArray[2]);
  Serial.println();
  
  delay(250);
}
