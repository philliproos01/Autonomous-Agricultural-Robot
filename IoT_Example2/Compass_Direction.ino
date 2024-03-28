#include <QMC5883LCompass.h>

QMC5883LCompass compass;

void setup() {
  Serial.begin(9600);
  compass.init();
  delay(500);
compass.setCalibrationOffsets(512.00, 1093.00, -658.00);
compass.setCalibrationScales(1.03, 0.79, 1.31);



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
