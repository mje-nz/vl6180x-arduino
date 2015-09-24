#include <Wire.h>
#include <VL6180X.h>

VL6180X sensor;

void setup() 
{
  Wire.begin();
  
  sensor.init();
  sensor.configureDefault();
  sensor.setTimeout(500);
}

void loop() 
{ 
  Serial.print(sensor.readRangeSingle());
  if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
  
  Serial.println();
}