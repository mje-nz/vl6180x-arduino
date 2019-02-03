/* This minimal example shows how to get single-shot range
measurements from the VL6180X.

This is no different than the Wire version in examples/, and is just here for
CI.  Note that you need -DVL6180X_USE_I2C_T3 for this to build.  In fact, if you
leave out the #include for i2c_t3.h or Wire.h you can switch between them with
that flag alone.  I'm not doing that here to make sure CI breaks if VL6180X.h
ends up including Wire.h.  */

#include <i2c_t3.h>
#include <VL6180X.h>

VL6180X sensor;

void setup() 
{
  Serial.begin(9600);
  Wire.begin();
  
  sensor.begin();
  sensor.setTimeout(500);
}

void loop() 
{ 
  Serial.print(sensor.readRangeSingleMillimeters());
  if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
  
  Serial.println();
}
