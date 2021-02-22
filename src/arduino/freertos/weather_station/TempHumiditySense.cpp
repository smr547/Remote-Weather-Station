#include "TempHumiditySense.h"
#include <SHT1x.h>


TempHumiditySensor::TempHumiditySensor() {
  sht = new SHT1x(SHT_DataPin, SHT_ClockPin);
}


float TempHumiditySensor::readTemperatureC(){
  return sht->readTemperatureC();
}


float TempHumiditySensor::readHumidity(){
  return sht->readHumidity();
}
/*
 * At present there seems to be no way of detecting if the sensor is absent
*/

bool TempHumiditySensor::isOk(){
  return true;
}
