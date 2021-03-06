#include <Arduino_FreeRTOS.h>
#include "TempHumiditySense.h"
#include "SHT1x.h"
#include <semphr.h>


TempHumiditySensor::TempHumiditySensor() {
  sht = new SHT15(SHT_DataPin, SHT_ClockPin);
  sht15_mutex = xSemaphoreCreateMutex();
}


float TempHumiditySensor::readTemperatureC(){
  float tempC;

  xSemaphoreTake(sht15_mutex, portMAX_DELAY);
  tempC = sht->readTemperatureC();
  xSemaphoreGive(sht15_mutex);
  return tempC;
}


float TempHumiditySensor::readHumidity(){
  float humid_pc;
  
  xSemaphoreTake(sht15_mutex, portMAX_DELAY);
  humid_pc = sht->readHumidity();
  xSemaphoreGive(sht15_mutex);
  return humid_pc;
}
/*
 * At present there seems to be no way of detecting if the sensor is absent
*/

bool TempHumiditySensor::isOk(){
  return true;
}
