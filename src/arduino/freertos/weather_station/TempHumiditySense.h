
#ifndef SRC_TEMP_HUMIDITY_SENSE_H_
#define SRC_TEMP_HUMIDITY_SENSE_H_

#define SHT_DataPin 4        // data pin for SHT temperature/humidity sensor
#define SHT_ClockPin 5       // clock pin for SHT temperature/humidity sensor
#include <Arduino_FreeRTOS.h>
#include <SHT1x.h>
#include <semphr.h>

/*
 * Combined Temperature and Humidity sensor.
 * 
 * Current hardware uses a Sparkfun STH15 breakout board. 
 * For details see https://www.sparkfun.com/products/retired/13683#:~:text=The%20SHT15%20Breakout%20is%20an,circuitry%20on%20one%20single%20chip.
 * 
 */
class TempHumiditySensor {
  private:

    SHT1x * sht;
    SemaphoreHandle_t sht15_mutex;  // Mutex required: Temp and humidity sensors are on the same one wire channel

  public:

    TempHumiditySensor();
    float readTemperatureC();
    float readHumidity();
    bool isOk();
};



#endif SRC_TEMP_HUMIDITY_SENSE_H_
