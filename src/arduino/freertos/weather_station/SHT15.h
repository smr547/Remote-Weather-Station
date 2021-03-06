/**
 * SHT15 Library
 *
 * Manages communication with SHT15 temperature / humidity sensors from 
 * Sensirion (www.sensirion.com).
 *
 * Based on previous work by:
 *    Jonathan Oxer <jon@oxer.com.au> / <www.practicalarduino.com>
 *    Maurice Ribble: <www.glacialwanderer.com/hobbyrobotics/?p=5>
 *    Wayne ?: <ragingreality.blogspot.com/2008/01/ardunio-and-sht15.html>
 *
 * Modified to:
 *   - improve timing on sensor data read
 *   - defer delay loops to FreeRTOS vTaskWait()
 *
 */
#ifndef SHT15_h
#define SHT15_h

#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

class SHT15
{
  public:
    SHT15(int dataPin, int clockPin);
    float readHumidity();
    float readTemperatureC();
    float readTemperatureF();
  private:
    int _dataPin;
    int _clockPin;
    int _numBits;
    float readTemperatureRaw();
    int shiftIn(int _dataPin, int _clockPin, int _numBits);
    int sendCommandSHT(int _command, int _dataPin, int _clockPin);
    int waitForResultSHT(int _dataPin);
    int getData16SHT(int _dataPin, int _clockPin);
    void skipCrcSHT(int _dataPin, int _clockPin);
};

#endif
