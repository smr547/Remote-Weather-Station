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

#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#include "SHT15.h"

SHT15::SHT15(int dataPin, int clockPin)
{
  _dataPin = dataPin;
  _clockPin = clockPin;
}


/* ================  Public methods ================ */

/**
   Reads the current temperature in degrees Celsius
   Return -99.0 if error
*/
float SHT15::readTemperatureC()
{
  int _val;                // Raw value returned from sensor
  float _temperature;      // Temperature derived from raw value

  // Conversion coefficients from SHT15 datasheet
  const float D1 = -40.0;  // for 14 Bit @ 5V
  const float D2 =   0.01; // for 14 Bit DEGC

  // Fetch raw value
  _val = readTemperatureRaw();
  if (_val < -40.0) {
    return _val;
  }

  // Convert raw value to degrees Celsius
  _temperature = (_val * D2) + D1;

  return (_temperature);
}

/**
   Reads the current temperature in degrees Fahrenheit
   Returns -99.0 on error
*/
float SHT15::readTemperatureF()
{
  int _val;                 // Raw value returned from sensor
  float _temperature;       // Temperature derived from raw value

  // Conversion coefficients from SHT15 datasheet
  const float D1 = -40.0;   // for 14 Bit @ 5V
  const float D2 =   0.018; // for 14 Bit DEGF

  // Fetch raw value
  _val = readTemperatureRaw();
  if (_val < -40.0) {
    return _val;
  }

  // Convert raw value to degrees Fahrenheit
  _temperature = (_val * D2) + D1;

  return (_temperature);
}

/**
   Reads current temperature-corrected relative humidity
   return a negative value if error
*/
float SHT15::readHumidity()
{
  int _val;                    // Raw humidity value returned from sensor
  float _linearHumidity;       // Humidity with linear correction applied
  float _correctedHumidity;    // Temperature-corrected humidity
  float _temperature;          // Raw temperature value

  // Conversion coefficients from SHT15 datasheet
  const float C1 = -4.0;       // for 12 Bit
  const float C2 =  0.0405;    // for 12 Bit
  const float C3 = -0.0000028; // for 12 Bit
  const float T1 =  0.01;      // for 14 Bit @ 5V
  const float T2 =  0.00008;   // for 14 Bit @ 5V

  // Command to send to the SHT15 to request humidity
  int _gHumidCmd = 0b00000101;

  // Fetch the value from the sensor
  int rc = sendCommandSHT(_gHumidCmd, _dataPin, _clockPin);
  if (rc)
    return -1.0;

  rc = waitForResultSHT(_dataPin);
  if (rc)
    return -1.0;
  _val = getData16SHT(_dataPin, _clockPin);
  skipCrcSHT(_dataPin, _clockPin);

  // Apply linear conversion to raw value
  _linearHumidity = C1 + C2 * _val + C3 * _val * _val;

  // Get current temperature for humidity correction
  _temperature = readTemperatureC();
  if (_temperature < -40.0)
    return -1.0;

  // Correct humidity value for current temperature
  _correctedHumidity = (_temperature - 25.0 ) * (T1 + T2 * _val) + _linearHumidity;

  return (_correctedHumidity);
}


/* ================  Private methods ================ */

/**
   Reads the current raw temperature value
   Return -99.0 if error
*/
float SHT15::readTemperatureRaw()
{
  int _val;

  // Command to send to the SHT15 to request Temperature
  int _gTempCmd  = 0b00000011;

  int rc = sendCommandSHT(_gTempCmd, _dataPin, _clockPin);
  if (rc)
    return -99.0;
  rc = waitForResultSHT(_dataPin);
  if (rc)
    return -99.0;
  _val = getData16SHT(_dataPin, _clockPin);
  skipCrcSHT(_dataPin, _clockPin);

  return (_val);
}

/**
*/
int SHT15::shiftIn(int _dataPin, int _clockPin, int _numBits)
{
  int ret = 0;
  int i;

  for (i = 0; i < _numBits; ++i)
  {
    digitalWrite(_clockPin, HIGH);
    //     delay(10);  // I don't know why I need this, but without it I don't get my 8 lsb of temp
    ret = (ret << 1) + digitalRead(_dataPin);
    digitalWrite(_clockPin, LOW);
  }

  return (ret);
}

/**
   Send a command to the sensor

   Return 0 if OK
   Return 1 or 2  if error in first or second ack
*/
int SHT15::sendCommandSHT(int _command, int _dataPin, int _clockPin)
{
  int ack;

  // Transmission Start
  pinMode(_dataPin, OUTPUT);
  pinMode(_clockPin, OUTPUT);
  digitalWrite(_dataPin, HIGH);
  digitalWrite(_clockPin, HIGH);
  digitalWrite(_dataPin, LOW);
  digitalWrite(_clockPin, LOW);
  digitalWrite(_clockPin, HIGH);
  digitalWrite(_dataPin, HIGH);
  digitalWrite(_clockPin, LOW);

  // The command (3 msb are address and must be 000, and last 5 bits are command)
  shiftOut(_dataPin, _clockPin, MSBFIRST, _command);

  // Verify we get the correct ack
  digitalWrite(_clockPin, HIGH);
  pinMode(_dataPin, INPUT);
  ack = digitalRead(_dataPin);
  if (ack != LOW) {
    // Serial.println("First ACK failied");
    return 1;
  }
  digitalWrite(_clockPin, LOW);
  ack = digitalRead(_dataPin);
  if (ack != HIGH) {
    // Serial.println("Second ACK failied");
    return 2;
  }

  return 0;
}

/**
   Wait for the sensor to indicate the result is ready

   Return 0 = ok, 1 = timeout
*/
int SHT15::waitForResultSHT(int _dataPin)
{
  int i;
  int ack;

  pinMode(_dataPin, INPUT);

  for (i = 0; i < 100; ++i)
  {
    delay(10);
    ack = digitalRead(_dataPin);

    if (ack == LOW) {
      break;
    }
  }

  if (ack == HIGH) {
    return 1; // no ack after 1 second
  }

  return 0;
}

/**
*/
int SHT15::getData16SHT(int _dataPin, int _clockPin)
{
  int val;

  // Get the most significant bits
  pinMode(_dataPin, INPUT);
  pinMode(_clockPin, OUTPUT);
  val = shiftIn(_dataPin, _clockPin, 8);
  val *= 256;

  // Send the required ack
  pinMode(_dataPin, OUTPUT);
  digitalWrite(_dataPin, HIGH);
  digitalWrite(_dataPin, LOW);
  digitalWrite(_clockPin, HIGH);
  digitalWrite(_clockPin, LOW);

  // Get the least significant bits
  pinMode(_dataPin, INPUT);
  val |= shiftIn(_dataPin, _clockPin, 8);

  return val;
}

/**
*/
void SHT15::skipCrcSHT(int _dataPin, int _clockPin)
{
  // Skip acknowledge to end trans (no CRC)
  pinMode(_dataPin, OUTPUT);
  pinMode(_clockPin, OUTPUT);

  digitalWrite(_dataPin, HIGH);
  digitalWrite(_clockPin, HIGH);
  digitalWrite(_clockPin, LOW);
}
