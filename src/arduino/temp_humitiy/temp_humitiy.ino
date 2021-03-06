/******************************************************************************
SHT15 Example
Joel Bartlett @ SparkFun Electronics
16 Sept 2015

This example shows how to get the temperature in F or C and humidity
Developed/Tested with:
SparkFun RedBoard 
Arduino IDE 1.6.5

Connections:
GND  -> A2
Vcc  -> A3
DATA -> A4
SCK  -> A5

Requires:
SHT1X Arduino Library
https://github.com/sparkfun/SHT15_Breakout/

This code is beerware.
Distributed as-is; no warranty is given. 
******************************************************************************/
#include "SHT15.h"

//variables for storing values
float tempC = 0;
float tempF = 0;
float humidity = 0;

//Create an instance of the SHT1X sensor
SHT15 *  sht15 = new SHT15(4, 5);//Data, SCK

//delacre output pins for powering the sensor


void setup()
{
  Serial.begin(9600); // Open serial connection to report values to host
}
//-------------------------------------------------------------------------------------------
void loop()
{
  readSensor();
  printOut();
  // while (1) {}
  delay(1000);
}
//-------------------------------------------------------------------------------------------
void readSensor()
{
  // Read values from the sensor
  tempC = sht15->readTemperatureC();
  if (tempC < -40.0) {
    Serial.println("Error reading temperature");
  }
  tempF = sht15->readTemperatureF();
  humidity = sht15->readHumidity();  
}
//-------------------------------------------------------------------------------------------
void printOut()
{
  Serial.print(" Temp = ");
  Serial.print(tempF);
  Serial.print("F, ");
  Serial.print(tempC);
  Serial.println("C");
  Serial.print(" Humidity = ");
  Serial.print(humidity); 
  Serial.println("%");
}
