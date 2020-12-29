
/*
Wire weather data as NMEA sentence
 
 */

#include <Wire.h>
#include <Adafruit_BMP085.h>
#include "cactus_io_SHT15.h"
#include <math.h>

#define rainGaugePin (3) // rain guage bucket tipping sensor
#define rainInterrupt (1)
#define windSpeedPin (2) // D2 pin is connected to the wind speed reed switch (one pulse per revolution)
#define windInterrupt (0)
#define windOffset 0;
int windDirectionPin = A3;

volatile unsigned long tips; // cup rotation counter used in interrupt routine
volatile unsigned long contactBounceTime; // Timer to avoid contact bounce in interrupt routine
volatile unsigned long rotations; // cup rotation counter used in interrupt routine

float windSpeed_kts;    // wind speed in knots


float rainfall_mm;    // aggregate rainfall in mm

int SHT_DataPin = 4;
int SHT_ClockPin = 5;

SHT15 sht = SHT15(SHT_DataPin, SHT_ClockPin);


/* BMP085 pressure and temperature sensor
 *  
 *  SDA -- Pin A4
 *  SCL -- Pin A5
 * 
 * hard coded in Wire library
 */


Adafruit_BMP085 bmp;
void setup()
{
  Serial.begin(9600);
  if (!bmp.begin())
  {
    Serial.println("Could not find a BMP085 sensor!");
    while (1) {
    }
  }

  pinMode(rainGaugePin, INPUT);
  tips = 0;
  attachInterrupt(rainInterrupt, isr_bucket_tip, FALLING);

  pinMode(windSpeedPin, INPUT);
  attachInterrupt(windInterrupt, isr_rotation, FALLING);
}


void loop()
{
  char buffer[8];
  // lets try to format an NMEA string (no checksum at this stage)

  sprintf(buffer, "%s" , "$TRXDA,");
  /* output sentence contains  
   *  
   *  0 - $TRXDA
   *  
   *  1 - temp deg C
   *  2 - pressure Pascals
   *  3 - humidity (% to 2 decimal places)
   *  4 - temp deg C
   *  5 - temp deg F
   *  6 - dew point deg C
   *  7 - rainfall mm -- this period
   *  8 - wind speed knots
   *  9 - wind direction (cardinal point)
   */
  sht.readSensor();
  Serial.print(buffer);
  Serial.print(bmp.readTemperature());
  Serial.print(",");
  Serial.print(bmp.readPressure()); 
  Serial.print(",");
  Serial.print(sht.getHumidity()); 
  Serial.print(",");
  Serial.print(sht.getTemperature_C()); 
  Serial.print(",");
  Serial.print(sht.getTemperature_F()); 
  Serial.print(",");
  Serial.print(sht.getDewPoint());

  // rain gauge
  // convert tips to rainfall_mm using the formula R=T*0.18
  cli(); // Disable interrupts
  rainfall_mm = tips * 0.18;
  tips = 0;
  sei(); // Enables interrupts
  Serial.print(",");
  Serial.print(rainfall_mm);


  // wind speed -- average over 10 minutes
  // TODO: support gusts


  // convert to knots using the formula V=P(2.25/T)*0.87
  // V_kts = P * (2.25/600) * 0.87
  // V_kts = P * 0.0032625
  // 
  cli(); // Disable interrupts -- prevents volitile variable changing during calcs

  windSpeed_kts = rotations * 0.0032625;
  rotations = 0;
  sei(); // Enables interrupts
  Serial.print(",");
  Serial.print(windSpeed_kts);

  // wind direction

  Serial.print(",");
  Serial.print(getWindDirection());

  // sleep for 10 mins

  Serial.print("\n");
  delay(600000);
}

// This is the function that the interrupt calls to increment the tip count
void isr_bucket_tip () {

  if ((millis() - contactBounceTime) > 15 ) { // debounce the switch contact.
    tips++;
    contactBounceTime = millis();
  }

}

// This is the function that the interrupt calls on each rotation of the anonometer
void isr_rotation () {

  if ((millis() - contactBounceTime) > 15 ) { // debounce the switch contact.
    rotations++;
    contactBounceTime = millis();
  }

}

int getWindDirection(void) {
  int vaneValue = analogRead(windDirectionPin);
  int direction = map(vaneValue, 0, 1023, 0, 360);
  direction = direction + windOffset;
  if (direction > 360) direction -= 360;
  if (direction < 0) direction += 360;
  return direction;
}











