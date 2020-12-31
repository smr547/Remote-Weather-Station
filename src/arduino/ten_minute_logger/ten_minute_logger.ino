
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


typedef struct observations {

  float temp_085_degC;     // air temperature degrees C as reported by the BMP085 pressure sensor
  int   pressure_Pa;       // barametric pressure in Pascals
  float humidity_Pcent;    // relative humidity as a percentage
  float temp_degC;         // temperature (degC) as reported by the SHT15
  float temp_degF;         // temperature (degF) as reported by the SHT15
  float dewpoint_degC;     // dew point (degC) as reported by the SHT15
  float rainfall_mm;       // rainfall aggregated since last observation
  float windSpeed_kts;     // windspeed in knots
  int   windDirection_deg; // wind direction in degrees

} 
Observations;

char * getNMEA(char * buff, Observations * obs);


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
  long sleep_ms = 600000L;
  Observations obs;      // storage structure for weather observations

    char buffer[8];
  // lets try to format an NMEA string (no checksum at this stage)

  obs.temp_085_degC = bmp.readTemperature();
  obs.pressure_Pa = bmp.readPressure();
  obs.humidity_Pcent = sht.getHumidity();
  obs.temp_degC   = sht.getTemperature_C();
  obs.temp_degF = sht.getTemperature_F();
  obs.dewpoint_degC = sht.getDewPoint();
  obs.rainfall_mm = getRainfall_mm();
  obs.windSpeed_kts = getWindspeed_kts();
  obs.windDirection_deg = getWindDirection_deg();

 
  Serial.print(getNMEA(buffer, &obs));
  delay(sleep_ms);
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

int getWindDirection_deg(void) {
  int vaneValue = analogRead(windDirectionPin);
  int direction = map(vaneValue, 0, 1023, 0, 360);
  direction = direction + windOffset;
  if (direction > 360) direction -= 360;
  if (direction < 0) direction += 360;
  return direction;
}

float getRainfall_mm() {
  // rainfall computed at 0.18mm per bucket tip
  cli(); // Disable interrupts so volatile variable don't change under us
  float rainfall_mm = tips * 0.18;
  tips = 0; 
  sei(); // Enables interrupts
  return rainfall_mm;
}

float getWindspeed_kts(void) {

  // wind speed -- average over 10 minutes
  // TODO: support gusts


  // convert to knots using the formula V=P(2.25/T)*0.87
  // V_kts = P * (2.25/600) * 0.87
  // V_kts = P * 0.0032625
  // 
  cli(); // Disable interrupts -- prevents volitile variable changing during calcs

  float windSpeed_kts = rotations * 0.0032625;
  rotations = 0;
  sei(); // Enables interrupts
  return windSpeed_kts;

}

char * getNMEA(char * buff, Observations * obs) {

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
   
   
   
   float temp_085_degC;     // air temperature degrees C as reported by the BMP085 pressure sensor
   int   pressure_Pa;       // barametric pressure in Pascals
   float humidity_Pcent;    // relative humidity as a percentage
   float temp_degC;         // temperature (degC) as reported by the SHT15
   float temp_degF;         // temperature (degF) as reported by the SHT15
   float dewpoint_degC;     // dew point (degC) as reported by the SHT15
   float rainfall_mm;       // rainfall aggregated since last observation
   float windSpeed_kts;     // windspeed in knots
   int   windDirection_deg; // wind direction in degrees
   
   */
  sprintf(buff, "$TRXDA,%f,%f,%f,%f,%f,%f,%f,%d\n", 
  obs->temp_degC, 
  obs->pressure_Pa/100.0,
  obs->humidity_Pcent,
  obs->temp_degC,
  obs->temp_degF,
  obs->dewpoint_degC,
  obs->rainfall_mm,
  obs->windSpeed_kts,
  obs->windDirection_deg);

  return buff;
}













