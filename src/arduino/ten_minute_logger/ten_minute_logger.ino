
/*
  Monitor weather station hardware and output an NMEA sentence containing observational data
  once every 10 minutes. Senors include:

  - Davis instruments Rain Gauge
  - Davis instruments Anemometer and wind direction sensor
  - Adafruit BMP085 air pressure sensor
  - SHT15 termperature and humidity sensor

  TODO: Add comms capability to log data to ThingSpeak IoT data collection portal
*/

#include <limits.h>
#include <stdint.h>

#include <Adafruit_BMP085.h>
#include "cactus_io_SHT15.h"

#define SHT_DataPin 4        // data pin for SHT temperature/humidity sensor
#define SHT_ClockPin 5       // clock pin for SHT temperature/humidity sensor


/*  Note:
    BMP085 pressure and temperature sensor
      SDA -- Pin A4
      SCL -- Pin A5
   hard coded in Wire library
*/

// globals constants

namespace {
constexpr long sleep_ms = 600000L; // ten minutes between each observation
}  // namespace

// global functions

/*
   Compute the number of milliseconds between the supplied from and to times
   noting that the time may have overflowed and wrapped to zero
*/
unsigned long getPeriod_msecs(unsigned long from_msecs, unsigned long to_msecs) {
  unsigned long result = 0;

  if (to_msecs < from_msecs) {
    // timer has wrapped around
    result = ULONG_MAX - from_msecs;
    result += to_msecs;
  } else {
    result = to_msecs - from_msecs;
  }
  return result;
}

// class definitions

/**
   Class RainGauge

   The rain gauge produced one electrical pulse per tip of the water collection bucket.
   The bucket holds the equivalent of 0.18mm of rainfall per bucket tip. Each buck tip causes
   a harware interrupt (0) on pin D3
*/
class RainGauge {

  private:
    static const int rainGaugePin = 3;  // rain guague connected to D3 via pullup resistor
    static const int rainInterrupt = 1; // tip of the bucket causes this interrupt
    static const float bucket_capacity = 0.18;

    volatile unsigned long tips = 0L; // cup rotation counter used in interrupt routine
    volatile unsigned long lastInterrupt = 0L; // Timer to avoid contact bounce in interrupt routine

    // Private constructor, obtain a RainGauge using RainGauge::instance().
    RainGauge() {
      pinMode(rainGaugePin, INPUT);
      attachInterrupt(rainInterrupt, []() {
        instance()->serviceInterrupt();
      }, FALLING);
    }

    /**
       Service a bucket-tip interrupt
    */
    void serviceInterrupt(void) {
      unsigned long now;
      unsigned long period_msecs;

      now = millis();
      period_msecs = getPeriod_msecs(lastInterrupt, now);

      if (period_msecs > 15 ) { // debounce the switch contact.
        tips++;
        lastInterrupt = now;
      }
    }

  public:
    static RainGauge* instance() {
      static RainGauge* inst = new RainGauge;
      return inst;
    }

    // Doesn't make sense to copy RainGauges.
    RainGauge(const RainGauge& other) = delete;

    /**
       Return the total rainfall during the integration period and reset
       the accumulator to zero
    */
    float getRainfall_mm() {
      // rainfall computed at 0.18mm per bucket tip
      cli(); // Disable interrupts so volatile variable don't change under us
      float rainfall_mm = tips * bucket_capacity;
      tips = 0;
      sei(); // Enables interrupts
      return rainfall_mm;
    }
};

/**
   Class WindMeter represents the Davis Anemometer connected to the weather station.

   For hardware details see: https://www.davisinstruments.com.au/product-page/6410-anemometer-for-vantage-pro
*/
class WindMeter {

  private:

    static constexpr int windSpeedPin = 2;  //D2 pin is connected to the wind speed reed switch (one pulse per revolution)
    static constexpr int windInterrupt = 0; // D2 pulse causes interrupt 0
    static constexpr int windOffset = 0;    // anemometer is aligned North/South
    static constexpr int windDirectionPin = A3; // wind direction is encode via a potentiometer reading 0V - 5V (0 to 360 degrees true)

    volatile unsigned int  rotations = 0;                      // number of rotations this period
    volatile unsigned long fastest_rot_msecs = 999999L;     // fastest rotation measured in milliseconds -- updates with 10 consecutive faster rotations
    volatile unsigned int  faster_count = 0;                   // increments when a faster rotation is measured
    volatile unsigned long aggregate_msecs = 0L;           // number of milliseconds for last n fast rotations
    volatile unsigned long last_interrupt_msecs = 0L;      // time of last interrupt

    // Private constructor, obtain a RainGauge using RainGauge::instance().
    WindMeter() {
      pinMode(windSpeedPin, INPUT);
      attachInterrupt(windInterrupt, []() {
        instance()->serviceInterrupt();
      }, FALLING);
    }

    /**
       Service the interrupt caused by a single rotation of the anemometer
    */
    void serviceInterrupt(void) {

      unsigned long now;
      unsigned long period_msecs;

      now = millis();
      period_msecs = getPeriod_msecs(last_interrupt_msecs, now);
      if (period_msecs > 15 ) { // debounce the switch contact.
        rotations++;     // accumulate the number of rotations so we can compute average speed over integration period
        last_interrupt_msecs = now;

        // record data so we can compute maximum wind gust during integration period

        if (period_msecs < fastest_rot_msecs) {
          faster_count++;                // this rotation beat past record
          aggregate_msecs += period_msecs; // sum the number of milliseconds over 10 rotations
          if (faster_count == 10) {
            fastest_rot_msecs = aggregate_msecs / 10; // previous record exceeded by 10 consecutive rotations
            faster_count = 0;
            aggregate_msecs = 0L;
          }
        } else { // didn't beat the record so zero the gust accumulators
          faster_count = 0L;
          aggregate_msecs = 0L;
        }
      }
    }

  public:

    static WindMeter* instance() {
      static WindMeter* inst = new WindMeter;
      return inst;
    }

    // Doesn't make sense to copy RainGauges.
    WindMeter(const WindMeter& other) = delete;
    /**
       Compute the wind direction by reading potentiometer voltage
       and convertion to degrees true
    */

    int getWindDirection_deg(void) {
      int vaneValue = analogRead(windDirectionPin);
      int direction = map(vaneValue, 0, 1023, 0, 360);
      direction = direction + windOffset;
      if (direction > 360) direction -= 360;
      if (direction < 0) direction += 360;
      return direction;
    }

    /**
       Compute and return the average wind speed in knots
    */
    float getWindspeed_kts(void) {

      // wind speed -- average over 10 minutes
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

    /**
       Compute and return the maximum wind gust speed over the integration period
    */
    float getGustSpeed_kts(void) {

      // v_kts = p * 1.9575 / t_sec
      // here p = 10 and t_sec = aggregate_msecs / 1000.0
      // therefor v_kts = 19575 / aggregate_msecs

      cli();
      float gust = 19575.0 / aggregate_msecs;

      // reset the accumulators for the next period

      fastest_rot_msecs = 99999L;
      faster_count = 0;
      aggregate_msecs = 0L;
      sei();

      return gust;
    }

};

/***
   Class Observations

   Represents all the observations that are made by the weather station within a fixed period of time
*/
class Observations {
  public:
    float temp_085_degC;     // air temperature degrees C as reported by the BMP085 pressure sensor
    int   pressure_Pa;       // barametric pressure in Pascals
    float humidity_Pcent;    // relative humidity as a percentage
    float temp_degC;         // temperature (degC) as reported by the SHT15
    float temp_degF;         // temperature (degF) as reported by the SHT15
    float dewpoint_degC;     // dew point (degC) as reported by the SHT15
    float rainfall_mm;       // rainfall aggregated since last observation
    float windSpeed_kts;     // windspeed in knots
    int   windDirection_deg; // wind direction in degrees
    float windGust_kts;      // wind gust ths period (sustained for 10 revs of anemometer)


    /**
       Read the sensors and record the results
    */
    void makeObservations(RainGauge* rainGauge, WindMeter* windMeter, SHT15 sht, Adafruit_BMP085 bmp ) {

      temp_085_degC = bmp.readTemperature();
      pressure_Pa = bmp.readPressure();
      humidity_Pcent = sht.getHumidity();
      temp_degC   = sht.getTemperature_C();
      temp_degF = sht.getTemperature_F();
      dewpoint_degC = sht.getDewPoint();
      rainfall_mm = rainGauge->getRainfall_mm();
      windSpeed_kts = windMeter->getWindspeed_kts();
      windDirection_deg = windMeter->getWindDirection_deg();
      windGust_kts = windMeter->getGustSpeed_kts();
    }

    /**
       Return a string in buffer containing the NMEA sentence with the following format:

          0 - $TRXDA

          1 - temp deg C
          2 - pressure Pascals
          3 - humidity (% to 2 decimal places)
          4 - temp deg C
          5 - temp deg F
          6 - dew point deg C
          7 - rainfall mm -- this period
          8 - wind speed knots
          9 - wind direction (cardinal point)
         10 - wind gust (knots) for this period (sustained for 10 revs of anemometer)

    */
    char * getNMEA(char * buff, int n) {
      snprintf(buff, n, "$TRXDA,%f,%f,%f,%f,%f,%f,%f,%d,%f",
               temp_degC,
               pressure_Pa / 100.0,
               humidity_Pcent,
               temp_degC,
               temp_degF,
               dewpoint_degC,
               rainfall_mm,
               windSpeed_kts,
               windDirection_deg,
               windGust_kts);

      return buff;
    }
};

// global variable

SHT15 sht = SHT15(SHT_DataPin, SHT_ClockPin);
WindMeter* windMeter = WindMeter::instance();
RainGauge* rainGauge = RainGauge::instance();
Observations obs;
Adafruit_BMP085 bmp;

/**
   Setup for the micro controller -- called once
*/
void setup() {
  Serial.begin(9600);
  if (!bmp.begin()) {
    Serial.println("Could not find a BMP085 sensor!");
    while (1) {}
  }
}

/**
   Main program loop -- called continuously
*/
void loop() {
  char buffer[80];         // String buffer for NMEA sentences

  delay(sleep_ms);
  obs.makeObservations(rainGauge, windMeter, sht, bmp);
  Serial.println(obs.getNMEA(buffer, sizeof(buffer)));
}
