#include <Arduino.h>
#include "Anemometer.h"
#include <limits.h>
#include "timing.h"
#include "signals.h"

// Private constructor, obtain a RainGauge using RainGauge::instance().
Anemometer::Anemometer() {
  pinMode(windSpeedPin, INPUT);
  attachInterrupt(windInterrupt, []() {
    instance()->serviceInterrupt();
  }, FALLING);
}

/**
   Service the interrupt caused by a single rotation of the anemometer
*/
void Anemometer::serviceInterrupt(void) {

  unsigned long now;
  unsigned long period_msecs;

  now = millis();
  period_msecs = getPeriod_msecs(last_interrupt_msecs, now);
  if (period_msecs > DEBOUNCE_MS ) { // debounce the switch contact.
    rotations++;     // accumulate the number of rotations so we can compute average speed over integration period
    last_interrupt_msecs = now;

    // record data so we can compute maximum wind gust during integration period

    if (period_msecs < fastest_rot_msecs) {
      faster_count++;                // this rotation beat past record
      aggregate_msecs += period_msecs; // sum the number of milliseconds over 10 rotations
      if (faster_count >= 10) {
        fastest_rot_msecs = aggregate_msecs / faster_count; // previous record exceeded by 10 consecutive rotations
        faster_count = 0;
        aggregate_msecs = 0L;
      }
    } else { // didn't beat the record so zero the gust accumulators
      faster_count = 0L;
      aggregate_msecs = 0L;
    }
  }
}




/**
   Compute the wind direction by reading potentiometer voltage
   and convertion to degrees true
*/

int Anemometer::getWindDirection_deg(void) {
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
float Anemometer::getWindspeed_kts(void) {

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
float Anemometer::getGustSpeed_kts(void) {

  // v_kts = p * 1.9575 / t_sec
  // here p = 10 and t_sec = aggregate_msecs / 1000.0
  // therefor v_kts = 19575 / aggregate_msecs

  float gust = 0.0;

  cli();
  if (fastest_rot_msecs != ULONG_MAX) {
    gust = 1955.0 / fastest_rot_msecs;
  }

  // reset the accumulators for the next period

  fastest_rot_msecs = ULONG_MAX;
  faster_count = 0;
  aggregate_msecs = 0L;
  sei();

  return gust;
}
