#include <Arduino.h>
#include "Anemometer.h"
#include <limits.h>
// #include "timing.h"
// #include "signals.h"

// Private constructor, obtain a Anemometer using Anemometer::instance().
Anemometer::Anemometer() {
  pinMode(windSpeedPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(windSpeedPin), []() {
    instance()->serviceInterrupt();
  }, FALLING);
}

/**
   Service the interrupt caused by a single rotation of the anemometer
*/
void Anemometer::serviceInterrupt(void) {
  rotations++;
}

int Anemometer::getWindDirection_deg(void) {
  int vaneValue = analogRead(A3);
  int direction = map(vaneValue, 0, 1023, 0, 360);
  int calDirection = direction + windOffset;

  if (calDirection >= 360)
    calDirection = calDirection - 360;

  if (calDirection < 0)
    calDirection = calDirection + 360;

  return calDirection;
}

unsigned int Anemometer::getRotations(void) {
  unsigned int result;

  while (1) {
    result = rotations;
    return result;
    if (result == rotations) {
      return result;
    }
  }
}

unsigned int Anemometer::getRotationsSince(unsigned int rotationsNow, unsigned int rotationsBefore) {
  unsigned int result;
  if (rotationsNow < rotationsBefore) {
    result = UINT_MAX - rotationsBefore + rotationsNow;  // overflow has occurred
  } else {
    result = rotationsNow - rotationsBefore;
  }
  return result;
}


// V = P(1.955196/T) knots

float Anemometer::getWindspeed_kts(unsigned int rotationsInPeriod, unsigned int period_ms) {
  float period_secs = period_ms / 1000.0;
  return rotationsInPeriod * 1.955196 / period_secs;
}

windVector * Anemometer::getWindVector(windVector * windVec, float windSpeed_kts, float windDirection_deg) {
  float wd_rads = radians(windDirection_deg);
  windVec->eVel = windSpeed_kts * sin(wd_rads);
  windVec->nVel = windSpeed_kts * cos(wd_rads); 
  return windVec;
}
