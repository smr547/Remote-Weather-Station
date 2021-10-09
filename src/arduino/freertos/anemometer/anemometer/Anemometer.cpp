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
