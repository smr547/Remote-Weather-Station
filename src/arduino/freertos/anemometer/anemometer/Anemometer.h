/*
   Anemometer.h

    Created on: Jan 25, 2021
        Author: stevenring
*/

#ifndef SRC_ANEMOMETER_H_
#define SRC_ANEMOMETER_H_

#include <limits.h>

/**
   Class Anemometer represents the Davis Anemometer connected to the weather station.

   For hardware details see: https://www.davisinstruments.com.au/product-page/6410-anemometer-for-vantage-pro
*/
class Anemometer {

  private:

    static constexpr int windSpeedPin = 2;  //D2 pin is connected to the wind speed reed switch (one pulse per revolution)
 //   static constexpr int windInterrupt = 0; // D2 pulse causes interrupt 0
    static constexpr int windOffset = 0;    // anemometer is aligned North/South
    static constexpr int windDirectionPin = A3; // wind direction is encode via a potentiometer reading 0V - 5V (0 to 360 degrees true)

    volatile unsigned int  rotations = 0;                      // number of rotations this periodxs
 //   volatile unsigned long last_interrupt_msecs = 0L;      // time of last interrupt

    // Private constructor, obtain a RainGauge using RainGauge::instance().
    Anemometer();

    /**
       Service the interrupt caused by a single rotation of the anemometer
    */
    void serviceInterrupt(void);

  public:

    static Anemometer* instance() {
      static Anemometer* inst = new Anemometer;
      return inst;
    }

    Anemometer(const Anemometer& other) = delete;
    int getWindDirection_deg(void);
    float getWindspeed_kts(unsigned int rotationsInPeriod, unsigned int period_ms);
    // float getGustSpeed_kts(void);
    unsigned int getRotations(void);
    unsigned int getRotationsSince(unsigned int rotationsNow, unsigned int rotationsBefore);
};

#endif /* SRC_ANEMOMETER_H_ */
