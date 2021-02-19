/*
   Observations.h

    Created on: Jan 25, 2021
        Author: stevenring
*/

#ifndef SRC_OBSERVATIONS_H_
#define SRC_OBSERVATIONS_H_

class Observations {
  private:
    Observations() {};
    //    virtual ~Observations();
  public:
    // Member variable

    float temp_C;
    float humidity_PC;
    float dewPoint_C;
    unsigned int pressure_Pa;
    float windSpeed_kts;
    float windGusts_kts;
    int windDirection_deg;
    // rain guage
    float rainfall_mm;
    float rain929_mm;
    // GPS data

    /*
    unsigned int year;
    unsigned short month;
    unsigned short day;
    unsigned short hrs;
    unsigned short mins;
    unsigned short secs;
    double lat_deg;
    double long_deg;
    double alt_m;
*/
    static Observations* instance() {
      static Observations* inst = new Observations;
      return inst;
    }

    // Doesn't make sense to copy Observations.
    Observations(const Observations& other) = delete;
};

#endif /* SRC_OBSERVATIONS_H_ */
