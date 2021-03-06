/*
   Observations.h

    Created on: Jan 25, 2021
        Author: stevenring
*/

#ifndef SRC_OBSERVATIONS_H_
#define SRC_OBSERVATIONS_H_

class Observations {
  private:
    Observations();
    //    virtual ~Observations();


  public:
    // Member variable

    int32_t pressure_Pa;
    int windDirection_deg;
    float temp_C;
    float humidity_PC;
    float dewPoint_C;
    float windSpeed_kts;
    float windGusts_kts;
    // rain guage
    float rainfall_mm;
    float rainfall929_mm;
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

    // power supply measures

    float v_battery_volts; // battery voltage
    float i_battery_amps; // amps to/from battery (-ve =  discharging, +ve = charging
    float v_solar_volts; // voltage across solar panel
    float i_solar_amps; // current from solar panel
    float v_in_volts;  // arduino voltage
    float i_in_amps;   // arduino current


    
    static Observations* instance() {
      static Observations* inst = new Observations;
      return inst;
    }

    // Doesn't make sense to copy Observations.
    Observations(const Observations& other) = delete;

    int getNMEA(char * buffer, int max_len);
};

#endif /* SRC_OBSERVATIONS_H_ */
