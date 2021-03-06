#include <arduino.h>
#include "Observations.h"
#include <stdio.h>



Observations::Observations() {

  temp_C = 0.0;
  humidity_PC = 0.0;
  dewPoint_C = 0.0;
  windSpeed_kts = 0.0;
  windGusts_kts = 0.0;
  rainfall_mm = 0.0;
  rainfall929_mm  = 0.0;
  pressure_Pa = 0;
  windDirection_deg = 0;

}

/**
   Returns, in the supplied buffer, a NMEA string with the following format:

      0 - $TRXDA
      1 - pressure Pascals
      2 - wind direction (degrees true)
      3 - temp_C,
      4 - humidity_PC,
      5 - dewPoint_C,
      6 - windSpeed_kts,
      7 - windGusts_kts,
      8 - rainfall_mm,
      9 - rainfall929_mm

     Need to respect the max_len of the supplied buff.
     Also need to work around non-support of the "%f" printf directive

     Returns the length of the string placed in the buffer.
     Returns a negative value in the event of an error

*/
int Observations::getNMEA(char * buff, int max_len) {


  float values[7] = {
    temp_C,
    humidity_PC,
    dewPoint_C,
    windSpeed_kts,
    windGusts_kts,
    rainfall_mm,
    rainfall929_mm
  };

  char str_temp[16];
  char * buffPtr = buff;
  int maxLen = max_len;
  int n;

  // integer values first

  n = snprintf(buffPtr, maxLen, "$TRXDA,%ld,%d", pressure_Pa, windDirection_deg);
  if (n < 0) return n;

  buffPtr += n;
  maxLen -= n;

  values[0] = temp_C; //testing

  // now the float values
  for (int i = 0; i < 7; i++) {
    dtostrf(values[i], 4, 2, str_temp);
    n = snprintf(buffPtr, maxLen, ",%s", str_temp);
    Serial.print("building float ");
    Serial.print(i);
    Serial.print(", ");
    Serial.print(values[i]);
    Serial.print(", ");
    Serial.println(str_temp);
    if (n < 0) {
#ifdef DEBUG
      Serial.println("Error formatting NMEA");
      delay(2);
#endif
      break;
    }
    buffPtr += n;
    maxLen -= n;
  }

  // all done

  return strlen(buff);

}
