#include "timing.h"
#include <limits.h>

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
