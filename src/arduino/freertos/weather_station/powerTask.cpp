#include "signals.h"
#include "powerTask.h"

/*
   This tasks monitors weather station power production and consumption
*/
void task_PowerTask(void *pvParameters) {
  (void) pvParameters;

  SIGNAL sig;

  for (;;) {
    // read the queue
#ifdef DEBUG
    Serial.println("Power waiting for signal");
    delay(2);
#endif
    if (xQueueReceive( q_PowerTask, &sig, portMAX_DELAY ) == pdPASS) {
      switch (sig) {
        case READ:  // read the wind gust value and record the result in Observations
          //observations->windGusts_kts = anemometer->getGustSpeed_kts();
#ifdef DEBUG
          Serial.println("Voltages been read");
          delay(2);
#endif
          break;

        default:
          break; // ignore unknown signal
      }
    }
  }
}
