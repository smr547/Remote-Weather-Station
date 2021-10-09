
/* Monitor the animometer and publish results as NMEA sentences to the Serial port


    First lets play with timers

*/
#include <Arduino_FreeRTOS.h>
// #include <task.h>
// #include <queue.h>
#include <timers.h>
// #include <limits.h>


// *************************************  GLOBALS *****************************
static TimerHandle_t timerHndl1Sec = NULL;

/**************************************** CALLBACKS ****************************

*/


void vTimerCallback1SecExpired(TimerHandle_t pxTimer) {
  Serial.println("Timer has fired");
}

//****************************************************************************
// Main code starts here


void setup() {

  Serial.begin(9600);

  // delay a bit so we don't miss output

 // vTaskDelay( pdMS_TO_TICKS(1000));
  Serial.println();
  Serial.println("FreeRTOS timer demo");

  timerHndl1Sec = xTimerCreate(
                    "timer1Sec", /* name */
                    pdMS_TO_TICKS(1000), /* period/time */
                    pdTRUE, /* auto reload */
                    (void*)0, /* timer ID */
                    vTimerCallback1SecExpired); /* callback */
  if (timerHndl1Sec == NULL) {
    Serial.println("Could not create software timer");
    for (;;); /* failure! */

  }

 // vTaskDelay( pdMS_TO_TICKS(1000));
  Serial.println("Starting timers");


  if (xTimerStart(timerHndl1Sec, portMAX_DELAY) != pdPASS) {
    Serial.println("Could not start software timer");
    for (;;); /* failure!?! */
  }

  // delete this task so only the timer is running

  vTaskDelete(NULL);
}



void loop() {
  // put your main code here, to run repeatedly:

}
