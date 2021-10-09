
/* Monitor the animometer and publish results as NMEA sentences to the Serial port


    First lets play with timers

*/
#include <Arduino_FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <timers.h>
#include "Anemometer.h"

#define STRING_BUF_LEN 40
#define TEST_OUTPUT_PIN 7
// #include <limits.h>




// *************************************  GLOBALS *****************************
static Anemometer* anemometer = Anemometer::instance();
static TimerHandle_t timerHndl1Sec = NULL;
static TimerHandle_t timerHndlTestOutput = NULL;
static QueueHandle_t serialQ = NULL;
static const uint8_t serialQ_len = 2;
static TaskHandle_t serialOutputTask = NULL;

static constexpr int windSpeedPin = 2;  //D2 pin is connected to the wind speed reed switch (one pulse per revolution)

bool test_output_var = false;

//**************************************** CALLBACKS ****************************

void vTimerCallback1SecExpired(TimerHandle_t pxTimer) {
  char buf[STRING_BUF_LEN];
  char fp_buf[16];
  unsigned int rotations; 
  static unsigned int lastRotations = 0;
  static unsigned long last_ms = 0;

  unsigned long now_ms = millis();
  unsigned long period_ms = now_ms - last_ms;

  

  rotations = anemometer->getRotations();
  float windSpeed_kts = anemometer->getWindspeed_kts(anemometer->getRotationsSince(rotations, lastRotations), period_ms);
  dtostrf(windSpeed_kts, 6, 1, fp_buf);
  sprintf(buf, "rotations=%d, (%d) %s knots", 
      rotations, 
      anemometer->getRotationsSince(rotations, lastRotations),
      fp_buf
      );
  serialOut(buf);
  lastRotations = rotations;
  last_ms = now_ms;
}


void vTimerCallbackHalfRotation(TimerHandle_t pxTimer) {
  
  digitalWrite(TEST_OUTPUT_PIN, test_output_var);
  test_output_var = !test_output_var;
}


//**************************************** TASKS ****************************


/**
   Task to handle all serial output
*/
void serialOutputCode(void * pvParameters) {

  char buff[STRING_BUF_LEN];

  //Initialize serial and wait for port to open:

  /*
    Serial.begin(9600);
    while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB
    vTaskDelay( pdMS_TO_TICKS(1000));
    }
  */
  while (1) {
    if (xQueueReceive(serialQ, &buff, 0) == pdTRUE) {
      Serial.println(buff);
    }
  }

}

void serialOut(char * myString) {
  if (strlen(myString) > STRING_BUF_LEN) {
    myString[STRING_BUF_LEN - 1] = '\0';
  }
  xQueueSend(serialQ, myString, 0);
}

//****************************************************************************
// Main code starts here


void setup() {

  pinMode(TEST_OUTPUT_PIN, OUTPUT);

  Serial.begin(115200);

  // create Q to handle messages to be output to the Serial port
  serialQ = xQueueCreate(serialQ_len, STRING_BUF_LEN);

  if (serialQ == NULL) {
    Serial.println("Serial Q could not be created");
  } else {
    Serial.println("Serial Q created");
  }

  if (anemometer == NULL) {
    Serial.println("anemometer is NOT assigned");
  }


  Serial.println((long) anemometer);


  // start the serial output task
  Serial.println("Starting serial output task");
  BaseType_t xReturned;
  xReturned = xTaskCreate(serialOutputCode,
                          "Serial output task",
                          256,
                          NULL,
                          2,
                          NULL) ;

  if (xReturned == pdPASS) {
    Serial.println("Serial output task created");
  } else {
    Serial.println("Serial output task NOT created");
  }

  
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

    timerHndlTestOutput = xTimerCreate(
                    "time_test_output", /* name */
                    pdMS_TO_TICKS(25), /* period/time */
                    pdTRUE, /* auto reload */
                    (void*)0, /* timer ID */
                    vTimerCallbackHalfRotation); /* callback */

  
  Serial.println("Starting timers");


  if (xTimerStart(timerHndl1Sec, portMAX_DELAY) != pdPASS) {
    Serial.println("Could not start software timer");
    for (;;); /* failure!?! */
  }

  if (xTimerStart(timerHndlTestOutput, portMAX_DELAY) != pdPASS) {
    Serial.println("Could not start software timer");
    for (;;); /* failure!?! */
  }



  Serial.println("At the end of setup()");

  // delete this task so only the timer is running

  // vTaskDelete(NULL);
}



void loop() {
  // put your main code here, to run repeatedly:

}
