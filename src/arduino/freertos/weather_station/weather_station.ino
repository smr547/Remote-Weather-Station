#include <Arduino_FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <timers.h>
#include <limits.h>


#define DEBUG 1
// #undef DEBUG

#include "Observations.h"

// define two tasks for Blink & AnalogRead
void TaskBlink( void *pvParameters );
void TaskAnalogRead( void *pvParameters );
void task_Sequencer(void *pvParameters);
void task_WindVaneReader(void *pvParameters);
void task_WindGustReader(void *pvParameters);
void task_TempReader(void *pvParameters);
void task_HumidityReader(void *pvParameters);
void task_DataReporter(void *pvParameters);
void reportTimerCallback(void);

/************************************************************************************
   Global constants
 ************************************************************************************/

namespace {
// constexpr long sleep_ms = 600000L; // ten minutes between each observation
constexpr int debounce_ms = 3;     // time window within which consecutive interrupts are ignored
}  // namespace


/*************************************************************************************
    Global functions
 *************************************************************************************/

#ifdef DEBUG
void pass_fail_msg(const char * msg, BaseType_t rc) {
  Serial.print(msg);
  Serial.print(": ");
  if (rc == pdPASS) {
    Serial.println("ok");
  }
  if (rc == pdFAIL) {
    Serial.println("FAILED!");

  }
  delay(2);
}
#endif
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

/*************************************************************************************
   SENSOR Classes (including IRQ handlers
 *************************************************************************************/

/**
   Class Anemometer represents the Davis Anemometer connected to the weather station.

   For hardware details see: https://www.davisinstruments.com.au/product-page/6410-anemometer-for-vantage-pro
*/
class Anemometer {

  private:

    static constexpr int windSpeedPin = 2;  //D2 pin is connected to the wind speed reed switch (one pulse per revolution)
    static constexpr int windInterrupt = 0; // D2 pulse causes interrupt 0
    static constexpr int windOffset = 0;    // anemometer is aligned North/South
    static constexpr int windDirectionPin = A3; // wind direction is encode via a potentiometer reading 0V - 5V (0 to 360 degrees true)

    volatile unsigned int  rotations = 0;                      // number of rotations this period
    volatile unsigned long fastest_rot_msecs = ULONG_MAX;     // fastest rotation measured in milliseconds -- updates with 10 consecutive faster rotations
    volatile unsigned int  faster_count = 0;                   // increments when a faster rotation is measured
    volatile unsigned long aggregate_msecs = 0L;           // number of milliseconds for last n fast rotations
    volatile unsigned long last_interrupt_msecs = 0L;      // time of last interrupt

    // Private constructor, obtain a RainGauge using RainGauge::instance().
    Anemometer() {
      pinMode(windSpeedPin, INPUT);
      attachInterrupt(windInterrupt, []() {
        instance()->serviceInterrupt();
      }, FALLING);
    }

    /**
       Service the interrupt caused by a single rotation of the anemometer
    */
    void serviceInterrupt(void) {

      unsigned long now;
      unsigned long period_msecs;

      now = millis();
      period_msecs = getPeriod_msecs(last_interrupt_msecs, now);
      if (period_msecs > debounce_ms ) { // debounce the switch contact.
        rotations++;     // accumulate the number of rotations so we can compute average speed over integration period
        last_interrupt_msecs = now;

        // record data so we can compute maximum wind gust during integration period

        if (period_msecs < fastest_rot_msecs) {
          faster_count++;                // this rotation beat past record
          aggregate_msecs += period_msecs; // sum the number of milliseconds over 10 rotations
          if (faster_count >= 10) {
            fastest_rot_msecs = aggregate_msecs / faster_count; // previous record exceeded by 10 consecutive rotations
            faster_count = 0;
            aggregate_msecs = 0L;
          }
        } else { // didn't beat the record so zero the gust accumulators
          faster_count = 0L;
          aggregate_msecs = 0L;
        }
      }
    }

  public:

    static Anemometer* instance() {
      static Anemometer* inst = new Anemometer;
      return inst;
    }

    // Doesn't make sense to copy RainGauges.
    Anemometer(const Anemometer& other) = delete;
    /**
       Compute the wind direction by reading potentiometer voltage
       and convertion to degrees true
    */

    int getWindDirection_deg(void) {
      int vaneValue = analogRead(windDirectionPin);
      int direction = map(vaneValue, 0, 1023, 0, 360);
      direction = direction + windOffset;
      if (direction > 360) direction -= 360;
      if (direction < 0) direction += 360;
      return direction;
    }

    /**
       Compute and return the average wind speed in knots
    */
    float getWindspeed_kts(void) {

      // wind speed -- average over 10 minutes
      // convert to knots using the formula V=P(2.25/T)*0.87
      // V_kts = P * (2.25/600) * 0.87
      // V_kts = P * 0.0032625
      //
      cli(); // Disable interrupts -- prevents volitile variable changing during calcs
      float windSpeed_kts = rotations * 0.0032625;
      rotations = 0;
      sei(); // Enables interrupts
      return windSpeed_kts;

    }

    /**
       Compute and return the maximum wind gust speed over the integration period
    */
    float getGustSpeed_kts(void) {

      // v_kts = p * 1.9575 / t_sec
      // here p = 10 and t_sec = aggregate_msecs / 1000.0
      // therefor v_kts = 19575 / aggregate_msecs

      float gust = 0.0;

      cli();
      if (fastest_rot_msecs != ULONG_MAX) {
        gust = 1955.0 / fastest_rot_msecs;
      }

      // reset the accumulators for the next period

      fastest_rot_msecs = ULONG_MAX;
      faster_count = 0;
      aggregate_msecs = 0L;
      sei();

      return gust;
    }
    /*
        String getDebugString(void) {
          String comma = ", ";
          String s = String(fastest_rot_msecs) + comma +
                     String(faster_count) + comma +
                     String(aggregate_msecs) + comma +
                     String(last_interrupt_msecs);

          return s;
        }
    */
};




/**
   Class RainGauge

   The rain gauge produced one electrical pulse per tip of the water collection bucket.
   The bucket holds the equivalent of 0.18mm of rainfall per bucket tip. Each buck tip causes
   a harware interrupt (0) on pin D3
*/
class RainGauge {

  private:
    static const int rainGaugePin = 3;  // rain guague connected to D3 via pullup resistor
    static const int rainInterrupt = 1; // tip of the bucket causes this interrupt
    static const float bucket_capacity = 0.18;

    volatile unsigned long tips = 0L; // cup rotation counter used in interrupt routine
    volatile unsigned long lastInterrupt = 0L; // Timer to avoid contact bounce in interrupt routine

    // Private constructor, obtain a RainGauge using RainGauge::instance().
    RainGauge() {
      pinMode(rainGaugePin, INPUT);
      attachInterrupt(rainInterrupt, []() {
        instance()->serviceInterrupt();
      }, FALLING);
    }

    /**
       Service a bucket-tip interrupt
    */
    void serviceInterrupt(void) {
      unsigned long now;
      unsigned long period_msecs;

      now = millis();
      period_msecs = getPeriod_msecs(lastInterrupt, now);

      if (period_msecs > debounce_ms ) { // debounce the switch contact.
        tips++;
        lastInterrupt = now;
      }
    }

  public:
    static RainGauge* instance() {
      static RainGauge* inst = new RainGauge;
      return inst;
    }

    // Doesn't make sense to copy RainGauges.
    RainGauge(const RainGauge& other) = delete;

    /**
       Return the total rainfall during the integration period and reset
       the accumulator to zero
    */
    float getRainfall_mm() {
      // rainfall computed at 0.18mm per bucket tip
      cli(); // Disable interrupts so volatile variable don't change under us
      float rainfall_mm = tips * bucket_capacity;
      tips = 0;
      sei(); // Enables interrupts
      return rainfall_mm;
    }
};



/**************************************************************************************************
   Global Variables
 **************************************************************************************************/
Anemometer* anemometer;
RainGauge* rainGauge;
Observations* observations;
int ticks;   // incremented each report period
int period_per_tick_millisecs;
int ticks_per_day;

// Signals

enum SIGNAL {REPORT, TICK, ALARM_9AM, READ};


// Timers

TimerHandle_t t_reportTimer;

// Sequencers's queue
TaskHandle_t th_Sequencer = NULL;
QueueHandle_t q_Sequencer = NULL;

// Wind vane reader's queue
TaskHandle_t th_WindVaneReader = NULL;
QueueHandle_t q_WindVaneReader = NULL;


// Wind gust reader's queue
TaskHandle_t th_WindGustReader = NULL;
QueueHandle_t q_WindGustReader = NULL;


// Wind gust reader's queue
TaskHandle_t th_TempReader = NULL;
QueueHandle_t q_TempReader = NULL;

// Wind gust reader's queue
TaskHandle_t th_HumidityReader = NULL;
QueueHandle_t q_HumidityReader = NULL;

// Rain gauge reader's queue
TaskHandle_t th_RainGaugeReader = NULL;
QueueHandle_t q_RainGaugeReader = NULL;

// DataReporter task and signal queue
TaskHandle_t th_DataReporter = NULL;
QueueHandle_t q_DataReporter = NULL;


TaskHandle_t th_Blink = NULL;
TaskHandle_t th_AnalogRead = NULL;

// the setup function runs once when you press reset or power the board
void setup() {

  BaseType_t rc;  // return code from FreeRTOS

  // initialize serial communication at 9600 bits per second for debugging
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB, on LEONARDO, MICRO, YUN, and other 32u4 based boards.
  }

#ifdef DEBUG
  Serial.println("in setup()");
  delay(2);

#endif

  // initialialise globals
  anemometer = Anemometer::instance();
  rainGauge = RainGauge::instance();
  observations = Observations::instance();
  ticks = 0;   // incremented each report period
  //  period_per_tick_millisecs = 600000; // ten minutes
  period_per_tick_millisecs = 20000;  // 5 second
  ticks_per_day = 86400000 / period_per_tick_millisecs;

  // setup timers

#ifdef DEBUG
  Serial.println("seting up timers");
  delay(2);
#endif

  t_reportTimer = xTimerCreate("ReportTimer", pdMS_TO_TICKS(period_per_tick_millisecs), pdTRUE, NULL, reportTimerCallback);

  // setup task queues
  Serial.println("seting queues");
  delay(2);
  q_Sequencer = xQueueCreate(5, sizeof(SIGNAL));
  q_WindVaneReader = xQueueCreate(1, sizeof(SIGNAL));
  q_WindGustReader = xQueueCreate(1, sizeof(SIGNAL)); 
  q_TempReader = xQueueCreate(1, sizeof(SIGNAL));
  q_HumidityReader = xQueueCreate(1, sizeof(SIGNAL));
  q_RainGaugeReader = xQueueCreate(2, sizeof(SIGNAL));
  q_DataReporter = xQueueCreate(1, sizeof(SIGNAL));

  // Now set up tasks to run independently.

#ifdef DEBUG
  Serial.println("seting up tasks");
  delay(2);
#endif
  rc = xTaskCreate(
         task_Sequencer
         ,  "SEQ"   // A name just for humans
         ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
         ,  NULL
         ,  3  // Priority
         ,  &th_Sequencer );

#ifdef DEBUG
  pass_fail_msg("create Sequencer Task", rc);
#endif


  rc = xTaskCreate(
         task_WindVaneReader
         ,  "VANE"   // A name just for humans
         ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
         ,  NULL
         ,  2  // Priority
         ,  &th_WindVaneReader );

#ifdef DEBUG
  pass_fail_msg("create WindVaneReader Task", rc);
#endif


  rc = xTaskCreate(
         task_WindGustReader
         ,  "GUST"   // A name just for humans
         ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
         ,  NULL
         ,  2  // Priority
         ,  &th_WindGustReader );

#ifdef DEBUG
  pass_fail_msg("create WindGustReader Task", rc);
#endif


  rc = xTaskCreate(
         task_TempReader
         ,  "TEMP"   // A name just for humans
         ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
         ,  NULL
         ,  2  // Priority
         ,  &th_TempReader );

#ifdef DEBUG
  pass_fail_msg("create TempReader Task", rc);
#endif


  rc = xTaskCreate(
         task_HumidityReader
         ,  "HUMID"   // A name just for humans
         ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
         ,  NULL
         ,  2  // Priority
         ,  &th_HumidityReader );

#ifdef DEBUG
  pass_fail_msg("create HumidityReader Task", rc);
#endif


  rc = xTaskCreate(
         task_RainGaugeReader
         ,  "RAIN"   // A name just for humans
         ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
         ,  NULL
         ,  2  // Priority
         ,  &th_RainGaugeReader );

#ifdef DEBUG
  pass_fail_msg("create WindGustReader Task", rc);
#endif



  rc = xTaskCreate(
         task_DataReporter
         ,  "REPORT"   // A name just for humans
         ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
         ,  NULL
         ,  2  // Priority
         ,  &th_DataReporter );

#ifdef DEBUG
  pass_fail_msg("create DataReporter Task", rc);
#endif


  Serial.println("Creating Blink task");
  delay(2);
  rc = xTaskCreate(
         TaskBlink
         ,  "Blink"   // A name just for humans
         ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
         ,  NULL
         ,  0  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
         ,  &th_Blink );

#ifdef DEBUG
  pass_fail_msg("create Blink Task", rc);
#endif

  /*
    rc = xTaskCreate(
           TaskAnalogRead
           ,  "AnalogRead"
           ,  128  // Stack size
           ,  NULL
           ,  1  // Priority
           ,  &th_AnalogRead );
    if (rc == pdPASS) {
      Serial.println("AnalogRead Task created");

    delay(2);
    }
    if (rc == pdFAIL) {
      Serial.print("AnalogRead Task failed");
    }
  */
  // start the timer

#ifdef DEBUG
  Serial.println("starting main timer");
  delay(2);
#endif

  rc = xTimerStart(t_reportTimer, 0);

#ifdef DEBUG
  pass_fail_msg("Timer start", rc);
#endif

#ifdef DEBUG
  delay(1000);
  Serial.println("starting FreeRTOS scheduler");
  delay(2);
#endif

  // Now the task scheduler, which takes over control of scheduling individual tasks, is automatically started.


}

void loop()
{

  // Empty. Things are done in Tasks.
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

void TaskBlink(void *pvParameters) {
  (void) pvParameters;

  /*
    Blink
    Turns on an LED on for one second, then off for one second, repeatedly.
    Most Arduinos have an on-board LED you can control. On the UNO, LEONARDO, MEGA, and ZERO
    it is attached to digital pin 13, on MKR1000 on pin 6. LED_BUILTIN takes care
    of use the correct LED pin whatever is the board used.

    The MICRO does not have a LED_BUILTIN available. For the MICRO board please substitute
    the LED_BUILTIN definition with either LED_BUILTIN_RX or LED_BUILTIN_TX.
    e.g. pinMode(LED_BUILTIN_RX, OUTPUT); etc.

    If you want to know what pin the on-board LED is connected to on your Arduino model, check
    the Technical Specs of your board  at https://www.arduino.cc/en/Main/Products

    This example code is in the public domain.
    modified 8 May 2014
    by Scott Fitzgerald

    modified 2 Sep 2016
    by Arturo Guadalupi
  */

  // initialize digital LED_BUILTIN on pin 13 as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  for (;;) {
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    vTaskDelay( 1000 / portTICK_PERIOD_MS ); // wait for one second
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    vTaskDelay( 1000 / portTICK_PERIOD_MS ); // wait for one second
#ifdef DEBUG
    Serial.println("LED flashed");
#endif
  }
}

/**
   Main controller is called the "Sequencer"
*/
void task_Sequencer(void *pvParameters) {

  SIGNAL sig_received;
  SIGNAL sig;
  BaseType_t rc;  // return code from FreeRTOS

  for (;;) {
    // WAITING
    // read the queue
#ifdef DEBUG
    pass_fail_msg("Sequencer waiting to read queue", pdPASS);
#endif
    if (xQueueReceive( q_Sequencer, &sig_received, portMAX_DELAY ) == pdPASS) {
      switch (sig_received) {
        case TICK:
          // 10 minute timer tick
#ifdef DEBUG
          Serial.println("Sequencer received TICK signal");

          delay(2);
#endif

          // send READ signal to WindVaneReader
          sig = READ;
          rc = xQueueSend( q_WindVaneReader, &sig, portMAX_DELAY );
#ifdef DEBUG
          pass_fail_msg("Sending READ to WindVaneReader", rc);
#endif

          // send READ signal to WindGustReader
          sig = READ;
          rc = xQueueSend( q_WindGustReader, &sig, portMAX_DELAY );
#ifdef DEBUG
          pass_fail_msg("Sending READ to WindGustReader", rc);
#endif

          // send READ signal to RainGaugeReader
          sig = READ;
          rc = xQueueSend( q_RainGaugeReader, &sig, portMAX_DELAY );
#ifdef DEBUG
          pass_fail_msg("Sending READ to RainGaugeReader", rc);
#endif


          // send READ signal to TempReader
          sig = READ;
          rc = xQueueSend( q_TempReader, &sig, portMAX_DELAY );
#ifdef DEBUG
          pass_fail_msg("Sending READ to TempReader", rc);
#endif


          // send READ signal to HumidityReader
          sig = READ;
          rc = xQueueSend( q_HumidityReader, &sig, portMAX_DELAY );
#ifdef DEBUG
          pass_fail_msg("Sending READ to HumidityReader", rc);
#endif

          // send REPORT signal to DataReporter
          sig = REPORT;
          rc = xQueueSend( q_DataReporter, &sig, portMAX_DELAY );
#ifdef DEBUG
          pass_fail_msg("Sending REPORT to DataReporter", rc);
#endif

          break;

        case ALARM_9AM:
          break;

        default:
          // error condition
          break;
      }
    }
  }
}

/*
   Callback for the report timer
*/
void reportTimerCallback(void) {
  SIGNAL sig = TICK;
  BaseType_t rc;  // return code from FreeRTOS

  rc = xQueueSend(q_Sequencer, &sig, 0);
#ifdef DEBUG
  pass_fail_msg("Timer fired, TICK queued to Sequencer", rc);
#endif

  ticks += 1;
  if (ticks >= ticks_per_day) {
    ticks = 0;
    sig = ALARM_9AM;
    xQueueSend(q_Sequencer, &sig, 0);
  }
}


/*
   Task to read the wind vane value
*/
void task_WindVaneReader(void *pvParameters) {
  (void) pvParameters;

  SIGNAL sig;

  for (;;) {
    // read the queue
#ifdef DEBUG
    Serial.println("WindVaneReader waiting for signal");
    delay(2);
#endif
    if (xQueueReceive( q_WindVaneReader, &sig, portMAX_DELAY ) == pdPASS) {
      switch (sig) {
        case READ:  // read the wind vane and record the result in Observations
#ifdef DEBUG
          Serial.println("WindVane has been read");
          delay(2);
#endif
          observations->windDirection_deg = anemometer->getWindDirection_deg();
          break;

        default:
          break; // ignore unknown signal
      }
    }
  }
}


/*
   This tasks reads the wind gust meter, records the results and resets the meter
*/
void task_WindGustReader(void *pvParameters) {
  (void) pvParameters;

  SIGNAL sig;

  for (;;) {
    // read the queue
#ifdef DEBUG
    Serial.println("WindGustReader waiting for signal");
    delay(2);
#endif
    if (xQueueReceive( q_WindGustReader, &sig, portMAX_DELAY ) == pdPASS) {
      switch (sig) {
        case READ:  // read the wind gust value and record the result in Observations
          observations->windGusts_kts = anemometer->getGustSpeed_kts();
#ifdef DEBUG
          Serial.println("WindGust has been read");
          delay(2);
#endif
          break;

        default:
          break; // ignore unknown signal
      }
    }
  }
}

/*
   This tasks reads the temperature
*/
void task_TempReader(void *pvParameters) {
  (void) pvParameters;

  SIGNAL sig;

  for (;;) {
    // read the queue
#ifdef DEBUG
    Serial.println("TempReaderReader waiting for signal");
    delay(2);
#endif
    if (xQueueReceive( q_TempReader, &sig, portMAX_DELAY ) == pdPASS) {
      switch (sig) {
        case READ:  // read the wind gust value and record the result in Observations
          //observations->windGusts_kts = anemometer->getGustSpeed_kts();
#ifdef DEBUG
          Serial.println("Temp has been read");
          delay(2);
#endif
          break;

        default:
          break; // ignore unknown signal
      }
    }
  }
}


/*
   This tasks reads the humitity
*/
void task_HumidityReader(void *pvParameters) {
  (void) pvParameters;

  SIGNAL sig;

  for (;;) {
    // read the queue
#ifdef DEBUG
    Serial.println("HumidityReaderReader waiting for signal");
    delay(2);
#endif
    if (xQueueReceive( q_HumidityReader, &sig, portMAX_DELAY ) == pdPASS) {
      switch (sig) {
        case READ:  // read the wind gust value and record the result in Observations
          //observations->windGusts_kts = anemometer->getGustSpeed_kts();
#ifdef DEBUG
          Serial.println("Humidity has been read");
          delay(2);
#endif
          break;

        default:
          break; // ignore unknown signal
      }
    }
  }
}


/*
   This tasks reads the rain gauge meter, records the results and resets the meter
   Also handles the 24 hour and last hour accumulators
*/
void task_RainGaugeReader(void *pvParameters) {
  (void) pvParameters;

  SIGNAL sig;

  for (;;) {
    // read the queue
#ifdef DEBUG
    Serial.println("RainGaugeReader waiting for signal");
    delay(2);
#endif
    if (xQueueReceive( q_RainGaugeReader, &sig, portMAX_DELAY ) == pdPASS) {
      switch (sig) {
        case READ:  // read the wind gust value and record the result in Observations
          // observations->windGusts_kts = anemometer->getGustSpeed_kts();
#ifdef DEBUG
          Serial.println("RainGauge has been read");
          delay(2);
#endif
          break;

        default:
          break; // ignore unknown signal
      }
    }
  }
}

/*
   The DataReporter tasks outputs the observations to the data recording service

*/
void task_DataReporter(void *pvParameters) {
  (void) pvParameters;

  SIGNAL sig;

  for (;;) {
    // read the queue
    Serial.println("Reading q_DataReporter");
    delay(2);
    if (xQueueReceive( q_DataReporter, &sig, portMAX_DELAY ) == pdPASS) {
      switch (sig) {
        case REPORT:  // report all observations
          Serial.println("A data report has been sent");
          delay(2);
          break;

        default:
          break; // ignore unknown signal
      }
    }
  }
}


void TaskAnalogRead(void *pvParameters) {
  (void) pvParameters;

  /*
    AnalogReadSerial
    Reads an analog input on pin 0, prints the result to the serial monitor.
    Graphical representation is available using serial plotter (Tools > Serial Plotter menu)
    Attach the center pin of a potentiometer to pin A0, and the outside pins to +5V and ground.
    This example code is in the public domain.
  */

  for (;;) {
    // read the input on analog pin 0:
    int sensorValue = analogRead(A0);
    // print out the value you read:
    //Serial.println(sensorValue);
    vTaskDelay(1);  // one tick delay (15ms) in between reads for stability
  }
}