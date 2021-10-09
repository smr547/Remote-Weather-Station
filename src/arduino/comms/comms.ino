
#include "Adafruit_FONA.h"


#define SIMCOM_7000 // SIM7000A/C/E/G
#define FONA_PWRKEY 6
#define FONA_RST 7
#define FONA_TX 10 // Microcontroller RX
#define FONA_RX 11 // Microcontroller TX


// We default to using software serial. If you want to use hardware serial
// (because softserial isnt supported) comment out the following three lines
// and uncomment the HardwareSerial line
#include <SoftwareSerial.h>
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);

// Use this one for LTE CAT-M/NB-IoT modules (like SIM7000)
// Notice how we don't include the reset pin because it's reserved for emergencies on the LTE module!
Adafruit_FONA_LTE fona = Adafruit_FONA_LTE();

void powerOnFona();
int waitForNetwork(int timeout_secs);
int sendSMS(char * sendto, char * message );


void setup() {
  Serial.begin(9600);
  Serial.println("Initialising fona");
  initialiseFona();
  Serial.println("Initialise complete");
}

void initialiseFona() {
  pinMode(FONA_RST, OUTPUT);
  digitalWrite(FONA_RST, HIGH); // Default state
  pinMode(FONA_PWRKEY, OUTPUT);

  // Turn on the module by pulsing PWRKEY low for a little bit
  // This amount of time depends on the specific module that's used
  powerOnFona();
  fonaSS.begin(115200); // Default SIM7000 shield baud ratej
  fonaSS.println("AT+IPR=9600"); // Set baud rate
  delay(100); // Short pause to let the command run
  fonaSS.begin(9600);
  if (! fona.begin(fonaSS)) {
    Serial.println(F("Couldn't find FONA"));
    while (1); // Don't proceed if it couldn't find the device
  }

  // Set modem to full functionality
  fona.setFunctionality(1); // AT+CFUN=1

  fona.setNetworkSettings(F("telstra.internet")); // For Telstra (Australia) SIM card - CAT-M1 (Band 28)
  fona.setPreferredMode(38); // Use LTE only, not 2G
  fona.setPreferredLTEMode(1); // Use LTE CAT-M only, not NB-IoT
  fona.setOperatingBand("CAT-M", 12); // AT&T uses band 12
  fona.enableRTC(true);

  fona.enableSleepMode(true);
  fona.set_eDRX(1, 4, "0010");
  fona.enablePSM(true);

  // Set the network status LED blinking pattern while connected to a network (see AT+SLEDS command)
  fona.setNetLED(true, 2, 64, 3000); // on/off, mode, timer_on, timer_off
  fona.setNetLED(false); // Disable network status LED
}


void loop() {
  uint16_t bat_voltage;
  uint16_t http_status;
  uint16_t data_length;
  char buffer[128];

  Serial.print(F("FONA> "));
  if (!enableGPRS()) {
    Serial.print("failed to enable GPRS");
  }

  if (fona.getBattVoltage(&bat_voltage)) {
    Serial.print("Battery voltage=");
    Serial.println(bat_voltage);
  } else {
    Serial.println("Could not read voltage");
  }


  if (waitForNetwork(10)) {
    /* 
    if (sendSMS("0417495268", "Hello from Steven's Arduino")) {
      Serial.print("SMS sent");
    } else {
      Serial.print("SMS NOT sent");
    }
*/
    if (fona.HTTP_init()) {
      sprintf(buffer, "api.thingspeak.com/update?api_key=0A9ZMA64KACDBZZ6&field1=%d", bat_voltage);
      if (fona.HTTP_GET_start(buffer, &http_status, &data_length )) {
        Serial.print("HTTP data sent");
      } else {
        Serial.print("HTTP data not sent");
      }
      fona.HTTP_GET_end();
      fona.HTTP_term();
    }
  }
  while (1 ) {}
}

/*
   Wait for "Registered on network" (status == 1) or timeout

   Returns:
     1 = registered on network
     0 = failed
*/
int waitForNetwork(int timeout_secs) {
  int time_secs = 0;
  int rc;

  while (1) {
    rc = fona.getNetworkStatus();
    if (rc == 1 | rc == 5)
      break;
    if (rc == 3 | rc == 4)
      return 0;

    delay(1000);
    time_secs++;
    if (time_secs < timeout_secs)
      return 0;
  }
  return 1;  // success
}

int enableGPRS(void) {
  return fona.enableGPRS(true);
}

int disableGPRS(void) {
  return fona.enableGPRS(false);
}



// Power on the module
void powerOnFona() {
  digitalWrite(FONA_PWRKEY, LOW);
  delay(100);
  digitalWrite(FONA_PWRKEY, HIGH);
}

void powerDown() {
  fona.powerDown();
}


/*
   POST supplied postData to URL and collect response
   data is supplied responseBuffer (up to the maximum lenth of the buffer
*/
/*
int postData(char * url, char * postData, char * responseBuffer, int maxResponseLength) {

  uint16_t statuscode;
  int16_t length;

  if (!fona.HTTP_POST_start(url, F("text/plain"), (uint8_t *) postData, strlen(postData), &statuscode, (uint16_t *)&length)) {
    return -1; // failed
  }

  // collect response
  int i = 0;
  while (i < length) {
    while (fona.available()) {
      char c = fona.read();
      responseBuffer[i] = c;
      i++;
    }
  }
  responseBuffer[i] = (char) 0;  // terminate the response string
  fona.HTTP_POST_end();
  return i;
}

*/

int sendSMS(char * sendto, char * message ) {
  return fona.sendSMS(sendto, message);
}
