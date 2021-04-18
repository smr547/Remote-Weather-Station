#include <math.h>

#define windSpeedPin (2) // D2 pin is connected to the wind speed reed switch (one pulse per revolution)
// #define windInterrupt (0)

volatile unsigned long rotations; // cup rotation counter used in interrupt routine
volatile unsigned long contactBounceTime; // Timer to avoid contact bounce in interrupt routine

float windSpeed_kts;    // wind speed in knots

void setup() {
  Serial.begin(9600);

  pinMode(windSpeedPin, INPUT);
  Serial.print("interrupt is ");
  Serial.println(digitalPinToInterrupt(windSpeedPin));
 // Serial.println(windInterrupt);
  //  attachInterrupt(digitalPinToInterrupt(rainGaugePin), isr_bucket_tip, FALLING);
  attachInterrupt(digitalPinToInterrupt(windSpeedPin), isr_rotation, FALLING);

  Serial.println("Davis Wind Speed Test");
  Serial.println("Rotations\t\tknots");

}

void loop() {
  rotations = 0;
  sei(); // Enables interrupts
  delay (3000); // Wait 10 seconds
  cli(); // Disable interrupts

  // convert to mp/h using the formula V=P(2.25/T)
  // V_mph = P(2.25/3) = P * 0.75
  // V_kts = P(2.25/3) = P * 0.75 * 0.87 = P * 0.6525
  // 

  windSpeed_kts = rotations * 0.6525;
  
  Serial.print(rotations); 
  Serial.print("\t\t");
  Serial.println(windSpeed_kts);

}

// This is the function that the interrupt calls on each rotation of the anonometer
void isr_rotation () {

  if ((millis() - contactBounceTime) > 15 ) { // debounce the switch contact.
    rotations++;
    contactBounceTime = millis();
  }

}
