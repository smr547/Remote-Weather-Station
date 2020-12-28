#include <math.h>

#define rainGaugePin (3) // rain guage bucket tipping sensor
#define rainInterrupt (1)

volatile unsigned long tips; // cup rotation counter used in interrupt routine
volatile unsigned long contactBounceTime; // Timer to avoid contact bounce in interrupt routine

float rainfall_mm;    // aggregate rainfall in mm

void setup() {
  Serial.begin(9600);

  pinMode(rainGaugePin, INPUT);
  Serial.print("interrupt is ");
//  Serial.println(digitalPinToInterrupt(rainGaugePin));
  Serial.println(rainInterrupt);
//  attachInterrupt(digitalPinToInterrupt(rainGaugePin), isr_bucket_tip, FALLING);
  attachInterrupt(rainInterrupt, isr_bucket_tip, FALLING);

  Serial.println("Davis Rain Guage Test");
  Serial.println("Tips\t\tmm");

  tips = 0;
}

void loop() {

  sei(); // Enables interrupts
  delay (10000); // Wait 10 seconds
  cli(); // Disable interrupts

  // convert tips to rainfall_mm using the formula R=T*0.18

  rainfall_mm = tips * 0.18;

  Serial.print(tips); Serial.print("\t\t");
  Serial.println(rainfall_mm);

}

// This is the function that the interrupt calls to increment the tip count
void isr_bucket_tip () {

  if ((millis() - contactBounceTime) > 15 ) { // debounce the switch contact.
    tips++;
    contactBounceTime = millis();
  }

}
