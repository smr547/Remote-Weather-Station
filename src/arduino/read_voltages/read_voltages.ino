
float batt_V;
float vin_V;
float batt_mA;
float mv_per_count = 3.0 * 3.10/635.0;
float mA_per_count = 500.0/1024.0;

void setup() {
  Serial.begin(115200);

}

void loop() {
  for (;;) {
    // read the input on analog pin 0:
    int a0Value = analogRead(A0);
    batt_V = a0Value * mv_per_count;
    
    
    int a1Value = analogRead(A1);
    vin_V = a1Value * mv_per_count;
    int a2Value = analogRead(A2);
    batt_mA = a2Value * mA_per_count;
    
    // print out the value you read:
    Serial.print("A0 = ");
    Serial.print(a0Value);
    Serial.print(", A1 = ");
    Serial.print(a1Value);
    Serial.print(", A2 = ");
    Serial.print(a2Value);
    Serial.println("");
    Serial.print("Battery = ");
    Serial.print(batt_V);
    Serial.print(" volts, Arduino VIN = ");
    Serial.print(vin_V);
    Serial.print(" volts, Battery current = ");
    Serial.print(batt_mA);
    Serial.println(" mA");
    delay(1000);
  }

}
