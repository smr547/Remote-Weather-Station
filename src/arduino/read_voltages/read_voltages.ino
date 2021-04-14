/*
 * Measure voltages and current from power suppy
 * 
 * ADC inputs:
 * ==========
 * 
 * A0 -- VIN pin on Arduino board via Ra0/Rb0 resistance divider
 * A1 -- Battery voltage via Ra1/Rb1 resistance divider and op-amp wired as unity gain voltage follower
 * A2 -- Voltage over Vs shunt resister via high gain inverting op amp 
 */

// define in-circuit values as constants
// A0:
#define Ra0 26300 // ohms
#define Rb0 8100.0  // trimpot TP1 ohms
#define G0 1.0      // no op amp

// A1:
#define Ra1 24170.0 // ohms
#define Rb1 7137  // trimpot TP1 ohms
#define G1 1.0      // op amp gain -- wired as unity buffer

// A2:
#define Ra2 66900.0 // ohms
#define Rb2 67.6    // trimpot TP2 ohms
#define Rs 0.01     // shunt resistance in ohms


#define VREF 5.0    // refernce voltage for ADC
#define ADC_MAX 1023 

float batt_V;
float vin_V;
float batt_mA;
float G2;
float mv_per_count_A0;
float mv_per_count_A1;
float mVs_per_count_A2;
float mA_per_count_A2;
float batt_power_mW;

void setup() {
  Serial.begin(115200);
  mv_per_count_A0 = 1000.0 * (VREF / (ADC_MAX+1.0)) * ((Ra0 + Rb0)/Rb0) / G0;
  mv_per_count_A1 = 1000.0 * (VREF / (ADC_MAX+1.0)) * ((Ra1 + Rb1)/Rb1) / G1;

 /*
 * current in shunt (batt_mA)
 * 
 * Vs = Is * Rs
 * Is = Vs / Rs
 * 
 * we measure Vs unsing ADC input A2
 * 
 */


  G2 = 1.0 + (Ra2 / Rb2);
  G2 = 1554.0;

  mVs_per_count_A2 = ((1000.0 * VREF) / (ADC_MAX+1.0)) / G2;
  mA_per_count_A2 = mVs_per_count_A2 / Rs;

  Serial.print("mv per count A0 = ");
  Serial.println(mv_per_count_A0);
  Serial.print("mv per count A1 = ");
  Serial.println(mv_per_count_A1);
  
  Serial.print("G2 = ");
  Serial.println(G2);
  Serial.print("mVs per count A2 = ");
  Serial.println(mVs_per_count_A2);
  Serial.print("mA per count A2 = ");
  Serial.println(mA_per_count_A2);

}

void loop() {
  for (;;) {
    // read the input on analog pin 0:
    int a0Value = analogRead(A0);
    vin_V = a0Value * mv_per_count_A0 / 1000.0;
    
    
    int a1Value = analogRead(A1);
    batt_V = a1Value * mv_per_count_A1 / 1000.0;
    
    int a2Value = analogRead(A2);
    batt_mA = a2Value * mA_per_count_A2;

    batt_power_mW = batt_mA * batt_V;
    
    // print out the value you read:
    Serial.print("A0 = ");
    Serial.print(a0Value);
    Serial.print(", A1 = ");
    Serial.print(a1Value);
    Serial.print(", A2 = ");
    Serial.print(a2Value);
    Serial.println("");
    Serial.print("Arduino VIN = ");
    Serial.print(vin_V);
    Serial.print(" volts, Battery = ");
    Serial.print(batt_V);    
    Serial.print(" volts, Battery current = ");
    Serial.print(batt_mA);      
    Serial.print(" mA, Battery power = ");
    Serial.print(batt_power_mW);
    Serial.println(" mW");
    delay(1000);
  }

}
