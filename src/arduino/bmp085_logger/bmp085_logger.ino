#include <Wire.h>
#include <Adafruit_BMP085.h>
Adafruit_BMP085 bmp;  // how does the code deteriming the pins for I2C SCL and SDA??
// Note: The BMP085 library uses the Wire library which specifies I2C on arduino uses: SCL = A5 and SDA = A4
// see https://www.arduino.cc/en/reference/wire

void setup()
{
  Serial.begin(115200);
  if (!bmp.begin())
  {
    Serial.println("Could not find a BMP085 sensor!");
    while (1) {
    }
  }
}
void loop()
{
  uint32_t pressure;

  pressure = bmp.readPressure();
  Serial.print(bmp.readTemperature());
  Serial.print(",");
  Serial.print(pressure /100.0);
  Serial.print("\n");
  delay(1000);
}
