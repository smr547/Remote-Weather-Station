#include <Adafruit_AM2315.h>
#include <Arduino.h>
#include <sensesp_app.h>
#include <sensors/analog_input.h>
#include <sensors/bmp280.h>
#include <sensors/digital_input.h>
#include <signalk/signalk_output.h>
#include <transforms/frequency.h>
#include <transforms/linear.h>
#include <transforms/typecast.h>

namespace {
class AM2315Value;

// Provides a small wrapper around Adafruit_AM2315 sensor support.
class AM2315 {
 public:
  AM2315(TwoWire *wire = &Wire) { sensor_ = new Adafruit_AM2315(wire); }

  void enable() {
    app.onRepeat(kReadIntervalMs, [this]() {
      float t, h;
      valid_ = sensor_->readTemperatureAndHumidity(&t, &h);

      temp_ = t + 273.15f;     // convert C to K
      humidity_ = h / 100.0f;  // convert percentage to ratio

      if (!valid_) {
        debugD("Failed to read temp and humidity from AM2315");
      }
    });
  }

  float temperature() { return temp_; }

  float humidity() { return humidity_; }

 private:
  static const uint kReadIntervalMs = 3000;

  Adafruit_AM2315 *sensor_;
  float temp_ = 0.0f;
  float humidity_ = 0.0f;
  bool valid_ = false;

  friend class AM2315Value;
};

// Represents a numeric sensor of either temperature (degrees K) or pressure (P)
// provided by an AM2315. Due to its design, the physical sensor will never be
// interrogated more than every 2.5 seconds or so, no matter what the user
// provides for read_delay_ms.
class AM2315Value : public NumericSensor {
 public:
  enum SensorType { temperature, humidity };

  AM2315Value(AM2315 *sensor, SensorType type, uint read_delay_ms,
              String config_path = "")
      : NumericSensor(std::move(config_path)),
        sensor_{sensor},
        type_{type},
        read_delay_ms_{read_delay_ms} {}

  void enable() override {
    app.onRepeat(read_delay_ms_, [this]() {
      if (sensor_->valid_) {
        output = type_ == SensorType::temperature ? sensor_->temperature()
                                                  : sensor_->humidity();
        notify();
      }
    });
  }

 private:
  AM2315 *sensor_;
  SensorType type_;
  uint read_delay_ms_;
};
}  // namespace

ReactESP app([]() {
#ifndef SERIAL_DEBUG_DISABLED
  SetupSerialDebug(115200);
#endif

  sensesp_app = new SensESPApp();

  {
    static const uint kReadDelayMillis = 5000;

    // Create a BMP280 for pressure.
    auto *bmp280 = new BMP280(0x76);

    auto *pressure =
        new BMP280Value(bmp280, BMP280Value::pressure, kReadDelayMillis);
    pressure->connect_to(new Linear(1.0, 0.0, "/sensors/pressure/calibrate"))
        ->connect_to(new SKOutputNumber("environment.outside.pressure"));
  }

  {
    static const uint kReadDelayMillis = 5000;

    // AM2315 for temperature.
    auto *am2315 = new AM2315();
    am2315->enable();

    auto *temp =
        new AM2315Value(am2315, AM2315Value::temperature, kReadDelayMillis);
    temp->connect_to(new Linear(1.0, 0.0, "/sensors/temperature/calibrate"))
        ->connect_to(new SKOutputNumber("environment.outside.temperature"));

    auto *humid =
        new AM2315Value(am2315, AM2315Value::humidity, kReadDelayMillis);
    humid->connect_to(new Linear(1.0, 0.0, "/sensors/humidity/calibrate"))
        ->connect_to(new SKOutputNumber("environment.outside.humidity"));
  }

  // Wind direction comes to us via ADC. A count of zero implies due north,
  // full count is 359 degrees. Signal K expects this in radians, so we scale it
  // to 0-2*PI. We report it as apparent wind as seen by a boat heading north,
  // with negative values being wind from port.
  {
    uint8_t pin = 36 /* ADC 0 */;
    uint read_interval_ms = 3 * 1000 /* read every 3s */;

    auto *sensor = new AnalogInput(pin, read_interval_ms, "", 2 * PI);
    sensor
        ->connect_to(new LambdaTransform<float, float>(
            [](float inRadians) {
              // Convert from true wind direction to apparent wind direction
              // with the boat heading true north.
              return inRadians < PI ? inRadians : (inRadians - 2 * PI);
            },
            "" /* no config */))
        ->connect_to(new SKOutputNumber("environment.wind.angleApparent"));
  }

  // Wind speed. 1Hz is 1.026m/s.
  {
    uint8_t pin = 27;
    uint read_interval_ms = 3 * 1000 /* read every 3s */;
    uint ignore_interval_ms = 5 /* 200 counts/s, or 205m/s of wind */;

    auto *sensor = new DigitalInputDebounceCounter(
        pin, INPUT_PULLUP, FALLING, read_interval_ms, ignore_interval_ms);
    sensor->connect_to(new Frequency(1.026, "/Outside/Windspeed/calibrate"))
        ->connect_to(new SKOutputNumber("environment.wind.speedApparent"));
  }

  // Rain sensor. Report count every 5 minutes.
  {
    const uint8_t pin = 35;
    const uint read_interval_ms = 5 * 60 * 1000 /* 5 minutes */;
    const unsigned int ignore_interval_ms = 200;  // switch is kinda noisy
    const float multiplier = 0.18;                // mm per count

    // There's no path in the Signal K spec for rain, so let's make one.
    auto *rain_meta = new SKMetadata();
    rain_meta->units_ = "mm";
    rain_meta->description_ = rain_meta->display_name_ =
        rain_meta->short_name_ = "Rainfall";

    auto *sensor = new DigitalInputDebounceCounter(
        pin, INPUT_PULLUP, FALLING, read_interval_ms, ignore_interval_ms);
    sensor->connect_to(new Typecast<int, float>())
        ->connect_to(new Linear(multiplier, 0.0, "/Outside/Rain/calibrate"))
        ->connect_to(
            new SKOutputNumber("environment.rain.volume5min", rain_meta));
  }

  sensesp_app->enable();
});