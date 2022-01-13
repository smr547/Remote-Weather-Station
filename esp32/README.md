Remote Weather Station
======================

An esp32 implementation of a weather station. Supported sensors are:

* rain gauge

* wind speed and direction

* humidity

* air pressure

* air temperature

# Pin configuration

* GPIO34 - digital wind speed counter
* GPIO36 (ADC0) - wind direction
* GPIO35 - rain gauge pulses (falling pulses, noisy reed switch)

## I2C bus

* I2C SDA GPIO21
* I2C SCL GPIO22

On the I2C bus we have:

* BME/BMP280 - pressure and temperature sensor
* AM2315 - temperature and humidity sensor