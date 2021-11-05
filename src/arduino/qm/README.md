# Weather Station using Quantum Leaps framework
 
Arduino implementation of the remote weather station  using State Machines.

The target hardware for this project is the [Arduino Mega 2560](https://store-usa.arduino.cc/products/arduino-mega-2560-rev3?selectedStore=us)

## Build

* This project uses the [Quantum Leaps Real Time Embedded Framework (RTEF)](https://www.state-machine.com/products/qp)
* All code for the project is store in the [model file](./weather_station.qm)
* This project utilises the [QEP Nano Framework](https://www.state-machine.com/qpn/index.html) and all code is written to the [QEP Nano API](https://www.state-machine.com/qpn/index.html)
* C application code (the ``weather_station.ino`` file) is generated using the [qm modelling tool](https://www.state-machine.com/products/qm/)
* Generated code is compiled and uploaded using the [Arduino IDE](https://www.arduino.cc/en/software)
