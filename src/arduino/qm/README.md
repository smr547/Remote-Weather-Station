# Weather Station using Quantum Leaps framework
 
Arduino implementation of the remote weather station using State Machines.

The target hardware for this project is the [Arduino Due](https://core-electronics.com.au/arduino-due.html)

## Build

* This project uses the [Quantum Leaps Real Time Embedded Framework (RTEF)](https://www.state-machine.com/products/qp)
* All code for the project is store in the [model file](./weather_station.qm)
* This project utilises the [QP/C++ framework](https://www.state-machine.com/qpcpp/index.html) and 
* all code is written to the [QP/C++ API](https://www.state-machine.com/qpcpp/api.html)
* C++ application code (the ``weather_station.ino`` file and related files) is generated using the [qm modelling tool](https://www.state-machine.com/products/qm/)
* Hand coded portions of the application follow the [Quantum Leaps C++ Coding Standard](https://www.state-machine.com/doc/AN_QL_Coding_Standard.pdf)
* Generated code is compiled and uploaded using the [Arduino IDE](https://www.arduino.cc/en/software)

## Development approach

As this is my first project using the QP/C++ framework and the QM modelling tool I will start by using the QM model
file for the Dining Philosopher problem. The functionality of the weather station will be added to the DPP code
until we have a fully functional Weather Station. The DPP code will then be removed.

## Step 1 -- the Anemometer

* The Anemometer report instantaneous wind velocity once per second
* Wind gusts are computed over a 3 second period
* Periodically the Anemometer reports average wind velocity and maximum wind gust velocity over the given interval
* Intervals are 1, 15, 30, 60 minutes
* All reports are written to the Serial port

The design feature an ISR and multiple active objects:

* an ISR to count anemometer rotations
* a single AO::Aneometer to report instataneous win velocity and send a Event::WindReport signal to
* a single AO::Guster which computes the average velocity over 3 seconds publishing Event::GustReport 
* instances of AO::IntervalReporter each of which subscribe to GustReports and produces a report at the prescribed interval

## Step 2 -- Rain Gauge 

To be added later

## Step 3 -- Temp, humidity and Pressure

To be added later

## Step 4 -- Voltage and system monitor

To be added

## Step 5 -- Serial command and control

To be addedd

## Step 6 -- 4G report posting

To be added
 
