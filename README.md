# Remote Weather Station
Self powered IoTs weather station for off-grid remote locations

## Introduction

This project aims to construct a remote weather station suitable for deployment on an outback block with little or no power or communications infrastructure on site. Key features include

* simplest possible hardware design
* "off the shelf" components
* powered by solar panels
* communicating via celluar network
* logging data to a web-based data repostiory
* web-based data viewer
* feature rich web-based display console

## Approach to development

The author is learning about micro electronic and so an incremental, multi-stage approach is being used.

1. basic design
1. benchtop prototype developed to 
1. incrementally add feature
1. package for deployment
1. deploy to trial site
1. develop web-based console while
1. learning lessions over extended trial period
1. finalise design
1. document fully (open source) - hardware and software
1. build final system (including spares) and deploy

Other considerations: 

* Sensor hardware will be sourced from the market
* Arduino-based microcontroller (and shields) for control and data processing
* Move to [STM32](https://www.st.com/en/microcontrollers-microprocessors/stm32-32-bit-arm-cortex-mcus.html) hardware if necessary

## System overview

<include system level schematic here>
  
## Prototype 1

The first prototype was built on the workbench in Reid, ACT, Australia. It comprised:

* [Anemometer from Davis instruments](https://www.davisinstruments.com/product/anemometer-for-vantage-pro2-vantage-pro/)
* [Rain gauge from Davis Instruments](https://www.davisinstruments.com/product/aerocone-rain-collector-with-vantage-pro2-mounting-base/)
* [BMP085 pressure sensor](https://www.sparkfun.com/products/retired/9694)
* [SHT15 temperature and humidity sensor](https://www.sparkfun.com/products/retired/13683)
* [Arduino Uno R3](https://store.arduino.cc/usa/arduino-uno-rev3) microcontoller board
* [Arduino Prototyping shield](https://www.jaycar.com.au/protoshield-basic-for-arduino/p/XC4214)

The arduino controller was conneced to a Raspberry Pi system that provided

* power and 
* serial communications support via USB
* data logging to [ThingsSpeak account](https://thingspeak.com/channels/1007595/private_show) via
* Python scripting

The arduino code issued a NMEA sentenced every 10 minutes on the USB serial port. The sentence encoded the following data:

1. current barimetric pressure (hPa)
1. current temperature (deg C)
1. current relative humidity (%)
1. current dew point (deg C)
1. average wind speed past 10 minutes (knots) 
1. max wind speed past 10 minutes (knots -- sampled every 15sec)
1. avg wind direction past 10 minutes (cardinal points)
1. current wind direction (cardinal points)
1. rain accumulation past 10 minutes (0.2mm precision)

## Intersting links
* [Wind reporting in the NAS](https://ral.ucar.edu/general/Summer2012/FPAW_2012_Summer_Presentations/Seg%204%20Surface%20Observations%20Riger%20Sultan/Armbruster%20Wind%20Reporting.pdf)
* [LTE shield instructable](https://www.instructables.com/LTE-NB-IoT-Shield-for-Arduino/)
* [LTE shield website](https://github.com/botletics/SIM7000-LTE-Shield)
* [Artic Climate Evaluator](http://www2.ensc.sfu.ca/~whitmore/courses/ensc305/projects/2011/2desi.pdf) -- an example of a great specification
* [Silicon Chip article on remote logging](https://www.siliconchip.com.au/Issue/2020/February/Remote+monitoring+station+messages+or+emails+by+4G%21)
* [ALDI mobile network information](https://apn-australia.gishan.net/en/apn/aldi)
* [Arduino C programming reference](https://www.arduino.cc/reference/en/)
* [SHT1x Arduino Library](https://github.com/practicalarduino/SHT1x)

