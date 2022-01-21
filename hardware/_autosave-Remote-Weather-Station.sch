EESchema Schematic File Version 5
EELAYER 36 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
Comment5 ""
Comment6 ""
Comment7 ""
Comment8 ""
Comment9 ""
$EndDescr
Connection ~ 6450 1750
Connection ~ 7500 1750
Wire Wire Line
	6150 1750 6450 1750
Wire Wire Line
	6450 1750 6450 1850
Wire Wire Line
	6450 1750 6750 1750
Wire Wire Line
	7350 1750 7500 1750
Wire Wire Line
	7500 1750 7500 1800
Wire Wire Line
	7500 1750 7600 1750
Wire Wire Line
	8000 2000 8000 2100
Text Notes 600  7675 0    118  ~ 0
Licensed under CERN OHL v.1.2
Text GLabel 6150 1750 0    50   Input ~ 0
3.3V
Text GLabel 7600 1750 2    50   Input ~ 0
5V
Text GLabel 8000 2000 2    50   Input ~ 0
GND
Text GLabel 8700 1800 0    50   Input ~ 0
GND
Text GLabel 8700 1900 0    50   Input ~ 0
5V
$Comp
L power:GND #PWR?
U 1 1 00000000
P 6450 2050
F 0 "#PWR?" H 6450 1800 50  0001 C CNN
F 1 "GND" H 6450 1850 50  0000 C CNN
F 2 "" H 6450 2050 50  0001 C CNN
F 3 "" H 6450 2050 50  0001 C CNN
	1    6450 2050
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 00000000
P 7050 2050
F 0 "#PWR?" H 7050 1800 50  0001 C CNN
F 1 "GND" H 7050 1850 50  0000 C CNN
F 2 "" H 7050 2050 50  0001 C CNN
F 3 "" H 7050 2050 50  0001 C CNN
	1    7050 2050
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 00000000
P 7500 2100
F 0 "#PWR?" H 7500 1850 50  0001 C CNN
F 1 "GND" H 7500 1900 50  0000 C CNN
F 2 "" H 7500 2100 50  0001 C CNN
F 3 "" H 7500 2100 50  0001 C CNN
	1    7500 2100
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 00000000
P 8000 2100
F 0 "#PWR?" H 8000 1850 50  0001 C CNN
F 1 "GND" H 8000 1900 50  0000 C CNN
F 2 "" H 8000 2100 50  0001 C CNN
F 3 "" H 8000 2100 50  0001 C CNN
	1    8000 2100
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Polarized_Small C?
U 1 1 00000000
P 6450 1950
F 0 "C?" H 6550 2022 50  0000 L CNN
F 1 "10uF" H 6550 1922 50  0000 L CNN
F 2 "" H 6450 1950 50  0001 C CNN
F 3 "~" H 6450 1950 50  0001 C CNN
	1    6450 1950
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x02_Female J?
U 1 1 00000000
P 8900 1800
F 0 "J?" H 8950 1800 50  0000 L CNN
F 1 "5V Power" H 8950 1700 50  0000 L CNN
F 2 "" H 8900 1800 50  0001 C CNN
F 3 "~" H 8900 1800 50  0001 C CNN
	1    8900 1800
	1    0    0    -1  
$EndComp
$Comp
L Device:C C?
U 1 1 00000000
P 7500 1950
F 0 "C?" H 7650 2000 50  0000 L CNN
F 1 "C" H 7650 1900 50  0000 L CNN
F 2 "" H 7538 1800 50  0001 C CNN
F 3 "~" H 7500 1950 50  0001 C CNN
	1    7500 1950
	1    0    0    -1  
$EndComp
$Comp
L Regulator_Linear:LM2936-3.3 U?
U 1 1 00000000
P 7050 1750
F 0 "U?" H 7050 2050 50  0000 C CNN
F 1 "LM2936-3.3" H 7050 1950 50  0000 C CNN
F 2 "" H 7050 1975 50  0001 C CIN
F 3 "http://www.ti.com/lit/ds/symlink/lm2936.pdf" H 7050 1700 50  0001 C CNN
	1    7050 1750
	-1   0    0    -1  
$EndComp
$EndSCHEMATC
