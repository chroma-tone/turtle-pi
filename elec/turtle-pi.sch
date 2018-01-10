EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
EELAYER 25 0
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
$EndDescr
$Comp
L PC817 U1
U 1 1 5A55CF89
P 2250 2550
F 0 "U1" H 2050 2750 50  0000 L CNN
F 1 "ZD1901" H 2250 2750 50  0000 L CNN
F 2 "DIP-4" H 2050 2350 50  0000 L CIN
F 3 "" H 2250 2550 50  0000 L CNN
	1    2250 2550
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR1
U 1 1 5A55D08E
P 1950 2200
F 0 "#PWR1" H 1950 2050 50  0001 C CNN
F 1 "+5V" H 1950 2340 50  0000 C CNN
F 2 "" H 1950 2200 50  0000 C CNN
F 3 "" H 1950 2200 50  0000 C CNN
	1    1950 2200
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR2
U 1 1 5A55D0A6
P 2350 3200
F 0 "#PWR2" H 2350 2950 50  0001 C CNN
F 1 "GND" H 2350 3050 50  0000 C CNN
F 2 "" H 2350 3200 50  0000 C CNN
F 3 "" H 2350 3200 50  0000 C CNN
	1    2350 3200
	1    0    0    -1  
$EndComp
$Comp
L R R1
U 1 1 5A55D0CA
P 1800 2900
F 0 "R1" V 1880 2900 50  0000 C CNN
F 1 "200" V 1800 2900 50  0000 C CNN
F 2 "" V 1730 2900 50  0000 C CNN
F 3 "" H 1800 2900 50  0000 C CNN
	1    1800 2900
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X01 P3
U 1 1 5A55D1B7
P 3100 2450
F 0 "P3" H 3100 2550 50  0000 C CNN
F 1 "RPI_GPIO14 (PIN 8)" V 3200 2450 50  0000 C CNN
F 2 "" H 3100 2450 50  0000 C CNN
F 3 "" H 3100 2450 50  0000 C CNN
	1    3100 2450
	1    0    0    -1  
$EndComp
Wire Wire Line
	1950 2650 1800 2650
Wire Wire Line
	1800 2650 1800 2750
Wire Wire Line
	1800 3050 1800 3200
Wire Wire Line
	1800 3200 2550 3200
Wire Wire Line
	2550 3200 2550 2650
Connection ~ 2350 3200
Wire Wire Line
	1950 2450 1950 2200
Wire Wire Line
	2900 2450 2550 2450
$Comp
L CONN_01X01 P1
U 1 1 5A55D2CB
P 1350 2300
F 0 "P1" H 1350 2400 50  0000 C CNN
F 1 "RPI 5V (PIN 6)" V 1450 2300 50  0000 C CNN
F 2 "" H 1350 2300 50  0000 C CNN
F 3 "" H 1350 2300 50  0000 C CNN
	1    1350 2300
	-1   0    0    1   
$EndComp
$Comp
L CONN_01X01 P2
U 1 1 5A55D3AA
P 1350 3150
F 0 "P2" H 1350 3250 50  0000 C CNN
F 1 "RPI_GND (PIN 4)" V 1450 3150 50  0000 C CNN
F 2 "" H 1350 3150 50  0000 C CNN
F 3 "" H 1350 3150 50  0000 C CNN
	1    1350 3150
	-1   0    0    1   
$EndComp
Wire Wire Line
	1550 2300 1950 2300
Connection ~ 1950 2300
Wire Wire Line
	1550 3150 1800 3150
Connection ~ 1800 3150
Text Notes 1200 1850 0    60   ~ 0
Wheel Encoder Prototype connections
Text Notes 2650 3000 0    60   ~ 0
NOTE: Pinout doesn't match real device...
Wire Notes Line
	2850 2900 2850 2550
Wire Notes Line
	2850 2550 2450 2550
$EndSCHEMATC
