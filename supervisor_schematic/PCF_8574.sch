EESchema Schematic File Version 4
EELAYER 30 0
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
L power:GNDD #PWR?
U 1 1 621C883B
P 5050 5050
F 0 "#PWR?" H 5050 4800 50  0001 C CNN
F 1 "GNDD" H 5054 4895 50  0000 C CNN
F 2 "" H 5050 5050 50  0001 C CNN
F 3 "" H 5050 5050 50  0001 C CNN
	1    5050 5050
	1    0    0    -1  
$EndComp
$Comp
L power:GNDD #PWR?
U 1 1 621C8EF7
P 4200 4450
F 0 "#PWR?" H 4200 4200 50  0001 C CNN
F 1 "GNDD" H 4204 4295 50  0000 C CNN
F 2 "" H 4200 4450 50  0001 C CNN
F 3 "" H 4200 4450 50  0001 C CNN
	1    4200 4450
	1    0    0    -1  
$EndComp
$Comp
L power:GNDD #PWR?
U 1 1 621C917F
P 5850 2900
F 0 "#PWR?" H 5850 2650 50  0001 C CNN
F 1 "GNDD" H 5854 2745 50  0000 C CNN
F 2 "" H 5850 2900 50  0001 C CNN
F 3 "" H 5850 2900 50  0001 C CNN
	1    5850 2900
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR?
U 1 1 621CB710
P 5050 3650
F 0 "#PWR?" H 5050 3500 50  0001 C CNN
F 1 "+5V" H 5065 3823 50  0000 C CNN
F 2 "" H 5050 3650 50  0001 C CNN
F 3 "" H 5050 3650 50  0001 C CNN
	1    5050 3650
	1    0    0    -1  
$EndComp
$Comp
L Transistor_BJT:BC548 Q?
U 1 1 621CC361
P 5750 2700
F 0 "Q?" H 5941 2746 50  0000 L CNN
F 1 "BC548" H 5941 2655 50  0000 L CNN
F 2 "Package_TO_SOT_THT:TO-92_Inline" H 5950 2625 50  0001 L CIN
F 3 "http://www.fairchildsemi.com/ds/BC/BC547.pdf" H 5750 2700 50  0001 L CNN
	1    5750 2700
	1    0    0    -1  
$EndComp
$Comp
L Interface_Expansion:PCF8574 U?
U 1 1 621CDD0E
P 5050 4350
F 0 "U?" H 5050 5231 50  0000 C CNN
F 1 "PCF8574" H 5050 5140 50  0000 C CNN
F 2 "" H 5050 4350 50  0001 C CNN
F 3 "http://www.nxp.com/documents/data_sheet/PCF8574_PCF8574A.pdf" H 5050 4350 50  0001 C CNN
	1    5050 4350
	1    0    0    -1  
$EndComp
Wire Wire Line
	4550 4450 4200 4450
Wire Wire Line
	4550 4350 4200 4350
Wire Wire Line
	4200 4350 4200 4450
Connection ~ 4200 4450
Wire Wire Line
	4550 4250 4200 4250
Wire Wire Line
	4200 4250 4200 4350
Connection ~ 4200 4350
Text GLabel 4550 3950 0    50   Input ~ 0
esp32_pin22
Text GLabel 4550 4050 0    50   Input ~ 0
esp32_pin23
$Comp
L power:+5V #PWR?
U 1 1 621CB7BB
P 6750 2650
F 0 "#PWR?" H 6750 2500 50  0001 C CNN
F 1 "+5V" H 6765 2823 50  0000 C CNN
F 2 "" H 6750 2650 50  0001 C CNN
F 3 "" H 6750 2650 50  0001 C CNN
	1    6750 2650
	0    1    1    0   
$EndComp
$Comp
L power:GNDD #PWR?
U 1 1 621C910A
P 6450 2650
F 0 "#PWR?" H 6450 2400 50  0001 C CNN
F 1 "GNDD" H 6454 2495 50  0000 C CNN
F 2 "" H 6450 2650 50  0001 C CNN
F 3 "" H 6450 2650 50  0001 C CNN
	1    6450 2650
	0    1    1    0   
$EndComp
$Comp
L Device:R_POT_US RV?
U 1 1 621CD528
P 6600 2650
F 0 "RV?" H 6533 2696 50  0000 R CNN
F 1 "R_POT_US" H 6533 2605 50  0000 R CNN
F 2 "" H 6600 2650 50  0001 C CNN
F 3 "~" H 6600 2650 50  0001 C CNN
	1    6600 2650
	0    -1   -1   0   
$EndComp
$Comp
L power:+5V #PWR?
U 1 1 621CB6A2
P 7450 2600
F 0 "#PWR?" H 7450 2450 50  0001 C CNN
F 1 "+5V" H 7465 2773 50  0000 C CNN
F 2 "" H 7450 2600 50  0001 C CNN
F 3 "" H 7450 2600 50  0001 C CNN
	1    7450 2600
	0    -1   -1   0   
$EndComp
$Comp
L power:+5V #PWR?
U 1 1 621CB015
P 8550 2200
F 0 "#PWR?" H 8550 2050 50  0001 C CNN
F 1 "+5V" H 8565 2373 50  0000 C CNN
F 2 "" H 8550 2200 50  0001 C CNN
F 3 "" H 8550 2200 50  0001 C CNN
	1    8550 2200
	1    0    0    -1  
$EndComp
$Comp
L power:GNDD #PWR?
U 1 1 621C905E
P 9050 2600
F 0 "#PWR?" H 9050 2350 50  0001 C CNN
F 1 "GNDD" H 9054 2445 50  0000 C CNN
F 2 "" H 9050 2600 50  0001 C CNN
F 3 "" H 9050 2600 50  0001 C CNN
	1    9050 2600
	0    -1   -1   0   
$EndComp
Wire Wire Line
	7450 2500 6600 2500
Wire Wire Line
	8450 2200 5850 2200
Wire Wire Line
	5850 2200 5850 2500
Wire Wire Line
	5550 4650 5850 4650
Wire Wire Line
	5850 4650 5850 3350
Wire Wire Line
	5850 3350 5500 3350
Wire Wire Line
	5500 3350 5500 2700
Wire Wire Line
	5500 2700 5550 2700
Wire Wire Line
	8550 3950 5550 3950
Wire Wire Line
	8650 4050 5550 4050
Wire Wire Line
	8750 4150 5550 4150
Wire Wire Line
	8850 4250 5550 4250
Wire Wire Line
	8850 3000 8850 4250
Wire Wire Line
	8750 3000 8750 4150
Wire Wire Line
	8650 3000 8650 4050
Wire Wire Line
	8550 3000 8550 3950
$Comp
L Display_Character:NHD-0420H1Z U?
U 1 1 621C1948
P 8250 2600
F 0 "U?" H 8250 1711 50  0000 C CNN
F 1 "NHD-0420H1Z" H 8250 1620 50  0000 C CNN
F 2 "Display:NHD-0420H1Z" H 8250 1700 50  0001 C CNN
F 3 "http://www.newhavendisplay.com/specs/NHD-0420H1Z-FSW-GBW-33V3.pdf" H 8350 2500 50  0001 C CNN
	1    8250 2600
	0    -1   -1   0   
$EndComp
Wire Wire Line
	7850 3000 7850 4350
Wire Wire Line
	7850 4350 5550 4350
Wire Wire Line
	7750 3000 7750 4450
Wire Wire Line
	7750 4450 5550 4450
Wire Wire Line
	5550 4550 7650 4550
Wire Wire Line
	7650 4550 7650 3000
$EndSCHEMATC
