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
L pspice:OPAMP OpAmp1
U 1 1 61FD4970
P 4300 3100
F 0 "OpAmp1" H 4644 3146 50  0001 L CNN
F 1 "OPAMP" H 4644 3100 50  0000 L CNN
F 2 "" H 4300 3100 50  0001 C CNN
F 3 "~" H 4300 3100 50  0001 C CNN
	1    4300 3100
	1    0    0    -1  
$EndComp
Wire Wire Line
	4000 3000 3550 3000
Text Label 3550 3000 0    50   ~ 0
3V
$Comp
L pspice:R R1
U 1 1 61FD5327
P 7500 4850
F 0 "R1" H 7568 4896 50  0000 L CNN
F 1 "200" H 7568 4805 50  0000 L CNN
F 2 "" H 7500 4850 50  0001 C CNN
F 3 "~" H 7500 4850 50  0001 C CNN
	1    7500 4850
	1    0    0    -1  
$EndComp
Text GLabel 7500 4600 0    50   Input ~ 0
Vin=5V
Wire Wire Line
	7500 5100 7500 5150
Connection ~ 7500 5150
Wire Wire Line
	7500 5150 7500 5250
Wire Wire Line
	7500 5150 7850 5150
Text GLabel 7850 5150 2    50   Input ~ 0
Vout=3V
$Comp
L pspice:R R2
U 1 1 61FD5B76
P 7500 5500
F 0 "R2" H 7568 5546 50  0000 L CNN
F 1 "300" H 7568 5455 50  0000 L CNN
F 2 "" H 7500 5500 50  0001 C CNN
F 3 "~" H 7500 5500 50  0001 C CNN
	1    7500 5500
	1    0    0    -1  
$EndComp
$Comp
L pspice:0 GND
U 1 1 61FD6CC4
P 7500 5800
F 0 "GND" H 7578 5783 50  0000 L CNN
F 1 "0" H 7500 5889 50  0001 C CNN
F 2 "" H 7500 5800 50  0001 C CNN
F 3 "~" H 7500 5800 50  0001 C CNN
	1    7500 5800
	1    0    0    -1  
$EndComp
$EndSCHEMATC
