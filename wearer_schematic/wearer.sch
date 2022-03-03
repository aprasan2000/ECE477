EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "Project \"Noggin\" - Hard Hat PCB"
Date "2022-02-27"
Rev ""
Comp "Purdue University ECE477 Team19"
Comment1 "Akshaj Prasannakumar"
Comment2 "Ryan Domanski"
Comment3 "Mooyeon Kim"
Comment4 "Dev Shah"
$EndDescr
$Comp
L Noggin:ESP32-WROOM-32D U3
U 1 1 620D2920
P 6000 2450
F 0 "U3" H 6700 2715 50  0000 C CNN
F 1 "ESP32-WROOM-32D" H 6700 2624 50  0000 C CNN
F 2 "noggin:ESP32WROOM32D" H 7250 2550 50  0001 L CNN
F 3 "https://www.mouser.com/datasheet/2/891/esp32-wroom-32d_esp32-wroom-32u_datasheet_en-1365844.pdf" H 7250 2450 50  0001 L CNN
F 4 "WiFi Modules (802.11) SMD Module, ESP32-D0WD, 32Mbits SPI flash, UART mode, PCB antenna" H 7250 2350 50  0001 L CNN "Description"
F 5 "3.1" H 7250 2250 50  0001 L CNN "Height"
F 6 "Espressif Systems" H 7250 2150 50  0001 L CNN "Manufacturer_Name"
F 7 "ESP32-WROOM-32D" H 7250 2050 50  0001 L CNN "Manufacturer_Part_Number"
F 8 "N/A" H 7250 1950 50  0001 L CNN "Mouser Part Number"
F 9 "https://www.mouser.co.uk/ProductDetail/Espressif-Systems/ESP32-WROOM-32D?qs=MLItCLRbWszx2KabkKPu5A%3D%3D" H 7250 1850 50  0001 L CNN "Mouser Price/Stock"
	1    6000 2450
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR024
U 1 1 621E738B
P 5150 2350
F 0 "#PWR024" H 5150 2100 50  0001 C CNN
F 1 "GND" H 5155 2177 50  0000 C CNN
F 2 "" H 5150 2350 50  0001 C CNN
F 3 "" H 5150 2350 50  0001 C CNN
	1    5150 2350
	-1   0    0    1   
$EndComp
$Comp
L Noggin:MQ-9B U1
U 1 1 621F40D6
P 1750 5800
F 0 "U1" H 1750 6481 50  0000 C CNN
F 1 "MQ-9B" H 1750 6390 50  0000 C CNN
F 2 "noggin:MQ-9B" H 1750 5700 50  0001 C CNN
F 3 "" H 1750 5700 50  0001 C CNN
	1    1750 5800
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR06
U 1 1 621F7E8D
P 1550 5250
F 0 "#PWR06" H 1550 5100 50  0001 C CNN
F 1 "+5V" V 1565 5378 50  0000 L CNN
F 2 "" H 1550 5250 50  0001 C CNN
F 3 "" H 1550 5250 50  0001 C CNN
	1    1550 5250
	0    -1   -1   0   
$EndComp
Wire Wire Line
	1750 5250 1750 5300
Text Notes 1850 6250 1    50   ~ 0
29Ω±3Ω
Wire Wire Line
	1550 5250 1750 5250
$Comp
L Noggin:AO3400A Q1
U 1 1 6220A7E3
P 1450 6950
F 0 "Q1" H 1880 7096 50  0000 L CNN
F 1 "AO3400A" H 1880 7005 50  0000 L CNN
F 2 "noggin:SOT95P280X125-3N" H 1900 6900 50  0001 L CNN
F 3 "http://aosmd.com/res/data_sheets/AO3400A.pdf" H 1900 6800 50  0001 L CNN
F 4 "30V N-Channel MOSFET" H 1900 6700 50  0001 L CNN "Description"
F 5 "1.25" H 1900 6600 50  0001 L CNN "Height"
F 6 "Alpha & Omega Semiconductors" H 1900 6500 50  0001 L CNN "Manufacturer_Name"
F 7 "AO3400A" H 1900 6400 50  0001 L CNN "Manufacturer_Part_Number"
F 8 "" H 1900 6300 50  0001 L CNN "Mouser Part Number"
F 9 "" H 1900 6200 50  0001 L CNN "Mouser Price/Stock"
F 10 "AO3400A" H 1900 6100 50  0001 L CNN "Arrow Part Number"
F 11 "https://www.arrow.com/en/products/ao3400a/alpha-and-omega-semiconductor?region=nac" H 1900 6000 50  0001 L CNN "Arrow Price/Stock"
	1    1450 6950
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR08
U 1 1 6220BBEA
P 1750 7150
F 0 "#PWR08" H 1750 6900 50  0001 C CNN
F 1 "GND" H 1755 6977 50  0000 C CNN
F 2 "" H 1750 7150 50  0001 C CNN
F 3 "" H 1750 7150 50  0001 C CNN
	1    1750 7150
	1    0    0    -1  
$EndComp
Text GLabel 950  6950 0    50   Input ~ 0
GPIO25
$Comp
L Device:R R1
U 1 1 6220E3FD
P 1200 6950
F 0 "R1" V 993 6950 50  0000 C CNN
F 1 "100" V 1084 6950 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 1130 6950 50  0001 C CNN
F 3 "~" H 1200 6950 50  0001 C CNN
	1    1200 6950
	0    1    1    0   
$EndComp
$Comp
L Device:R R4
U 1 1 622115D7
P 2350 6700
F 0 "R4" H 2420 6746 50  0000 L CNN
F 1 "36" H 2420 6655 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 2280 6700 50  0001 C CNN
F 3 "~" H 2350 6700 50  0001 C CNN
	1    2350 6700
	1    0    0    -1  
$EndComp
Wire Wire Line
	1750 6350 1750 6450
Wire Wire Line
	1750 6450 2350 6450
Wire Wire Line
	1750 6450 1750 6550
Connection ~ 1750 6450
$Comp
L power:GND #PWR011
U 1 1 62218983
P 2350 7150
F 0 "#PWR011" H 2350 6900 50  0001 C CNN
F 1 "GND" H 2355 6977 50  0000 C CNN
F 2 "" H 2350 7150 50  0001 C CNN
F 3 "" H 2350 7150 50  0001 C CNN
	1    2350 7150
	1    0    0    -1  
$EndComp
Wire Wire Line
	1450 6950 1350 6950
Wire Wire Line
	1050 6950 950  6950
Wire Wire Line
	1350 5750 1300 5750
Wire Wire Line
	1300 5750 1300 5900
Wire Wire Line
	1300 5900 1350 5900
Wire Wire Line
	2150 5900 2200 5900
Wire Wire Line
	2200 5900 2200 5750
Wire Wire Line
	2200 5750 2150 5750
Wire Wire Line
	1300 5750 1200 5750
Connection ~ 1300 5750
$Comp
L Device:R R6
U 1 1 621DA192
P 2500 5400
F 0 "R6" H 2570 5446 50  0000 L CNN
F 1 "1800" H 2570 5355 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 2430 5400 50  0001 C CNN
F 3 "~" H 2500 5400 50  0001 C CNN
	1    2500 5400
	1    0    0    -1  
$EndComp
$Comp
L Device:R R7
U 1 1 621DB81E
P 2500 5850
F 0 "R7" H 2570 5896 50  0000 L CNN
F 1 "2900" H 2570 5805 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 2430 5850 50  0001 C CNN
F 3 "~" H 2500 5850 50  0001 C CNN
	1    2500 5850
	1    0    0    -1  
$EndComp
Wire Wire Line
	2500 5550 2500 5600
Wire Wire Line
	2500 5200 2500 5250
$Comp
L power:+5V #PWR02
U 1 1 621ECBFC
P 1200 5750
F 0 "#PWR02" H 1200 5600 50  0001 C CNN
F 1 "+5V" V 1215 5878 50  0000 L CNN
F 2 "" H 1200 5750 50  0001 C CNN
F 3 "" H 1200 5750 50  0001 C CNN
	1    1200 5750
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR012
U 1 1 621EE04F
P 2500 6100
F 0 "#PWR012" H 2500 5850 50  0001 C CNN
F 1 "GND" H 2505 5927 50  0000 C CNN
F 2 "" H 2500 6100 50  0001 C CNN
F 3 "" H 2500 6100 50  0001 C CNN
	1    2500 6100
	1    0    0    -1  
$EndComp
Wire Wire Line
	2200 5750 2300 5750
Connection ~ 2200 5750
Wire Wire Line
	2500 6000 2500 6100
Wire Wire Line
	2500 5600 2850 5600
Connection ~ 2500 5600
Wire Wire Line
	2500 5600 2500 5700
Text GLabel 2850 5600 2    50   Input ~ 0
ADC1_CH5
Wire Wire Line
	2300 5200 2500 5200
Wire Wire Line
	2300 5200 2300 5750
Text Notes 1550 5000 0    50   ~ 0
Gas Sensor
$Comp
L 74xx:74HC14 U2
U 1 1 62236DAF
P 5100 5450
F 0 "U2" H 5100 5767 50  0000 C CNN
F 1 "74HC14" H 5100 5676 50  0000 C CNN
F 2 "Package_DIP:DIP-14_W7.62mm" H 5100 5450 50  0001 C CNN
F 3 "http://www.ti.com/lit/gpn/sn74HC14" H 5100 5450 50  0001 C CNN
	1    5100 5450
	1    0    0    -1  
$EndComp
$Comp
L 74xx:74HC14 U2
U 2 1 6223B3CC
P 5100 6550
F 0 "U2" H 5100 6867 50  0000 C CNN
F 1 "74HC14" H 5100 6776 50  0000 C CNN
F 2 "Package_DIP:DIP-14_W7.62mm" H 5100 6550 50  0001 C CNN
F 3 "http://www.ti.com/lit/gpn/sn74HC14" H 5100 6550 50  0001 C CNN
	2    5100 6550
	1    0    0    -1  
$EndComp
Text GLabel 5400 5450 2    50   Input ~ 0
GPIO5
Text GLabel 5400 6550 2    50   Input ~ 0
GPIO17
$Comp
L Device:R R13
U 1 1 62240379
P 4450 5450
F 0 "R13" H 4520 5496 50  0000 L CNN
F 1 "10k" H 4520 5405 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 4380 5450 50  0001 C CNN
F 3 "~" H 4450 5450 50  0001 C CNN
	1    4450 5450
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R14
U 1 1 6224144C
P 4450 6550
F 0 "R14" H 4520 6596 50  0000 L CNN
F 1 "10k" H 4520 6505 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 4380 6550 50  0001 C CNN
F 3 "~" H 4450 6550 50  0001 C CNN
	1    4450 6550
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R10
U 1 1 62241B22
P 4200 5200
F 0 "R10" H 4270 5246 50  0000 L CNN
F 1 "10k" H 4270 5155 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 4130 5200 50  0001 C CNN
F 3 "~" H 4200 5200 50  0001 C CNN
	1    4200 5200
	1    0    0    -1  
$EndComp
$Comp
L Device:R R11
U 1 1 622423BC
P 4200 6300
F 0 "R11" H 4270 6346 50  0000 L CNN
F 1 "10k" H 4270 6255 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 4130 6300 50  0001 C CNN
F 3 "~" H 4200 6300 50  0001 C CNN
	1    4200 6300
	1    0    0    -1  
$EndComp
$Comp
L Switch:SW_Push SW4
U 1 1 62246FBA
P 3900 6550
F 0 "SW4" H 3900 6835 50  0000 C CNN
F 1 "SW_Push" H 3900 6744 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 3900 6750 50  0001 C CNN
F 3 "~" H 3900 6750 50  0001 C CNN
	1    3900 6550
	1    0    0    -1  
$EndComp
$Comp
L Switch:SW_Push SW3
U 1 1 62248632
P 3900 5450
F 0 "SW3" H 3900 5735 50  0000 C CNN
F 1 "SW_Push" H 3900 5644 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 3900 5650 50  0001 C CNN
F 3 "~" H 3900 5650 50  0001 C CNN
	1    3900 5450
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR015
U 1 1 62249770
P 3600 6850
F 0 "#PWR015" H 3600 6600 50  0001 C CNN
F 1 "GND" H 3605 6677 50  0000 C CNN
F 2 "" H 3600 6850 50  0001 C CNN
F 3 "" H 3600 6850 50  0001 C CNN
	1    3600 6850
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR014
U 1 1 62249FB3
P 3600 5750
F 0 "#PWR014" H 3600 5500 50  0001 C CNN
F 1 "GND" H 3605 5577 50  0000 C CNN
F 2 "" H 3600 5750 50  0001 C CNN
F 3 "" H 3600 5750 50  0001 C CNN
	1    3600 5750
	1    0    0    -1  
$EndComp
Wire Wire Line
	3600 5450 3700 5450
Wire Wire Line
	4100 5450 4200 5450
Wire Wire Line
	4200 5350 4200 5450
Connection ~ 4200 5450
Wire Wire Line
	4200 5450 4300 5450
Wire Wire Line
	4600 5450 4700 5450
Wire Wire Line
	4700 5550 4700 5450
Connection ~ 4700 5450
Wire Wire Line
	4700 5450 4800 5450
Wire Wire Line
	4100 6550 4200 6550
Wire Wire Line
	4200 6450 4200 6550
Connection ~ 4200 6550
Wire Wire Line
	4200 6550 4300 6550
Wire Wire Line
	3700 6550 3600 6550
Wire Wire Line
	4600 6550 4700 6550
Wire Wire Line
	4700 6650 4700 6550
Connection ~ 4700 6550
Wire Wire Line
	4700 6550 4800 6550
$Comp
L power:+3.3V #PWR018
U 1 1 6225590F
P 4200 5050
F 0 "#PWR018" H 4200 4900 50  0001 C CNN
F 1 "+3.3V" H 4215 5223 50  0000 C CNN
F 2 "" H 4200 5050 50  0001 C CNN
F 3 "" H 4200 5050 50  0001 C CNN
	1    4200 5050
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR019
U 1 1 622563CE
P 4200 6150
F 0 "#PWR019" H 4200 6000 50  0001 C CNN
F 1 "+3.3V" H 4215 6323 50  0000 C CNN
F 2 "" H 4200 6150 50  0001 C CNN
F 3 "" H 4200 6150 50  0001 C CNN
	1    4200 6150
	1    0    0    -1  
$EndComp
Text Notes 3900 4750 0    50   ~ 0
Button Debouncing
$Comp
L power:GND #PWR022
U 1 1 6226459A
P 4700 5750
F 0 "#PWR022" H 4700 5500 50  0001 C CNN
F 1 "GND" H 4705 5577 50  0000 C CNN
F 2 "" H 4700 5750 50  0001 C CNN
F 3 "" H 4700 5750 50  0001 C CNN
	1    4700 5750
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR023
U 1 1 62264AA7
P 4700 6850
F 0 "#PWR023" H 4700 6600 50  0001 C CNN
F 1 "GND" H 4705 6677 50  0000 C CNN
F 2 "" H 4700 6850 50  0001 C CNN
F 3 "" H 4700 6850 50  0001 C CNN
	1    4700 6850
	1    0    0    -1  
$EndComp
Wire Wire Line
	3600 5450 3600 5750
Wire Wire Line
	3600 6550 3600 6850
Text GLabel 6000 2650 0    50   Input ~ 0
EN
$Comp
L power:+3.3V #PWR016
U 1 1 62270593
P 4050 950
F 0 "#PWR016" H 4050 800 50  0001 C CNN
F 1 "+3.3V" H 4065 1123 50  0000 C CNN
F 2 "" H 4050 950 50  0001 C CNN
F 3 "" H 4050 950 50  0001 C CNN
	1    4050 950 
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR017
U 1 1 622738DC
P 4250 1850
F 0 "#PWR017" H 4250 1600 50  0001 C CNN
F 1 "GND" H 4255 1677 50  0000 C CNN
F 2 "" H 4250 1850 50  0001 C CNN
F 3 "" H 4250 1850 50  0001 C CNN
	1    4250 1850
	1    0    0    -1  
$EndComp
Wire Wire Line
	4050 1250 4050 1300
Wire Wire Line
	4050 1300 4450 1300
Wire Wire Line
	4450 1300 4450 1350
Connection ~ 4050 1300
Wire Wire Line
	4050 1300 4050 1350
Wire Wire Line
	4050 1750 4050 1800
Wire Wire Line
	4050 1800 4250 1800
Wire Wire Line
	4250 1800 4250 1850
Wire Wire Line
	4450 1800 4250 1800
Connection ~ 4250 1800
Text GLabel 4800 1300 2    50   Input ~ 0
EN
Wire Wire Line
	4450 1300 4800 1300
Connection ~ 4450 1300
$Comp
L Device:R R9
U 1 1 6226F706
P 4050 1100
F 0 "R9" H 4120 1146 50  0000 L CNN
F 1 "10k" H 4120 1055 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 3980 1100 50  0001 C CNN
F 3 "~" H 4050 1100 50  0001 C CNN
	1    4050 1100
	-1   0    0    1   
$EndComp
Text Notes 4600 800  0    50   ~ 0
Reset
$Comp
L Noggin:RFM9-LoRa-Radio U6
U 1 1 621C1E3B
P 10250 5650
F 0 "U6" V 9425 5600 50  0000 C CNN
F 1 "RFM9-LoRa-Radio" V 9516 5600 50  0000 C CNN
F 2 "noggin:RFM9x" H 10250 5650 50  0001 C CNN
F 3 "" H 10250 5650 50  0001 C CNN
	1    10250 5650
	0    1    1    0   
$EndComp
Text GLabel 6000 4350 0    50   Input ~ 0
SPI_SCK
Text GLabel 9900 5650 0    50   Input ~ 0
SPI_SCK
Text Notes 9950 4700 0    50   ~ 0
LoRa Module
Text GLabel 9900 6100 0    50   Input ~ 0
SPI_CS
Text GLabel 6000 4250 0    50   Input ~ 0
SPI_CS
Text GLabel 9900 5800 0    50   Input ~ 0
SPI_MISO
Text GLabel 9900 5950 0    50   Input ~ 0
SPI_MOSI
Text GLabel 7400 2450 2    50   Input ~ 0
SPI_MISO
Text GLabel 7400 2550 2    50   Input ~ 0
SPI_MOSI
Text GLabel 9900 5500 0    50   Input ~ 0
GPIO4
Text GLabel 9900 6250 0    50   Input ~ 0
GPIO16
$Comp
L power:+3.3V #PWR040
U 1 1 621C829E
P 9500 5000
F 0 "#PWR040" H 9500 4850 50  0001 C CNN
F 1 "+3.3V" H 9515 5173 50  0000 C CNN
F 2 "" H 9500 5000 50  0001 C CNN
F 3 "" H 9500 5000 50  0001 C CNN
	1    9500 5000
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR041
U 1 1 621C99CC
P 9500 5200
F 0 "#PWR041" H 9500 4950 50  0001 C CNN
F 1 "GND" H 9505 5027 50  0000 C CNN
F 2 "" H 9500 5200 50  0001 C CNN
F 3 "" H 9500 5200 50  0001 C CNN
	1    9500 5200
	-1   0    0    -1  
$EndComp
$Comp
L Noggin:U1V10F5 VR1
U 1 1 621A437E
P 1050 3900
F 0 "VR1" V 1350 4000 50  0000 C CNN
F 1 "U1V10F5" V 1250 3900 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 1050 3650 50  0001 C CNN
F 3 "" H 1050 3650 50  0001 C CNN
	1    1050 3900
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR03
U 1 1 621B9659
P 1250 3900
F 0 "#PWR03" H 1250 3650 50  0001 C CNN
F 1 "GND" H 1255 3727 50  0000 C CNN
F 2 "" H 1250 3900 50  0001 C CNN
F 3 "" H 1250 3900 50  0001 C CNN
	1    1250 3900
	0    -1   -1   0   
$EndComp
$Comp
L power:+3.3V #PWR04
U 1 1 621B9FBC
P 1350 3700
F 0 "#PWR04" H 1350 3550 50  0001 C CNN
F 1 "+3.3V" H 1365 3873 50  0000 C CNN
F 2 "" H 1350 3700 50  0001 C CNN
F 3 "" H 1350 3700 50  0001 C CNN
	1    1350 3700
	1    0    0    -1  
$EndComp
Wire Wire Line
	1250 3800 1350 3800
Wire Wire Line
	1350 3800 1350 3700
$Comp
L power:+5V #PWR05
U 1 1 621BF0C8
P 1350 4100
F 0 "#PWR05" H 1350 3950 50  0001 C CNN
F 1 "+5V" H 1365 4273 50  0000 C CNN
F 2 "" H 1350 4100 50  0001 C CNN
F 3 "" H 1350 4100 50  0001 C CNN
	1    1350 4100
	-1   0    0    1   
$EndComp
Wire Wire Line
	1250 4000 1350 4000
Wire Wire Line
	1350 4000 1350 4100
Text GLabel 9900 5350 0    50   Input ~ 0
GPIO2
$Comp
L Noggin:LSM6DSLTR U4
U 1 1 621EF8C3
P 8500 1100
F 0 "U4" H 9400 1487 60  0000 C CNN
F 1 "LSM6DSLTR" H 9400 1381 60  0000 C CNN
F 2 "noggin:LSM6DSLTR" H 9400 1340 60  0001 C CNN
F 3 "" H 8500 1100 60  0000 C CNN
	1    8500 1100
	1    0    0    -1  
$EndComp
Text GLabel 8500 1400 0    50   Input ~ 0
GPIO18
$Comp
L power:GND #PWR037
U 1 1 621C86E4
P 8200 1800
F 0 "#PWR037" H 8200 1550 50  0001 C CNN
F 1 "GND" H 8205 1627 50  0000 C CNN
F 2 "" H 8200 1800 50  0001 C CNN
F 3 "" H 8200 1800 50  0001 C CNN
	1    8200 1800
	-1   0    0    -1  
$EndComp
Wire Wire Line
	8200 1600 8200 1700
Connection ~ 8200 1700
Wire Wire Line
	8200 1700 8200 1800
$Comp
L power:+3.3V #PWR043
U 1 1 621D8CE0
P 10850 1700
F 0 "#PWR043" H 10850 1550 50  0001 C CNN
F 1 "+3.3V" H 10865 1873 50  0000 C CNN
F 2 "" H 10850 1700 50  0001 C CNN
F 3 "" H 10850 1700 50  0001 C CNN
	1    10850 1700
	0    1    1    0   
$EndComp
Text GLabel 10300 1600 2    50   Input ~ 0
GPIO19
Wire Wire Line
	8200 1600 8500 1600
Wire Wire Line
	8200 1700 8500 1700
Wire Wire Line
	8500 1300 8200 1300
Wire Wire Line
	8200 1300 8200 1600
Connection ~ 8200 1600
Wire Wire Line
	8500 1200 8200 1200
Wire Wire Line
	8200 1200 8200 1300
Connection ~ 8200 1300
Wire Wire Line
	10300 1700 10700 1700
$Comp
L Device:C_Small C8
U 1 1 621FE0DC
P 10700 1900
F 0 "C8" H 10792 1946 50  0000 L CNN
F 1 "100 nF" H 10792 1855 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 10700 1900 50  0001 C CNN
F 3 "~" H 10700 1900 50  0001 C CNN
	1    10700 1900
	1    0    0    -1  
$EndComp
Wire Wire Line
	10700 1800 10700 1700
Connection ~ 10700 1700
Wire Wire Line
	10700 1700 10850 1700
$Comp
L power:GND #PWR042
U 1 1 62200D10
P 10700 2000
F 0 "#PWR042" H 10700 1750 50  0001 C CNN
F 1 "GND" H 10705 1827 50  0000 C CNN
F 2 "" H 10700 2000 50  0001 C CNN
F 3 "" H 10700 2000 50  0001 C CNN
	1    10700 2000
	-1   0    0    -1  
$EndComp
$Comp
L Device:C_Small C6
U 1 1 62203F11
P 7800 1650
F 0 "C6" H 7600 1700 50  0000 L CNN
F 1 "100 nF" H 7400 1600 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 7800 1650 50  0001 C CNN
F 3 "~" H 7800 1650 50  0001 C CNN
	1    7800 1650
	1    0    0    -1  
$EndComp
Wire Wire Line
	7800 1750 7800 1800
Wire Wire Line
	7800 1800 8200 1800
Connection ~ 8200 1800
Wire Wire Line
	8500 1500 7800 1500
Wire Wire Line
	7800 1500 7800 1550
$Comp
L Device:R R16
U 1 1 6220D6DC
P 10400 950
F 0 "R16" H 10470 996 50  0000 L CNN
F 1 "10k" H 10470 905 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 10330 950 50  0001 C CNN
F 3 "~" H 10400 950 50  0001 C CNN
	1    10400 950 
	1    0    0    -1  
$EndComp
$Comp
L Device:R R17
U 1 1 62212F2D
P 10800 1050
F 0 "R17" H 10870 1096 50  0000 L CNN
F 1 "10k" H 10870 1005 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 10730 1050 50  0001 C CNN
F 3 "~" H 10800 1050 50  0001 C CNN
	1    10800 1050
	1    0    0    -1  
$EndComp
Wire Wire Line
	10300 1100 10400 1100
Wire Wire Line
	10300 1200 10800 1200
Wire Wire Line
	10400 1100 10450 1100
Connection ~ 10400 1100
Text GLabel 10850 1200 2    50   Input ~ 0
SCL
Wire Wire Line
	10800 1200 10850 1200
Connection ~ 10800 1200
Wire Wire Line
	7800 1500 7800 650 
Wire Wire Line
	7800 650  10400 650 
Wire Wire Line
	10800 650  10800 900 
Connection ~ 7800 1500
Wire Wire Line
	10400 800  10400 650 
Connection ~ 10400 650 
Wire Wire Line
	10400 650  10800 650 
Wire Wire Line
	10300 1300 11100 1300
Wire Wire Line
	11100 1300 11100 650 
Wire Wire Line
	11100 650  10800 650 
Connection ~ 10800 650 
Text GLabel 7400 3650 2    50   Input ~ 0
SDA
Text GLabel 7400 3950 2    50   Input ~ 0
SCL
Text GLabel 6000 2950 0    50   Input ~ 0
ADC1_CH6
Text GLabel 6000 3350 0    50   Input ~ 0
GPIO25
Text GLabel 6000 3250 0    50   Input ~ 0
ADC1_CH5
Text Notes 950  3350 0    50   ~ 0
5V Power
Text Notes 700  4500 0    50   ~ 0
5V Step-Up\nVoltage Regulator
Text GLabel 10450 1100 2    50   Input ~ 0
SDA
$Comp
L Noggin:SEN-11574 U5
U 1 1 62262824
P 3800 4000
F 0 "U5" H 4690 4046 50  0000 L CNN
F 1 "SEN-11574" H 4690 3955 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Horizontal" H 3800 3950 50  0001 C CNN
F 3 "" H 3800 3950 50  0001 C CNN
	1    3800 4000
	-1   0    0    1   
$EndComp
Text Notes 3900 4350 2    50   ~ 0
Pulse Sensor
$Comp
L power:GND #PWR038
U 1 1 62268C21
P 4200 4150
F 0 "#PWR038" H 4200 3900 50  0001 C CNN
F 1 "GND" H 4205 3977 50  0000 C CNN
F 2 "" H 4200 4150 50  0001 C CNN
F 3 "" H 4200 4150 50  0001 C CNN
	1    4200 4150
	0    -1   -1   0   
$EndComp
$Comp
L power:+3.3V #PWR039
U 1 1 622698BE
P 4200 3850
F 0 "#PWR039" H 4200 3700 50  0001 C CNN
F 1 "+3.3V" H 4215 4023 50  0000 C CNN
F 2 "" H 4200 3850 50  0001 C CNN
F 3 "" H 4200 3850 50  0001 C CNN
	1    4200 3850
	0    1    1    0   
$EndComp
Wire Wire Line
	4200 4000 4500 4000
Text GLabel 6000 3150 0    50   Input ~ 0
ADC1_CH4
Text GLabel 4500 4000 2    50   Input ~ 0
ADC1_CH4
$Comp
L power:GND #PWR036
U 1 1 6226E33E
P 7550 4150
F 0 "#PWR036" H 7550 3900 50  0001 C CNN
F 1 "GND" H 7555 3977 50  0000 C CNN
F 2 "" H 7550 4150 50  0001 C CNN
F 3 "" H 7550 4150 50  0001 C CNN
	1    7550 4150
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR035
U 1 1 6226EBB4
P 7400 4250
F 0 "#PWR035" H 7400 4000 50  0001 C CNN
F 1 "GND" H 7405 4077 50  0000 C CNN
F 2 "" H 7400 4250 50  0001 C CNN
F 3 "" H 7400 4250 50  0001 C CNN
	1    7400 4250
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR031
U 1 1 6226ED9D
P 6000 3850
F 0 "#PWR031" H 6000 3600 50  0001 C CNN
F 1 "GND" H 6005 3677 50  0000 C CNN
F 2 "" H 6000 3850 50  0001 C CNN
F 3 "" H 6000 3850 50  0001 C CNN
	1    6000 3850
	0    1    1    0   
$EndComp
Text GLabel 6000 3450 0    50   Input ~ 0
GPIO26
Text GLabel 6000 3550 0    50   Input ~ 0
GPIO27
Text GLabel 6000 3650 0    50   Input ~ 0
GPIO14
Text GLabel 7400 2750 2    50   Input ~ 0
GPIO2
Text GLabel 7400 2950 2    50   Input ~ 0
GPIO4
Text GLabel 7400 3050 2    50   Input ~ 0
GPIO16
Text GLabel 7400 3350 2    50   Input ~ 0
GPIO18
Text GLabel 7400 3450 2    50   Input ~ 0
GPIO19
Text GLabel 7400 3250 2    50   Input ~ 0
GPIO5
Text GLabel 7400 3150 2    50   Input ~ 0
GPIO17
Text GLabel 8500 1100 0    50   Input ~ 0
SDO
Text GLabel 7400 4050 2    50   Input ~ 0
SDO
Wire Wire Line
	7400 4150 7550 4150
$Comp
L 74xx:74HC14 U2
U 7 1 62296A70
P 6400 5000
F 0 "U2" V 6033 5000 50  0000 C CNN
F 1 "74HC14" V 6124 5000 50  0000 C CNN
F 2 "Package_DIP:DIP-14_W7.62mm" H 6400 5000 50  0001 C CNN
F 3 "http://www.ti.com/lit/gpn/sn74HC14" H 6400 5000 50  0001 C CNN
	7    6400 5000
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR028
U 1 1 6229B493
P 5900 5000
F 0 "#PWR028" H 5900 4750 50  0001 C CNN
F 1 "GND" H 5905 4827 50  0000 C CNN
F 2 "" H 5900 5000 50  0001 C CNN
F 3 "" H 5900 5000 50  0001 C CNN
	1    5900 5000
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR034
U 1 1 6229BD9A
P 6900 5000
F 0 "#PWR034" H 6900 4850 50  0001 C CNN
F 1 "+3.3V" H 6915 5173 50  0000 C CNN
F 2 "" H 6900 5000 50  0001 C CNN
F 3 "" H 6900 5000 50  0001 C CNN
	1    6900 5000
	1    0    0    -1  
$EndComp
$Comp
L 74xx:74HC14 U2
U 3 1 6229D1FF
P 6200 5600
F 0 "U2" H 6200 5917 50  0000 C CNN
F 1 "74HC14" H 6200 5826 50  0000 C CNN
F 2 "Package_DIP:DIP-14_W7.62mm" H 6200 5600 50  0001 C CNN
F 3 "http://www.ti.com/lit/gpn/sn74HC14" H 6200 5600 50  0001 C CNN
	3    6200 5600
	1    0    0    -1  
$EndComp
$Comp
L 74xx:74HC14 U2
U 4 1 6229FE38
P 6200 6100
F 0 "U2" H 6200 6417 50  0000 C CNN
F 1 "74HC14" H 6200 6326 50  0000 C CNN
F 2 "Package_DIP:DIP-14_W7.62mm" H 6200 6100 50  0001 C CNN
F 3 "http://www.ti.com/lit/gpn/sn74HC14" H 6200 6100 50  0001 C CNN
	4    6200 6100
	1    0    0    -1  
$EndComp
$Comp
L 74xx:74HC14 U2
U 5 1 622C073F
P 7000 5600
F 0 "U2" H 7000 5917 50  0000 C CNN
F 1 "74HC14" H 7000 5826 50  0000 C CNN
F 2 "Package_DIP:DIP-14_W7.62mm" H 7000 5600 50  0001 C CNN
F 3 "http://www.ti.com/lit/gpn/sn74HC14" H 7000 5600 50  0001 C CNN
	5    7000 5600
	1    0    0    -1  
$EndComp
$Comp
L 74xx:74HC14 U2
U 6 1 622C2769
P 7000 6100
F 0 "U2" H 7000 6417 50  0000 C CNN
F 1 "74HC14" H 7000 6326 50  0000 C CNN
F 2 "Package_DIP:DIP-14_W7.62mm" H 7000 6100 50  0001 C CNN
F 3 "http://www.ti.com/lit/gpn/sn74HC14" H 7000 6100 50  0001 C CNN
	6    7000 6100
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR029
U 1 1 622C4BD0
P 5900 5600
F 0 "#PWR029" H 5900 5350 50  0001 C CNN
F 1 "GND" H 5905 5427 50  0000 C CNN
F 2 "" H 5900 5600 50  0001 C CNN
F 3 "" H 5900 5600 50  0001 C CNN
	1    5900 5600
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR030
U 1 1 622C5CDE
P 5900 6100
F 0 "#PWR030" H 5900 5850 50  0001 C CNN
F 1 "GND" H 5905 5927 50  0000 C CNN
F 2 "" H 5900 6100 50  0001 C CNN
F 3 "" H 5900 6100 50  0001 C CNN
	1    5900 6100
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR033
U 1 1 622C61C1
P 6700 6100
F 0 "#PWR033" H 6700 5850 50  0001 C CNN
F 1 "GND" H 6705 5927 50  0000 C CNN
F 2 "" H 6700 6100 50  0001 C CNN
F 3 "" H 6700 6100 50  0001 C CNN
	1    6700 6100
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR032
U 1 1 622C6842
P 6700 5600
F 0 "#PWR032" H 6700 5350 50  0001 C CNN
F 1 "GND" H 6705 5427 50  0000 C CNN
F 2 "" H 6700 5600 50  0001 C CNN
F 3 "" H 6700 5600 50  0001 C CNN
	1    6700 5600
	1    0    0    -1  
$EndComp
Wire Notes Line
	3200 650  550  650 
Wire Notes Line
	550  650  550  3100
Wire Notes Line
	550  3100 3200 3100
Wire Notes Line
	3200 3100 3200 650 
Wire Notes Line
	1750 4550 1750 3250
Wire Notes Line
	1750 3250 550  3250
Wire Notes Line
	550  3250 550  4550
Wire Notes Line
	550  4550 1750 4550
Wire Notes Line
	550  7450 3300 7450
Wire Notes Line
	3300 7450 3300 4850
Wire Notes Line
	3300 4850 550  4850
Wire Notes Line
	550  4850 550  7450
Wire Notes Line
	3450 7250 6900 7250
Wire Notes Line
	6900 7250 6900 6450
Wire Notes Line
	6900 6450 7400 6450
Wire Notes Line
	7400 6450 7400 4550
Wire Notes Line
	7400 4550 3450 4550
Wire Notes Line
	3450 4550 3450 7250
Wire Notes Line
	2350 4400 2350 3700
Wire Notes Line
	2350 3700 5000 3700
Wire Notes Line
	5000 3700 5000 4400
Wire Notes Line
	5000 4400 2350 4400
Wire Notes Line
	10750 4550 9300 4550
Wire Notes Line
	9300 4550 9300 6400
Wire Notes Line
	9300 6400 10750 6400
Wire Notes Line
	10750 6400 10750 4550
Wire Notes Line
	11150 550  7350 550 
Wire Notes Line
	11150 2300 11150 550 
$Comp
L Connector_Generic:Conn_01x06 J2
U 1 1 6232A885
P 5800 1100
F 0 "J2" H 5880 1092 50  0000 L CNN
F 1 "Conn_01x06" H 5880 1001 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x06_P2.54mm_Vertical" H 5800 1100 50  0001 C CNN
F 3 "~" H 5800 1100 50  0001 C CNN
	1    5800 1100
	1    0    0    -1  
$EndComp
Text Notes 5200 750  0    50   ~ 0
Programming Header
$Comp
L power:GND #PWR026
U 1 1 6232C84D
P 5450 900
F 0 "#PWR026" H 5450 650 50  0001 C CNN
F 1 "GND" H 5455 727 50  0000 C CNN
F 2 "" H 5450 900 50  0001 C CNN
F 3 "" H 5450 900 50  0001 C CNN
	1    5450 900 
	0    1    1    0   
$EndComp
Text GLabel 7400 3850 2    50   Input ~ 0
TX
Text GLabel 5600 1000 0    50   Input ~ 0
TX
Text GLabel 7400 3750 2    50   Input ~ 0
RX
Text GLabel 5600 1100 0    50   Input ~ 0
RX
$Comp
L power:+3.3V #PWR027
U 1 1 6232EEA6
P 5450 1200
F 0 "#PWR027" H 5450 1050 50  0001 C CNN
F 1 "+3.3V" H 5465 1373 50  0000 C CNN
F 2 "" H 5450 1200 50  0001 C CNN
F 3 "" H 5450 1200 50  0001 C CNN
	1    5450 1200
	0    -1   -1   0   
$EndComp
Text GLabel 5600 1300 0    50   Input ~ 0
EN
Text GLabel 7400 2850 2    50   Input ~ 0
GPIO0
Text GLabel 5600 1400 0    50   Input ~ 0
GPIO0
Wire Wire Line
	5450 900  5600 900 
Wire Wire Line
	5450 1200 5600 1200
Wire Notes Line
	5150 650  6400 650 
Wire Notes Line
	6400 650  6400 1550
Wire Notes Line
	6400 1550 5150 1550
Wire Notes Line
	5150 1550 5150 650 
$Comp
L power:GND #PWR010
U 1 1 6219E17C
P 2100 1550
F 0 "#PWR010" H 2100 1300 50  0001 C CNN
F 1 "GND" H 2105 1377 50  0000 C CNN
F 2 "" H 2100 1550 50  0001 C CNN
F 3 "" H 2100 1550 50  0001 C CNN
	1    2100 1550
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR01
U 1 1 621B8E68
P 1150 1800
F 0 "#PWR01" H 1150 1550 50  0001 C CNN
F 1 "GND" H 1155 1627 50  0000 C CNN
F 2 "" H 1150 1800 50  0001 C CNN
F 3 "" H 1150 1800 50  0001 C CNN
	1    1150 1800
	1    0    0    -1  
$EndComp
Wire Wire Line
	1250 1450 1150 1450
Wire Wire Line
	1150 1450 1150 1700
Text Notes 1100 850  0    50   ~ 0
Power
Text Notes 2200 1900 0    50   ~ 0
3.3V Step-Up Step-Down\nVoltage Regulator
$Comp
L Device:R R2
U 1 1 6223F31D
P 1750 2050
F 0 "R2" H 1820 2096 50  0000 L CNN
F 1 "22k" H 1820 2005 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 1680 2050 50  0001 C CNN
F 3 "~" H 1750 2050 50  0001 C CNN
	1    1750 2050
	-1   0    0    1   
$EndComp
$Comp
L Device:R R3
U 1 1 62242505
P 1750 2550
F 0 "R3" H 1820 2596 50  0000 L CNN
F 1 "68k" H 1820 2505 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 1680 2550 50  0001 C CNN
F 3 "~" H 1750 2550 50  0001 C CNN
	1    1750 2550
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR07
U 1 1 62242CEE
P 1750 2700
F 0 "#PWR07" H 1750 2450 50  0001 C CNN
F 1 "GND" H 1755 2527 50  0000 C CNN
F 2 "" H 1750 2700 50  0001 C CNN
F 3 "" H 1750 2700 50  0001 C CNN
	1    1750 2700
	1    0    0    -1  
$EndComp
Wire Wire Line
	1750 2200 1750 2300
Wire Wire Line
	1750 2300 1800 2300
Connection ~ 1750 2300
Wire Wire Line
	1750 2300 1750 2400
Text GLabel 1800 2300 2    50   Input ~ 0
ADC1_CH6
Wire Wire Line
	1750 1450 1750 1900
Wire Wire Line
	1750 1450 2100 1450
Text Notes 1900 2550 0    50   ~ 0
Battery\nMonitoring
$Comp
L power:+3.3V #PWR025
U 1 1 620D7784
P 5150 2550
F 0 "#PWR025" H 5150 2400 50  0001 C CNN
F 1 "+3.3V" H 5165 2723 50  0000 C CNN
F 2 "" H 5150 2550 50  0001 C CNN
F 3 "" H 5150 2550 50  0001 C CNN
	1    5150 2550
	-1   0    0    1   
$EndComp
$Comp
L Device:C_Small C5
U 1 1 621E55A2
P 5650 2450
F 0 "C5" H 5742 2496 50  0000 L CNN
F 1 "0.1uF" H 5742 2405 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 5650 2450 50  0001 C CNN
F 3 "~" H 5650 2450 50  0001 C CNN
	1    5650 2450
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C4
U 1 1 621F0155
P 5300 2450
F 0 "C4" H 5392 2496 50  0000 L CNN
F 1 "10uF" H 5392 2405 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 5300 2450 50  0001 C CNN
F 3 "~" H 5300 2450 50  0001 C CNN
	1    5300 2450
	1    0    0    -1  
$EndComp
Wire Wire Line
	5650 2550 6000 2550
Wire Wire Line
	5300 2550 5150 2550
Wire Wire Line
	5300 2550 5650 2550
Connection ~ 5300 2550
Connection ~ 5650 2550
Wire Wire Line
	6000 2450 6000 2350
Wire Wire Line
	6000 2350 5650 2350
Wire Wire Line
	5650 2350 5300 2350
Connection ~ 5650 2350
Wire Wire Line
	5300 2350 5150 2350
Connection ~ 5300 2350
$Comp
L Device:C_Small C1
U 1 1 6221322F
P 4450 1450
F 0 "C1" H 4542 1496 50  0000 L CNN
F 1 "1uF" H 4542 1405 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 4450 1450 50  0001 C CNN
F 3 "~" H 4450 1450 50  0001 C CNN
	1    4450 1450
	1    0    0    -1  
$EndComp
Wire Wire Line
	4450 1550 4450 1800
$Comp
L Device:C_Small C2
U 1 1 621F1B60
P 4700 5650
F 0 "C2" H 4792 5696 50  0000 L CNN
F 1 "0.1uF" H 4792 5605 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 4700 5650 50  0001 C CNN
F 3 "~" H 4700 5650 50  0001 C CNN
	1    4700 5650
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C3
U 1 1 621F2F4E
P 4700 6750
F 0 "C3" H 4792 6796 50  0000 L CNN
F 1 "0.1uF" H 4792 6705 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 4700 6750 50  0001 C CNN
F 3 "~" H 4700 6750 50  0001 C CNN
	1    4700 6750
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C7
U 1 1 621FB93F
P 9500 5100
F 0 "C7" H 9592 5146 50  0000 L CNN
F 1 "100 nF" H 9592 5055 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 9500 5100 50  0001 C CNN
F 3 "~" H 9500 5100 50  0001 C CNN
	1    9500 5100
	1    0    0    -1  
$EndComp
Wire Wire Line
	9900 5050 9900 5000
Connection ~ 9500 5000
Connection ~ 9500 5200
Wire Wire Line
	9500 5000 9900 5000
Wire Wire Line
	9500 5200 9900 5200
$Comp
L Connector:Conn_01x02_Male J1
U 1 1 622461A7
P 950 1700
F 0 "J1" H 1058 1881 50  0000 C CNN
F 1 "Conn_01x02_Male" H 1058 1790 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 950 1700 50  0001 C CNN
F 3 "~" H 950 1700 50  0001 C CNN
	1    950  1700
	1    0    0    -1  
$EndComp
$Comp
L Switch:SW_SPST SW1
U 1 1 62270423
P 1450 1450
F 0 "SW1" H 1450 1685 50  0000 C CNN
F 1 "SW_SPST" H 1450 1594 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 1450 1450 50  0001 C CNN
F 3 "~" H 1450 1450 50  0001 C CNN
	1    1450 1450
	1    0    0    -1  
$EndComp
Wire Wire Line
	1650 1450 1750 1450
Connection ~ 1750 1450
$Comp
L Switch:SW_Push SW2
U 1 1 622753D1
P 4050 1550
F 0 "SW2" H 4050 1835 50  0000 C CNN
F 1 "SW_Push" H 4050 1744 50  0000 C CNN
F 2 "Button_Switch_THT:SW_PUSH_6mm" H 4050 1750 50  0001 C CNN
F 3 "~" H 4050 1750 50  0001 C CNN
	1    4050 1550
	0    -1   -1   0   
$EndComp
Wire Notes Line
	3700 2100 5000 2100
Wire Notes Line
	3700 650  5000 650 
Wire Notes Line
	7950 2500 7950 3900
Wire Notes Line
	9100 2500 7950 2500
Wire Notes Line
	9100 3900 9100 2500
Wire Notes Line
	7950 3900 9100 3900
Text GLabel 8250 2750 2    50   Input ~ 0
GPIO14
Text GLabel 8650 2850 2    50   Input ~ 0
GPIO27
Text Notes 8450 2650 0    50   ~ 0
LEDs
Wire Wire Line
	8250 2750 8250 2900
Wire Wire Line
	8650 2900 8650 2850
Wire Wire Line
	8250 3200 8250 3300
Wire Wire Line
	8650 3300 8650 3200
$Comp
L Device:R R15
U 1 1 621C0204
P 8650 3050
F 0 "R15" H 8720 3096 50  0000 L CNN
F 1 "68" H 8720 3005 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 8580 3050 50  0001 C CNN
F 3 "~" H 8650 3050 50  0001 C CNN
	1    8650 3050
	-1   0    0    1   
$EndComp
$Comp
L Device:R R12
U 1 1 621BECEF
P 8250 3050
F 0 "R12" H 8320 3096 50  0000 L CNN
F 1 "68" H 8320 3005 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 8180 3050 50  0001 C CNN
F 3 "~" H 8250 3050 50  0001 C CNN
	1    8250 3050
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR021
U 1 1 621F0037
P 8650 3600
F 0 "#PWR021" H 8650 3350 50  0001 C CNN
F 1 "GND" H 8655 3427 50  0000 C CNN
F 2 "" H 8650 3600 50  0001 C CNN
F 3 "" H 8650 3600 50  0001 C CNN
	1    8650 3600
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR020
U 1 1 621EF66F
P 8250 3600
F 0 "#PWR020" H 8250 3350 50  0001 C CNN
F 1 "GND" H 8255 3427 50  0000 C CNN
F 2 "" H 8250 3600 50  0001 C CNN
F 3 "" H 8250 3600 50  0001 C CNN
	1    8250 3600
	1    0    0    -1  
$EndComp
$Comp
L Device:LED D1
U 1 1 620D647E
P 8250 3450
F 0 "D1" H 8243 3666 50  0000 C CNN
F 1 "LED" H 8243 3575 50  0000 C CNN
F 2 "LED_SMD:LED_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 8250 3450 50  0001 C CNN
F 3 "~" H 8250 3450 50  0001 C CNN
	1    8250 3450
	0    -1   -1   0   
$EndComp
$Comp
L Device:LED D2
U 1 1 620D5497
P 8650 3450
F 0 "D2" H 8643 3666 50  0000 C CNN
F 1 "LED" H 8643 3575 50  0000 C CNN
F 2 "LED_SMD:LED_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 8650 3450 50  0001 C CNN
F 3 "~" H 8650 3450 50  0001 C CNN
	1    8650 3450
	0    -1   -1   0   
$EndComp
$Comp
L Noggin:PS1240P02BT LS1
U 1 1 621084ED
P 10050 3900
F 0 "LS1" H 10678 3896 50  0000 L CNN
F 1 "PS1240P02BT" H 10678 3805 50  0000 L CNN
F 2 "noggin:PS1240P02BT" H 10700 4000 50  0001 L CNN
F 3 "https://product.tdk.com/system/files/dam/doc/product/sw_piezo/sw_piezo/piezo-buzzer/catalog/piezoelectronic_buzzer_ps_en.pdf" H 10700 3900 50  0001 L CNN
F 4 "Piezoelectric Buzzers, Sound Pressure Level=70dB min, Freq.=4000Hz nom" H 10700 3800 50  0001 L CNN "Description"
F 5 "TDK" H 10700 3600 50  0001 L CNN "Manufacturer_Name"
F 6 "PS1240P02BT" H 10700 3500 50  0001 L CNN "Manufacturer_Part_Number"
F 7 "810-PS1240P02BT" H 10700 3400 50  0001 L CNN "Mouser Part Number"
F 8 "https://www.mouser.co.uk/ProductDetail/TDK/PS1240P02BT?qs=d7g9p1yFhWaZXSY9MjKMkw%3D%3D" H 10700 3300 50  0001 L CNN "Mouser Price/Stock"
F 9 "PS1240P02BT" H 10700 3200 50  0001 L CNN "Arrow Part Number"
F 10 "https://www.arrow.com/en/products/ps1240p02bt/tdk?region=nac" H 10700 3100 50  0001 L CNN "Arrow Price/Stock"
	1    10050 3900
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R8
U 1 1 6210A8A7
P 9800 4000
F 0 "R8" V 9593 4000 50  0000 C CNN
F 1 "1K" V 9684 4000 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 9730 4000 50  0001 C CNN
F 3 "~" H 9800 4000 50  0001 C CNN
	1    9800 4000
	0    1    1    0   
$EndComp
Wire Wire Line
	9950 4000 10050 4000
Wire Wire Line
	10050 4000 10050 3900
$Comp
L power:GND #PWR013
U 1 1 621E2094
P 10150 4000
F 0 "#PWR013" H 10150 3750 50  0001 C CNN
F 1 "GND" H 10155 3827 50  0000 C CNN
F 2 "" H 10150 4000 50  0001 C CNN
F 3 "" H 10150 4000 50  0001 C CNN
	1    10150 4000
	-1   0    0    -1  
$EndComp
Wire Wire Line
	10150 3900 10150 4000
Text GLabel 9650 4000 0    50   Input ~ 0
GPIO26
Text Notes 9650 2650 0    50   ~ 0
Buzzer
Wire Notes Line
	10300 4250 10300 2500
Wire Notes Line
	10300 2500 9250 2500
Wire Notes Line
	9250 2500 9250 4250
Wire Notes Line
	9250 4250 10300 4250
$Comp
L Device:R R5
U 1 1 62205901
P 2350 7000
F 0 "R5" H 2420 7046 50  0000 L CNN
F 1 "36" H 2420 6955 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 2280 7000 50  0001 C CNN
F 3 "~" H 2350 7000 50  0001 C CNN
	1    2350 7000
	1    0    0    -1  
$EndComp
Wire Wire Line
	2350 6450 2350 6550
$Comp
L Noggin:S9V11F3S5C3 VR2
U 1 1 62215EA1
P 2300 1450
F 0 "VR2" V 1850 1500 50  0000 L CNN
F 1 "S9V11F3S5C3" V 1950 1300 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x06_P2.54mm_Vertical" H 2200 900 50  0001 C CNN
F 3 "" H 2200 900 50  0001 C CNN
	1    2300 1450
	0    1    1    0   
$EndComp
Wire Notes Line
	5000 2100 5000 650 
Wire Notes Line
	3700 650  3700 2100
Wire Notes Line
	7350 550  7350 2300
Wire Notes Line
	7350 2300 11150 2300
Text Notes 7550 2150 0    50   ~ 0
Accelerometer
Wire Wire Line
	2100 1650 2050 1650
Wire Wire Line
	2050 1650 2050 1750
$Comp
L power:+3.3V #PWR09
U 1 1 6219FA43
P 2050 1750
F 0 "#PWR09" H 2050 1600 50  0001 C CNN
F 1 "+3.3V" H 2065 1923 50  0000 C CNN
F 2 "" H 2050 1750 50  0001 C CNN
F 3 "" H 2050 1750 50  0001 C CNN
	1    2050 1750
	-1   0    0    1   
$EndComp
$EndSCHEMATC
