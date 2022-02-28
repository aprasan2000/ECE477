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
L Noggin:ESP32-WROOM-32D ANT?
U 1 1 620D2920
P 6000 2450
F 0 "ANT?" H 6700 2715 50  0000 C CNN
F 1 "ESP32-WROOM-32D" H 6700 2624 50  0000 C CNN
F 2 "ESP32WROOM32D" H 7250 2550 50  0001 L CNN
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
L Device:LED D?
U 1 1 620D5497
P 4750 3950
F 0 "D?" H 4743 4166 50  0000 C CNN
F 1 "LED" H 4743 4075 50  0000 C CNN
F 2 "" H 4750 3950 50  0001 C CNN
F 3 "~" H 4750 3950 50  0001 C CNN
	1    4750 3950
	0    -1   -1   0   
$EndComp
$Comp
L power:+3.3V #PWR?
U 1 1 620D7784
P 5900 2550
F 0 "#PWR?" H 5900 2400 50  0001 C CNN
F 1 "+3.3V" H 5915 2723 50  0000 C CNN
F 2 "" H 5900 2550 50  0001 C CNN
F 3 "" H 5900 2550 50  0001 C CNN
	1    5900 2550
	0    -1   -1   0   
$EndComp
$Comp
L Device:LED D?
U 1 1 620D647E
P 4350 3950
F 0 "D?" H 4343 4166 50  0000 C CNN
F 1 "LED" H 4343 4075 50  0000 C CNN
F 2 "" H 4350 3950 50  0001 C CNN
F 3 "~" H 4350 3950 50  0001 C CNN
	1    4350 3950
	0    -1   -1   0   
$EndComp
$Comp
L Noggin:PS1240P02BT LS?
U 1 1 621084ED
P 3150 4700
F 0 "LS?" H 3778 4696 50  0000 L CNN
F 1 "PS1240P02BT" H 3778 4605 50  0000 L CNN
F 2 "PS1240P02BT" H 3800 4800 50  0001 L CNN
F 3 "https://product.tdk.com/system/files/dam/doc/product/sw_piezo/sw_piezo/piezo-buzzer/catalog/piezoelectronic_buzzer_ps_en.pdf" H 3800 4700 50  0001 L CNN
F 4 "Piezoelectric Buzzers, Sound Pressure Level=70dB min, Freq.=4000Hz nom" H 3800 4600 50  0001 L CNN "Description"
F 5 "TDK" H 3800 4400 50  0001 L CNN "Manufacturer_Name"
F 6 "PS1240P02BT" H 3800 4300 50  0001 L CNN "Manufacturer_Part_Number"
F 7 "810-PS1240P02BT" H 3800 4200 50  0001 L CNN "Mouser Part Number"
F 8 "https://www.mouser.co.uk/ProductDetail/TDK/PS1240P02BT?qs=d7g9p1yFhWaZXSY9MjKMkw%3D%3D" H 3800 4100 50  0001 L CNN "Mouser Price/Stock"
F 9 "PS1240P02BT" H 3800 4000 50  0001 L CNN "Arrow Part Number"
F 10 "https://www.arrow.com/en/products/ps1240p02bt/tdk?region=nac" H 3800 3900 50  0001 L CNN "Arrow Price/Stock"
	1    3150 4700
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R?
U 1 1 6210A8A7
P 2900 4800
F 0 "R?" V 2693 4800 50  0000 C CNN
F 1 "1K" V 2784 4800 50  0000 C CNN
F 2 "" V 2830 4800 50  0001 C CNN
F 3 "~" H 2900 4800 50  0001 C CNN
	1    2900 4800
	0    1    1    0   
$EndComp
Wire Wire Line
	3050 4800 3150 4800
$Comp
L Noggin:S9V11F3S5C3 VR1
U 1 1 621966CE
P 2300 1650
F 0 "VR1" V 1850 1550 50  0000 L CNN
F 1 "S9V11F3S5C3" V 1950 1550 50  0000 L CNN
F 2 "" H 2200 1100 50  0001 C CNN
F 3 "" H 2200 1100 50  0001 C CNN
	1    2300 1650
	0    1    1    0   
$EndComp
$Comp
L Device:Battery_Cell BT?
U 1 1 62197E53
P 1150 2050
F 0 "BT?" H 850 2200 50  0000 L CNN
F 1 "Battery_Cell" H 550 2100 50  0000 L CNN
F 2 "" V 1150 2110 50  0001 C CNN
F 3 "~" V 1150 2110 50  0001 C CNN
	1    1150 2050
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR?
U 1 1 6219FA43
P 2000 900
F 0 "#PWR?" H 2000 750 50  0001 C CNN
F 1 "+3.3V" H 2015 1073 50  0000 C CNN
F 2 "" H 2000 900 50  0001 C CNN
F 3 "" H 2000 900 50  0001 C CNN
	1    2000 900 
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 6219E17C
P 2100 1500
F 0 "#PWR?" H 2100 1250 50  0001 C CNN
F 1 "GND" H 2105 1327 50  0000 C CNN
F 2 "" H 2100 1500 50  0001 C CNN
F 3 "" H 2100 1500 50  0001 C CNN
	1    2100 1500
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 621B8E68
P 1150 2150
F 0 "#PWR?" H 1150 1900 50  0001 C CNN
F 1 "GND" H 1155 1977 50  0000 C CNN
F 2 "" H 1150 2150 50  0001 C CNN
F 3 "" H 1150 2150 50  0001 C CNN
	1    1150 2150
	1    0    0    -1  
$EndComp
Wire Wire Line
	2100 1400 2000 1400
$Comp
L Switch:SW_SPST SW?
U 1 1 621C2767
P 1450 1600
F 0 "SW?" H 1450 1835 50  0000 C CNN
F 1 "SW_SPST" H 1450 1744 50  0000 C CNN
F 2 "" H 1450 1600 50  0001 C CNN
F 3 "~" H 1450 1600 50  0001 C CNN
	1    1450 1600
	1    0    0    -1  
$EndComp
Wire Wire Line
	1250 1600 1150 1600
Wire Wire Line
	1150 1600 1150 1850
Wire Wire Line
	6000 2550 5900 2550
Wire Wire Line
	3150 4800 3150 4700
$Comp
L power:GND #PWR?
U 1 1 621E2094
P 3250 4800
F 0 "#PWR?" H 3250 4550 50  0001 C CNN
F 1 "GND" H 3255 4627 50  0000 C CNN
F 2 "" H 3250 4800 50  0001 C CNN
F 3 "" H 3250 4800 50  0001 C CNN
	1    3250 4800
	-1   0    0    -1  
$EndComp
Wire Wire Line
	3250 4700 3250 4800
$Comp
L power:GND #PWR?
U 1 1 621E738B
P 6000 2450
F 0 "#PWR?" H 6000 2200 50  0001 C CNN
F 1 "GND" H 6005 2277 50  0000 C CNN
F 2 "" H 6000 2450 50  0001 C CNN
F 3 "" H 6000 2450 50  0001 C CNN
	1    6000 2450
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 621EF66F
P 4350 4100
F 0 "#PWR?" H 4350 3850 50  0001 C CNN
F 1 "GND" H 4355 3927 50  0000 C CNN
F 2 "" H 4350 4100 50  0001 C CNN
F 3 "" H 4350 4100 50  0001 C CNN
	1    4350 4100
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 621F0037
P 4750 4100
F 0 "#PWR?" H 4750 3850 50  0001 C CNN
F 1 "GND" H 4755 3927 50  0000 C CNN
F 2 "" H 4750 4100 50  0001 C CNN
F 3 "" H 4750 4100 50  0001 C CNN
	1    4750 4100
	1    0    0    -1  
$EndComp
$Comp
L Noggin:MQ-9B U?
U 1 1 621F40D6
P 1750 6100
F 0 "U?" H 1750 6781 50  0000 C CNN
F 1 "MQ-9B" H 1750 6690 50  0000 C CNN
F 2 "" H 1750 6000 50  0001 C CNN
F 3 "" H 1750 6000 50  0001 C CNN
	1    1750 6100
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR?
U 1 1 621F7E8D
P 1550 5550
F 0 "#PWR?" H 1550 5400 50  0001 C CNN
F 1 "+5V" V 1565 5678 50  0000 L CNN
F 2 "" H 1550 5550 50  0001 C CNN
F 3 "" H 1550 5550 50  0001 C CNN
	1    1550 5550
	0    -1   -1   0   
$EndComp
Wire Wire Line
	1750 5550 1750 5600
Text Notes 1850 6550 1    50   ~ 0
29Ω±3Ω
Wire Wire Line
	1550 5550 1750 5550
$Comp
L Noggin:AO3400A Q?
U 1 1 6220A7E3
P 1450 7250
F 0 "Q?" H 1880 7396 50  0000 L CNN
F 1 "AO3400A" H 1880 7305 50  0000 L CNN
F 2 "SOT95P280X125-3N" H 1900 7200 50  0001 L CNN
F 3 "http://aosmd.com/res/data_sheets/AO3400A.pdf" H 1900 7100 50  0001 L CNN
F 4 "30V N-Channel MOSFET" H 1900 7000 50  0001 L CNN "Description"
F 5 "1.25" H 1900 6900 50  0001 L CNN "Height"
F 6 "Alpha & Omega Semiconductors" H 1900 6800 50  0001 L CNN "Manufacturer_Name"
F 7 "AO3400A" H 1900 6700 50  0001 L CNN "Manufacturer_Part_Number"
F 8 "" H 1900 6600 50  0001 L CNN "Mouser Part Number"
F 9 "" H 1900 6500 50  0001 L CNN "Mouser Price/Stock"
F 10 "AO3400A" H 1900 6400 50  0001 L CNN "Arrow Part Number"
F 11 "https://www.arrow.com/en/products/ao3400a/alpha-and-omega-semiconductor?region=nac" H 1900 6300 50  0001 L CNN "Arrow Price/Stock"
	1    1450 7250
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 6220BBEA
P 1750 7450
F 0 "#PWR?" H 1750 7200 50  0001 C CNN
F 1 "GND" H 1755 7277 50  0000 C CNN
F 2 "" H 1750 7450 50  0001 C CNN
F 3 "" H 1750 7450 50  0001 C CNN
	1    1750 7450
	1    0    0    -1  
$EndComp
Text GLabel 950  7250 0    50   Input ~ 0
GPIO25
$Comp
L Device:R R?
U 1 1 6220E3FD
P 1200 7250
F 0 "R?" V 993 7250 50  0000 C CNN
F 1 "100" V 1084 7250 50  0000 C CNN
F 2 "" V 1130 7250 50  0001 C CNN
F 3 "~" H 1200 7250 50  0001 C CNN
	1    1200 7250
	0    1    1    0   
$EndComp
$Comp
L Device:R R?
U 1 1 622115D7
P 2350 7000
F 0 "R?" H 2420 7046 50  0000 L CNN
F 1 "71.167" H 2420 6955 50  0000 L CNN
F 2 "" V 2280 7000 50  0001 C CNN
F 3 "~" H 2350 7000 50  0001 C CNN
	1    2350 7000
	1    0    0    -1  
$EndComp
Wire Wire Line
	1750 6650 1750 6750
Wire Wire Line
	1750 6750 2350 6750
Wire Wire Line
	2350 6750 2350 6850
Wire Wire Line
	1750 6750 1750 6850
Connection ~ 1750 6750
$Comp
L power:GND #PWR?
U 1 1 62218983
P 2350 7150
F 0 "#PWR?" H 2350 6900 50  0001 C CNN
F 1 "GND" H 2355 6977 50  0000 C CNN
F 2 "" H 2350 7150 50  0001 C CNN
F 3 "" H 2350 7150 50  0001 C CNN
	1    2350 7150
	1    0    0    -1  
$EndComp
Wire Wire Line
	1450 7250 1350 7250
Wire Wire Line
	1050 7250 950  7250
$Comp
L Device:R R?
U 1 1 621BECEF
P 4350 3550
F 0 "R?" H 4420 3596 50  0000 L CNN
F 1 "R" H 4420 3505 50  0000 L CNN
F 2 "" V 4280 3550 50  0001 C CNN
F 3 "~" H 4350 3550 50  0001 C CNN
	1    4350 3550
	-1   0    0    1   
$EndComp
$Comp
L Device:R R?
U 1 1 621C0204
P 4750 3550
F 0 "R?" H 4820 3596 50  0000 L CNN
F 1 "R" H 4820 3505 50  0000 L CNN
F 2 "" V 4680 3550 50  0001 C CNN
F 3 "~" H 4750 3550 50  0001 C CNN
	1    4750 3550
	-1   0    0    1   
$EndComp
Wire Wire Line
	4750 3800 4750 3700
Wire Wire Line
	4350 3700 4350 3800
Wire Wire Line
	4750 3400 4750 3350
Wire Wire Line
	4350 3250 4350 3400
Wire Wire Line
	1350 6050 1300 6050
Wire Wire Line
	1300 6050 1300 6200
Wire Wire Line
	1300 6200 1350 6200
Wire Wire Line
	2150 6200 2200 6200
Wire Wire Line
	2200 6200 2200 6050
Wire Wire Line
	2200 6050 2150 6050
Wire Wire Line
	1300 6050 1200 6050
Connection ~ 1300 6050
$Comp
L Device:R R?
U 1 1 621DA192
P 2500 5700
F 0 "R?" H 2570 5746 50  0000 L CNN
F 1 "1800" H 2570 5655 50  0000 L CNN
F 2 "" V 2430 5700 50  0001 C CNN
F 3 "~" H 2500 5700 50  0001 C CNN
	1    2500 5700
	1    0    0    -1  
$EndComp
$Comp
L Device:R R?
U 1 1 621DB81E
P 2500 6150
F 0 "R?" H 2570 6196 50  0000 L CNN
F 1 "2900" H 2570 6105 50  0000 L CNN
F 2 "" V 2430 6150 50  0001 C CNN
F 3 "~" H 2500 6150 50  0001 C CNN
	1    2500 6150
	1    0    0    -1  
$EndComp
Wire Wire Line
	2500 5850 2500 5900
Wire Wire Line
	2500 5500 2500 5550
$Comp
L power:+5V #PWR?
U 1 1 621ECBFC
P 1200 6050
F 0 "#PWR?" H 1200 5900 50  0001 C CNN
F 1 "+5V" V 1215 6178 50  0000 L CNN
F 2 "" H 1200 6050 50  0001 C CNN
F 3 "" H 1200 6050 50  0001 C CNN
	1    1200 6050
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 621EE04F
P 2500 6400
F 0 "#PWR?" H 2500 6150 50  0001 C CNN
F 1 "GND" H 2505 6227 50  0000 C CNN
F 2 "" H 2500 6400 50  0001 C CNN
F 3 "" H 2500 6400 50  0001 C CNN
	1    2500 6400
	1    0    0    -1  
$EndComp
Wire Wire Line
	2200 6050 2300 6050
Connection ~ 2200 6050
Wire Wire Line
	2500 6300 2500 6400
Wire Wire Line
	2500 5900 2850 5900
Connection ~ 2500 5900
Wire Wire Line
	2500 5900 2500 6000
Text GLabel 2850 5900 2    50   Input ~ 0
ADC1_CH5
Wire Wire Line
	2300 5500 2500 5500
Wire Wire Line
	2300 5500 2300 6050
$Comp
L Device:CP1 C?
U 1 1 62221CB4
P 2250 1000
F 0 "C?" V 2502 1000 50  0000 C CNN
F 1 "10uF" V 2411 1000 50  0000 C CNN
F 2 "" H 2250 1000 50  0001 C CNN
F 3 "~" H 2250 1000 50  0001 C CNN
	1    2250 1000
	0    -1   -1   0   
$EndComp
Wire Wire Line
	2000 900  2000 1000
Wire Wire Line
	2100 1000 2000 1000
Connection ~ 2000 1000
Wire Wire Line
	2000 1000 2000 1400
Text Notes 1550 5300 0    50   ~ 0
Gas Sensor
Text Notes 1300 1200 0    50   ~ 0
Power
$Comp
L 74xx:74HC14 U?
U 1 1 62236DAF
P 5150 5700
F 0 "U?" H 5150 6017 50  0000 C CNN
F 1 "74HC14" H 5150 5926 50  0000 C CNN
F 2 "" H 5150 5700 50  0001 C CNN
F 3 "http://www.ti.com/lit/gpn/sn74HC14" H 5150 5700 50  0001 C CNN
	1    5150 5700
	1    0    0    -1  
$EndComp
$Comp
L 74xx:74HC14 U?
U 2 1 6223B3CC
P 5150 6800
F 0 "U?" H 5150 7117 50  0000 C CNN
F 1 "74HC14" H 5150 7026 50  0000 C CNN
F 2 "" H 5150 6800 50  0001 C CNN
F 3 "http://www.ti.com/lit/gpn/sn74HC14" H 5150 6800 50  0001 C CNN
	2    5150 6800
	1    0    0    -1  
$EndComp
Text GLabel 5450 5700 2    50   Input ~ 0
GPIO5
Text GLabel 5450 6800 2    50   Input ~ 0
GPIO17
$Comp
L Device:CP1 C?
U 1 1 6223D7AE
P 4750 5950
F 0 "C?" V 5002 5950 50  0000 C CNN
F 1 "0.1uF" V 4911 5950 50  0000 C CNN
F 2 "" H 4750 5950 50  0001 C CNN
F 3 "~" H 4750 5950 50  0001 C CNN
	1    4750 5950
	1    0    0    -1  
$EndComp
$Comp
L Device:CP1 C?
U 1 1 6223FB3E
P 4750 7050
F 0 "C?" V 5002 7050 50  0000 C CNN
F 1 "0.1uF" V 4911 7050 50  0000 C CNN
F 2 "" H 4750 7050 50  0001 C CNN
F 3 "~" H 4750 7050 50  0001 C CNN
	1    4750 7050
	1    0    0    -1  
$EndComp
$Comp
L Device:R R?
U 1 1 62240379
P 4500 5700
F 0 "R?" H 4570 5746 50  0000 L CNN
F 1 "10k" H 4570 5655 50  0000 L CNN
F 2 "" V 4430 5700 50  0001 C CNN
F 3 "~" H 4500 5700 50  0001 C CNN
	1    4500 5700
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R?
U 1 1 6224144C
P 4500 6800
F 0 "R?" H 4570 6846 50  0000 L CNN
F 1 "10k" H 4570 6755 50  0000 L CNN
F 2 "" V 4430 6800 50  0001 C CNN
F 3 "~" H 4500 6800 50  0001 C CNN
	1    4500 6800
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R?
U 1 1 62241B22
P 4250 5450
F 0 "R?" H 4320 5496 50  0000 L CNN
F 1 "10k" H 4320 5405 50  0000 L CNN
F 2 "" V 4180 5450 50  0001 C CNN
F 3 "~" H 4250 5450 50  0001 C CNN
	1    4250 5450
	1    0    0    -1  
$EndComp
$Comp
L Device:R R?
U 1 1 622423BC
P 4250 6550
F 0 "R?" H 4320 6596 50  0000 L CNN
F 1 "10k" H 4320 6505 50  0000 L CNN
F 2 "" V 4180 6550 50  0001 C CNN
F 3 "~" H 4250 6550 50  0001 C CNN
	1    4250 6550
	1    0    0    -1  
$EndComp
$Comp
L Switch:SW_Push SW?
U 1 1 62246FBA
P 3950 6800
F 0 "SW?" H 3950 7085 50  0000 C CNN
F 1 "SW_Push" H 3950 6994 50  0000 C CNN
F 2 "" H 3950 7000 50  0001 C CNN
F 3 "~" H 3950 7000 50  0001 C CNN
	1    3950 6800
	1    0    0    -1  
$EndComp
$Comp
L Switch:SW_Push SW?
U 1 1 62248632
P 3950 5700
F 0 "SW?" H 3950 5985 50  0000 C CNN
F 1 "SW_Push" H 3950 5894 50  0000 C CNN
F 2 "" H 3950 5900 50  0001 C CNN
F 3 "~" H 3950 5900 50  0001 C CNN
	1    3950 5700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 62249770
P 3650 7200
F 0 "#PWR?" H 3650 6950 50  0001 C CNN
F 1 "GND" H 3655 7027 50  0000 C CNN
F 2 "" H 3650 7200 50  0001 C CNN
F 3 "" H 3650 7200 50  0001 C CNN
	1    3650 7200
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 62249FB3
P 3650 6100
F 0 "#PWR?" H 3650 5850 50  0001 C CNN
F 1 "GND" H 3655 5927 50  0000 C CNN
F 2 "" H 3650 6100 50  0001 C CNN
F 3 "" H 3650 6100 50  0001 C CNN
	1    3650 6100
	1    0    0    -1  
$EndComp
Wire Wire Line
	3650 5700 3750 5700
Wire Wire Line
	4150 5700 4250 5700
Wire Wire Line
	4250 5600 4250 5700
Connection ~ 4250 5700
Wire Wire Line
	4250 5700 4350 5700
Wire Wire Line
	4650 5700 4750 5700
Wire Wire Line
	4750 5800 4750 5700
Connection ~ 4750 5700
Wire Wire Line
	4750 5700 4850 5700
Wire Wire Line
	4150 6800 4250 6800
Wire Wire Line
	4250 6700 4250 6800
Connection ~ 4250 6800
Wire Wire Line
	4250 6800 4350 6800
Wire Wire Line
	3750 6800 3650 6800
Wire Wire Line
	4650 6800 4750 6800
Wire Wire Line
	4750 6900 4750 6800
Connection ~ 4750 6800
Wire Wire Line
	4750 6800 4850 6800
$Comp
L power:+3.3V #PWR?
U 1 1 6225590F
P 4250 5300
F 0 "#PWR?" H 4250 5150 50  0001 C CNN
F 1 "+3.3V" H 4265 5473 50  0000 C CNN
F 2 "" H 4250 5300 50  0001 C CNN
F 3 "" H 4250 5300 50  0001 C CNN
	1    4250 5300
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR?
U 1 1 622563CE
P 4250 6400
F 0 "#PWR?" H 4250 6250 50  0001 C CNN
F 1 "+3.3V" H 4265 6573 50  0000 C CNN
F 2 "" H 4250 6400 50  0001 C CNN
F 3 "" H 4250 6400 50  0001 C CNN
	1    4250 6400
	1    0    0    -1  
$EndComp
Text Notes 3950 5000 0    50   ~ 0
Button Debouncing
$Comp
L power:GND #PWR?
U 1 1 6226459A
P 4750 6100
F 0 "#PWR?" H 4750 5850 50  0001 C CNN
F 1 "GND" H 4755 5927 50  0000 C CNN
F 2 "" H 4750 6100 50  0001 C CNN
F 3 "" H 4750 6100 50  0001 C CNN
	1    4750 6100
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 62264AA7
P 4750 7200
F 0 "#PWR?" H 4750 6950 50  0001 C CNN
F 1 "GND" H 4755 7027 50  0000 C CNN
F 2 "" H 4750 7200 50  0001 C CNN
F 3 "" H 4750 7200 50  0001 C CNN
	1    4750 7200
	1    0    0    -1  
$EndComp
Wire Wire Line
	3650 5700 3650 6100
Wire Wire Line
	3650 6800 3650 7200
Text GLabel 6000 2650 0    50   Input ~ 0
EN
$Comp
L power:+3.3V #PWR?
U 1 1 62270593
P 3750 1200
F 0 "#PWR?" H 3750 1050 50  0001 C CNN
F 1 "+3.3V" H 3765 1373 50  0000 C CNN
F 2 "" H 3750 1200 50  0001 C CNN
F 3 "" H 3750 1200 50  0001 C CNN
	1    3750 1200
	1    0    0    -1  
$EndComp
$Comp
L Device:CP1 C?
U 1 1 62270C52
P 4150 1750
F 0 "C?" V 4402 1750 50  0000 C CNN
F 1 "1uF" V 4311 1750 50  0000 C CNN
F 2 "" H 4150 1750 50  0001 C CNN
F 3 "~" H 4150 1750 50  0001 C CNN
	1    4150 1750
	1    0    0    -1  
$EndComp
$Comp
L Switch:SW_SPST SW?
U 1 1 6227237A
P 3750 1800
F 0 "SW?" H 3750 2035 50  0000 C CNN
F 1 "SW_SPST" H 3750 1944 50  0000 C CNN
F 2 "" H 3750 1800 50  0001 C CNN
F 3 "~" H 3750 1800 50  0001 C CNN
	1    3750 1800
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 622738DC
P 3950 2100
F 0 "#PWR?" H 3950 1850 50  0001 C CNN
F 1 "GND" H 3955 1927 50  0000 C CNN
F 2 "" H 3950 2100 50  0001 C CNN
F 3 "" H 3950 2100 50  0001 C CNN
	1    3950 2100
	1    0    0    -1  
$EndComp
Wire Wire Line
	3750 1500 3750 1550
Wire Wire Line
	3750 1550 4150 1550
Wire Wire Line
	4150 1550 4150 1600
Connection ~ 3750 1550
Wire Wire Line
	3750 1550 3750 1600
Wire Wire Line
	3750 2000 3750 2050
Wire Wire Line
	3750 2050 3950 2050
Wire Wire Line
	3950 2050 3950 2100
Wire Wire Line
	4150 1900 4150 2050
Wire Wire Line
	4150 2050 3950 2050
Connection ~ 3950 2050
Text GLabel 4500 1550 2    50   Input ~ 0
EN
Wire Wire Line
	4150 1550 4500 1550
Connection ~ 4150 1550
$Comp
L Device:R R?
U 1 1 6226F706
P 3750 1350
F 0 "R?" H 3820 1396 50  0000 L CNN
F 1 "10k" H 3820 1305 50  0000 L CNN
F 2 "" V 3680 1350 50  0001 C CNN
F 3 "~" H 3750 1350 50  0001 C CNN
	1    3750 1350
	-1   0    0    1   
$EndComp
Text Notes 3950 850  0    50   ~ 0
Reset
$Comp
L Noggin:RFM9-LoRa-Radio U?
U 1 1 621C1E3B
P 10250 5650
F 0 "U?" V 9425 5600 50  0000 C CNN
F 1 "RFM9-LoRa-Radio" V 9516 5600 50  0000 C CNN
F 2 "" H 10250 5650 50  0001 C CNN
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
Text Notes 2100 2150 0    50   ~ 0
3.3V Step-Up Step-Down\nVoltage Regulator Module
Text GLabel 9900 5500 0    50   Input ~ 0
GPIO4
Text GLabel 9900 6250 0    50   Input ~ 0
GPIO16
$Comp
L power:+3.3V #PWR?
U 1 1 621C829E
P 9650 5050
F 0 "#PWR?" H 9650 4900 50  0001 C CNN
F 1 "+3.3V" H 9665 5223 50  0000 C CNN
F 2 "" H 9650 5050 50  0001 C CNN
F 3 "" H 9650 5050 50  0001 C CNN
	1    9650 5050
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 621C99CC
P 9800 5200
F 0 "#PWR?" H 9800 4950 50  0001 C CNN
F 1 "GND" H 9805 5027 50  0000 C CNN
F 2 "" H 9800 5200 50  0001 C CNN
F 3 "" H 9800 5200 50  0001 C CNN
	1    9800 5200
	0    1    -1   0   
$EndComp
Wire Wire Line
	9900 5050 9650 5050
Wire Wire Line
	9900 5200 9800 5200
$Comp
L Noggin:U1V10F5 VR2
U 1 1 621A437E
P 1050 3900
F 0 "VR2" V 1350 4000 50  0000 C CNN
F 1 "U1V10F5" V 1250 3900 50  0000 C CNN
F 2 "" H 1050 3650 50  0001 C CNN
F 3 "" H 1050 3650 50  0001 C CNN
	1    1050 3900
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 621B9659
P 1250 3900
F 0 "#PWR?" H 1250 3650 50  0001 C CNN
F 1 "GND" H 1255 3727 50  0000 C CNN
F 2 "" H 1250 3900 50  0001 C CNN
F 3 "" H 1250 3900 50  0001 C CNN
	1    1250 3900
	0    -1   -1   0   
$EndComp
$Comp
L power:+3.3V #PWR?
U 1 1 621B9FBC
P 1350 3700
F 0 "#PWR?" H 1350 3550 50  0001 C CNN
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
L power:+5V #PWR?
U 1 1 621BF0C8
P 1350 4100
F 0 "#PWR?" H 1350 3950 50  0001 C CNN
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
Text GLabel 2750 4800 0    50   Input ~ 0
GPIO26
$Comp
L Noggin:LSM6DSLTR U?
U 1 1 621EF8C3
P 8500 1100
F 0 "U?" H 9400 1487 60  0000 C CNN
F 1 "LSM6DSLTR" H 9400 1381 60  0000 C CNN
F 2 "QFN_6DSLTR_STM" H 9400 1340 60  0001 C CNN
F 3 "" H 8500 1100 60  0000 C CNN
	1    8500 1100
	1    0    0    -1  
$EndComp
Text GLabel 8500 1400 0    50   Input ~ 0
GPIO18
$Comp
L power:GND #PWR?
U 1 1 621C86E4
P 8200 1800
F 0 "#PWR?" H 8200 1550 50  0001 C CNN
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
L power:+3.3V #PWR?
U 1 1 621D8CE0
P 10850 1700
F 0 "#PWR?" H 10850 1550 50  0001 C CNN
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
L Device:C_Small C?
U 1 1 621FE0DC
P 10700 1900
F 0 "C?" H 10792 1946 50  0000 L CNN
F 1 "100 nF" H 10792 1855 50  0000 L CNN
F 2 "" H 10700 1900 50  0001 C CNN
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
L power:GND #PWR?
U 1 1 62200D10
P 10700 2000
F 0 "#PWR?" H 10700 1750 50  0001 C CNN
F 1 "GND" H 10705 1827 50  0000 C CNN
F 2 "" H 10700 2000 50  0001 C CNN
F 3 "" H 10700 2000 50  0001 C CNN
	1    10700 2000
	-1   0    0    -1  
$EndComp
$Comp
L Device:C_Small C?
U 1 1 62203F11
P 7800 1650
F 0 "C?" H 7600 1700 50  0000 L CNN
F 1 "100 nF" H 7400 1600 50  0000 L CNN
F 2 "" H 7800 1650 50  0001 C CNN
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
L Device:R R?
U 1 1 6220D6DC
P 10400 950
F 0 "R?" H 10470 996 50  0000 L CNN
F 1 "10k" H 10470 905 50  0000 L CNN
F 2 "" V 10330 950 50  0001 C CNN
F 3 "~" H 10400 950 50  0001 C CNN
	1    10400 950 
	1    0    0    -1  
$EndComp
$Comp
L Device:R R?
U 1 1 62212F2D
P 10800 1050
F 0 "R?" H 10870 1096 50  0000 L CNN
F 1 "10k" H 10870 1005 50  0000 L CNN
F 2 "" V 10730 1050 50  0001 C CNN
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
Text Notes 3000 3450 0    50   ~ 0
Buzzer
Text Notes 4450 3100 0    50   ~ 0
LEDs
Text GLabel 6000 3250 0    50   Input ~ 0
ADC1_CH5
$Comp
L Device:R R?
U 1 1 6223F31D
P 1750 2200
F 0 "R?" H 1820 2246 50  0000 L CNN
F 1 "22k" H 1820 2155 50  0000 L CNN
F 2 "" V 1680 2200 50  0001 C CNN
F 3 "~" H 1750 2200 50  0001 C CNN
	1    1750 2200
	-1   0    0    1   
$EndComp
$Comp
L Device:R R?
U 1 1 62242505
P 1750 2700
F 0 "R?" H 1820 2746 50  0000 L CNN
F 1 "68k" H 1820 2655 50  0000 L CNN
F 2 "" V 1680 2700 50  0001 C CNN
F 3 "~" H 1750 2700 50  0001 C CNN
	1    1750 2700
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 62242CEE
P 1750 2850
F 0 "#PWR?" H 1750 2600 50  0001 C CNN
F 1 "GND" H 1755 2677 50  0000 C CNN
F 2 "" H 1750 2850 50  0001 C CNN
F 3 "" H 1750 2850 50  0001 C CNN
	1    1750 2850
	1    0    0    -1  
$EndComp
Wire Wire Line
	1750 2350 1750 2450
Wire Wire Line
	1750 2450 1800 2450
Connection ~ 1750 2450
Wire Wire Line
	1750 2450 1750 2550
Text GLabel 1800 2450 2    50   Input ~ 0
ADC1_CH6
Wire Wire Line
	1650 1600 1750 1600
Wire Wire Line
	1750 1600 1750 2050
Connection ~ 1750 1600
Wire Wire Line
	1750 1600 2100 1600
Text Notes 950  3350 0    50   ~ 0
5V Power
Text Notes 700  4500 0    50   ~ 0
5V Step-Up\nVoltage Regulator Module
Text GLabel 10450 1100 2    50   Input ~ 0
SDA
$Comp
L Noggin:SEN-11574 U?
U 1 1 62262824
P 9450 4050
F 0 "U?" H 10340 4096 50  0000 L CNN
F 1 "SEN-11574" H 10340 4005 50  0000 L CNN
F 2 "" H 9450 4000 50  0001 C CNN
F 3 "" H 9450 4000 50  0001 C CNN
	1    9450 4050
	1    0    0    -1  
$EndComp
Text Notes 9550 3650 0    50   ~ 0
Pulse Sensor
$Comp
L power:GND #PWR?
U 1 1 62268C21
P 9050 3900
F 0 "#PWR?" H 9050 3650 50  0001 C CNN
F 1 "GND" H 9055 3727 50  0000 C CNN
F 2 "" H 9050 3900 50  0001 C CNN
F 3 "" H 9050 3900 50  0001 C CNN
	1    9050 3900
	0    1    1    0   
$EndComp
$Comp
L power:+3.3V #PWR?
U 1 1 622698BE
P 9050 4200
F 0 "#PWR?" H 9050 4050 50  0001 C CNN
F 1 "+3.3V" H 9065 4373 50  0000 C CNN
F 2 "" H 9050 4200 50  0001 C CNN
F 3 "" H 9050 4200 50  0001 C CNN
	1    9050 4200
	0    -1   -1   0   
$EndComp
Wire Wire Line
	9050 4050 8750 4050
Text GLabel 6000 3150 0    50   Input ~ 0
ADC1_CH4
Text GLabel 8750 4050 0    50   Input ~ 0
ADC1_CH4
$Comp
L power:GND #PWR?
U 1 1 6226E33E
P 7550 4150
F 0 "#PWR?" H 7550 3900 50  0001 C CNN
F 1 "GND" H 7555 3977 50  0000 C CNN
F 2 "" H 7550 4150 50  0001 C CNN
F 3 "" H 7550 4150 50  0001 C CNN
	1    7550 4150
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 6226EBB4
P 7400 4250
F 0 "#PWR?" H 7400 4000 50  0001 C CNN
F 1 "GND" H 7405 4077 50  0000 C CNN
F 2 "" H 7400 4250 50  0001 C CNN
F 3 "" H 7400 4250 50  0001 C CNN
	1    7400 4250
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 6226ED9D
P 6000 3850
F 0 "#PWR?" H 6000 3600 50  0001 C CNN
F 1 "GND" H 6005 3677 50  0000 C CNN
F 2 "" H 6000 3850 50  0001 C CNN
F 3 "" H 6000 3850 50  0001 C CNN
	1    6000 3850
	0    1    1    0   
$EndComp
Text Notes 1900 2700 0    50   ~ 0
Battery\nMonitoring
Text GLabel 6000 3450 0    50   Input ~ 0
GPIO26
Text GLabel 4750 3350 2    50   Input ~ 0
GPIO27
Text GLabel 6000 3550 0    50   Input ~ 0
GPIO27
Text GLabel 4350 3250 2    50   Input ~ 0
GPIO14
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
$Comp
L power:GND #PWR?
U 1 1 622272CD
P 2400 1000
F 0 "#PWR?" H 2400 750 50  0001 C CNN
F 1 "GND" H 2405 827 50  0000 C CNN
F 2 "" H 2400 1000 50  0001 C CNN
F 3 "" H 2400 1000 50  0001 C CNN
	1    2400 1000
	0    -1   -1   0   
$EndComp
Text GLabel 8500 1100 0    50   Input ~ 0
SDO
Text GLabel 7400 4050 2    50   Input ~ 0
SDO
Wire Wire Line
	7400 4150 7550 4150
Text Notes 7850 2500 0    50   ~ 0
(unsure)
$Comp
L 74xx:74HC14 U?
U 7 1 62296A70
P 6450 5250
F 0 "U?" V 6083 5250 50  0000 C CNN
F 1 "74HC14" V 6174 5250 50  0000 C CNN
F 2 "" H 6450 5250 50  0001 C CNN
F 3 "http://www.ti.com/lit/gpn/sn74HC14" H 6450 5250 50  0001 C CNN
	7    6450 5250
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 6229B493
P 5950 5250
F 0 "#PWR?" H 5950 5000 50  0001 C CNN
F 1 "GND" H 5955 5077 50  0000 C CNN
F 2 "" H 5950 5250 50  0001 C CNN
F 3 "" H 5950 5250 50  0001 C CNN
	1    5950 5250
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR?
U 1 1 6229BD9A
P 6950 5250
F 0 "#PWR?" H 6950 5100 50  0001 C CNN
F 1 "+3.3V" H 6965 5423 50  0000 C CNN
F 2 "" H 6950 5250 50  0001 C CNN
F 3 "" H 6950 5250 50  0001 C CNN
	1    6950 5250
	1    0    0    -1  
$EndComp
$Comp
L 74xx:74HC14 U?
U 3 1 6229D1FF
P 6250 5850
F 0 "U?" H 6250 6167 50  0000 C CNN
F 1 "74HC14" H 6250 6076 50  0000 C CNN
F 2 "" H 6250 5850 50  0001 C CNN
F 3 "http://www.ti.com/lit/gpn/sn74HC14" H 6250 5850 50  0001 C CNN
	3    6250 5850
	1    0    0    -1  
$EndComp
$Comp
L 74xx:74HC14 U?
U 4 1 6229FE38
P 6250 6350
F 0 "U?" H 6250 6667 50  0000 C CNN
F 1 "74HC14" H 6250 6576 50  0000 C CNN
F 2 "" H 6250 6350 50  0001 C CNN
F 3 "http://www.ti.com/lit/gpn/sn74HC14" H 6250 6350 50  0001 C CNN
	4    6250 6350
	1    0    0    -1  
$EndComp
$Comp
L 74xx:74HC14 U?
U 5 1 622C073F
P 7050 5850
F 0 "U?" H 7050 6167 50  0000 C CNN
F 1 "74HC14" H 7050 6076 50  0000 C CNN
F 2 "" H 7050 5850 50  0001 C CNN
F 3 "http://www.ti.com/lit/gpn/sn74HC14" H 7050 5850 50  0001 C CNN
	5    7050 5850
	1    0    0    -1  
$EndComp
$Comp
L 74xx:74HC14 U?
U 6 1 622C2769
P 7050 6350
F 0 "U?" H 7050 6667 50  0000 C CNN
F 1 "74HC14" H 7050 6576 50  0000 C CNN
F 2 "" H 7050 6350 50  0001 C CNN
F 3 "http://www.ti.com/lit/gpn/sn74HC14" H 7050 6350 50  0001 C CNN
	6    7050 6350
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 622C4BD0
P 5950 5850
F 0 "#PWR?" H 5950 5600 50  0001 C CNN
F 1 "GND" H 5955 5677 50  0000 C CNN
F 2 "" H 5950 5850 50  0001 C CNN
F 3 "" H 5950 5850 50  0001 C CNN
	1    5950 5850
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 622C5CDE
P 5950 6350
F 0 "#PWR?" H 5950 6100 50  0001 C CNN
F 1 "GND" H 5955 6177 50  0000 C CNN
F 2 "" H 5950 6350 50  0001 C CNN
F 3 "" H 5950 6350 50  0001 C CNN
	1    5950 6350
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 622C61C1
P 6750 6350
F 0 "#PWR?" H 6750 6100 50  0001 C CNN
F 1 "GND" H 6755 6177 50  0000 C CNN
F 2 "" H 6750 6350 50  0001 C CNN
F 3 "" H 6750 6350 50  0001 C CNN
	1    6750 6350
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 622C6842
P 6750 5850
F 0 "#PWR?" H 6750 5600 50  0001 C CNN
F 1 "GND" H 6755 5677 50  0000 C CNN
F 2 "" H 6750 5850 50  0001 C CNN
F 3 "" H 6750 5850 50  0001 C CNN
	1    6750 5850
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
	3450 750  4700 750 
Wire Notes Line
	4700 750  4700 2350
Wire Notes Line
	4700 2350 3450 2350
Wire Notes Line
	3450 2350 3450 750 
Wire Notes Line
	1750 4550 1750 3250
Wire Notes Line
	1750 3250 550  3250
Wire Notes Line
	550  3250 550  4550
Wire Notes Line
	550  4550 1750 4550
Wire Notes Line
	500  7750 3300 7750
Wire Notes Line
	3300 7750 3300 5150
Wire Notes Line
	3300 5150 550  5150
Wire Notes Line
	550  5150 550  7750
Wire Notes Line
	3450 5050 3450 3300
Wire Notes Line
	3450 3300 2350 3300
Wire Notes Line
	2350 3300 2350 5050
Wire Notes Line
	2350 5050 3450 5050
Wire Notes Line
	4050 4400 5200 4400
Wire Notes Line
	5200 4400 5200 3000
Wire Notes Line
	5200 3000 4050 3000
Wire Notes Line
	4050 3000 4050 4400
Wire Notes Line
	3450 7500 6950 7500
Wire Notes Line
	6950 7500 6950 6500
Wire Notes Line
	6950 6500 7450 6500
Wire Notes Line
	7450 6500 7450 4800
Wire Notes Line
	7450 4800 3500 4800
Wire Notes Line
	3500 4800 3500 7500
Wire Notes Line
	10900 3500 10900 4350
Wire Notes Line
	10900 4350 8250 4350
Wire Notes Line
	8250 4350 8250 3500
Wire Notes Line
	8250 3500 10900 3500
Wire Notes Line
	10750 4550 9300 4550
Wire Notes Line
	9300 4550 9300 6400
Wire Notes Line
	9300 6400 10750 6400
Wire Notes Line
	10750 6400 10750 4550
Wire Notes Line
	11150 550  7250 550 
Wire Notes Line
	7250 550  7250 1900
Wire Notes Line
	7250 1900 8000 1900
Wire Notes Line
	8000 1900 8000 2300
Wire Notes Line
	8000 2300 11150 2300
Wire Notes Line
	11150 2300 11150 550 
$Comp
L Connector_Generic:Conn_01x06 J?
U 1 1 6232A885
P 5950 1400
F 0 "J?" H 6030 1392 50  0000 L CNN
F 1 "Conn_01x06" H 6030 1301 50  0000 L CNN
F 2 "" H 5950 1400 50  0001 C CNN
F 3 "~" H 5950 1400 50  0001 C CNN
	1    5950 1400
	1    0    0    -1  
$EndComp
Text Notes 5350 1050 0    50   ~ 0
ESPFlasher Programming Header
$Comp
L power:GND #PWR?
U 1 1 6232C84D
P 5600 1200
F 0 "#PWR?" H 5600 950 50  0001 C CNN
F 1 "GND" H 5605 1027 50  0000 C CNN
F 2 "" H 5600 1200 50  0001 C CNN
F 3 "" H 5600 1200 50  0001 C CNN
	1    5600 1200
	0    1    1    0   
$EndComp
Text GLabel 7400 3850 2    50   Input ~ 0
TX
Text GLabel 5750 1300 0    50   Input ~ 0
TX
Text GLabel 7400 3750 2    50   Input ~ 0
RX
Text GLabel 5750 1400 0    50   Input ~ 0
RX
$Comp
L power:+3.3V #PWR?
U 1 1 6232EEA6
P 5600 1500
F 0 "#PWR?" H 5600 1350 50  0001 C CNN
F 1 "+3.3V" H 5615 1673 50  0000 C CNN
F 2 "" H 5600 1500 50  0001 C CNN
F 3 "" H 5600 1500 50  0001 C CNN
	1    5600 1500
	0    -1   -1   0   
$EndComp
Text GLabel 5750 1600 0    50   Input ~ 0
EN
Text GLabel 7400 2850 2    50   Input ~ 0
GPIO0
Text GLabel 5750 1700 0    50   Input ~ 0
GPIO0
Wire Wire Line
	5600 1200 5750 1200
Wire Wire Line
	5600 1500 5750 1500
Wire Notes Line
	5300 950  6600 950 
Wire Notes Line
	6600 950  6600 1850
Wire Notes Line
	6600 1850 5300 1850
Wire Notes Line
	5300 1850 5300 950 
$EndSCHEMATC
