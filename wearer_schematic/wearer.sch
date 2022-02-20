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
P -1300 1400
F 0 "OpAmp1" H -956 1446 50  0001 L CNN
F 1 "OPAMP" H -956 1400 50  0000 L CNN
F 2 "" H -1300 1400 50  0001 C CNN
F 3 "~" H -1300 1400 50  0001 C CNN
	1    -1300 1400
	1    0    0    -1  
$EndComp
Wire Wire Line
	-1600 1300 -2050 1300
Text Label -2050 1300 0    50   ~ 0
3V
$Comp
L pspice:R R1
U 1 1 61FD5327
P -1550 2450
F 0 "R1" H -1482 2496 50  0000 L CNN
F 1 "200" H -1482 2405 50  0000 L CNN
F 2 "" H -1550 2450 50  0001 C CNN
F 3 "~" H -1550 2450 50  0001 C CNN
	1    -1550 2450
	1    0    0    -1  
$EndComp
Text GLabel -1550 2200 0    50   Input ~ 0
Vin=5V
Wire Wire Line
	-1550 2700 -1550 2750
Connection ~ -1550 2750
Wire Wire Line
	-1550 2750 -1550 2850
Wire Wire Line
	-1550 2750 -1200 2750
Text GLabel -1200 2750 2    50   Input ~ 0
Vout=3V
$Comp
L pspice:R R2
U 1 1 61FD5B76
P -1550 3100
F 0 "R2" H -1482 3146 50  0000 L CNN
F 1 "300" H -1482 3055 50  0000 L CNN
F 2 "" H -1550 3100 50  0001 C CNN
F 3 "~" H -1550 3100 50  0001 C CNN
	1    -1550 3100
	1    0    0    -1  
$EndComp
$Comp
L pspice:0 GND
U 1 1 61FD6CC4
P -1550 3400
F 0 "GND" H -1472 3383 50  0000 L CNN
F 1 "0" H -1550 3489 50  0001 C CNN
F 2 "" H -1550 3400 50  0001 C CNN
F 3 "~" H -1550 3400 50  0001 C CNN
	1    -1550 3400
	1    0    0    -1  
$EndComp
$Comp
L Noggin:ESP32-WROOM-32D ANT?
U 1 1 620D2920
P 6150 2450
F 0 "ANT?" H 6850 2715 50  0000 C CNN
F 1 "ESP32-WROOM-32D" H 6850 2624 50  0000 C CNN
F 2 "ESP32WROOM32D" H 7400 2550 50  0001 L CNN
F 3 "https://www.mouser.com/datasheet/2/891/esp32-wroom-32d_esp32-wroom-32u_datasheet_en-1365844.pdf" H 7400 2450 50  0001 L CNN
F 4 "WiFi Modules (802.11) SMD Module, ESP32-D0WD, 32Mbits SPI flash, UART mode, PCB antenna" H 7400 2350 50  0001 L CNN "Description"
F 5 "3.1" H 7400 2250 50  0001 L CNN "Height"
F 6 "Espressif Systems" H 7400 2150 50  0001 L CNN "Manufacturer_Name"
F 7 "ESP32-WROOM-32D" H 7400 2050 50  0001 L CNN "Manufacturer_Part_Number"
F 8 "N/A" H 7400 1950 50  0001 L CNN "Mouser Part Number"
F 9 "https://www.mouser.co.uk/ProductDetail/Espressif-Systems/ESP32-WROOM-32D?qs=MLItCLRbWszx2KabkKPu5A%3D%3D" H 7400 1850 50  0001 L CNN "Mouser Price/Stock"
	1    6150 2450
	1    0    0    -1  
$EndComp
$Comp
L Noggin:SEN-11574 U?
U 1 1 620D4740
P 8750 2650
F 0 "U?" H 9640 2696 50  0000 L CNN
F 1 "SEN-11574" H 9640 2605 50  0000 L CNN
F 2 "" H 8750 2600 50  0001 C CNN
F 3 "" H 8750 2600 50  0001 C CNN
	1    8750 2650
	1    0    0    -1  
$EndComp
$Comp
L Device:LED D?
U 1 1 620D5497
P 5450 3450
F 0 "D?" H 5443 3666 50  0000 C CNN
F 1 "LED" H 5443 3575 50  0000 C CNN
F 2 "" H 5450 3450 50  0001 C CNN
F 3 "~" H 5450 3450 50  0001 C CNN
	1    5450 3450
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 620D6E37
P 7950 1900
F 0 "#PWR?" H 7950 1650 50  0001 C CNN
F 1 "GND" H 7955 1727 50  0000 C CNN
F 2 "" H 7950 1900 50  0001 C CNN
F 3 "" H 7950 1900 50  0001 C CNN
	1    7950 1900
	-1   0    0    1   
$EndComp
$Comp
L power:+3.3V #PWR?
U 1 1 620D7784
P 5150 2150
F 0 "#PWR?" H 5150 2000 50  0001 C CNN
F 1 "+3.3V" H 5165 2323 50  0000 C CNN
F 2 "" H 5150 2150 50  0001 C CNN
F 3 "" H 5150 2150 50  0001 C CNN
	1    5150 2150
	1    0    0    -1  
$EndComp
Wire Wire Line
	6150 2550 5150 2550
Wire Wire Line
	5150 2550 5150 2250
Wire Wire Line
	6150 3450 5600 3450
$Comp
L Device:LED D?
U 1 1 620D647E
P 5450 3100
F 0 "D?" H 5443 3316 50  0000 C CNN
F 1 "LED" H 5443 3225 50  0000 C CNN
F 2 "" H 5450 3100 50  0001 C CNN
F 3 "~" H 5450 3100 50  0001 C CNN
	1    5450 3100
	1    0    0    -1  
$EndComp
Wire Wire Line
	6150 3350 5600 3350
Wire Wire Line
	5600 3350 5600 3100
Wire Wire Line
	5300 3100 4850 3100
Connection ~ 4850 3100
Wire Wire Line
	4850 3100 4850 3450
Wire Wire Line
	5300 3450 4850 3450
Wire Wire Line
	7550 2650 8350 2650
Wire Wire Line
	8350 2500 8050 2500
Wire Wire Line
	8350 2800 7750 2800
Wire Wire Line
	7750 2800 7750 2250
Wire Wire Line
	7750 2250 5150 2250
Connection ~ 5150 2250
Wire Wire Line
	5150 2250 5150 2150
Wire Wire Line
	8050 1900 7950 1900
Wire Wire Line
	4850 1900 7950 1900
Wire Wire Line
	4850 1900 4850 3100
Connection ~ 7950 1900
Text Notes 7150 5050 0    50   ~ 0
Button Debouncing\n
$Comp
L power:+3.3V #PWR?
U 1 1 62104884
P 6600 4900
F 0 "#PWR?" H 6600 4750 50  0001 C CNN
F 1 "+3.3V" H 6615 5073 50  0000 C CNN
F 2 "" H 6600 4900 50  0001 C CNN
F 3 "" H 6600 4900 50  0001 C CNN
	1    6600 4900
	1    0    0    -1  
$EndComp
$Comp
L 74xx:74HC14 U?
U 1 1 6210488A
P 5900 5550
F 0 "U?" H 5900 5233 50  0000 C CNN
F 1 "74HC14" H 5900 5324 50  0000 C CNN
F 2 "" H 5900 5550 50  0001 C CNN
F 3 "http://www.ti.com/lit/gpn/sn74HC14" H 5900 5550 50  0001 C CNN
	1    5900 5550
	-1   0    0    1   
$EndComp
Text Notes 5650 5850 0    50   ~ 0
Schmitt Trigger\n
$Comp
L Device:R R?
U 1 1 62104891
P 6600 5050
F 0 "R?" H 6670 5096 50  0000 L CNN
F 1 "R" H 6670 5005 50  0000 L CNN
F 2 "" V 6530 5050 50  0001 C CNN
F 3 "~" H 6600 5050 50  0001 C CNN
	1    6600 5050
	1    0    0    -1  
$EndComp
$Comp
L Device:R R?
U 1 1 62104897
P 7050 5550
F 0 "R?" V 6843 5550 50  0000 C CNN
F 1 "R" V 6934 5550 50  0000 C CNN
F 2 "" V 6980 5550 50  0001 C CNN
F 3 "~" H 7050 5550 50  0001 C CNN
	1    7050 5550
	0    1    1    0   
$EndComp
$Comp
L Switch:SW_MEC_5G SW?
U 1 1 6210489D
P 7700 5550
F 0 "SW?" H 7700 5835 50  0000 C CNN
F 1 "SW_MEC_5G" H 7700 5744 50  0000 C CNN
F 2 "" H 7700 5750 50  0001 C CNN
F 3 "http://www.apem.com/int/index.php?controller=attachment&id_attachment=488" H 7700 5750 50  0001 C CNN
	1    7700 5550
	1    0    0    -1  
$EndComp
$Comp
L Device:CP1 C?
U 1 1 621048A3
P 7400 5950
F 0 "C?" V 7652 5950 50  0000 C CNN
F 1 "CP1" V 7561 5950 50  0000 C CNN
F 2 "" H 7400 5950 50  0001 C CNN
F 3 "~" H 7400 5950 50  0001 C CNN
	1    7400 5950
	0    -1   -1   0   
$EndComp
$Comp
L power:GNDREF #PWR?
U 1 1 621048A9
P 8350 5950
F 0 "#PWR?" H 8350 5700 50  0001 C CNN
F 1 "GNDREF" H 8355 5777 50  0000 C CNN
F 2 "" H 8350 5950 50  0001 C CNN
F 3 "" H 8350 5950 50  0001 C CNN
	1    8350 5950
	1    0    0    -1  
$EndComp
Wire Wire Line
	6600 5200 6600 5550
Wire Wire Line
	6600 5550 6200 5550
Wire Wire Line
	6600 5550 6800 5550
Connection ~ 6600 5550
Wire Wire Line
	7200 5550 7500 5550
Wire Wire Line
	7900 5550 8000 5550
Wire Wire Line
	8350 5550 8350 5950
Wire Wire Line
	7550 5950 8000 5950
Wire Wire Line
	7250 5950 6800 5950
Wire Wire Line
	6800 5950 6800 5550
Connection ~ 6800 5550
Wire Wire Line
	6800 5550 6900 5550
Wire Wire Line
	8000 5950 8000 5550
Connection ~ 8000 5550
Wire Wire Line
	8000 5550 8350 5550
Wire Wire Line
	5600 5550 5600 3550
Wire Wire Line
	5600 3550 6150 3550
$Comp
L Noggin:PS1240P02BT LS?
U 1 1 621084ED
P 8500 3100
F 0 "LS?" H 9128 3096 50  0000 L CNN
F 1 "PS1240P02BT" H 9128 3005 50  0000 L CNN
F 2 "PS1240P02BT" H 9150 3200 50  0001 L CNN
F 3 "https://product.tdk.com/system/files/dam/doc/product/sw_piezo/sw_piezo/piezo-buzzer/catalog/piezoelectronic_buzzer_ps_en.pdf" H 9150 3100 50  0001 L CNN
F 4 "Piezoelectric Buzzers, Sound Pressure Level=70dB min, Freq.=4000Hz nom" H 9150 3000 50  0001 L CNN "Description"
F 5 "TDK" H 9150 2800 50  0001 L CNN "Manufacturer_Name"
F 6 "PS1240P02BT" H 9150 2700 50  0001 L CNN "Manufacturer_Part_Number"
F 7 "810-PS1240P02BT" H 9150 2600 50  0001 L CNN "Mouser Part Number"
F 8 "https://www.mouser.co.uk/ProductDetail/TDK/PS1240P02BT?qs=d7g9p1yFhWaZXSY9MjKMkw%3D%3D" H 9150 2500 50  0001 L CNN "Mouser Price/Stock"
F 9 "PS1240P02BT" H 9150 2400 50  0001 L CNN "Arrow Part Number"
F 10 "https://www.arrow.com/en/products/ps1240p02bt/tdk?region=nac" H 9150 2300 50  0001 L CNN "Arrow Price/Stock"
	1    8500 3100
	1    0    0    -1  
$EndComp
Wire Wire Line
	8500 3200 8050 3200
Wire Wire Line
	8050 1900 8050 2500
Connection ~ 8050 2500
Wire Wire Line
	8050 2500 8050 3200
$Comp
L Device:R R?
U 1 1 6210A8A7
P 7850 3050
F 0 "R?" V 7643 3050 50  0000 C CNN
F 1 "1K" V 7734 3050 50  0000 C CNN
F 2 "" V 7780 3050 50  0001 C CNN
F 3 "~" H 7850 3050 50  0001 C CNN
	1    7850 3050
	0    1    1    0   
$EndComp
Wire Wire Line
	7550 3050 7700 3050
Wire Wire Line
	8000 3050 8500 3050
Wire Wire Line
	8500 3050 8500 3100
Text Notes 2450 2450 0    50   ~ 0
Supervisor\n
Text Notes 6700 1700 0    50   ~ 0
Wearer\n
$Comp
L Device:LED D?
U 1 1 62113A04
P 1500 3500
F 0 "D?" H 1493 3716 50  0000 C CNN
F 1 "LED" H 1493 3625 50  0000 C CNN
F 2 "" H 1500 3500 50  0001 C CNN
F 3 "~" H 1500 3500 50  0001 C CNN
	1    1500 3500
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR?
U 1 1 6211423A
P 1550 2650
F 0 "#PWR?" H 1550 2500 50  0001 C CNN
F 1 "+3.3V" H 1565 2823 50  0000 C CNN
F 2 "" H 1550 2650 50  0001 C CNN
F 3 "" H 1550 2650 50  0001 C CNN
	1    1550 2650
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 62114AE7
P 1850 2350
F 0 "#PWR?" H 1850 2100 50  0001 C CNN
F 1 "GND" H 1855 2177 50  0000 C CNN
F 2 "" H 1850 2350 50  0001 C CNN
F 3 "" H 1850 2350 50  0001 C CNN
	1    1850 2350
	-1   0    0    1   
$EndComp
Wire Wire Line
	1900 3000 1850 3000
Wire Wire Line
	1900 3100 1550 3100
Wire Wire Line
	1550 3100 1550 2650
Wire Wire Line
	1900 3500 1650 3500
Wire Wire Line
	1850 2350 1850 2450
Wire Wire Line
	1350 2450 1850 2450
Connection ~ 1850 2450
Wire Wire Line
	1850 2450 1850 3000
Wire Wire Line
	1350 2450 1350 3500
$Comp
L Noggin:ESP32-WROOM-32D ANT?
U 1 1 62112557
P 1900 3000
F 0 "ANT?" H 2600 3265 50  0000 C CNN
F 1 "ESP32-WROOM-32D" H 2600 3174 50  0000 C CNN
F 2 "ESP32WROOM32D" H 3150 3100 50  0001 L CNN
F 3 "https://www.mouser.com/datasheet/2/891/esp32-wroom-32d_esp32-wroom-32u_datasheet_en-1365844.pdf" H 3150 3000 50  0001 L CNN
F 4 "WiFi Modules (802.11) SMD Module, ESP32-D0WD, 32Mbits SPI flash, UART mode, PCB antenna" H 3150 2900 50  0001 L CNN "Description"
F 5 "3.1" H 3150 2800 50  0001 L CNN "Height"
F 6 "Espressif Systems" H 3150 2700 50  0001 L CNN "Manufacturer_Name"
F 7 "ESP32-WROOM-32D" H 3150 2600 50  0001 L CNN "Manufacturer_Part_Number"
F 8 "N/A" H 3150 2500 50  0001 L CNN "Mouser Part Number"
F 9 "https://www.mouser.co.uk/ProductDetail/Espressif-Systems/ESP32-WROOM-32D?qs=MLItCLRbWszx2KabkKPu5A%3D%3D" H 3150 2400 50  0001 L CNN "Mouser Price/Stock"
	1    1900 3000
	1    0    0    -1  
$EndComp
Text Notes 2900 5550 0    50   ~ 0
Button Debouncing\n
$Comp
L power:+3.3V #PWR?
U 1 1 6213F0C2
P 2350 5400
F 0 "#PWR?" H 2350 5250 50  0001 C CNN
F 1 "+3.3V" H 2365 5573 50  0000 C CNN
F 2 "" H 2350 5400 50  0001 C CNN
F 3 "" H 2350 5400 50  0001 C CNN
	1    2350 5400
	1    0    0    -1  
$EndComp
$Comp
L 74xx:74HC14 U?
U 1 1 6213F0C8
P 1650 6050
F 0 "U?" H 1650 5733 50  0000 C CNN
F 1 "74HC14" H 1650 5824 50  0000 C CNN
F 2 "" H 1650 6050 50  0001 C CNN
F 3 "http://www.ti.com/lit/gpn/sn74HC14" H 1650 6050 50  0001 C CNN
	1    1650 6050
	-1   0    0    1   
$EndComp
Text Notes 1400 6350 0    50   ~ 0
Schmitt Trigger\n
$Comp
L Device:R R?
U 1 1 6213F0CF
P 2350 5550
F 0 "R?" H 2420 5596 50  0000 L CNN
F 1 "R" H 2420 5505 50  0000 L CNN
F 2 "" V 2280 5550 50  0001 C CNN
F 3 "~" H 2350 5550 50  0001 C CNN
	1    2350 5550
	1    0    0    -1  
$EndComp
$Comp
L Device:R R?
U 1 1 6213F0D5
P 2800 6050
F 0 "R?" V 2593 6050 50  0000 C CNN
F 1 "R" V 2684 6050 50  0000 C CNN
F 2 "" V 2730 6050 50  0001 C CNN
F 3 "~" H 2800 6050 50  0001 C CNN
	1    2800 6050
	0    1    1    0   
$EndComp
$Comp
L Switch:SW_MEC_5G SW?
U 1 1 6213F0DB
P 3450 6050
F 0 "SW?" H 3450 6335 50  0000 C CNN
F 1 "SW_MEC_5G" H 3450 6244 50  0000 C CNN
F 2 "" H 3450 6250 50  0001 C CNN
F 3 "http://www.apem.com/int/index.php?controller=attachment&id_attachment=488" H 3450 6250 50  0001 C CNN
	1    3450 6050
	1    0    0    -1  
$EndComp
$Comp
L Device:CP1 C?
U 1 1 6213F0E1
P 3150 6450
F 0 "C?" V 3402 6450 50  0000 C CNN
F 1 "CP1" V 3311 6450 50  0000 C CNN
F 2 "" H 3150 6450 50  0001 C CNN
F 3 "~" H 3150 6450 50  0001 C CNN
	1    3150 6450
	0    -1   -1   0   
$EndComp
$Comp
L power:GNDREF #PWR?
U 1 1 6213F0E7
P 4100 6450
F 0 "#PWR?" H 4100 6200 50  0001 C CNN
F 1 "GNDREF" H 4105 6277 50  0000 C CNN
F 2 "" H 4100 6450 50  0001 C CNN
F 3 "" H 4100 6450 50  0001 C CNN
	1    4100 6450
	1    0    0    -1  
$EndComp
Wire Wire Line
	2350 5700 2350 6050
Wire Wire Line
	2350 6050 1950 6050
Wire Wire Line
	2350 6050 2550 6050
Connection ~ 2350 6050
Wire Wire Line
	2950 6050 3250 6050
Wire Wire Line
	3650 6050 3750 6050
Wire Wire Line
	4100 6050 4100 6450
Wire Wire Line
	3300 6450 3750 6450
Wire Wire Line
	3000 6450 2550 6450
Wire Wire Line
	2550 6450 2550 6050
Connection ~ 2550 6050
Wire Wire Line
	2550 6050 2650 6050
Wire Wire Line
	3750 6450 3750 6050
Connection ~ 3750 6050
Wire Wire Line
	3750 6050 4100 6050
Wire Wire Line
	1900 3800 1350 3800
Wire Wire Line
	1350 3800 1350 6050
$EndSCHEMATC
