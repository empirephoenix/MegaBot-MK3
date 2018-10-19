EESchema Schematic File Version 4
LIBS:secondarycontroller-cache
EELAYER 26 0
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
L secondarycontroller-rescue:Arduino_Uno_Shield-arduino XA1
U 1 1 5BBE520D
P 4100 2750
F 0 "XA1" H 4100 4137 60  0000 C CNN
F 1 "Arduino_Uno_Shield" H 4100 4031 60  0000 C CNN
F 2 "Arduino:Arduino_Uno_Shield" H 5900 6500 60  0001 C CNN
F 3 "https://store.arduino.cc/arduino-uno-rev3" H 5900 6500 60  0001 C CNN
	1    4100 2750
	1    0    0    -1  
$EndComp
$Comp
L secondarycontroller-rescue:Conn_01x02_Female-Connector J3
U 1 1 5BBE53F0
P 1900 3600
F 0 "J3" H 1794 3275 50  0000 C CNN
F 1 "VIN" H 1794 3366 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 1900 3600 50  0001 C CNN
F 3 "~" H 1900 3600 50  0001 C CNN
	1    1900 3600
	-1   0    0    1   
$EndComp
Wire Wire Line
	2100 3500 2600 3500
Wire Wire Line
	2600 3500 2600 3800
Wire Wire Line
	2600 3800 2800 3800
Wire Wire Line
	2100 3600 2550 3600
Wire Wire Line
	2550 3600 2550 3300
Wire Wire Line
	2550 3300 2800 3300
$Comp
L secondarycontroller-rescue:C-Device CF15
U 1 1 5BBE583A
P 2400 4300
F 0 "CF15" V 2148 4300 50  0000 C CNN
F 1 "C" V 2239 4300 50  0000 C CNN
F 2 "Capacitor_THT:C_Rect_L41.5mm_W13.0mm_P37.50mm_MKS4" H 2438 4150 50  0001 C CNN
F 3 "~" H 2400 4300 50  0001 C CNN
	1    2400 4300
	0    1    1    0   
$EndComp
$Comp
L secondarycontroller-rescue:SW_Push-Switch SW2
U 1 1 5BBE645A
P 4100 4300
F 0 "SW2" H 4100 4585 50  0000 C CNN
F 1 "Reset" H 4100 4494 50  0000 C CNN
F 2 "Button_Switch_THT:SW_PUSH_6mm" H 4100 4500 50  0001 C CNN
F 3 "" H 4100 4500 50  0001 C CNN
	1    4100 4300
	1    0    0    -1  
$EndComp
$Comp
L secondarycontroller-rescue:Conn_01x04_Female-Connector J4
U 1 1 5BBE6815
P 6250 1600
F 0 "J4" H 6278 1576 50  0000 L CNN
F 1 "APA102C" H 6278 1485 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical" H 6250 1600 50  0001 C CNN
F 3 "~" H 6250 1600 50  0001 C CNN
	1    6250 1600
	1    0    0    -1  
$EndComp
$Comp
L secondarycontroller-rescue:SW_Push-Switch SW1
U 1 1 5BBE694D
P 5600 2800
F 0 "SW1" H 5600 3085 50  0000 C CNN
F 1 "Calibrate" H 5600 2994 50  0000 C CNN
F 2 "Button_Switch_THT:SW_PUSH_6mm" H 5600 3000 50  0001 C CNN
F 3 "" H 5600 3000 50  0001 C CNN
	1    5600 2800
	1    0    0    -1  
$EndComp
Wire Wire Line
	5400 3700 5600 3700
Wire Wire Line
	5600 3700 5600 4300
Wire Wire Line
	5600 4300 4300 4300
Wire Wire Line
	3900 4300 3250 4300
Connection ~ 2550 3600
Wire Wire Line
	2550 4300 2550 3600
Wire Wire Line
	2800 3700 2700 3700
Wire Wire Line
	2700 3700 2700 4400
Wire Wire Line
	2700 4400 2900 4400
Wire Wire Line
	2900 4400 2900 5400
Wire Wire Line
	700  3700 2700 3700
Connection ~ 2700 3700
Wire Wire Line
	5850 5650 5850 2600
Wire Wire Line
	5850 2600 5400 2600
Wire Wire Line
	5400 2500 5900 2500
Wire Wire Line
	5900 2500 5900 5700
Wire Wire Line
	5900 5700 550  5700
Wire Wire Line
	550  5700 550  6150
Wire Wire Line
	550  6150 2300 6150
Connection ~ 2550 4300
Wire Wire Line
	2800 2900 1550 2900
Wire Wire Line
	1550 2900 1550 4300
Wire Wire Line
	1550 4300 2250 4300
$Comp
L secondarycontroller-rescue:Conn_01x03_Female-Connector J1
U 1 1 5BBEDF33
P 6200 2300
F 0 "J1" H 6227 2326 50  0000 L CNN
F 1 "Receiver" H 6227 2235 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 6200 2300 50  0001 C CNN
F 3 "~" H 6200 2300 50  0001 C CNN
	1    6200 2300
	1    0    0    -1  
$EndComp
Wire Wire Line
	6000 2200 6000 2050
Wire Wire Line
	6000 2050 6300 2050
Wire Wire Line
	6300 2050 6300 2800
Wire Wire Line
	6300 5000 3250 5000
Wire Wire Line
	3250 5000 3250 4300
Connection ~ 3250 4300
Wire Wire Line
	5650 1600 6050 1600
Wire Wire Line
	5400 2000 5700 2000
Wire Wire Line
	6050 1500 5850 1500
Wire Wire Line
	5850 2050 6000 2050
Connection ~ 6000 2050
Wire Wire Line
	6050 1800 6050 1900
Wire Wire Line
	6050 1900 6200 1900
Wire Wire Line
	6200 1900 6200 5400
Wire Wire Line
	6200 5400 2900 5400
Wire Wire Line
	5850 1500 5850 2050
$Comp
L secondarycontroller-rescue:Conn_01x02_Female-Connector J5
U 1 1 5BBF72BA
P 2300 2100
F 0 "J5" H 2194 1775 50  0000 C CNN
F 1 "I2C BUS" H 2194 1866 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 2300 2100 50  0001 C CNN
F 3 "~" H 2300 2100 50  0001 C CNN
	1    2300 2100
	-1   0    0    1   
$EndComp
Wire Wire Line
	2800 2000 2500 2000
Wire Wire Line
	2500 2100 2800 2100
$Comp
L Connector:Conn_01x07_Female J2
U 1 1 5BCA6134
P 2500 6150
F 0 "J2" H 2527 6176 50  0000 L CNN
F 1 "Conn_01x07_Female" H 2527 6085 50  0000 L CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x07_P2.54mm_Vertical" H 2500 6150 50  0001 C CNN
F 3 "~" H 2500 6150 50  0001 C CNN
	1    2500 6150
	1    0    0    -1  
$EndComp
Wire Wire Line
	700  3700 700  5850
Wire Wire Line
	700  5850 2300 5850
Wire Wire Line
	5850 5650 1900 5650
Wire Wire Line
	1900 5650 1900 6050
Wire Wire Line
	1900 6050 2300 6050
Wire Wire Line
	2900 5400 2900 5550
Wire Wire Line
	2900 5550 1650 5550
Wire Wire Line
	1650 5550 1650 6450
Wire Wire Line
	1650 6450 2300 6450
Connection ~ 2900 5400
Wire Wire Line
	2300 6250 1800 6250
Wire Wire Line
	1800 6250 1800 5600
Wire Wire Line
	1800 5600 6600 5600
Wire Wire Line
	6600 5600 6600 2250
Wire Wire Line
	6600 2250 5800 2250
Wire Wire Line
	5800 2300 5400 2300
Wire Wire Line
	2550 4300 3250 4300
Wire Wire Line
	2550 5750 2200 5750
Wire Wire Line
	2200 5750 2200 5950
Wire Wire Line
	2200 5950 2300 5950
Wire Wire Line
	2200 5950 2200 6350
Wire Wire Line
	2200 6350 2300 6350
Connection ~ 2200 5950
Wire Wire Line
	2550 4300 2550 5750
Wire Wire Line
	5800 2250 5800 2300
Wire Wire Line
	5800 2800 6300 2800
Connection ~ 6300 2800
Wire Wire Line
	6300 2800 6300 5000
Wire Wire Line
	6000 2400 6000 2700
Wire Wire Line
	6000 2700 5400 2700
Wire Wire Line
	5400 1900 5650 1900
Wire Wire Line
	5650 1600 5650 1900
Wire Wire Line
	5700 1700 5700 2000
Wire Wire Line
	5700 1700 6050 1700
$EndSCHEMATC
