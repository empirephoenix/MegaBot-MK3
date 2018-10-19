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
F 2 "" H 5900 6500 60  0001 C CNN
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
F 2 "Capacitor_THT:C_Rect_L41.5mm_W19.0mm_P37.50mm_MKS4" H 2438 4150 50  0001 C CNN
F 3 "~" H 2400 4300 50  0001 C CNN
	1    2400 4300
	0    1    1    0   
$EndComp
$Comp
L secondarycontroller-rescue:DIODE-pspice D2
U 1 1 5BBE5A38
P 2550 5100
F 0 "D2" V 2596 4972 50  0000 R CNN
F 1 "DIODE" V 2505 4972 50  0000 R CNN
F 2 "Diode_THT:D_5W_P12.70mm_Horizontal" H 2550 5100 50  0001 C CNN
F 3 "~" H 2550 5100 50  0001 C CNN
	1    2550 5100
	1    0    0    -1  
$EndComp
$Comp
L secondarycontroller-rescue:SW_Push-Switch SW2
U 1 1 5BBE645A
P 4100 4300
F 0 "SW2" H 4100 4585 50  0000 C CNN
F 1 "Reset" H 4100 4494 50  0000 C CNN
F 2 "Button_Switch_THT:SW_DIP_SPSTx01_Piano_10.8x4.1mm_W7.62mm_P2.54mm" H 4100 4500 50  0001 C CNN
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
P 5600 2100
F 0 "SW1" H 5600 2385 50  0000 C CNN
F 1 "Calibrate" H 5600 2294 50  0000 C CNN
F 2 "Button_Switch_THT:SW_DIP_SPSTx01_Piano_10.8x4.1mm_W7.62mm_P2.54mm" H 5600 2300 50  0001 C CNN
F 3 "" H 5600 2300 50  0001 C CNN
	1    5600 2100
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
$Comp
L secondarycontroller-rescue:MCU3386-mcu3386 U1
U 1 1 5BBE76D9
P 1300 4750
F 0 "U1" H 1350 4865 50  0000 C CNN
F 1 "MCU3386" H 1350 4774 50  0000 C CNN
F 2 "" H 1300 4750 50  0001 C CNN
F 3 "" H 1300 4750 50  0001 C CNN
	1    1300 4750
	1    0    0    -1  
$EndComp
Wire Wire Line
	2550 4300 2550 3600
$Comp
L secondarycontroller-rescue:DIODE-pspice D1
U 1 1 5BBE875F
P 2200 5200
F 0 "D1" H 2200 4935 50  0000 C CNN
F 1 "DIODE" H 2200 5026 50  0000 C CNN
F 2 "Diode_THT:D_5W_P12.70mm_Horizontal" H 2200 5200 50  0001 C CNN
F 3 "~" H 2200 5200 50  0001 C CNN
	1    2200 5200
	-1   0    0    1   
$EndComp
Wire Wire Line
	1900 5200 2000 5200
Wire Wire Line
	1900 5100 2350 5100
Wire Wire Line
	2750 4300 2750 4500
Connection ~ 2750 4300
Wire Wire Line
	2750 4300 2550 4300
Wire Wire Line
	2400 5200 2800 5200
Connection ~ 2800 3800
Wire Wire Line
	2800 3800 2800 5200
Wire Wire Line
	2800 3700 2700 3700
Wire Wire Line
	2700 3700 2700 4400
Wire Wire Line
	2700 4400 2900 4400
Wire Wire Line
	2900 4400 2900 5400
Wire Wire Line
	2900 5400 1900 5400
Wire Wire Line
	2750 5100 2750 5300
Wire Wire Line
	2750 5300 1900 5300
Connection ~ 2750 5100
Wire Wire Line
	800  5150 600  5150
Wire Wire Line
	600  5150 600  4500
Wire Wire Line
	600  4500 2750 4500
Connection ~ 2750 4500
Wire Wire Line
	2750 4500 2750 5100
Wire Wire Line
	800  5250 700  5250
Wire Wire Line
	700  5250 700  3700
Wire Wire Line
	700  3700 2700 3700
Connection ~ 2700 3700
Wire Wire Line
	800  5050 650  5050
Wire Wire Line
	650  5050 650  5650
Wire Wire Line
	650  5650 5850 5650
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
	550  5700 550  4950
Wire Wire Line
	550  4950 800  4950
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
	6000 2400 5400 2400
Wire Wire Line
	6000 2200 6000 2050
Wire Wire Line
	6000 2050 6300 2050
Wire Wire Line
	6300 2050 6300 5000
Wire Wire Line
	6300 5000 3250 5000
Wire Wire Line
	3250 5000 3250 4300
Connection ~ 3250 4300
Wire Wire Line
	3250 4300 2750 4300
Wire Wire Line
	5400 1900 5650 1900
Wire Wire Line
	5650 1900 5650 1600
Wire Wire Line
	5650 1600 6050 1600
Wire Wire Line
	5400 2000 5700 2000
Wire Wire Line
	5700 2000 5700 1700
Wire Wire Line
	5700 1700 6050 1700
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
Connection ~ 2900 5400
Wire Wire Line
	5800 2100 5850 2100
Wire Wire Line
	5850 2100 5850 2050
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
Connection ~ 5850 2050
$EndSCHEMATC
