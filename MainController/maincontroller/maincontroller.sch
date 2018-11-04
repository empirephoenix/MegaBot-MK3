EESchema Schematic File Version 4
LIBS:maincontroller-cache
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
L maincontroller-rescue:SW_Push-Switch Reset1
U 1 1 5BB7D413
P 5600 5100
F 0 "Reset1" H 5600 5385 50  0000 C CNN
F 1 "SW_Push" H 5600 5294 50  0000 C CNN
F 2 "Button_Switch_THT:SW_DIP_SPSTx01_Piano_10.8x4.1mm_W7.62mm_P2.54mm" H 5600 5300 50  0001 C CNN
F 3 "" H 5600 5300 50  0001 C CNN
	1    5600 5100
	1    0    0    -1  
$EndComp
$Comp
L maincontroller-rescue:Conn_01x06-Connector_Generic J2
U 1 1 5BB7E71D
P 9200 6000
F 0 "J2" H 9280 5992 50  0000 L CNN
F 1 "Receiver" H 9280 5901 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x06_P2.54mm_Vertical" H 9200 6000 50  0001 C CNN
F 3 "~" H 9200 6000 50  0001 C CNN
	1    9200 6000
	1    0    0    -1  
$EndComp
$Comp
L maincontroller-rescue:Arduino_Mega2560_Shield-arduino XA1
U 1 1 5BB7EBA8
P 7200 3950
F 0 "XA1" H 7200 1570 60  0000 C CNN
F 1 "Arduino_Mega2560_Shield" H 7200 1464 60  0000 C CNN
F 2 "Arduino:Arduino_Mega2560_Shield" H 7900 6700 60  0001 C CNN
F 3 "https://store.arduino.cc/arduino-mega-2560-rev3" H 7900 6700 60  0001 C CNN
	1    7200 3950
	1    0    0    -1  
$EndComp
$Comp
L maincontroller-rescue:Conn_01x04-Connector_Generic J5
U 1 1 5BB7F576
P 2900 6250
F 0 "J5" H 2980 6242 50  0000 L CNN
F 1 "OneWire" H 2980 6151 50  0000 L CNN
F 2 "Connector_JST:JST_EH_B04B-EH-A_1x04_P2.50mm_Vertical" H 2900 6250 50  0001 C CNN
F 3 "~" H 2900 6250 50  0001 C CNN
	1    2900 6250
	-1   0    0    1   
$EndComp
$Comp
L maincontroller-rescue:Conn_01x05_Female-Connector J4
U 1 1 5BB7E5BD
P 2050 3350
F 0 "J4" H 2077 3326 50  0000 L CNN
F 1 "Analog_Motor" H 2077 3235 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x05_P2.54mm_Vertical" H 2050 3350 50  0001 C CNN
F 3 "~" H 2050 3350 50  0001 C CNN
	1    2050 3350
	-1   0    0    1   
$EndComp
$Comp
L maincontroller-rescue:Conn_01x04-Connector_Generic J1
U 1 1 5BB80B10
P 9600 2600
F 0 "J1" H 9680 2592 50  0000 L CNN
F 1 "LED" H 9680 2501 50  0000 L CNN
F 2 "Connector_JST:JST_EH_B04B-EH-A_1x04_P2.50mm_Vertical" H 9600 2600 50  0001 C CNN
F 3 "~" H 9600 2600 50  0001 C CNN
	1    9600 2600
	1    0    0    -1  
$EndComp
$Comp
L maincontroller-rescue:MCP4725-MCP4725 Vorne_links1
U 1 1 5BB8C41D
P 2750 2600
F 0 "Vorne_links1" V 3064 2468 50  0000 C CNN
F 1 "MCP4725" V 2973 2468 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x06_P2.54mm_Vertical" H 2800 2050 50  0001 C CNN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/22039d.pdf" H 2800 2900 50  0001 C CNN
	1    2750 2600
	0    -1   -1   0   
$EndComp
$Comp
L maincontroller-rescue:MCP4725-MCP4725 Vorne_rechts1
U 1 1 5BB8C793
P 2750 3500
F 0 "Vorne_rechts1" V 3064 3368 50  0000 C CNN
F 1 "MCP4725" V 2973 3368 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x06_P2.54mm_Vertical" H 2800 2950 50  0001 C CNN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/22039d.pdf" H 2800 3800 50  0001 C CNN
	1    2750 3500
	0    -1   -1   0   
$EndComp
$Comp
L maincontroller-rescue:MCP4725-MCP4725 Hinten_links1
U 1 1 5BB8C852
P 2750 4400
F 0 "Hinten_links1" V 3064 4268 50  0000 C CNN
F 1 "MCP4725" V 2973 4268 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x06_P2.54mm_Vertical" H 2800 3850 50  0001 C CNN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/22039d.pdf" H 2800 4700 50  0001 C CNN
	1    2750 4400
	0    -1   -1   0   
$EndComp
$Comp
L maincontroller-rescue:MCP4725-MCP4725 Hinten_rechts1
U 1 1 5BB8C992
P 2750 5300
F 0 "Hinten_rechts1" V 3064 5168 50  0000 C CNN
F 1 "MCP4725" V 2973 5168 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x06_P2.54mm_Vertical" H 2800 4750 50  0001 C CNN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/22039d.pdf" H 2800 5600 50  0001 C CNN
	1    2750 5300
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3100 2500 3200 2500
Wire Wire Line
	3200 2500 3200 3400
Wire Wire Line
	3200 5200 3100 5200
Wire Wire Line
	3100 3400 3200 3400
Connection ~ 3200 3400
Wire Wire Line
	3200 3400 3200 4300
Wire Wire Line
	3100 4300 3200 4300
Connection ~ 3200 4300
Wire Wire Line
	3200 4300 3200 5200
Wire Wire Line
	3200 5200 4700 5200
Connection ~ 3200 5200
Wire Wire Line
	3100 2600 3300 2600
Wire Wire Line
	3300 2600 3300 3500
Wire Wire Line
	3300 3500 3100 3500
Wire Wire Line
	3300 3500 3300 4400
Wire Wire Line
	3300 4400 3100 4400
Connection ~ 3300 3500
Wire Wire Line
	3300 4400 3300 5300
Wire Wire Line
	3300 5300 3100 5300
Connection ~ 3300 4400
Wire Wire Line
	3100 6150 3200 6150
Wire Wire Line
	3200 5200 3200 6050
Wire Wire Line
	3100 6050 3200 6050
Connection ~ 3200 6050
Wire Wire Line
	3200 6050 3200 6150
Wire Wire Line
	5900 5800 5300 5800
Wire Wire Line
	3300 5800 3300 5300
Connection ~ 3300 5300
Wire Wire Line
	5800 5100 5800 4900
Wire Wire Line
	5800 4900 5900 4900
Wire Wire Line
	5400 5100 5400 5200
Connection ~ 5400 5200
$Comp
L maincontroller-rescue:R-Device R1
U 1 1 5BB91DD8
P 3250 3000
F 0 "R1" V 3043 3000 50  0000 C CNN
F 1 "R380" V 3134 3000 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0309_L9.0mm_D3.2mm_P12.70mm_Horizontal" V 3180 3000 50  0001 C CNN
F 3 "~" H 3250 3000 50  0001 C CNN
	1    3250 3000
	0    1    1    0   
$EndComp
$Comp
L maincontroller-rescue:R-Device R2
U 1 1 5BB922C9
P 3250 3900
F 0 "R2" V 3043 3900 50  0000 C CNN
F 1 "R380" V 3134 3900 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0309_L9.0mm_D3.2mm_P12.70mm_Horizontal" V 3180 3900 50  0001 C CNN
F 3 "~" H 3250 3900 50  0001 C CNN
	1    3250 3900
	0    1    1    0   
$EndComp
$Comp
L maincontroller-rescue:R-Device R3
U 1 1 5BB9238D
P 3250 4800
F 0 "R3" V 3043 4800 50  0000 C CNN
F 1 "R380" V 3134 4800 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0309_L9.0mm_D3.2mm_P12.70mm_Horizontal" V 3180 4800 50  0001 C CNN
F 3 "~" H 3250 4800 50  0001 C CNN
	1    3250 4800
	0    1    1    0   
$EndComp
$Comp
L maincontroller-rescue:R-Device R4
U 1 1 5BB923E1
P 3250 5700
F 0 "R4" V 3043 5700 50  0000 C CNN
F 1 "R380" V 3134 5700 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0309_L9.0mm_D3.2mm_P12.70mm_Horizontal" V 3180 5700 50  0001 C CNN
F 3 "~" H 3250 5700 50  0001 C CNN
	1    3250 5700
	0    1    1    0   
$EndComp
Wire Wire Line
	3400 3100 3400 3000
Wire Wire Line
	3400 3200 3400 3900
Wire Wire Line
	3400 4800 3500 4800
Wire Wire Line
	3500 4800 3500 3300
Wire Wire Line
	3400 5700 3600 5700
Wire Wire Line
	3600 5700 3600 3400
Wire Wire Line
	3400 3100 3400 3150
Wire Wire Line
	3400 3150 2250 3150
Connection ~ 3400 3100
Wire Wire Line
	3400 3900 3400 4050
Wire Wire Line
	3400 4050 2600 4050
Wire Wire Line
	2600 4050 2600 3250
Wire Wire Line
	2600 3250 2250 3250
Connection ~ 3400 3900
Wire Wire Line
	3400 4950 2550 4950
Wire Wire Line
	2550 3350 2250 3350
Wire Wire Line
	3400 4800 3400 4950
Wire Wire Line
	2550 3350 2550 4950
Connection ~ 3400 4800
Wire Wire Line
	3400 5700 3400 5850
Wire Wire Line
	3400 5850 2450 5850
Wire Wire Line
	2450 5850 2450 3450
Wire Wire Line
	2450 3450 2250 3450
Connection ~ 3400 5700
Wire Wire Line
	8500 1800 8650 1800
Wire Wire Line
	8650 1800 8650 1200
Wire Wire Line
	8650 1200 5300 1200
Wire Wire Line
	5300 1200 5300 5500
Wire Wire Line
	5300 5500 3100 5500
Wire Wire Line
	3100 5400 5250 5400
Wire Wire Line
	5250 5400 5250 1150
Wire Wire Line
	5250 1150 8700 1150
Wire Wire Line
	8700 1150 8700 1900
Wire Wire Line
	8700 1900 8500 1900
Wire Wire Line
	8500 2000 8800 2000
Wire Wire Line
	8800 2000 8800 1050
Wire Wire Line
	8800 1050 5100 1050
Wire Wire Line
	5100 1050 5100 4600
Wire Wire Line
	5100 4600 3100 4600
Wire Wire Line
	3100 4500 5050 4500
Wire Wire Line
	5050 4500 5050 1000
Wire Wire Line
	5050 1000 8850 1000
Wire Wire Line
	8850 1000 8850 2100
Wire Wire Line
	8850 2100 8500 2100
Wire Wire Line
	4950 3700 4950 900 
Wire Wire Line
	4950 900  8950 900 
Wire Wire Line
	8950 900  8950 2200
Wire Wire Line
	8950 2200 8500 2200
Wire Wire Line
	8500 2300 9000 2300
Wire Wire Line
	9000 2300 9000 850 
Wire Wire Line
	9000 850  4900 850 
Wire Wire Line
	4900 850  4900 3600
Wire Wire Line
	4900 3600 3100 3600
Wire Wire Line
	3100 2800 4800 2800
Wire Wire Line
	4800 2800 4800 750 
Wire Wire Line
	4800 750  9100 750 
Wire Wire Line
	9100 750  9100 2400
Wire Wire Line
	9100 2400 8500 2400
Wire Wire Line
	8500 2500 9150 2500
Wire Wire Line
	9150 2500 9150 700 
Wire Wire Line
	9150 700  4750 700 
Wire Wire Line
	4750 700  4750 2700
Wire Wire Line
	4750 2700 3100 2700
Wire Wire Line
	8300 6200 8300 6350
Wire Wire Line
	8300 6350 5400 6350
Wire Wire Line
	8350 6300 8350 6400
Wire Wire Line
	8350 6400 5300 6400
Wire Wire Line
	5300 6400 5300 5800
Connection ~ 5300 5800
Wire Wire Line
	5300 5800 4100 5800
$Comp
L maincontroller-rescue:R-Device R5
U 1 1 5BBD15F0
P 3250 6250
F 0 "R5" V 3043 6250 50  0000 C CNN
F 1 "R215" V 3134 6250 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0411_L9.9mm_D3.6mm_P12.70mm_Horizontal" V 3180 6250 50  0001 C CNN
F 3 "~" H 3250 6250 50  0001 C CNN
	1    3250 6250
	0    1    1    0   
$EndComp
Wire Wire Line
	3400 6250 3700 6250
Wire Wire Line
	3700 6250 3700 3500
Wire Wire Line
	3100 3700 4950 3700
Wire Wire Line
	3100 6350 3800 6350
Wire Wire Line
	3800 6350 3800 5900
$Comp
L maincontroller-rescue:R-Device R6
U 1 1 5BBDE040
P 3950 5900
F 0 "R6" V 3743 5900 50  0000 C CNN
F 1 "R4K7" V 3834 5900 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0309_L9.0mm_D3.2mm_P12.70mm_Horizontal" V 3880 5900 50  0001 C CNN
F 3 "~" H 3950 5900 50  0001 C CNN
	1    3950 5900
	0    1    1    0   
$EndComp
Connection ~ 4100 5800
Wire Wire Line
	4100 5800 3300 5800
Wire Wire Line
	4100 5900 4100 5800
Connection ~ 3800 5900
Wire Wire Line
	3800 5900 3800 3750
Wire Wire Line
	8350 6300 8800 6300
Wire Wire Line
	8300 6200 8700 6200
Wire Wire Line
	8700 6200 8700 6350
Wire Wire Line
	8700 6350 8900 6350
Wire Wire Line
	8900 5600 9200 5600
Wire Wire Line
	9250 5650 8800 5650
Wire Wire Line
	5400 5200 5400 5400
Wire Wire Line
	5900 5400 5800 5400
Wire Wire Line
	5400 5400 5400 6350
Connection ~ 5400 5400
Wire Wire Line
	8500 2700 9400 2700
Wire Wire Line
	8500 2600 9400 2600
Wire Wire Line
	9200 2800 9200 3600
Wire Wire Line
	9200 2800 9400 2800
Wire Wire Line
	9400 2500 9250 2500
Wire Wire Line
	9250 2500 9250 5650
Wire Wire Line
	3800 3750 5450 3750
Wire Wire Line
	3700 3500 5400 3500
Wire Wire Line
	5750 3100 5750 3300
Wire Wire Line
	5750 3300 5900 3300
Wire Wire Line
	5700 3200 5700 3400
Wire Wire Line
	5700 3400 5900 3400
Wire Wire Line
	5650 3500 5900 3500
Wire Wire Line
	5650 3300 5650 3500
Wire Wire Line
	5600 3600 5900 3600
Wire Wire Line
	3400 3100 5750 3100
Wire Wire Line
	3400 3200 5700 3200
Wire Wire Line
	3500 3300 5650 3300
Wire Wire Line
	3600 3400 5600 3400
Wire Wire Line
	5400 3500 5400 2950
Wire Wire Line
	5400 2950 5850 2950
Wire Wire Line
	5450 3000 5800 3000
Wire Wire Line
	5800 3000 5800 3200
Wire Wire Line
	5800 3200 5900 3200
Wire Wire Line
	5850 3100 5900 3100
Wire Wire Line
	5850 2950 5850 3100
$Comp
L maincontroller-rescue:Conn_01x04-Connector_Generic J6
U 1 1 5BD2D1DF
P 3950 4100
F 0 "J6" H 3870 3675 50  0000 C CNN
F 1 "Hall_In" H 3870 3766 50  0000 C CNN
F 2 "Connector_JST:JST_EH_B04B-EH-A_1x05_P2.50mm_Vertical" H 3950 4100 50  0001 C CNN
F 3 "~" H 3950 4100 50  0001 C CNN
	1    3950 4100
	-1   0    0    1   
$EndComp
$Comp
L maincontroller-rescue:R-Device R7
U 1 1 5BD32A4E
P 4300 3900
F 0 "R7" V 4093 3900 50  0000 C CNN
F 1 "R30k" V 4184 3900 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P2.54mm_Vertical" V 4230 3900 50  0001 C CNN
F 3 "~" H 4300 3900 50  0001 C CNN
	1    4300 3900
	0    1    1    0   
$EndComp
Wire Wire Line
	5600 3400 5600 3600
Wire Wire Line
	5450 3750 5450 3000
$Comp
L maincontroller-rescue:R-Device R8
U 1 1 5BD88C7C
P 4300 4000
F 0 "R8" V 4093 4000 50  0000 C CNN
F 1 "R30k" V 4184 4000 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P2.54mm_Vertical" V 4230 4000 50  0001 C CNN
F 3 "~" H 4300 4000 50  0001 C CNN
	1    4300 4000
	0    1    1    0   
$EndComp
$Comp
L maincontroller-rescue:R-Device R9
U 1 1 5BD88CBE
P 4300 4100
F 0 "R9" V 4093 4100 50  0000 C CNN
F 1 "R30k" V 4184 4100 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P2.54mm_Vertical" V 4230 4100 50  0001 C CNN
F 3 "~" H 4300 4100 50  0001 C CNN
	1    4300 4100
	0    1    1    0   
$EndComp
$Comp
L maincontroller-rescue:R-Device R10
U 1 1 5BD88D02
P 4300 4200
F 0 "R10" V 4093 4200 50  0000 C CNN
F 1 "R30k" V 4184 4200 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P2.54mm_Vertical" V 4230 4200 50  0001 C CNN
F 3 "~" H 4300 4200 50  0001 C CNN
	1    4300 4200
	0    1    1    0   
$EndComp
Wire Wire Line
	4700 4700 4700 5200
Connection ~ 4700 4700
Connection ~ 4700 5200
Wire Wire Line
	4700 5200 5400 5200
Wire Wire Line
	4450 3900 4550 3900
Wire Wire Line
	4450 4100 4750 4100
Wire Wire Line
	4450 4200 4850 4200
Wire Wire Line
	4850 4400 4850 4200
Connection ~ 4850 4200
Wire Wire Line
	4750 4400 4750 4100
Connection ~ 4750 4100
Wire Wire Line
	4650 4400 4650 4000
Wire Wire Line
	4450 4000 4650 4000
Connection ~ 4650 4000
Wire Wire Line
	4550 4400 4550 3900
Connection ~ 4550 3900
Wire Wire Line
	5950 4000 5900 4000
Wire Wire Line
	5850 3900 5900 3900
Wire Wire Line
	5850 4000 5850 3900
Wire Wire Line
	5700 3900 5700 3750
Wire Wire Line
	5700 3750 5950 3750
Wire Wire Line
	5950 3750 5950 4000
Wire Wire Line
	5750 3800 5750 4200
Wire Wire Line
	5750 3800 5900 3800
Wire Wire Line
	5800 4100 5800 3700
Wire Wire Line
	5800 3700 5900 3700
Wire Wire Line
	8750 5800 8500 5800
$Comp
L maincontroller-rescue:Conn_01x02-Connector_Generic Bus1
U 1 1 5BC4BCB9
P 5450 2850
F 0 "Bus1" H 5370 2525 50  0000 C CNN
F 1 "Conn_01x02" H 5370 2616 50  0000 C CNN
F 2 "Connector_JST:JST_EH_B02B-EH-A_1x02_P2.50mm_Vertical" H 5450 2850 50  0001 C CNN
F 3 "~" H 5450 2850 50  0001 C CNN
	1    5450 2850
	-1   0    0    1   
$EndComp
Wire Wire Line
	5900 2800 5800 2800
Wire Wire Line
	5800 2800 5800 2750
Wire Wire Line
	5800 2750 5650 2750
Wire Wire Line
	5650 2850 5900 2850
Wire Wire Line
	5900 2850 5900 2900
$Comp
L maincontroller-rescue:Conn_01x05_Female-Connector relays1
U 1 1 5BBFA89A
P 9700 3400
F 0 "relays1" H 9727 3426 50  0000 L CNN
F 1 "Conn_01x05_Female-Connector" H 9727 3335 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 9700 3400 50  0001 C CNN
F 3 "~" H 9700 3400 50  0001 C CNN
	1    9700 3400
	1    0    0    -1  
$EndComp
Wire Wire Line
	9500 3200 9100 3200
Wire Wire Line
	9100 3200 9100 2800
Wire Wire Line
	9100 2800 8500 2800
Wire Wire Line
	9500 3600 9200 3600
Connection ~ 9200 3600
Wire Wire Line
	9200 3600 9200 5600
Wire Wire Line
	9000 5800 8950 5800
Wire Wire Line
	8950 5800 8950 5850
Wire Wire Line
	8950 5850 8550 5850
Wire Wire Line
	8550 5850 8550 6000
Wire Wire Line
	8550 6000 8500 6000
Wire Wire Line
	9000 5900 8750 5900
Wire Wire Line
	8750 5900 8750 5800
Wire Wire Line
	9000 6000 8700 6000
Wire Wire Line
	8700 6000 8700 5900
Wire Wire Line
	8700 5900 8500 5900
Wire Wire Line
	9000 6100 8500 6100
Wire Wire Line
	5900 5300 5800 5300
Wire Wire Line
	5800 5300 5800 5400
Connection ~ 5800 5400
Wire Wire Line
	5800 5400 5400 5400
Wire Wire Line
	5900 5200 5800 5200
Wire Wire Line
	5800 5200 5800 5300
Connection ~ 5800 5300
Wire Wire Line
	5900 5500 5800 5500
Wire Wire Line
	5800 5500 5800 5400
Wire Wire Line
	5900 5600 5800 5600
Wire Wire Line
	5800 5600 5800 5500
Connection ~ 5800 5500
Wire Wire Line
	8800 5650 8800 6300
Wire Wire Line
	8900 5600 8900 6200
Wire Wire Line
	9000 6300 8800 6300
Connection ~ 8800 6300
Wire Wire Line
	9000 6200 8900 6200
Connection ~ 8900 6200
Wire Wire Line
	8900 6200 8900 6350
Wire Wire Line
	5900 4100 5950 4100
Wire Wire Line
	5950 4100 5950 4750
Wire Wire Line
	5950 4750 2250 4750
Wire Wire Line
	2250 4750 2250 3550
Wire Wire Line
	9500 3500 9500 3600
Wire Wire Line
	9500 3300 9500 3200
Wire Wire Line
	8900 3400 8900 3200
Wire Wire Line
	8900 3200 8500 3200
Wire Wire Line
	8900 3400 9500 3400
$Comp
L Connector:Conn_01x01_Male MotorControllerPower
U 1 1 5BDDE388
P 9750 3000
F 0 "MotorControllerPower" H 9723 2930 50  0000 R CNN
F 1 "Conn_01x01_Male" H 9723 3021 50  0000 R CNN
F 2 "" H 9750 3000 50  0001 C CNN
F 3 "~" H 9750 3000 50  0001 C CNN
	1    9750 3000
	-1   0    0    1   
$EndComp
Wire Wire Line
	9550 3000 8500 3000
Wire Wire Line
	4550 4700 4700 4700
Wire Wire Line
	4700 4700 4850 4700
Wire Wire Line
	4550 3900 5700 3900
Wire Wire Line
	4650 4000 5850 4000
Wire Wire Line
	4750 4100 5800 4100
Wire Wire Line
	4850 4200 5750 4200
$EndSCHEMATC
