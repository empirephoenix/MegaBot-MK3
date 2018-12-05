#include "MCP4725.h"
#include <SoftWire.h>
#include <OneWire.h>

/*
1  PG5 OC0B      Digital pin 04  PWM
2 PE0 RXD0  PCINT8    Digital pin 00    RX0 (USART0 Receive Pin, Programming Data Input or Pin Change Interrupt 8)
3 PE1 TXD0      Digital pin 01    TX0 (USART0 Transmit Pin or Programming Data Output)
4 PE2 XCK0  AIN0          (USART0 External Clock  Input/Output)
5 PE3 OC3A  AIN1    Digital pin 05  PWM
6 PE4 OC3B  INT4    Digital pin 02  PWM
7 PE5 OC3C  INT5    Digital pin 03  PWM
8 PE6 T3  INT6
9 PE7 CLKO  INT7  ICP3
10  VCC VCC
11  GND GND
12  PH0 RXD2      Digital pin 17    RX2 (USART2 Receive Pin)
13  PH1 TXD2      Digital pin 16    TX2 (USART2 Transmit Pin)
14  PH2 XCK2            (USART2 External Clock  Input/Output)
15  PH3 OC4A      Digital pin 06  PWM
16  PH4 OC4B      Digital pin 07  PWM
17  PH5 OC4C      Digital pin 08  PWM
18  PH6 OC2B      Digital pin 09  PWM
19  PB0 SS  PCINT0    Digital pin 53    SS  (SPI Slave Select input or Pin Change Interrupt 0)
20  PB1 SCK PCINT1    Digital pin 52    SCK (SPI Bus Serial Clock or Pin Change Interrupt 1)
21  PB2 MOSI  PCINT2    Digital pin 51    MOSI  (SPI Bus Master Output/Slave Input or Pin Change Interrupt 2)
22  PB3 MISO  PCINT3    Digital pin 50    MISO  (SPI Bus Master Input/Slave Output or Pin Change Interrupt 3)
23  PB4 OC2A  PCINT4    Digital pin 10  PWM
24  PB5 OC1A  PCINT5    Digital pin 11  PWM
25  PB6 OC1B  PCINT6    Digital pin 12  PWM
26  PB7 OC0A  PCINT7  OC1C  Digital pin 13  PWM
27  PH7 T4
28  PG3 TOSC2
29  PG4 TOSC1
30  RESET RESET
31  VCC VCC
32  GND GND
33  XTAL2 XTAL2
34  XTAL1 XTAL1
35  PL0 ICP4      Digital pin 49
36  PL1 ICP5      Digital pin 48
37  PL2 T5      Digital pin 47
38  PL3 OC5A      Digital pin 46  PWM
39  PL4 OC5B      Digital pin 45  PWM
40  PL5 OC5C      Digital pin 44  PWM
41  PL6       Digital pin 43
42  PL7       Digital pin 42
43  PD0 SCL INT0    Digital pin 21  SCL   (TWI Serial Clock or External Interrupt0 Input)
44  PD1 SDA INT1    Digital pin 20  SDA   (TWI Serial Data or External Interrupt1 Input )
45  PD2 RXD1  INT2    Digital pin 19    RX1 (USART1 Receive Pin or External Interrupt2 Input)
46  PD3 TXD1  INT3    Digital pin 18    TX1 (USART1 Transmit Pin or External Interrupt3 Input)
47  PD4 ICP1
48  PD5 XCK1            (USART1 External Clock  Input/Output)
49  PD6 T1
50  PD7 T0      Digital pin 38
51  PG0 WR      Digital pin 41
52  PG1 RD      Digital pin 40
53  PC0 A8      Digital pin 37
54  PC1 A9      Digital pin 36
55  PC2 A10     Digital pin 35
56  PC3 A11     Digital pin 34
57  PC4 A12     Digital pin 33
58  PC5 A13     Digital pin 32
59  PC6 A14     Digital pin 31
60  PC7 A15     Digital pin 30
61  VCC VCC
62  GND GND
63  PJ0 RXD3  PCINT9    Digital pin 15    RX3 (USART3 Receive Pin or Pin Change Interrupt 9)
64  PJ1 TXD3  PCINT10   Digital pin 14    TX3 (USART3 Transmit Pin or Pin Change Interrupt 10)
65  PJ2 XCK3  PCINT11         (USART3 External Clock  Input/Output or Pin Change Interrupt 11)
66  PJ3   PCINT12
67  PJ4   PCINT13
68  PJ5   PCINT14
69  PJ6   PCINT 15
70  PG2 ALE     Digital pin 39
71  PA7 AD7     Digital pin 29
72  PA6 AD6     Digital pin 28
73  PA5 AD5     Digital pin 27
74  PA4 AD4     Digital pin 26
75  PA3 AD3     Digital pin 25
76  PA2 AD2     Digital pin 24
77  PA1 AD1     Digital pin 23
78  PA0 AD0     Digital pin 22
79  PJ7
80  VCC VCC
81  GND GND
82  PK7 ADC15 PCINT23   Analog pin 15
83  PK6 ADC14 PCINT22   Analog pin 14
84  PK5 ADC13 PCINT21   Analog pin 13
85  PK4 ADC12 PCINT20   Analog pin 12
86  PK3 ADC11 PCINT19   Analog pin 11
87  PK2 ADC10 PCINT18   Analog pin 10
88  PK1 ADC9  PCINT17   Analog pin 09
89  PK0 ADC8  PCINT16   Analog pin 08
90  PF7 ADC7  PCINT15   Analog pin 07
91  PF6 ADC6  PCINT14   Analog pin 06
92  PF5 ADC5  TMS   Analog pin 05
93  PF4 ADC4  TMK   Analog pin 04
94  PF3 ADC3      Analog pin 03
95  PF2 ADC2      Analog pin 02
96  PF1 ADC1      Analog pin 01
97  PF0 ADC0      Analog pin 00
98  AREF        Analog Reference
99  GND GND
100 AVCC  VCC
 */


#ifndef MAINCONTROLLER_HPP_
#define MAINCONTROLLER_HPP_

#define NUM_CHANNELS 4
#define NUM_HALL_SENSORS 1
#define HALL_SENSOR_VORNE_RECHTS 0

#define STEERING 0
#define THROTTLE 2
#define FORWARD 1
#define MODE 3

#define PIN_BRAKING_STAGE_ONE 24
#define PIN_BRAKING_STAGE_TWO 12
#define PIN_REVERSE A10
#define PIN_MOTOR_CONTROLLER_PWR 22
#define PIN_IBUTTON A1
#define PIN_IBUTTON_LED A0

#define LED_COUNT 3
#define PIN_LED_DATA 10
#define PIN_LED_CLOCK 11


#define LED_TOGGLE_TIME 10000
#define THRESHOLD 250000

#define VORNE_LINKS_INPUT A9 //HALL-IN PIN4
#define VORNE_RECHTS_INPUT A8 //HALL-IN PIN3
#define HINTEN_LINKS_INPUT A6 //HALL-IN PIN2
#define HINTEN_RECHTS_INPUT A7 //HALL-IN PIN1
#define SPEED_AVERAGING_TIME 250
#define HALL_PULSES_PER_ROTATION 30.0
#define WHEEL_DIAMETER_CM 78

#define TRIGGER 30

#define VORNE_LINKS_OUTPUT A2
#define VORNE_RECHTS_OUTPUT A3
#define HINTEN_LINKS_OUTPUT A4
#define HINTEN_RECHTS_OUTPUT A5

SoftWire sw1(9,8);
SoftWire sw2(7,6);
SoftWire sw3(5,4);
SoftWire sw4(3,2);
MCP4725 vorne_links;
MCP4725 vorne_rechts;
MCP4725 hinten_links;
MCP4725 hinten_rechts;

OneWire ibutton(PIN_IBUTTON);

void out(byte r, byte g, byte b);
void setup();
void loop();
void fail();
void enable();
void startup();
void handleKeys();

#endif /* MAINCONTROLLER_HPP_ */
