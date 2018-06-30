#define NUM_CHANNELS 8
#define CHANNEL_1 0
#define CHANNEL_2 1
#define CHANNEL_3 2
#define CHANNEL_4 3
#define CHANNEL_5 4
#define CHANNEL_6 5
#define CHANNEL_7 6
#define CHANNEL_8 7

#include "MCP4725.h"

byte last_flank[NUM_CHANNELS];

volatile int receiver_input[NUM_CHANNELS];
volatile int raw_inputs[NUM_CHANNELS];

//not volatile only interrupt handler
unsigned long current_time_int0, upflank_time[NUM_CHANNELS];
unsigned long loop_timer;

/* @name  setup()
 *1  PG5 OC0B      Digital pin 04  PWM   
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
 *
 *
 * @desc  This function is the function that gets run on arduino startup.
 */

MCP4725 m1;
void setup() {
  Serial.begin(115200);
  
  PCICR |= (1 << PCIE0);          //Set PCIE0 to enable PCMSK0 scan.
  PCMSK0 |= B11111111;

  m1.begin(0x62);
  m1.setFastMode();
  m1.setVoltageFast(128);

  Serial.println("Initializing gyro...");
  Serial.println("done.");
}

/*
 * @name  loop()
 *
 * @desc  Main loop of arduino. Used to do all the calculations and generate the pulses for the ESCs.
 */
void loop() {
  limit_receiver_input(CHANNEL_1);
  limit_receiver_input(CHANNEL_2);
  limit_receiver_input(CHANNEL_3);
  limit_receiver_input(CHANNEL_4);
  limit_receiver_input(CHANNEL_5);
  limit_receiver_input(CHANNEL_6);
  limit_receiver_input(CHANNEL_7);
  limit_receiver_input(CHANNEL_8);

  Serial.print(raw_inputs[CHANNEL_1]);
  Serial.print("\t");
  Serial.print(raw_inputs[CHANNEL_2]);
  Serial.print("\t");
  Serial.print(raw_inputs[CHANNEL_3]);
  Serial.print("\t");
  Serial.print(raw_inputs[CHANNEL_4]);
  Serial.print("\t");
  Serial.print(raw_inputs[CHANNEL_5]);
  Serial.print("\t");
  Serial.print(raw_inputs[CHANNEL_6]);
  Serial.print("\t");
  Serial.print(raw_inputs[CHANNEL_7]);
  Serial.print("\t");
  Serial.print(raw_inputs[CHANNEL_8]);
  Serial.print("\n");
}

/* @name  ISR(PCINT0_vect) 
 *
 * @desc  Interrupt service routine for handling the joystick inputs.
 */ 

 
 

 
ISR(PCINT0_vect){
  current_time_int0 = micros();

  //Channel 1
  if(PINB & B00000001){
    if(last_flank[CHANNEL_1] == 0){
      last_flank[CHANNEL_1] = 1;
      upflank_time[CHANNEL_1] = current_time_int0;
    }
  }
  else if(last_flank[CHANNEL_1] == 1){
    last_flank[CHANNEL_1] = 0;
    raw_inputs[CHANNEL_1] = current_time_int0 - upflank_time[CHANNEL_1];
  }

  //Channel 2
  if(PINB & B00000010){
    if(last_flank[CHANNEL_2] == 0){
      last_flank[CHANNEL_2] = 1;
      upflank_time[CHANNEL_2] = current_time_int0;
    }
  }
  else if(last_flank[CHANNEL_2] == 1){
    last_flank[CHANNEL_2] = 0;
    raw_inputs[CHANNEL_2] = current_time_int0 - upflank_time[CHANNEL_2];
  }

  //Channel 3
  if(PINB & B00000100){
    if(last_flank[CHANNEL_3] == 0){
      last_flank[CHANNEL_3] = 1;
      upflank_time[CHANNEL_3] = current_time_int0;
    }
  }
  else if(last_flank[CHANNEL_3] == 1){
    last_flank[CHANNEL_3] = 0;
    raw_inputs[CHANNEL_3] = current_time_int0 - upflank_time[CHANNEL_3];
  }

  //Channel 4
  if(PINB & B00001000){
    if(last_flank[CHANNEL_4] == 0){
      last_flank[CHANNEL_4] = 1;
      upflank_time[CHANNEL_4] = current_time_int0;
    }
  }
  else if(last_flank[CHANNEL_4] == 1){
    last_flank[CHANNEL_4] = 0;
    raw_inputs[CHANNEL_4] = current_time_int0 - upflank_time[CHANNEL_4];
  }

  //Channel 5 (Hovermode)
  if(PINB & B00010000){
     if(last_flank[CHANNEL_5] == 0){
       last_flank[CHANNEL_5] = 1;
       upflank_time[CHANNEL_5] = current_time_int0;
     }
  }
  else if(last_flank[CHANNEL_5] == 1){
     last_flank[CHANNEL_5] = 0;
     raw_inputs[CHANNEL_5] = current_time_int0 - upflank_time[CHANNEL_5];
  }

    //Channel 5 (Hovermode)
  if(PINB & B00100000){
     if(last_flank[CHANNEL_6] == 0){
       last_flank[CHANNEL_6] = 1;
       upflank_time[CHANNEL_6] = current_time_int0;
     }
  }
  else if(last_flank[CHANNEL_6] == 1){
     last_flank[CHANNEL_6] = 0;
     raw_inputs[CHANNEL_6] = current_time_int0 - upflank_time[CHANNEL_6];
  }

      //Channel 5 (Hovermode)
  if(PINB & B01000000){
     if(last_flank[CHANNEL_7] == 0){
       last_flank[CHANNEL_7] = 1;
       upflank_time[CHANNEL_7] = current_time_int0;
     }
  }
  else if(last_flank[CHANNEL_7] == 1){
     last_flank[CHANNEL_7] = 0;
     raw_inputs[CHANNEL_7] = current_time_int0 - upflank_time[CHANNEL_7];
  }

    
  if(PINB & B10000000){
     if(last_flank[CHANNEL_8] == 0){
       last_flank[CHANNEL_8] = 1;
       upflank_time[CHANNEL_8] = current_time_int0;
     }
  }
  else if(last_flank[CHANNEL_8] == 1){
     last_flank[CHANNEL_8] = 0;
     raw_inputs[CHANNEL_8] = current_time_int0 - upflank_time[CHANNEL_8];
  }
}

void limit_receiver_input(byte n) {
  int rc_signal = raw_inputs[n];
  if (rc_signal > 2000) rc_signal = 2000;
  if (rc_signal < 1000) rc_signal = 1000;
  receiver_input[n] = rc_signal;
}
