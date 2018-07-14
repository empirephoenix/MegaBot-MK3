#include "MainController.hpp"
#include <Arduino.h>

#define NUM_CHANNELS 8
#define CHANNEL_1 0
#define CHANNEL_2 1
#define CHANNEL_3 2
#define CHANNEL_4 3
#define CHANNEL_5 4
#define CHANNEL_6 5
#define CHANNEL_7 6
#define CHANNEL_8 7

byte last_flank[NUM_CHANNELS];

volatile int receiver_input[NUM_CHANNELS];
volatile int raw_inputs[NUM_CHANNELS];

//not volatile only interrupt handler
unsigned long current_time_int0, upflank_time[NUM_CHANNELS];
unsigned long loop_timer;
int mappedTo;


SoftWire sw1(8,9);
SoftWire sw2(6,7);
MCP4725 m1;
MCP4725 m2;
void setup() {
  Serial.begin(115200);

  PCICR |= (1 << PCIE0);          //Set PCIE0 to enable PCMSK0 scan.
  PCMSK0 |= B11111111;

  m1.begin(0x60, &sw1);
  m2.begin(0x60, &sw2);

  Serial.println("Done init");
}

void loop() {
	if (Serial.available() > 1) {

		mappedTo = Serial.parseInt();
		m1.setVoltage(mappedTo);
		m2.setVoltage(4095-mappedTo);
		Serial.println(mappedTo);
	}


}


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
