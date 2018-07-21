#include "MainController.hpp"
#include <APA102.h>
#include <Arduino.h>

byte last_flank[NUM_CHANNELS];

volatile int receiver_input[NUM_CHANNELS];
volatile int raw_inputs[NUM_CHANNELS];

//not volatile only interrupt handler
unsigned long current_time_int0, upflank_time[NUM_CHANNELS];
unsigned long loop_timer;
int motorPower;
byte mode;
int throttleAvrg;

int calibrateCount = 0;

APA102<LED_DATA, LED_CLOCK> ledStrip;
rgb_color leds[LED_COUNT];

void out(int idx, byte r, byte g, byte b) {
	leds[idx].red = r;
	leds[idx].green = g;
	leds[idx].blue = b;
}

void setup() {
  Serial.begin(115200);

  pinMode(12, INPUT);

  PCICR |= (1 << PCIE0);          //Set PCIE0 to enable PCMSK0 scan.
  PCMSK0 |= 0x0F;

  // each begin hangs until module is connected
  m1.begin(0x60, &sw1);
  m2.begin(0x60, &sw2);
  m3.begin(0x60, &sw3);
  m4.begin(0x60, &sw4);

  Serial.println("Done init");
  //calibrate();
  out(0,0,0,0);
  out(1,0,0,0);
  out(2,0,0,0);

  throttleAvrg = 1500;
}

void loop() {
/*	if (digitalRead(12)) {
		calibrateCount++;
		out (0, COLOR_CALIBRATE_HOLD);
		if (calibrateCount== 1000) {
			calibrateCount = 0;
			out (0, COLOR_CALIBRATE_EXECUTE);
			calibrate();
		}
	} else {
		calibrateCount = 0;
	}*/


	if (Serial.available() > 1) {
		int mappedTo = Serial.parseInt();
		m1.setVoltage(mappedTo);
		m2.setVoltage(mappedTo);
		m3.setVoltage(mappedTo);
		m4.setVoltage(mappedTo);
		Serial.println(mappedTo);
	}

	throttleAvrg = throttleAvrg * 0.9 + raw_inputs[THROTTLE] * 0.1;

	if (raw_inputs[MODE] > 1750){
		mode = 2;
		out(0,0,127,0);
	} else if (raw_inputs[MODE] > 1250) {
		mode = 1;
		out(0,0,0,127);
	} else {
		mode = 0;
		out(0,127,0,0);
	}

	if (throttleAvrg > 1520) {
		int mappedThrottle = map(raw_inputs[THROTTLE],1520,2000,0,255);
		if (raw_inputs[REVERSE] > 1250) {
			out(1,0,mappedThrottle,0);
		} else {
			out(1,0,0,mappedThrottle);
		}
	} else if (throttleAvrg < 1400){
		if (throttleAvrg > 1050) {
			out(1,127,0,0);
		} else {
			out(1,255,0,0);
		}
	} else {
		out(1,127,127,0);
	}

	if (raw_inputs[STEERING] > 1540) {
		out(2,0,127,0);
	} else if (raw_inputs[STEERING] < 1460){
		out(2,127,0,0);
	} else {
		out(2,127,127,127);
	}
	ledStrip.write(leds, LED_COUNT, 4);
}

void calibrate() {
	//m1.setVoltageAndSave(0);
	m2.setVoltageAndSave(0);

	// TODO: Handle updating of LEDs while calibrating (blocking function)
	//ledStrip.write(leds, LED_COUNT, 4);
}

ISR(PCINT0_vect){
  current_time_int0 = micros();

  //Channel 1
  if(PINB & B00000001){
    if(last_flank[MODE] == 0){
      last_flank[MODE] = 1;
      upflank_time[MODE] = current_time_int0;
    }
  }
  else if(last_flank[MODE] == 1){
    last_flank[MODE] = 0;
    raw_inputs[MODE] = current_time_int0 - upflank_time[MODE];
  }

  //Channel 2
  if(PINB & B00000010){
    if(last_flank[REVERSE] == 0){
      last_flank[REVERSE] = 1;
      upflank_time[REVERSE] = current_time_int0;
    }
  }
  else if(last_flank[REVERSE] == 1){
    last_flank[REVERSE] = 0;
    raw_inputs[REVERSE] = current_time_int0 - upflank_time[REVERSE];
  }

  //Channel 3
  if(PINB & B00000100){
    if(last_flank[STEERING] == 0){
      last_flank[STEERING] = 1;
      upflank_time[STEERING] = current_time_int0;
    }
  }
  else if(last_flank[STEERING] == 1){
    last_flank[STEERING] = 0;
    raw_inputs[STEERING] = current_time_int0 - upflank_time[STEERING];
  }

  //Channel 4
  if(PINB & B00001000){
    if(last_flank[THROTTLE] == 0){
      last_flank[THROTTLE] = 1;
      upflank_time[THROTTLE] = current_time_int0;
    }
  }
  else if(last_flank[THROTTLE] == 1){
    last_flank[THROTTLE] = 0;
    raw_inputs[THROTTLE] = current_time_int0 - upflank_time[THROTTLE];
  }
}
