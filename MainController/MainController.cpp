#include "MainController.hpp"
#include "keys.hpp"
#include <APA102.h>
#include <OneWire.h>
#include <Arduino.h>

byte last_flank[NUM_CHANNELS];

volatile int receiver_input[NUM_CHANNELS];
volatile int raw_inputs[NUM_CHANNELS];

//not volatile only interrupt handler
unsigned long current_time_int0, upflank_time[NUM_CHANNELS];
unsigned long loop_timer, curTime;
byte mode, buf[8];
int motorPower, throttleAvrg, calibrateCount = 0;
bool enabled, forward, fastModeAvailable;

APA102<LED_DATA, LED_CLOCK> ledStrip;
rgb_color leds[LED_COUNT];

void out(int idx, byte r, byte g, byte b) {
	leds[idx].red = r;
	leds[idx].green = g;
	leds[idx].blue = b;
}

void setup() {
	Serial.begin(115200);

	pinMode(PIN_CALIBRATE, INPUT);
	pinMode(PIN_BRAKING_STAGE_ONE, OUTPUT);
	pinMode(PIN_BRAKING_STAGE_TWO, OUTPUT);
	pinMode(PIN_MOTOR_CONTROLLER_PWR, OUTPUT);
	pinMode(PIN_REVERSE, OUTPUT);
	pinMode(PIN_IBUTTON, OUTPUT);

	analogWrite(PIN_IBUTTON_LED, 0);

	PCICR |= (1 << PCIE0);          //Set PCIE0 to enable PCMSK0 scan.
	PCMSK0 |= 0x0F;

	// each begin hangs until module is connected
	vorne_links.begin(0x60, &sw1);
	vorne_rechts.begin(0x60, &sw2);
	hinten_rechts.begin(0x60, &sw4);
	hinten_links.begin(0x60, &sw3);

	Serial.println("Done init");
	//calibrate();
	out(0, 0, 0, 0);
	out(1, 0, 0, 0);
	out(2, 0, 0, 0);

	throttleAvrg = 1500;
	forward = true;

	startup();
	fail();
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

	/*if (Serial.available() > 1) {
	 int mappedTo = Serial.parseInt();

	 Serial.println(mappedTo);
	 }*/

	if (enabled) {
		throttleAvrg = throttleAvrg * 0.9 + raw_inputs[THROTTLE] * 0.1;

		// Mode switch TODO: Later needed
		if (fastModeAvailable){
			if (raw_inputs[MODE] > 1750) {
				mode = 2;
				out(0, 0, 127, 0);
			} else if (raw_inputs[MODE] > 1250) {
				mode = 1;
				out(0, 0, 0, 127);
			} else {
				mode = 0;
				out(0, 127, 0, 0);
			}
		} else {
			mode = 2;
			out(0, 0, 127, 0);
		}

		forward = raw_inputs[FORWARD] < 1250;
		// Throttle / Braking
		if (throttleAvrg > 1520) { //Throttle

			//Throttle calculation
			int mappedThrottle = map(raw_inputs[THROTTLE], 1520, 2000, 0, 255);
			int thr = map(raw_inputs[THROTTLE], 1520, 2000, 1000, 4095);
			if (forward) {
				out(1, 0, 0, mappedThrottle);
				vorne_links.setVoltage(thr);
				vorne_rechts.setVoltage(thr);
				hinten_rechts.setVoltage(thr);
				hinten_links.setVoltage(thr);
			} else { // Reverse
				out(1, 0, mappedThrottle, 0);
				vorne_links.setVoltage(thr);
				vorne_rechts.setVoltage(thr);
				hinten_rechts.setVoltage(0);
				hinten_links.setVoltage(0);
			}

		} else if (throttleAvrg < 1400) { //Braking
			vorne_links.setVoltage(0);
			vorne_rechts.setVoltage(0);
			hinten_rechts.setVoltage(0);
			hinten_links.setVoltage(0);
			if (throttleAvrg > 1050) {
				out(1, 127, 0, 0);
				digitalWrite(PIN_BRAKING_STAGE_ONE, HIGH);
				digitalWrite(PIN_BRAKING_STAGE_TWO, LOW);
			} else {
				digitalWrite(PIN_BRAKING_STAGE_ONE, HIGH);
				digitalWrite(PIN_BRAKING_STAGE_TWO, HIGH);
				out(1, 255, 0, 0);
			}
		} else {
			vorne_links.setVoltage(0);
			vorne_rechts.setVoltage(0);
			hinten_rechts.setVoltage(0);
			hinten_links.setVoltage(0);
			out(1, 127, 127, 0);
			digitalWrite(PIN_BRAKING_STAGE_ONE, LOW);
			digitalWrite(PIN_BRAKING_STAGE_TWO, LOW);
			handleKeys();
		}

		digitalWrite(PIN_REVERSE, forward);

		if (raw_inputs[STEERING] > 1540) {
			out(2, 0, 127, 0);
		} else if (raw_inputs[STEERING] < 1460) {
			out(2, 127, 0, 0);
		} else {
			out(2, 127, 127, 127);
		}

		unsigned long tmp[4];
		tmp[0] = upflank_time[0];
		tmp[1] = upflank_time[1];
		tmp[2] = upflank_time[2];
		tmp[3] = upflank_time[3];

		bool error = false;
		curTime = micros();
		if (curTime - tmp[0] > THRESHOLD)
			error = true;
		if (curTime - tmp[1] > THRESHOLD)
			error = true;
		if (curTime - tmp[2] > THRESHOLD)
			error = true;
		if (curTime - tmp[3] > THRESHOLD)
			error = true;
		if (error) {
			out(0, 255, 0, 0);
			out(1, 255, 0, 0);
			out(2, 255, 0, 0);
		}
	} else {
		handleKeys();
	}

	ledStrip.write(leds, LED_COUNT, 4);
}

void calibrate() {
	//vorne_links.setVoltageAndSave(0);
	vorne_rechts.setVoltageAndSave(0);

	// TODO: Handle updating of LEDs while calibrating (blocking function)
	//ledStrip.write(leds, LED_COUNT, 4);
}

void validateDAC(byte idx) {
}

void handleKeys() {

	if (!ibutton.search(buf)) {
		ibutton.reset_search();
		return;
	} else {
		if (buf[0] == 0x01) { // check if its an iButton
			for (int x = 0; x < 8; x++) {
				Serial.print(buf[x], HEX);
				Serial.print(" ");
			}
			Serial.println("");
			//delay(1000);
			bool found = false;
			for (byte x = 0; x < NUM_KEYS; x++) {
				if (memcmp((void*) keys[x], (void*) buf, 8) == 0) {
					found = true;
					fastModeAvailable = keys[x][9];
					Serial.println(fastModeAvailable);
					enable();
					return;
				}
			}
		}
	}

	for (int i = 0; i < 20; i++) {
		digitalWrite(PIN_IBUTTON_LED, i % 2);
		delay(100);
	}

	fail();

	/*if (ibutton_led_status) {
	 ibutton_led_status = false;
	 }
	 digitalWrite(PIN_IBUTTON_LED, ibutton_led_status);*/
}

bool checkValidKey() {
	for (int x = 0; x < 8; x++) {
		Serial.print(buf[x], HEX);
		Serial.print(" ");
	}
	Serial.println("");
//delay(1000);

	for (byte x = 0; x < NUM_KEYS; x++) {
		if (memcmp((void*) keys[x], (void*) buf, 8) == 0) {
			//Serial.print("verified key ");
			//Serial.println(x);
			if (keys[x][9]) {

			}
			return true;
		}
	}
	return false;
//Serial.println("");
}

void startup() {
	bool error = false;
	vorne_links.setVoltage(0);
	vorne_rechts.setVoltage(0);
	hinten_rechts.setVoltage(0);
	hinten_links.setVoltage(0);
	delay(5);
	if (analogRead(0) > 10)
		error = true;
	if (analogRead(1) > 10)
		error = true;
	if (analogRead(2) > 10)
		error = true;
	if (analogRead(3) > 10)
		error = true;

	if (error) {
		out(0, 255, 0, 0);
		out(1, 0, 0, 0);
		out(2, 0, 127, 0);
		ledStrip.write(leds, LED_COUNT, 4);
		Serial.println(analogRead(0) / 1024 * 5);
		Serial.println(analogRead(1) / 1024 * 5);
		Serial.println(analogRead(2) / 1024 * 5);
		Serial.println(analogRead(3) / 1024 * 5);
		while (error)
			;
	}

	vorne_links.setVoltage(1024);
	vorne_rechts.setVoltage(1024);
	hinten_rechts.setVoltage(1024);
	hinten_links.setVoltage(1024);
	delay(5);
	if (analogRead(0) < 220)
		error = true;
	if (analogRead(1) < 220)
		error = true;
	if (analogRead(2) < 220)
		error = true;
	if (analogRead(3) < 220)
		error = true;

	if (error) {
		out(0, 255, 0, 0);
		out(1, 0, 0, 0);
		out(2, 0, 0, 127);
		ledStrip.write(leds, LED_COUNT, 4);
		Serial.println(analogRead(0) / 1024 * 5);
		Serial.println(analogRead(1) / 1024 * 5);
		Serial.println(analogRead(2) / 1024 * 5);
		Serial.println(analogRead(3) / 1024 * 5);
		while (error)
			;
	}
}

void enable() {
	enabled = true;
	digitalWrite(PIN_MOTOR_CONTROLLER_PWR, LOW);
	digitalWrite(PIN_BRAKING_STAGE_ONE, LOW);
	digitalWrite(PIN_BRAKING_STAGE_TWO, LOW);
	digitalWrite(PIN_IBUTTON_LED, LOW);
}

void fail() {
	enabled = false;
	digitalWrite(PIN_MOTOR_CONTROLLER_PWR, HIGH);
	digitalWrite(PIN_BRAKING_STAGE_ONE, HIGH);
	digitalWrite(PIN_BRAKING_STAGE_TWO, HIGH);
	digitalWrite(PIN_IBUTTON_LED, HIGH);
}

ISR(PCINT0_vect) {
	current_time_int0 = micros();

//Channel 1
	if (PINB & B00000001) {
		if (last_flank[MODE] == 0) {
			last_flank[MODE] = 1;
			upflank_time[MODE] = current_time_int0;
		}
	} else if (last_flank[MODE] == 1) {
		last_flank[MODE] = 0;
		raw_inputs[MODE] = current_time_int0 - upflank_time[MODE];
	}

//Channel 2
	if (PINB & B00000010) {
		if (last_flank[FORWARD] == 0) {
			last_flank[FORWARD] = 1;
			upflank_time[FORWARD] = current_time_int0;
		}
	} else if (last_flank[FORWARD] == 1) {
		last_flank[FORWARD] = 0;
		raw_inputs[FORWARD] = current_time_int0 - upflank_time[FORWARD];
	}

//Channel 3
	if (PINB & B00000100) {
		if (last_flank[STEERING] == 0) {
			last_flank[STEERING] = 1;
			upflank_time[STEERING] = current_time_int0;
		}
	} else if (last_flank[STEERING] == 1) {
		last_flank[STEERING] = 0;
		raw_inputs[STEERING] = current_time_int0 - upflank_time[STEERING];
	}

//Channel 4
	if (PINB & B00001000) {
		if (last_flank[THROTTLE] == 0) {
			last_flank[THROTTLE] = 1;
			upflank_time[THROTTLE] = current_time_int0;
		}
	} else if (last_flank[THROTTLE] == 1) {
		last_flank[THROTTLE] = 0;
		raw_inputs[THROTTLE] = current_time_int0 - upflank_time[THROTTLE];
	}
}
