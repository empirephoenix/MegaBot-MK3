#include "MainController.hpp"
#include "keys.hpp"
#include <APA102.h>
#include <OneWire.h>
#include <Arduino.h>

byte last_flank_rank0[NUM_CHANNELS];
byte last_flank_rank2[NUM_HALL_SENSORS];

volatile int receiver_input[NUM_CHANNELS];
volatile int raw_inputs[NUM_CHANNELS];
volatile int raw_inputs_rank2[NUM_HALL_SENSORS];

//not volatile only interrupt handler
unsigned long current_time_int0, current_time_int_rank2, upflank_time0[NUM_CHANNELS], upflank_time_rank2[NUM_HALL_SENSORS];
unsigned long loop_timer, curTime;
byte mode, buf[8];
int motorPower, throttleAvrg, calibrateCount = 0;
float currentSpeed = 0;
unsigned long nextSpeedCheck = 0;
volatile unsigned int vlkss = 0;
bool enabled, forward, fastModeAvailable;

APA102<PIN_LED_DATA, PIN_LED_CLOCK> ledStrip;
rgb_color leds[LED_COUNT];

void out(int idx, byte r, byte g, byte b) {
	leds[idx].red = r;
	leds[idx].green = g;
	leds[idx].blue = b;
}

void countvri(){
	vlkss++;
}

void setup() {
	Serial.begin(115200);

	pinMode(PIN_BRAKING_STAGE_ONE, OUTPUT);
	pinMode(PIN_BRAKING_STAGE_TWO, OUTPUT);
	pinMode(PIN_MOTOR_CONTROLLER_PWR, OUTPUT);
	pinMode(PIN_REVERSE, OUTPUT);
	pinMode(PIN_IBUTTON, OUTPUT);

	analogWrite(PIN_IBUTTON_LED, 0);

	PCICR |= (1 << PCIE0);          //Set PCIE0 to enable PCMSK0 scan.
	PCMSK0 |= 0x0F;

	PCICR |= (1 << PCIE2);
	PCMSK2 |= 0x03;
	//88  PK1 ADC9  PCINT17   Analog pin 09
	//89  PK0 ADC8  PCINT16   Analog pin 08


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
	nextSpeedCheck = millis();
	startup();
	fail();
}

void stopMotors() {
	//Braking
	vorne_links.setVoltage(0);
	vorne_rechts.setVoltage(0);
	hinten_rechts.setVoltage(0);
	hinten_links.setVoltage(0);
}

void handleNormalThrust() {
	long int tledLevel;
	tledLevel = map(raw_inputs[THROTTLE], 1520, 2000, 0, 255);
	out(1, 0, forward ? tledLevel : 0, forward ? 0 : tledLevel);

	int thr = map(raw_inputs[THROTTLE], 1520, 2000, 1000, 4095);
	vorne_links.setVoltage(thr);
	vorne_rechts.setVoltage(thr);
	hinten_rechts.setVoltage(forward ? thr : 0);
	hinten_links.setVoltage(forward ? thr : 0);
}

void handleBraking() {
	//Braking
	stopMotors();
	bool maxBrake = throttleAvrg < 1050;
	out(1, maxBrake ? 255 : 127, 0, 0);
	digitalWrite(PIN_BRAKING_STAGE_ONE, HIGH);
	digitalWrite(PIN_BRAKING_STAGE_TWO, maxBrake ? HIGH : LOW);
}

void handlePause() {
	stopMotors();
	out(1, 127, 127, 0);
	digitalWrite(PIN_BRAKING_STAGE_ONE, LOW);
	digitalWrite(PIN_BRAKING_STAGE_TWO, LOW);
	handleKeys();
	forward = raw_inputs[FORWARD] < 1250;
}

void updateTurningStatusLED() {
	if (raw_inputs[STEERING] > 1540) {
		out(2, 0, 127, 0);
	} else if (raw_inputs[STEERING] < 1460) {
		out(2, 127, 0, 0);
	} else {
		out(2, 127, 127, 127);
	}
}

void handleModeInput() {
	// Mode switch TODO: Later needed
	if (fastModeAvailable) {
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
}

void checkSpeed(){
	nextSpeedCheck = millis()+1000;
	Serial.print("Speed per second ");
	Serial.println(vlkss);
	vlkss = 0;
//	vrkss = 0;
//	hlkss = 0;
//	hrkss = 0;
}

void loop() {
	Serial.print("Test ");
	Serial.print(raw_inputs_rank2[HALL_SENSOR2]);
	Serial.println();
	if(nextSpeedCheck < millis()){
//		checkSpeed();
	}

	if (enabled) {
		throttleAvrg = throttleAvrg * 0.9 + raw_inputs[THROTTLE] * 0.1;

		handleModeInput();

		// Throttle / Braking
		if (throttleAvrg > 1520) { //Throttle
			handleNormalThrust();
		} else if (throttleAvrg < 1400) { //Braking
			handleBraking();
		} else if (currentSpeed < 1) {
			handlePause();
		}

		digitalWrite(PIN_REVERSE, forward);

		updateTurningStatusLED();

		unsigned long tmp[4];
		tmp[0] = upflank_time0[0];
		tmp[1] = upflank_time0[1];
		tmp[2] = upflank_time0[2];
		tmp[3] = upflank_time0[3];

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
	} else if (currentSpeed < 1) {
		handleKeys();
	}

	ledStrip.write(leds, LED_COUNT, 4);
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
}

bool checkValidKey() {
	for (int x = 0; x < 8; x++) {
		Serial.print(buf[x], HEX);
		Serial.print(" ");
	}
	Serial.println("");

	for (byte x = 0; x < NUM_KEYS; x++) {
		if (memcmp((void*) keys[x], (void*) buf, 8) == 0) {
			return true;
		}
	}
	return false;
}

void hangWithDACError(bool error) {
	out(0, 255, 0, 0);
	out(1, 0, 0, 0);
	ledStrip.write(leds, LED_COUNT, 4);
	Serial.println("VORNE_LINKS_OUTPUT");
	Serial.println(analogRead(VORNE_LINKS_OUTPUT) / 1024 * 5);
	Serial.println("VORNE_RECHTS_OUTPUT");
	Serial.println(analogRead(VORNE_RECHTS_OUTPUT) / 1024 * 5);
	Serial.println("HINTEN_LINKS_OUTPUT");
	Serial.println(analogRead(HINTEN_LINKS_OUTPUT) / 1024 * 5);
	Serial.println("HINTEN_RECHTS_OUTPUT");
	Serial.println(analogRead(HINTEN_RECHTS_OUTPUT) / 1024 * 5);
	while (error)
		;

}

void startup() {
	bool error = false;
	stopMotors();
	delay(5);
	Serial.println("Initial to 0V DAC test");
	if (analogRead(VORNE_LINKS_OUTPUT) > 10)
		error = true;
	if (analogRead(VORNE_RECHTS_OUTPUT) > 10)
		error = true;
	if (analogRead(HINTEN_LINKS_OUTPUT) > 10)
		error = true;
	if (analogRead(HINTEN_RECHTS_OUTPUT) > 10)
		error = true;

	if (error) {
		Serial.println("Abort startup, initial validation of DAC failed");
		out(2, 0, 127, 0);
		hangWithDACError(error);
	}
	Serial.println("Low Test passed, starting 5V test");

	vorne_links.setVoltage(1024);
	vorne_rechts.setVoltage(1024);
	hinten_rechts.setVoltage(1024);
	hinten_links.setVoltage(1024);
	delay(5);
	if (analogRead(VORNE_LINKS_OUTPUT) < 220)
		error = true;
	if (analogRead(VORNE_RECHTS_OUTPUT) < 220)
		error = true;
	if (analogRead(HINTEN_LINKS_OUTPUT) < 220)
		error = true;
	if (analogRead(HINTEN_RECHTS_OUTPUT) < 220)
		error = true;

	if (error) {
		out(2, 0, 0, 127);
		hangWithDACError(error);
	}
	Serial.println("Startup tests passed");
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

ISR(PCINT2_vect) {
	current_time_int_rank2 = micros();

	//Sensor 1
	if (PINK & B00000001) {
		if (last_flank_rank2[HALL_SENSOR1] == 0) {
			last_flank_rank2[HALL_SENSOR1] = 1;
			upflank_time_rank2[HALL_SENSOR1] = current_time_int_rank2;
		}
	} else if (last_flank_rank2[HALL_SENSOR1] == 1) {
		last_flank_rank2[HALL_SENSOR1] = 0;
		raw_inputs_rank2[HALL_SENSOR1] = current_time_int_rank2 - upflank_time_rank2[HALL_SENSOR1];
	}

	//Sensor 2
	if (PINK & B00000010) {
		if (last_flank_rank2[HALL_SENSOR2] == 0) {
			last_flank_rank2[HALL_SENSOR2] = 1;
			upflank_time_rank2[HALL_SENSOR2] = current_time_int_rank2;
		}
	} else if (last_flank_rank2[HALL_SENSOR2] == 1) {
		last_flank_rank2[HALL_SENSOR2] = 0;
		raw_inputs_rank2[HALL_SENSOR2] = current_time_int_rank2 - upflank_time_rank2[HALL_SENSOR2];
	}
}

ISR(PCINT0_vect) {
	current_time_int0 = micros();

//Channel 1
	if (PINB & B00000001) {
		if (last_flank_rank0[MODE] == 0) {
			last_flank_rank0[MODE] = 1;
			upflank_time0[MODE] = current_time_int0;
		}
	} else if (last_flank_rank0[MODE] == 1) {
		last_flank_rank0[MODE] = 0;
		raw_inputs[MODE] = current_time_int0 - upflank_time0[MODE];
	}

//Channel 2
	if (PINB & B00000010) {
		if (last_flank_rank0[FORWARD] == 0) {
			last_flank_rank0[FORWARD] = 1;
			upflank_time0[FORWARD] = current_time_int0;
		}
	} else if (last_flank_rank0[FORWARD] == 1) {
		last_flank_rank0[FORWARD] = 0;
		raw_inputs[FORWARD] = current_time_int0 - upflank_time0[FORWARD];
	}

//Channel 3
	if (PINB & B00000100) {
		if (last_flank_rank0[STEERING] == 0) {
			last_flank_rank0[STEERING] = 1;
			upflank_time0[STEERING] = current_time_int0;
		}
	} else if (last_flank_rank0[STEERING] == 1) {
		last_flank_rank0[STEERING] = 0;
		raw_inputs[STEERING] = current_time_int0 - upflank_time0[STEERING];
	}

//Channel 4
	if (PINB & B00001000) {
		if (last_flank_rank0[THROTTLE] == 0) {
			last_flank_rank0[THROTTLE] = 1;
			upflank_time0[THROTTLE] = current_time_int0;
		}
	} else if (last_flank_rank0[THROTTLE] == 1) {
		last_flank_rank0[THROTTLE] = 0;
		raw_inputs[THROTTLE] = current_time_int0 - upflank_time0[THROTTLE];
	}
}
