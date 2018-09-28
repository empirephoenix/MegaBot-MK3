#include "SecondaryController.h"
#include <EEPROM.h>
#include <APA102.h>

#define PIN_EXTEND 9
#define PIN_RETRACT 11
#define PIN_SERVO_POSITION A1
#define SERVO_STOP_CYCLES 60
#define DEADBAND 2
#define CALIBRATE_ANTI_BOUNCE_COUNT 1000

#define EEPROM_MIN_MAX_ADDR_OFFSET 0
#define NUM_CHANNELS 2
#define CHANNEL_1 0
#define CHANNEL_2 1

#define PIN_LED_DATA 7
#define PIN_LED_CLOCK 4
#define COLOR_DIM_GREEN (0, 32, 0)
#define COLOR_EXTEND (128,128,0)
#define COLOR_RETRACT (0,255,0)
#define COLOR_CALIBRATE_HOLD (0,0,128)
#define COLOR_CALIBRATE_EXECUTE (0,0,255)
#define COLOR_MOTOR_BLOCKING (200,255,255)
#define COLOR_ERROR (255,0,0)

APA102<PIN_LED_DATA, PIN_LED_CLOCK> ledStrip;
rgb_color leds[1];

float targetPos = 0;
float curPos = 0;
int calibrateCount = 0;

int delta;
int min = -1;
int max = -1;

int lastPos = 0, lastDir = 0;
int blockingCount = 0;
bool blocking = false;
bool error = false;

byte last_flank[NUM_CHANNELS];
volatile int receiver_input[NUM_CHANNELS];
volatile int raw_inputs[NUM_CHANNELS];
unsigned volatile long diff;
//not volatile only interrupt handler
unsigned long current_time_int0;
unsigned long upflank_time[NUM_CHANNELS];
unsigned long loop_timer;

void out(byte r, byte g, byte b) {
	leds[0].red = r;
	leds[0].green = g;
	leds[0].blue = b;
	ledStrip.write(leds, 1, 4);
}

void eepromWriteInt(int adr, int wert) {
	byte low = wert & 0xFF;
	byte high = (wert >> 8) & 0xFF;
	EEPROM.write(adr, low); // dauert 3,3ms
	EEPROM.write(adr + 1, high);
	return;
}

int eepromReadInt(int adr) {
	byte low = EEPROM.read(adr);
	byte high = EEPROM.read(adr + 1);
	return low + ((high << 8) & 0xFF00);
}

void calibrate() {
	boolean calibrating = true;
	while (calibrating) {
		int prior = analogRead(PIN_SERVO_POSITION);
		move(-26);
		delay(200);
		int after = analogRead(PIN_SERVO_POSITION);
		if (abs(prior - after) < DEADBAND * 2) {
			min = (prior + after) / 2;
			calibrating = false;
		}
	}
	calibrating = true;
	while (calibrating) {
		int prior = analogRead(PIN_SERVO_POSITION);
		move(26);
		delay(200);
		int after = analogRead(PIN_SERVO_POSITION);
		if (abs(prior - after) < DEADBAND * 2) {
			max = (prior + after) / 2;
			calibrating = false;
		}
	}
	Serial.print("min ");
	Serial.print(min);
	Serial.print(" max ");
	Serial.println(max);
	eepromWriteInt(EEPROM_MIN_MAX_ADDR_OFFSET + 0, min);
	eepromWriteInt(EEPROM_MIN_MAX_ADDR_OFFSET + 2, max);
	middleSteering();
}

void move(int delta) {
	int speed = abs(delta) > 10 ? 0 : 128;
	analogWrite(PIN_EXTEND, delta > 0 ? 255 : speed);
	analogWrite(PIN_RETRACT, delta < 0 ? 255 : speed);
}

void stop() {
	analogWrite(PIN_EXTEND, 255);
	analogWrite(PIN_RETRACT, 255);
}

void setup() {

	pinMode(13, INPUT_PULLUP);
	pinMode(PIN_EXTEND, OUTPUT);
	pinMode(PIN_RETRACT, OUTPUT);

	leds[0].red = 0;
	leds[0].green = 32;
	leds[0].blue = 0;
	void extend(int delta);
	Serial.begin(115200);
	pinMode(PIN_SERVO_POSITION, INPUT);

	min = eepromReadInt(0);
	max = eepromReadInt(2);
	Serial.print("reading calibration min ");
	Serial.print(min);
	Serial.print(" max ");
	Serial.println(max);
	if (min + 200 > max) {
		calibrate();
	}

	PCICR |= (1 << PCIE0);          //Set PCIE0 to enable PCMSK0 scan.
	PCMSK0 |= B00000101;

	curPos = analogRead(PIN_SERVO_POSITION);
	middleSteering();
	Serial.println(targetPos);
}

void middleSteering() {
	targetPos = min + (max - min) / 2;
}

void processCalibrate() {
	if (!digitalRead(13)) {
		calibrateCount++;
		out COLOR_CALIBRATE_HOLD;
		if (calibrateCount == CALIBRATE_ANTI_BOUNCE_COUNT) {
			updateLED();
			calibrate();
			calibrateCount = 0;
		}
	} else {
		calibrateCount = 0;
	}
}

void updateLED() {
	if (error) {
		out COLOR_ERROR;
		return;
	}

	if (calibrateCount == CALIBRATE_ANTI_BOUNCE_COUNT) {
		out COLOR_CALIBRATE_EXECUTE;
		return;
	}

	if (calibrateCount > 0) {
		out COLOR_CALIBRATE_HOLD;
		return;
	}

	if(blocking) {
		out COLOR_MOTOR_BLOCKING;
		return;
	}

	if (delta > DEADBAND) {
		out COLOR_EXTEND;
		return;
	}

	if (delta < -DEADBAND) {
		out COLOR_RETRACT;
		return;
	}

	out COLOR_DIM_GREEN;

}

void loop() {
	diff = micros() - current_time_int0;
	int sample = analogRead(PIN_SERVO_POSITION);
	if (sample >= min && sample <= max) {
		curPos = curPos * 0.9 + sample * 0.1;
		targetPos = targetPos * 0.9 + map(raw_inputs[0], 1000, 2000, min, max) * 0.1;
	} else {
		curPos = targetPos;
	}

	error = (diff > 250000 || raw_inputs[0] < 800);
	Serial.print(raw_inputs[0]);
	Serial.print(" ");
	Serial.println(diff);
	if (error) {
		middleSteering();
		Serial.println("error");
	}

	antiFlickeringAndMovement();
	processCalibrate();
	updateLED();
}

void antiFlickeringAndMovement() {
	delta = targetPos - curPos;
	if (blocking) {
		if (--blockingCount == 0)
			blocking = false;
		stop();
	} else {
		if (delta < - DEADBAND) {
			if (lastDir) {
				blocking = true;
				blockingCount = SERVO_STOP_CYCLES;
				lastDir = !lastDir;
				return;
			}

			move(delta);
		} else if (delta > DEADBAND) {
			if (!lastDir) {
				blocking = true;
				blockingCount = SERVO_STOP_CYCLES;
				lastDir = !lastDir;
				return;
			}

			move(delta);
		} else {
			stop();
		}
	}
	lastPos = curPos;
}

ISR(PCINT0_vect) {
	current_time_int0 = micros();

	//Channel 1
	if (PINB & B00000001) {
		if (last_flank[CHANNEL_1] == 0) {
			last_flank[CHANNEL_1] = 1;
			upflank_time[CHANNEL_1] = current_time_int0;
		}
	} else if (last_flank[CHANNEL_1] == 1) {
		last_flank[CHANNEL_1] = 0;
		raw_inputs[CHANNEL_1] = current_time_int0 - upflank_time[CHANNEL_1];
	}
}
