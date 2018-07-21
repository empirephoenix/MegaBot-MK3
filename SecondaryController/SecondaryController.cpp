#include "SecondaryController.h"
#include <EEPROM.h>
#include <APA102.h>

#define EXTEND 9
#define RETRACT 11

#define LED_DATA 7
#define LED_CLOCK 4

#define EEPROM_MIN_MAX_ADDR_OFFSET 0
#define NUM_CHANNELS 2
#define CHANNEL_1 0
#define CHANNEL_2 1
#define DELTA_DRIVE_START 50
#define DELTA_DRIVE_MIN_PWR 180
#define SERVO_STOP_CYCLES 60

#define DEADBAND 5

#define COLOR_DIM_GREEN (0, 32, 0)
#define COLOR_EXTEND (128,128,0)
#define COLOR_RETRACT (0,255,0)
#define COLOR_CALIBRATE_HOLD (0,0,128)
#define COLOR_CALIBRATE_EXECUTE (0,0,255)
#define COLOR_MOTOR_BLOCKING (200,255,255)
#define COLOR_ERROR (255,0,0)

APA102<LED_DATA, LED_CLOCK> ledStrip;
rgb_color leds[1];

int targetPos = 600;
int total = 0;
int curPos = 0;
int calibrateCount = 0;

int min = -1;
int max = -1;

int lastPos = 0, lastDir = 0;
int blockingCount = 0;
bool blocking = false;

byte last_flank[NUM_CHANNELS];
volatile int receiver_input[NUM_CHANNELS];
volatile int raw_inputs[NUM_CHANNELS];
//not volatile only interrupt handler
unsigned long current_time_int0;
unsigned long upflank_time[NUM_CHANNELS];
unsigned long loop_timer;
unsigned long diff;

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
		int prior = analogRead(A1);
		extend(26);
		delay(200);
		int after = analogRead(A1);
		if (abs(prior - after) < DEADBAND * 2) {
			min = (prior + after) / 2;
			calibrating = false;
		}
	}
	calibrating = true;
	while (calibrating) {
		int prior = analogRead(A1);
		retract(26);
		delay(200);
		int after = analogRead(A1);
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
	targetPos = (max - min) / 2 + min;
}

int deltaDriveCalc(int delta) {
	if (delta > DELTA_DRIVE_START) {
		return 0;
	}
	int pwr = map(delta, 0, DELTA_DRIVE_START, DELTA_DRIVE_MIN_PWR, 0);
	return pwr;
}

void retract(int delta) {
	analogWrite(EXTEND, 255);
	analogWrite(RETRACT, 0);
}

void extend(int delta) {
	analogWrite(EXTEND, 0);
	analogWrite(RETRACT, 255);
}

void stop() {
	analogWrite(EXTEND, 255);
	analogWrite(RETRACT, 255);
}

void setup() {

	pinMode(13, INPUT_PULLUP);
	pinMode(EXTEND, OUTPUT);
	pinMode(RETRACT, OUTPUT);

	leds[0].red = 0;
	leds[0].green = 32;
	leds[0].blue = 0;

	Serial.begin(115200);
	pinMode(A1, INPUT);

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

	curPos = analogRead(A1);
}

void loop() {
	if (!digitalRead(13)) {
		calibrateCount++;
		out COLOR_CALIBRATE_HOLD;
		if (calibrateCount== 1000) {
			calibrateCount = 0;
			out COLOR_CALIBRATE_EXECUTE;
			calibrate();
		}
	} else {
		calibrateCount = 0;
	}

	curPos = curPos * 0.95 + analogRead(A1) * 0.05;
	if (raw_inputs[0] > 900 && raw_inputs[0] < 2100)
		targetPos = targetPos * 0.8
				+ map(raw_inputs[0], 1000, 2000, min, max) * 0.2;

	//Serial.println(raw_inputs[0]);

	int delta = abs(targetPos - curPos);

	lastPos = curPos;

	if (blocking) {
		if (--blockingCount == 0)
			blocking = false;
		stop();
		out COLOR_MOTOR_BLOCKING;
	} else {
		if (curPos < (targetPos - DEADBAND)) {
			if (lastDir) {
				blocking = true;
				blockingCount = SERVO_STOP_CYCLES;
				lastDir = !lastDir;
				return;
			}

			retract(delta);
			if (calibrateCount < 5) out COLOR_RETRACT;

		} else if (curPos > (targetPos + DEADBAND)) {
			if (!lastDir) {
				blocking = true;
				blockingCount = SERVO_STOP_CYCLES;
				lastDir = !lastDir;
				return;
			}

			extend(delta);
			if (calibrateCount < 5)	out COLOR_EXTEND;
		} else {
			stop();
			out COLOR_DIM_GREEN;
		}
	}

	diff = micros() - current_time_int0;

	if(diff > 250000) {
		out COLOR_ERROR;
	}


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
