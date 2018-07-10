#include "SecondaryController.h"
#include <Arduino.h>
#include <EEPROM.h>
#include <FastLED.h>


#define EXTEND 9
#define RETRACT 11
#define LED_DATA 7
#define EEPROM_MIN_MAX_ADDR_OFFSET 0
#define NUM_CHANNELS 2
#define CHANNEL_1 0
#define CHANNEL_2 1
#define DELTA_DRIVE_START 50
#define DELTA_DRIVE_MIN_PWR 230
#define SERVO_STOP_CYCLES 50

#define COLOR_DIM_GREEN CRGB(0, 32, 0)
#define COLOR_EXTEND CRGB::Olive
#define COLOR_RETRACT CRGB::Lime
#define COLOR_CALIBRATE_HOLD CRGB::Aqua
#define COLOR_CALIBRATE_EXECUTE CRGB::Yellow
#define COLOR_MOTOR_BLOCKING CRGB::White
#define COLOR_ERROR CRGB::Red

CRGB leds[1];

int targetPos = 600;
int deadBand = 2;
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
		if (abs(prior - after) < deadBand * 2) {
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
		if (abs(prior - after) < deadBand * 2) {
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
	analogWrite(RETRACT, deltaDriveCalc(delta));
}

void extend(int delta) {
	analogWrite(EXTEND, deltaDriveCalc(delta));
	analogWrite(RETRACT, 255);
}

void stop() {
	analogWrite(EXTEND, 255);
	analogWrite(RETRACT, 255);
}

void setup() {

	pinMode(13, INPUT_PULLUP);

	FastLED.addLeds<WS2812, LED_DATA, GRB>(leds, 1);
	leds[0] = CRGB(0,32,0);
	FastLED.setBrightness(64);
	FastLED.show();

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
		leds[0] = COLOR_CALIBRATE_HOLD;
		Serial.println(calibrateCount);
		if (calibrateCount== 2000) {
			calibrateCount = 0;
			leds[0] = COLOR_CALIBRATE_EXECUTE;
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
		leds[0] = COLOR_MOTOR_BLOCKING;
	} else {
		if (curPos < (targetPos - deadBand)) {
			if (lastDir) {
				blocking = true;
				blockingCount = SERVO_STOP_CYCLES;
				lastDir = !lastDir;
				return;

			}
			retract(delta);
			if (calibrateCount < 5)
				leds[0] = COLOR_RETRACT;
		} else if (curPos > (targetPos + deadBand)) {
			if (!lastDir) {
				blocking = true;
				blockingCount = SERVO_STOP_CYCLES;
				lastDir = !lastDir;
				return;
			}
			extend(delta);
			if (calibrateCount < 5)
				leds[0] = COLOR_EXTEND;
		} else {
			stop();
			leds[0] = COLOR_DIM_GREEN;
		}
	}

	Serial.print(micros());
	Serial.print(" ");
	Serial.print(current_time_int0);
	Serial.print(" ");
	diff = micros() - current_time_int0;
	Serial.println(diff);

	if(diff > 250000) {
		leds[0] = COLOR_ERROR;
		FastLED.show();
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
		FastLED.show();
	}

	//Channel 2
	if (PINB & B00000100) {
		if (last_flank[CHANNEL_2] == 0) {
			last_flank[CHANNEL_2] = 1;
			upflank_time[CHANNEL_2] = current_time_int0;
		}
	} else if (last_flank[CHANNEL_2] == 1) {
		last_flank[CHANNEL_2] = 0;
		raw_inputs[CHANNEL_2] = current_time_int0 - upflank_time[CHANNEL_2];
	}
}
