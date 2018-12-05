#include "SecondaryController.h"
#include <EEPROM.h>
#include <APA102.h>

#define PIN_EXTEND 8
#define PIN_RETRACT 10
#define PIN_FAILURE 11
#define PIN_SERVO_POSITION A0
#define SERVO_STOP_CYCLES 1
#define DEADBAND 2
#define CALIBRATE_ANTI_BOUNCE_COUNT 1000

#define EEPROM_MIN_MAX_ADDR_OFFSET 0
#define PIN_RECEIVER 12

#define PIN_LED_DATA 5
#define PIN_LED_CLOCK 4
#define COLOR_DIM_GREEN (0, 32, 0)
#define COLOR_EXTEND (128,128,0)
#define COLOR_RETRACT (0,255,0)
#define COLOR_CALIBRATE_HOLD (0,0,128)
#define COLOR_CALIBRATE_EXECUTE (0,0,255)
#define COLOR_ERROR (255,0,0)

APA102<PIN_LED_DATA, PIN_LED_CLOCK> ledStrip;
rgb_color leds[1];
RunningMedian filterdInput = RunningMedian(100);
RunningMedian filterdSensor = RunningMedian(100);

float targetPos = 0;
float curPos = 0;
int calibrateCount = 0;

int min = -1;
int max = -1;

volatile int receiver_input = 0;
volatile int raw_inputs = 0;
volatile long diff = 0;
//not volatile only interrupt handler
volatile unsigned long current_time_int0 = 0;
volatile  unsigned long upflank_time0 = 0;

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

	*digitalPinToPCMSK(PIN_RECEIVER) |= bit (digitalPinToPCMSKbit(PIN_RECEIVER));  // enable pin
	PCIFR  |= bit (digitalPinToPCICRbit(PIN_RECEIVER)); // clear any outstanding interrupt
	PCICR  |= bit (digitalPinToPCICRbit(PIN_RECEIVER)); // enable interrupt for the group

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
			updateLED(0, false);
			calibrate();
			calibrateCount = 0;
		}
	} else {
		calibrateCount = 0;
	}
}

void updateLED(float delta, bool error) {
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
	long int potiMappedRawInput = 0;
	if (sample >= min && sample <= max && raw_inputs > 800 && raw_inputs < 2200) {
		filterdSensor.add(sample);
		curPos = filterdSensor.getMedian();
		potiMappedRawInput = map(raw_inputs, 1000, 2000, min, max);
		filterdInput.add(potiMappedRawInput);
		targetPos = filterdInput.getMedian();
		Serial.print(curPos);
		Serial.print(" ");
		Serial.println(targetPos);
	} else {
		curPos = targetPos;
	}
	bool error = (diff > 250000 || raw_inputs < 800);

	if (error) {
		middleSteering();
		Serial.print("error ");
	}
	float delta = antiFlickeringAndMovement();

	processCalibrate();
	updateLED(delta, error);
}

float antiFlickeringAndMovement() {
	float delta = targetPos - curPos;
	if (delta < - DEADBAND || delta > DEADBAND) {
		move(delta);
		return delta;
	} else {
		stop();
		return 0;
	}
}

ISR(PCINT0_vect) {
	current_time_int0 = micros();

	//Channel 1
	if (digitalRead(PIN_RECEIVER)) {
		upflank_time0 = current_time_int0;
	} else  {
		if(current_time_int0 > upflank_time0){
			raw_inputs =  current_time_int0 - upflank_time0;
		}
	}
}
