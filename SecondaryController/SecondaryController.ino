#define EXTEND 11
#define RETRACT 9
#define NUM_READINGS 10
#define EEPROM_MIN_MAX_ADDR_OFFSET 0
#define NUM_CHANNELS 2
#define CHANNEL_1 0
#define CHANNEL_2 1
#define DELTA_DRIVE_START 50
#define DELTA_DRIVE_MIN_PWR 230

#include <EEPROM.h>

int targetPos = 600;
int deadBand = 5;
int total = 0;
int curpos  = 0;

int min = -1;
int max = -1;

int readings[NUM_READINGS];
int readIndex = 0;
int lastPos = 0;

byte last_flank[NUM_CHANNELS];
volatile int receiver_input[NUM_CHANNELS];
volatile int raw_inputs[NUM_CHANNELS];
//not volatile only interrupt handler
unsigned long current_time_int0, upflank_time[NUM_CHANNELS];
unsigned long loop_timer;

void setup() {
  for (int thisReading = 0; thisReading < NUM_READINGS; thisReading++) {
    readings[thisReading] = 0;
  }

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
  Serial.print(delta);
  Serial.print(" ");
  Serial.println(pwr);
  return pwr;
}

void extend(int delta) {
  analogWrite(EXTEND, 255);
  analogWrite(RETRACT, deltaDriveCalc(delta));
}

void retract(int delta) {
  analogWrite(EXTEND, deltaDriveCalc(delta));
  analogWrite(RETRACT, 255);
}

void stop() {
  analogWrite(EXTEND, 255);
  analogWrite(RETRACT, 255);
}

void loop() {
  total = total - readings[readIndex];
  readings[readIndex] = analogRead(A1);
  total = total + readings[readIndex];
  readIndex = readIndex + 1;
  if (readIndex >= NUM_READINGS) {
    readIndex = 0;
  }
  int curPos = total / NUM_READINGS;

  if (Serial.available() > 1) {
    int servoSignal = Serial.parseInt();
    targetPos = map(servoSignal, 1000, 2000, min, max);
    Serial.print("New target");
    Serial.println(targetPos);
    if (servoSignal == -1) {
      calibrate();
    }
  }
  int delta = abs(targetPos - curPos);
  if (curPos < (targetPos - deadBand)) {
    retract(delta);
  } else if (curPos > (targetPos + deadBand)) {
    extend(delta);
  } else {
    stop();
  }

  limit_receiver_input(CHANNEL_1);
  limit_receiver_input(CHANNEL_2);
  targetPos = map(receiver_input[0], 1000, 2000, min, max);
}

void limit_receiver_input(byte n) {
  int rc_signal = raw_inputs[n];
  if (rc_signal > 2000) rc_signal = 2000;
  if (rc_signal < 1000) rc_signal = 1000;
  receiver_input[n] = rc_signal;
}

ISR(PCINT0_vect) {
  current_time_int0 = micros();

  //Channel 1
  if (PINB & B00000001) {
    if (last_flank[CHANNEL_1] == 0) {
      last_flank[CHANNEL_1] = 1;
      upflank_time[CHANNEL_1] = current_time_int0;
    }
  }
  else if (last_flank[CHANNEL_1] == 1) {
    last_flank[CHANNEL_1] = 0;
    raw_inputs[CHANNEL_1] = current_time_int0 - upflank_time[CHANNEL_1];
  }

  //Channel 2
  if (PINB & B00000100) {
    if (last_flank[CHANNEL_2] == 0) {
      last_flank[CHANNEL_2] = 1;
      upflank_time[CHANNEL_2] = current_time_int0;
    }
  }
  else if (last_flank[CHANNEL_2] == 1) {
    last_flank[CHANNEL_2] = 0;
    raw_inputs[CHANNEL_2] = current_time_int0 - upflank_time[CHANNEL_2];
  }
}
