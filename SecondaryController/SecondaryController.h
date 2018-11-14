#include <Arduino.h>
#include "RunningMedian.h"

#ifndef SECONDARYCONTROLLER_H_
#define SECONDARYCONTROLLER_H_

float antiFlickeringAndMovement();
void processCalibrate();
void calibrate();
void stop();
void move(int delta);
void limit_receiver_input(byte n);
void eepromWriteInt(int adr, int wert);
void middleSteering();
void updateLED(float delta, bool error);

int eepromReadInt(int adr);
int deltaDriveCalc(int delta);

#endif /* SECONDARYCONTROLLER_H_ */
