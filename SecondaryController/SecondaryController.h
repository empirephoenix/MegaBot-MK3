#include <Arduino.h>

#ifndef SECONDARYCONTROLLER_H_
#define SECONDARYCONTROLLER_H_

void calibrate();
void stop();
void retract(int delta);
void extend(int delta);
void limit_receiver_input(byte n);
void eepromWriteInt(int adr, int wert);

int eepromReadInt(int adr);
int deltaDriveCalc(int delta);

#endif /* SECONDARYCONTROLLER_H_ */
