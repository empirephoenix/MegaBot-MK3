#include "Arduino.h"
#include <SoftWire.h>

#define WRITEDAC        (0x40)  //(0100 0000) Writes to the DAC
#define WRITEDACEEPROM  (0x60)  //(0110 0000) Writes to the DAC and the EEPROM (persisting the assigned value after reset)

class MCP4725{
 public:
  MCP4725();
  void begin(uint8_t addr, SoftWire* sw);
  void setFastMode();  
  void setVoltageAndSave(uint16_t output);
  void setVoltage(uint16_t output);
  void setVoltageFast(uint16_t output);
 private:
  uint8_t buffer[64];
  SoftWire* sw;
  uint8_t _i2caddr;
};
