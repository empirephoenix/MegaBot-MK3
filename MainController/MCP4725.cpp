#include "Arduino.h"
#include "MCP4725.h"

/*See page 19 of the MCP4726 DataSheet.
 *[C2,C1]=00 -- Fast Mode
 *[C2,C1,C0]=010 --- Write to DAC register only
 *[C2,C1,C0]=011 --- Write to DAC and EEPROM registers
 *[PD1,PD0]=00 -- Power Normal mode
 *[PD1,PD0]=01 -- Power Down mode with 1k pull down
 *[PD1,PD0]=10 -- Power Down mode with 100k pull down
 *[PD1,PD0]=11 -- Power Down mode with 500k pull down
 *The contents of bytes 2nd,3rd,4th are orgnized differently
 *based on whether the DAC is operated in Normal-Speed or Fast-Speed. Let
 *D denote data byte, x denote don't-care. Then:
 *
 *For Normal-Speed:
 *[Addr.Byte]+[C2,C1,C0,x,x,PD1,PD0,x]+[D11,D10,D9,D8,D7,D6,D5,D4]+[D3,D2,D1,D0,x,x,x,x]
 *
 *For Fast-Speed:
 *[Addr.Byte]+[C2,C1,PD1,PD0,D11,D10,D9,D8],[D7,D6,D5,D4,D3,D2,D1,D0]
 *
 *The address byte for our dac is (1,1,0,0,0,1,A0). By default A0=GND, but we can connect it to VCC.
 *For A0=GND the hex value of address is 0x62 (1100010)
 *For A0=VCC the hex value of address is 0x63 (1100011)
 *We must pass the address in our Arduino code in the argument of the function .begin(). But the argument must
 *be an 8-bit value and the address is only 7-bit long. What is happening internally is left shifting by 1 bit.
 *You can see this if you trace the definition of the .begin() function and the definition of the twi_setAddress()
 *in the file twi.c. The register TWAR is set to (address << 1). The R/W bit (bit 8) is actually determined based 
 *on the function you send after .begin()
 */

MCP4725::MCP4725() {}

void MCP4725::begin(uint8_t addr, SoftWire* _sw) {
  _i2caddr = addr;
  sw = _sw;
  sw->setRxBuffer(&buffer, 64);
  sw->setTxBuffer(&buffer, 64);
}
void MCP4725::setFastMode(){
#ifdef TWBR // in case this library is used on a chip that does not have TWBR reg defined.  
  TWBR = ((F_CPU / 400000L) - 16) / 2; // Set I2C frequency for the ATMega to 400kHz
#endif  
}
/*
 *Wire.write() takes an 8-bit unsigned int. We thus need to convert our 16-bit
 *argument to 2 8-bit variables. 16 to 8-bit, conversion keeps the 8 LSBs.
 */
void MCP4725::setVoltageAndSave(uint16_t output){
 /*For Normal-Speed: (Only normal speed includes the C3 bit, needed for writing to EEPROM)
  *[Addr.Byte]+[C2,C1,C0,x,x,PD1,PD0,x]+[D11,D10,D9,D8,D7,D6,D5,D4]+[D3,D2,D1,D0,x,x,x,x]  */
  sw->beginTransmission(_i2caddr);
  sw->write(WRITEDACEEPROM); //[C2,C1,C0,x,x,PD1,PD0,x]=[0,1,1,0,0,0,0,0]
  /*The 12-bit output is in 16-bit form (0,0,0,0,D11.D10.D9.D8.D7.D6.D5.D4,D3.D2.D1.D0).*/
  uint8_t firstbyte=(output>>4);//(0,0,0,0,0,0,0,0,D11.D10.D9.D8.D7.D6.D5.D4) of which only the 8 LSB's survive
  output = output << 12;	//(D3.D2.D1.D0,0,0,0,0,0,0,0,0,0,0,0,0) 
  uint8_t secndbyte=(output>>8);//(0,0,0,0,0,0,0,0,D3,D2,D1,D0,0,0,0,0)	of which only the 8 LSB's survive.
  sw->write(firstbyte);
  sw->write(secndbyte);
  sw->endTransmission();
}
void MCP4725::setVoltage(uint16_t output){
 /*For Normal-Speed:
  *[Addr.Byte]+[C2,C1,C0,x,x,PD1,PD0,x]+[D11,D10,D9,D8,D7,D6,D5,D4]+[D3,D2,D1,D0,x,x,x,x]  */
	sw->beginTransmission(_i2caddr);
  sw->write(WRITEDAC); //[C2,C1,C0,x,x,PD1,PD0,x]=[0,1,0,0,0,0,0,0]
  /*The 12-bit output is in 16-bit form (0,0,0,0,D11.D10.D9.D8.D7.D6.D5.D4,D3.D2.D1.D0).*/
  uint8_t firstbyte=(output>>4);//(0,0,0,0,0,0,0,0,D11.D10.D9.D8.D7.D6.D5.D4) of which only the 8 LSB's survive
  output = output << 12;	//(D3.D2.D1.D0,0,0,0,0,0,0,0,0,0,0,0,0) 
  uint8_t secndbyte=(output>>8);//(0,0,0,0,0,0,0,0,D3,D2,D1,D0,0,0,0,0)	of which only the 8 LSB's survive.
  sw->write(firstbyte);
  sw->write(secndbyte);
  sw->endTransmission();
}
void MCP4725::setVoltageFast( uint16_t output){
 /*For Fast-Speed:
  *[Addr.Byte]+[C2,C1,PD1,PD0,D11,D10,D9,D8],[D7,D6,D5,D4,D3,D2,D1,D0]  */
	sw->beginTransmission(_i2caddr);
  //output is a 12-bit value in 16-bit form, namely: [0,0,0,0,D11,D10,D9,D8,D7,D6,D5,D4,D3,D2,D1,D0]
  uint8_t firstbyte=(output>>8); //[0,0,0,0,0,0,0,0,0,0,0,0,D11,D10,D9,D8] only the 8 LSB's survive
  uint8_t secndbyte=(output); //only the 8 LSB's survive.
  sw->write(firstbyte);  // Upper data bits (0,0,0,0,D11,D10,D9,D8)
  sw->write(secndbyte);  // Lower data bits (D7,D6,D5,D4,D3,D2,D1,D0)
  sw->endTransmission();
}

