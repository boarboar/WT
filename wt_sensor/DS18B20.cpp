
#include "Energia.h"
#include "DS18B20.h"

// list of commands DS18B20:

#define DS1820_WRITE_SCRATCHPAD 	0x4E
#define DS1820_READ_SCRATCHPAD      0xBE
#define DS1820_COPY_SCRATCHPAD 		0x48
#define DS1820_READ_EEPROM 			0xB8
#define DS1820_READ_PWRSUPPLY 		0xB4
#define DS1820_SEARCHROM 			0xF0
#define DS1820_SKIP_ROM             0xCC
#define DS1820_READROM 				0x33
#define DS1820_MATCHROM 			0x55
#define DS1820_ALARMSEARCH 			0xEC
#define DS1820_CONVERT_T            0x44

#define DS18_MEAS_12_DLY	750

extern const uint16_t port_to_dir[]; // fix
extern const uint16_t port_to_ren[]; // fix

//#define OW_LO {	*_OWPORTDIR |= _OWPORTPIN;	*_OWPORTREN &= ~_OWPORTPIN; *_OWPORTOUT &= ~_OWPORTPIN; }
//#define OW_HI {	*_OWPORTDIR |= _OWPORTPIN;	*_OWPORTREN &= ~_OWPORTPIN; *_OWPORTOUT |= _OWPORTPIN; }
//#define OW_RLS { *_OWPORTDIR &= ~_OWPORTPIN; *_OWPORTREN |= _OWPORTPIN; *_OWPORTOUT |= _OWPORTPIN; }

#define OW_LO {	*_OWPORTREN &= ~_OWPORTPIN;  *_OWPORTOUT &= ~_OWPORTPIN; *_OWPORTDIR |= _OWPORTPIN;}
#define OW_HI {	*_OWPORTREN &= ~_OWPORTPIN; *_OWPORTOUT |= _OWPORTPIN;  *_OWPORTDIR |= _OWPORTPIN;}
#define OW_RLS { *_OWPORTDIR &= ~_OWPORTPIN; *_OWPORTREN |= _OWPORTPIN; *_OWPORTOUT &= ~_OWPORTPIN; }

#define OW_IN (*_OWPORTIN & _OWPORTPIN)

//int16_t _stemp = 9999;
//uint32_t _lastCall=0;

DS18B20::DS18B20(uint8_t OWPIN)
{
	uint8_t OWPORT = digitalPinToPort(OWPIN);
	_OWPORTPIN = digitalPinToBitMask(OWPIN);
	_OWPORTDIR = portDirRegister(OWPORT);
	_OWPORTREN = portRenRegister(OWPORT);
	_OWPORTIN = portInputRegister(OWPORT);
	_OWPORTOUT = portOutputRegister(OWPORT);
}

/***************************************************************/

int16_t DS18B20::GetData16_1()
{
    if(reset()) return DS18_MEAS_FAIL;
    write_byte(0xcc); // skip ROM command
    write_byte(0x44); // convert T command
    OW_HI
	return 0;
}

int16_t DS18B20::GetData16_2()
{
    uint16_t temp;
    if(reset()) return DS18_MEAS_FAIL;
    write_byte(0xcc); // skip ROM command
    write_byte(0xbe); // read scratchpad command
    temp = ReadDS1820();
    return((int16_t)temp);	
}

int16_t DS18B20::GetData16(uint8_t res)
{
	if (res == 9) { res = 100; }
	else if (res==10) { res = 200; }
	else if (res==11) { res = 400; }
	else res = 800;  	
    uint16_t temp;
    if(reset()) return DS18_MEAS_FAIL;
    write_byte(0xcc); // skip ROM command
    write_byte(0x44); // convert T command
    OW_HI
	delay(res); 
    if(reset()) return DS18_MEAS_FAIL;
    write_byte(0xcc); // skip ROM command
    write_byte(0xbe); // read scratchpad command
    temp = ReadDS1820();
    return((int16_t)temp);	
}

uint16_t DS18B20::ReadDS1820 ( void )
{
  unsigned int i;
  uint16_t byte = 0;
  for(i = 16; i > 0; i--){
    byte >>= 1;
    if (read_bit()) {
      byte |= 0x8000;
    }
  }
  return byte;
}
int DS18B20::reset(void)
{
	OW_LO
	delayMicroseconds(500); // 480us minimum  // try 500
	noInterrupts();
	OW_RLS
	delayMicroseconds(80); // slave waits 15-60us  // try 80 or 40
	interrupts();
	if (OW_IN) return 1; // line should be pulled down by slave
	delayMicroseconds(300); // slave TX presence pulse 60-240us
	if (!OW_IN) return 2; // line should be "released" by slave
	return 0;
}

void DS18B20::write_bit(int bit)
{
  delayMicroseconds(1); // recovery, min 1us
  OW_HI
  if (bit) {
    noInterrupts();
    OW_LO
    delayMicroseconds(5); // max 15us
    OW_RLS	// input
	interrupts();
    delayMicroseconds(56);
  }
  else {
    noInterrupts();
    OW_LO
    delayMicroseconds(60); // min 60us
    OW_RLS	// input
	interrupts();
    delayMicroseconds(1);
  }
 }

//#####################################################################

int DS18B20::read_bit()
{
  int bit=0;
  delayMicroseconds(1);
  noInterrupts();
  OW_LO
  delayMicroseconds(5); // hold min 1us
  OW_RLS
  delayMicroseconds(10); // 15us window
  if (OW_IN) {
    bit = 1;
  }
  interrupts();
  delayMicroseconds(46); // rest of the read slot
  return bit;
}

//#####################################################################

void DS18B20::write_byte(uint8_t byte)
{
  int i;
  for(i = 0; i < 8; i++)
  {
    write_bit(byte & 1);
    byte >>= 1;
  }
}

//#####################################################################

uint8_t DS18B20::read_byte()
{
  unsigned int i;
  uint8_t byte = 0;
  for(i = 0; i < 8; i++)
  {
    byte >>= 1;
    if (read_bit()) byte |= 0x80;
  }
  return byte;
}

