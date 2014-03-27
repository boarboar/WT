#ifndef DS18B20_h
#define DS18B20_h

// tw
// Port and pins definition:

// [Ground] ----x         x              x----------------- [+5v]
//			      |					
//                                  |
//                                  |	  
//                                Digital pin

#include <inttypes.h>

#define DS18_MEAS_FAIL	9999

class DS18B20
{
  private:
	volatile uint8_t * _OWPORTDIR;
	volatile uint8_t * _OWPORTREN;
	volatile uint8_t * _OWPORTIN;
	volatile uint8_t * _OWPORTOUT;
	uint8_t _OWPORTPIN;
	uint16_t ReadDS1820(void);
	void write_bit(int bit);
	int read_bit();
	void write_byte(uint8_t byte);
	uint8_t read_byte();
	
  public:
    DS18B20( uint8_t pin); // 1-wire pin
	int16_t GetData16(uint8_t res); // temp (*16integer)	
	int16_t GetData16_1();
	int16_t GetData16_2();
	int reset();
};

#endif
