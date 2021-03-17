#include "main.h"


#define FILTER_BUF 2500



typedef struct
{
	uint8_t filled; // filter filled flag
	
	uint8_t error; // error flag;
	
	uint8_t init; //
	
	float buffer[FILTER_BUF]; // buffer
	
	float sum; // sum of buf
	
	float output; // output of the filter
	
	uint16_t counter1; // counter for filling buffer in
	
	uint16_t counter2; // counter for filtration
	
	float input;
		  
} moving_average_type;


#define High			0x01
#define Low				0x00
#define average		 100
#define Pole_Pairs 11
#define Vdc 				12
#define Pi 3.1415926535897932384
#define CALIBRATION 0

#define window_Roll 10000
