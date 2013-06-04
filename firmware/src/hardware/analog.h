/*  analog.h - Contains definitions and function prototypes for using the
	ADC to detect analog signals on the pot.
	author:  Tom Lauwers
*/

#include <inttypes.h>

// Define values to switch from reading one analog value to another
#define C0_READ 0x00
#define C1_READ 0x01
#define C2_READ 0x02
#define C3_READ 0x03
#define C4_READ 0x04
#define C5_READ 0x05
#define POT     0x06
#define BUTTON2 0x07

// Function description can be found in analog.c
void init_analog(void);
inline uint16_t analog(uint8_t index);


