/*
	analog.c - Contains the functions for manipulating the FWRboard's ADC
	author:  Tom Lauwers
	edited:  pkv
*/

#include <analog.h>
#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>

/* Initializes the ADC, sets which Analog inputs can be used */
void init_analog(void) 
{
	// ADC Status Register A
	// Bit 7 - ADEN is set (enables analog)
	// Bit 6 - Start conversion bit is not set
	// Bit 5 - Disable Auto Trigger (for free running mode)
	// Bit 4 - Clear ADC Flag (required to run free-running mode)
	// Bit 3 - Disable ADC Interrupt (required to run free-running mode)
	// Bits 2-0 - Set to create a clock divisor of 128
	ADCSRA = _BV(ADEN) | _BV(ADIF) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);

	// ADMUX register
	// Bit 7,6 - Set voltage reference to AVcc (0b01)
	// Bit 5 - Set ADLAR bit for left adjust to do simple 8-bit reads (0b1)
	// Bit 4:0 - Sets the current channel, set to ADC0 (0b00000)
	ADMUX = _BV(REFS0) | _BV(ADLAR);
}


/* Returns the current 10-bit analog value as a 16-bit integer
   
   examples:
   value = analog(0) - returns the value of ADC0
   value = analog(7) - returns the value of ADC7
*/
inline uint16_t analog(uint8_t index)
{
	// Set internal mux to the specified index
	ADMUX = (ADMUX & 0xF8) | (index & 0x7);

	// Start conversion
	ADCSRA |= _BV(ADSC);
	while ( !(ADCSRA & _BV(ADIF)) );
	return ADC;
}

ISR(ADC_vect)
{
  // This is just here to catch ADC interrupts 
  // if the ADC is free running.  
  // No code needs to be in here.
}

