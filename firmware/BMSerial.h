/*
Modified version of SoftwareSerial.cpp (formerly NewSoftSerial.cpp) - 
Multi-instance software serial library for Arduino/Wiring
-- Interrupt-driven receive and other improvements by ladyada
   (http://ladyada.net)
-- Tuning, circular buffer, derivation from class Print/Stream,
   multi-instance support, porting to 8MHz processors,
   various optimizations, PROGMEM delay tables, inverse logic and 
   direct port writing by Mikal Hart (http://www.arduiniana.org)
-- Pin change interrupt macros by Paul Stoffregen (http://www.pjrc.com)
-- 20MHz processor support by Garrett Mace (http://www.macetech.com)
-- ATmega1280/2560 support by Brett Hagman (http://www.roguerobotics.com/)

-- Added support for half duplex single pin operation
-- Added readln,readdec,readhex,readbin
-- Added read timeout option

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef BMSerial_h
#define BMSerial_h

#include <inttypes.h>
#include <Stream.h>
#include <HardwareSerial.h>

/******************************************************************************
* Definitions
******************************************************************************/

#define _SS_MAX_RX_BUFF 64 // RX buffer size
#define _SS_VERSION 11 // software version of this library
#ifndef GCC_VERSION
#define GCC_VERSION (__GNUC__ * 10000 + __GNUC_MINOR__ * 100 + __GNUC_PATCHLEVEL__)
#endif

#ifndef USB_PID_DUE 

#define digitalPinToPCIFR(p)    (((p) >= 0 && (p) <= 21) ? (&PCIFR) : ((uint8_t *)NULL))
#define digitalPinToPCIFRbit(p) (((p) <= 7) ? 2 : (((p) <= 13) ? 0 : 1))

#endif

class BMSerial : public Stream
{
private:
	// per object data
	uint8_t _receivePin;
	uint8_t _receiveBitMask;
	volatile uint8_t *_receivePortRegister;
	uint8_t _transmitPin;
	uint8_t _transmitBitMask;
	volatile uint8_t *_transmitPortRegister;
	volatile uint8_t *_transmitModeRegister;
	
	uint16_t _rx_delay_centering;
	uint16_t _rx_delay_intrabit;
	uint16_t _rx_delay_stopbit;
	uint16_t _tx_delay;
	
	uint16_t _buffer_overflow:1;
	uint16_t _inverse_logic:1;
	
	// static data
	static char _receive_buffer[_SS_MAX_RX_BUFF]; 
	static volatile uint8_t _receive_buffer_tail;
	static volatile uint8_t _receive_buffer_head;
	static BMSerial *active_object;

	// private methods
#ifndef USB_PID_DUE 
	void recv();
	uint8_t rx_pin_read();
	void tx_pin_write(uint8_t pin_state);
	// private static method for timing
	static inline void tunedDelay(uint16_t delay);
#endif

	void setTXRX(uint8_t transmitPin,uint8_t receivePin);

	int8_t bintoint(uint8_t c);
	int8_t dectoint(uint8_t c);
	int8_t hextoint(uint8_t c);

public:
	// public methods
	BMSerial(uint8_t receivePin, uint8_t transmitPin, bool inverse_logic = false);
	~BMSerial();
	void begin(long speed);
	bool listen();
	void end();
	bool is_listening() { return this == active_object; }
	bool overflow() { bool ret = _buffer_overflow; _buffer_overflow = false; return ret; }
	static int library_version() { return _SS_VERSION; }
	int16_t peek();

	virtual size_t write(uint8_t byte);
	virtual int16_t read();
	virtual int available();
	virtual void flush();

#ifndef USB_PID_DUE 
	// public only for easy access by interrupt handlers
	static inline void handle_interrupt();
#endif

	int16_t read(uint32_t timeout);
	uint16_t readln(char *data,uint32_t timeout);

	uint32_t readdec(uint32_t timeout,bool ignoreleading=true);
	uint32_t readhex(uint32_t timeout,bool ignoreleading=true);
	uint32_t readbin(uint32_t timeout,bool ignoreleading=true);
	
};

// Arduino 0012 workaround
#undef int
#undef char
#undef long
#undef byte
#undef float
#undef abs
#undef round

#endif
