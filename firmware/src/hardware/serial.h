/*
  XMEGA serial library for use with avr-gcc.
  
  author:  Pras Velagapudi
  
  based on: http://blog.frankvh.com/2009/11/14/atmel-xmega-printf-howto/
*/

#ifndef SERIAL_H
#define	SERIAL_H

#include <stdio.h>
#include <avr/io.h>
  
struct SerialConfig 
{
  USART_t *uart;
  PORT_t *port;
  int rxPin;
  int txPin;
};

struct BaudConfig
{
  uint16_t bsel;
  uint8_t bscale;
};

// Serial rates computed using:
// http://prototalk.net/forums/showthread.php?t=188
const BaudConfig BAUD_1200 =   { 3331, 15 };
const BaudConfig BAUD_4800 =   { 3325, 13 };
const BaudConfig BAUD_9600 =   { 3317, 12 };
const BaudConfig BAUD_38400 =  { 3269, 10 };
const BaudConfig BAUD_115200 = { 1047, 10 };

class Serial
{
 protected:
  FILE* _stream;
  Serial(bool isDefault);
  virtual ~Serial() = 0;

 public:
  FILE *stream();
  virtual bool available() = 0;
};


template <const SerialConfig &_serial>
class SerialHW : public Serial
{
 public:  
  /**
   * Init USART.  Transmit only (we're not receiving anything) 
   * We use USARTC0, transmit pin on PC3.
   * Want 9600 baud. Have a 2 MHz clock. BSCALE = 0
   * BSEL = ( 2000000 / (2^0 * 16*9600)) -1 = 12
   * Fbaud = 2000000 / (2^0 * 16 * (12+1))  = 9615 bits/sec
   */
 SerialHW(BaudConfig baud, bool isDefault = false) 
   : Serial(isDefault) 
  {
    
    // Set the TxD pin high and the RxD pin low
    _serial.port->OUTSET = _BV(_serial.txPin);
    _serial.port->OUTCLR = _BV(_serial.rxPin);
    
    // Set the TxD pin as an output and the RxD pin as an input
    _serial.port->DIRSET = _BV(_serial.txPin);
    _serial.port->DIRCLR = _BV(_serial.rxPin);
    
    // Set baud rate & frame format
    _serial.uart->BAUDCTRLA = (uint8_t) baud.bsel;
    _serial.uart->BAUDCTRLB = (baud.bscale << 4) | (baud.bsel >> 8);

    // Set mode of operation
    // (async, no parity, 8 bit data, 1 stop bit)
    _serial.uart->CTRLA = 0; // no interrupts enabled
    _serial.uart->CTRLC = USART_CMODE_ASYNCHRONOUS_gc | USART_CHSIZE_8BIT_gc | USART_PMODE_DISABLED_gc; 
    
    // Enable transmitter and receiver
    _serial.uart->CTRLB = USART_TXEN_bm | USART_RXEN_bm;

    // Connect up the transmit and receive functions to a serial stream 
    _stream = fdevopen(uart_putchar, uart_getchar);
  }

  ~SerialHW() {
    // Connect up the transmit and receive functions to a serial stream
    _stream = fdevopen(uart_putchar, uart_getchar);

    // Set the TxD pin and the RxD pin low
    _serial.port->OUTCLR = _BV(_serial.txPin) | _BV(_serial.rxPin);
    
    // Set the TxD pin and the RxD pin as an input
    _serial.port->DIRCLR = _BV(_serial.txPin) | _BV(_serial.rxPin);
  }
  
  /**
   * Low-level function which puts a value into the
   * Tx buffer for transmission.  
   */
  static int uart_putchar(char c, FILE *stream) 
  {
    // Wait for the transmit buffer to be empty
    while ( !(_serial.uart->STATUS & USART_DREIF_bm) );
    
    // Put our character into the transmit buffer
    _serial.uart->DATA = c;

    return 0;
  }

  /**
   * Low-level function which waits for the Rx buffer
   * to be filled, and then reads one character out of it.
   * Note that this function blocks on read - it will wait until
   * something fills the Rx buffer.
   */
  static int uart_getchar(FILE *stream)
  {
    // Wait for the receive buffer is filled
    while( !( _serial.uart->STATUS & USART_RXCIF_bm) );
  
    // Read the receive buffer and clear the read complete flag
    _serial.uart->STATUS = USART_RXCIF_bm;
    return _serial.uart->DATA;
  }

  bool available()
  {
    return (_serial.uart->STATUS & USART_RXCIF_bm) ? true : false;
  }
};

#endif	/* SERIAL_H */
