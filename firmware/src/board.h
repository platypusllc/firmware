/*
 * Board configuration file for Rev A of the Platypus control board.
 *
 * author: Pras Velagapudi <pras@senseplatypus.com>
 */

#ifndef _BOARD_H_
#define _BOARD_H_

#include "led.h"
#include "serial.h"
#include "servo.h"
#include "task.h"

#warning Set up board configuration with correct port values!

// Configure the LED on the board
LedConfig UserLed = { &PORTC, PIN5, false };

// Serial configurations for external and bluetooth ports
SerialConfig SerialBluetooth = { &USARTE1, &PORTE, PIN6, PIN7 };
SerialConfig SerialExternal  = { &USARTE0, &PORTE, PIN2, PIN3 };

// Serial configurations for each sensor port
SerialConfig Serial1 = { &USARTC0, &PORTC, PIN2, PIN3 };
SerialConfig Serial2 = { &USARTC1, &PORTC, PIN6, PIN7 };
SerialConfig Serial3 = { &USARTD1, &PORTD, PIN6, PIN7 };
SerialConfig Serial4 = { &USARTD0, &PORTD, PIN2, PIN3 };

// Servo PWM configurations for each sensor port
ServoConfig0 Motor  = { &TCF0, &PORTF, PIN1 };
ServoConfig1 Servo1 = { &TCE1, &PORTE, PIN5 };
ServoConfig0 Servo2 = { &TCE0, &PORTE, PIN1 };
ServoConfig1 Servo3 = { &TCD1, &PORTD, PIN5 };
ServoConfig0 Servo4 = { &TCD0, &PORTD, PIN1 };

// Set up general purpose task timer
TaskConfig UserTask = { &TCC0 }; // No one is using TCC0

// Helper function to change to high-speed clock adapted from:
// http://fivevolt.blogspot.com/2010/11/enabling-32mhz-operation-on-avr-xmega.html
void setClockTo32MHz() 
{
  CCP = CCP_IOREG_gc;              // disable register security for oscillator update
  OSC.CTRL = OSC_RC32MEN_bm;       // enable 32MHz oscillator
  while(!(OSC.STATUS & OSC_RC32MRDY_bm)); // wait for oscillator to be ready
  CCP = CCP_IOREG_gc;              // disable register security for clock update
  CLK.CTRL = CLK_SCLKSEL_RC32M_gc; // switch to 32MHz clock
}

// Startup function for basic board functionality
void initBoard()
{
  setClockTo32MHz();

  // Enable BT CTS line forever
  PORTF.DIRSET = PIN5_bm;
  PORTF.OUTCLR = PIN5_bm;
}

#endif /* _BOARD_H_ */
