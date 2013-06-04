/* 
 * File:   servo.h
 * Author: Pras Velagapudi
 *
 * Created on December 21, 2012, 10:17 AM
 * This code is adapted from the sample code at:
 * http://bradsprojects.wordpress.com/2010/05/03/servo-control-with-an-xmega/
 */

#ifndef SERVO_H
#define	SERVO_H

#include "portability.h"
#include <inttypes.h>
#include <avr/io.h>

#if (F_CPU != 32000000UL)
#error Servo timer settings do not match CPU frequency!
#endif

#define MIN_PULSE_WIDTH_US      1000L    // the shortest pulse sent to a servo 
#define MAX_PULSE_WIDTH_US      2000L    // the longest pulse sent to a servo 
#define DEFAULT_PULSE_WIDTH_US  ((MAX_PULSE_WIDTH_US + MIN_PULSE_WIDTH_US) / 2)
#define REFRESH_INTERVAL_US     20000L   // minimum time to refresh servos
#define SERVO_SCALE_FACTOR      (65535L / (MAX_PULSE_WIDTH_US - MIN_PULSE_WIDTH_US))

struct ServoConfig0
{
  TC0_t *timer;
  PORT_t *port;
  int pin;
};

struct ServoConfig1
{
  TC1_t *timer;
  PORT_t *port;
  int pin;
};

class Servo
{
 protected:
  Servo();
  virtual ~Servo() = 0;

  volatile int _position;
  virtual void update(uint16_t position) = 0;

 public:
  void set(int16_t position);
  int16_t get(void);
  void setRaw(uint16_t position);
  uint16_t getRaw(void);
};

template<const ServoConfig0 &_config>
class ServoHW0 : public Servo
{
 public:
 ServoHW0() : Servo() {
    // Initialize the timer to the default position
    update(_position);

    // Set up the timer pin as an output
    _config.port->OUTCLR = _BV(_config.pin);
    _config.port->DIRSET = _BV(_config.pin);

    // Set up the timer to a 0.5MHz tick resolution, so we
    // can convert timings easily (1 tick = 2uS)
    _config.timer->PER = REFRESH_INTERVAL_US >> 1; // Set the PWM resolution
    _config.timer->CTRLB = TC0_CCBEN_bm | TC_WGMODE_SS_gc; // Use compare channel B

    // Start the timer running
    _config.timer->CTRLA = TC_CLKSEL_DIV64_gc; // 32MHz / 64 = 0.5Mhz
  }

  ~ServoHW0() {
    // Disable the timer
    _config.timer->CTRLA = 0;

    // Set up the timer pin as an input
    _config.port->OUTCLR = _BV(_config.pin);
    _config.port->DIRCLR = _BV(_config.pin);
  }
  
 private:
  void update(uint16_t position)
  {
    // Configure the timer period to match the servo setting
    _config.timer->CCBBUF = position >> 1; // 2 uS = 1 ticks @ 32MHz/64
  }
};

template<const ServoConfig1 &_config>
class ServoHW1 : public Servo
{
 public:
 ServoHW1() : Servo() {
    // Initialize the timer to the default position
    update(_position);

    // Set up the timer pin as an output
    _config.port->OUTCLR = _BV(_config.pin);
    _config.port->DIRSET = _BV(_config.pin);

    // Set up the timer to a 0.5MHz tick resolution, so we
    // can convert timings easily (1 tick = 2uS)
    _config.timer->PER = REFRESH_INTERVAL_US >> 1; // Set the PWM resolution
    _config.timer->CTRLB = TC1_CCBEN_bm | TC_WGMODE_SS_gc; // Use compare channel B

    // Start the timer running
    _config.timer->CTRLA = TC_CLKSEL_DIV64_gc; // 32MHz / 64 = 0.5Mhz
  }

  ~ServoHW1() {
    // Disable the timer
    _config.timer->CTRLA = 0;

    // Set up the timer pin as an input
    _config.port->OUTCLR = _BV(_config.pin);
    _config.port->DIRCLR = _BV(_config.pin);
  }
  
 private:
  void update(uint16_t position)
  {
    // Configure the timer period to match the servo setting
    _config.timer->CCBBUF = position >> 1; // 2 uS = 1 ticks @ 32MHz/64
  }
};

#endif	/* SERVO_H */

