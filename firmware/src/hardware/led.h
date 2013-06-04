/* 
 * File:   led.h
 * Author: pkv
 *
 * Created on December 21, 2012, 12:31 AM
 */

#ifndef LED_H
#define	LED_H

#include <avr/io.h>

struct LedConfig {
  PORT_t *port;
  int outputPin;
  bool isActiveHigh;
};

class Led
{
 protected:
  Led();
  virtual ~Led() = 0;

 public:
  void on(void);
  void off(void);
  void toggle(void);
  virtual void set(bool enabled) = 0;
  virtual bool get() = 0;
};

template<const LedConfig &_led>
class LedHW : public Led
{
 public:
 LedHW() : Led() { 
    
    // Turn the LED off by default
    if (_led.isActiveHigh) {
      _led.port->OUTCLR = _BV(_led.outputPin);
    } else {
      _led.port->OUTSET = _BV(_led.outputPin);
    }

    // Set the LED port output direction
    _led.port->DIRSET = _BV(_led.outputPin);
  }

  ~LedHW(void) {

    // Set the LED pin to be an input and low
    _led.port->OUTCLR = _BV(_led.outputPin);
    _led.port->DIRCLR = _BV(_led.outputPin);
  }
  
  bool get(void) {
    return (!(_led.port->OUT & _BV(_led.outputPin)) ^ _led.isActiveHigh);
  }

  void set(bool enabled)
  {
    // Turn the LED on or off as specified
    // (invert the logic when active low)
    if (_led.isActiveHigh ^ enabled) {
      _led.port->OUTCLR = _BV(_led.outputPin);
    } else {
      _led.port->OUTSET = _BV(_led.outputPin);
    }
  }
};

#endif	/* LED_H */

