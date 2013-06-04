#include "servo.h"

Servo::Servo()
{
  _position = DEFAULT_PULSE_WIDTH_US;
}

Servo::~Servo() { }

void Servo::set(int16_t position)
{
  _position = (position / SERVO_SCALE_FACTOR) + DEFAULT_PULSE_WIDTH_US;
  update(_position);
}

int16_t Servo::get()
{
  return (_position - DEFAULT_PULSE_WIDTH_US) * SERVO_SCALE_FACTOR;
}

void Servo::setRaw(uint16_t position)
{
  // Do valid bounds checking/clipping                                               
  if (position > MAX_PULSE_WIDTH_US)
    position = MAX_PULSE_WIDTH_US;
  if (position < MIN_PULSE_WIDTH_US)
    position = MIN_PULSE_WIDTH_US;

  // Set to unscaled position in pulse width                                         
  _position = position;
  update(_position);
}

uint16_t Servo::getRaw()
{
  return _position;
}
