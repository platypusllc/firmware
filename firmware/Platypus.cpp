#include "Platypus.h"

using namespace platypus;

LED::LED() 
: r_(0), g_(0), b_(0)
{
  pinMode(board::LED.R, OUTPUT);
  pinMode(board::LED.G, OUTPUT);
  pinMode(board::LED.B, OUTPUT);
}

LED::~LED()
{
  pinMode(board::LED.R, INPUT);
  pinMode(board::LED.G, INPUT);
  pinMode(board::LED.B, INPUT);
}

void LED::set(uint8_t red, uint8_t green, uint8_t blue)
{
  R(red);
  G(green);
  B(blue);
}

void LED::R(uint8_t red)
{
 r_ = red;
 digitalWrite(board::LED.R, !r_);
}

const uint8_t LED::R()
{
 return r_;
}

void LED::G(uint8_t green)
{
 g_ = green; 
 digitalWrite(board::LED.G, !g_);
}

const uint8_t LED::G()
{
 return g_;
}

void LED::B(uint8_t blue)
{
 b_ = blue; 
 digitalWrite(board::LED.B, !b_);
}

const uint8_t LED::B()
{
 return b_;
}
