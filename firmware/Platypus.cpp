#include "Platypus.h"

using namespace platypus;

LED::LED() 
: r_(0), g_(0), b_(0)
{  
  pinMode(board::LED.R, OUTPUT);
  digitalWrite(board::LED.R, HIGH);
  
  pinMode(board::LED.G, OUTPUT);
  digitalWrite(board::LED.G, HIGH);
  
  pinMode(board::LED.B, OUTPUT);
  digitalWrite(board::LED.B, HIGH);
}

LED::~LED()
{
  pinMode(board::LED.R, INPUT);
  pinMode(board::LED.G, INPUT);
  pinMode(board::LED.B, INPUT);
}

void LED::set(int red, int green, int blue)
{
  R(red);
  G(green);
  B(blue);
}

void LED::R(int red)
{
 r_ = red;
 digitalWrite(board::LED.R, !r_);
}

int LED::R()
{
 return r_;
}

void LED::G(int green)
{
 g_ = green; 
 digitalWrite(board::LED.G, !g_);
}

int LED::G()
{
 return g_;
}

void LED::B(int blue)
{
 b_ = blue; 
 digitalWrite(board::LED.B, !b_);
}

int LED::B()
{
 return b_;
}

Motor::Motor(int channel)
: enable_(board::MOTOR[channel].ENABLE), enabled_(false), velocity_(0)
{
  servo_.attach(board::MOTOR[channel].SERVO);
  pinMode(enable_, OUTPUT);
  digitalWrite(enable_, LOW);
}

Motor::~Motor()
{
  pinMode(enable_, INPUT);
  digitalWrite(enable_, LOW);
  servo_.detach();
}
    
void Motor::velocity(float velocity)
{
  if (velocity > 1.0) {
    velocity = 1.0;
  }
  if (velocity < -1.0) {
     velocity = -1.0; 
  }
  velocity_ = velocity;
  
  float command = (velocity * 500) + 1500;
  servo_.writeMicroseconds(command);
}

float Motor::velocity()
{
  return velocity_;
}

void Motor::enable(bool isOn)
{
  enabled_ = isOn;
  digitalWrite(enable_, enabled_);
}

bool Motor::enabled()
{
  return enabled_;
}
    
void Motor::enable()
{
  enable(true);
}

void Motor::disable()
{
  enable(false);  
}
    
float Motor::current()
{
 // TODO: fill me in. 
}

void VaporPro::arm() 
{
  disable();
  delay(500);
  enable();
  
  velocity(1.0);
  delay(5500);

  velocity(-1.0);
  delay(3500);

  velocity(0.0);
  delay(8500);
}

