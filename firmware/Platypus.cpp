#include "Platypus.h"

using namespace platypus;

// Velocity decay/ramping constants
const float VELOCITY_ALPHA = 0.1;
const float VELOCITY_THRESHOLD = 0.001;

// TODO: Correct default initialization of these sensors.
platypus::Motor *platypus::motors[board::NUM_MOTORS];
platypus::Sensor *platypus::sensors[board::NUM_SENSORS];

// TODO: Switch to using HardwareSerial.
USARTClass *platypus::SERIAL_PORTS[4] = {
  NULL,
  &Serial1,
  &Serial2,
  &Serial3,
};

SerialHandler_t platypus::SERIAL_HANDLERS[4] = {
  {NULL, NULL},
  {NULL, NULL},
  {NULL, NULL},
  {NULL, NULL}
};

void serialEvent1() 
{
  if (SERIAL_HANDLERS[1].handler != NULL) 
  {
    (*SERIAL_HANDLERS[1].handler)(SERIAL_HANDLERS[1].data);
  }
}

void serialEvent2() 
{
  if (SERIAL_HANDLERS[2].handler != NULL) 
  {
    (*SERIAL_HANDLERS[2].handler)(SERIAL_HANDLERS[2].data);
  }
}

void serialEvent3() 
{ 
  if (SERIAL_HANDLERS[3].handler != NULL) 
  {
    (*SERIAL_HANDLERS[3].handler)(SERIAL_HANDLERS[3].data);
  }
}

uint32_t platypus::swap(uint32_t bytes)
{
  return ((bytes << 24) & 0xFF000000)
         | ((bytes <<  8) & 0x00FF0000)
         | ((bytes >>  8) & 0x0000FF00)
         | ((bytes >> 24) & 0x000000FF);
}

/**
 * Cooperative task schedulers for Platypus motors and sensors.
 */
void platypusLoop_()
{
  // TODO: Currently, this runs loops in series, which is wrong.
  // TODO: Parallelize these cooperative loops.

  //Serial.println("In Platypus Loop");
  
  // Run each motor loop task once.
  for (int motorIdx = 0; motorIdx < board::NUM_MOTORS; ++motorIdx)
  {
    Motor *motor = platypus::motors[motorIdx];
    if (motor != NULL)
    {
      platypus::Motor::onLoop_(motor);
    }
  }

  // Run each sensor loop task once.  
  for (int sensorIdx = 0; sensorIdx < board::NUM_SENSORS; ++sensorIdx)
  {
    Sensor *sensor = platypus::sensors[sensorIdx];
    if (sensor != NULL) 
    {
      platypus::Sensor::onLoop_(sensor);
    }
  }

  yield();
}

void platypus::init()
{
  Scheduler.startLoop(platypusLoop_);
}

Led::Led()
  : r_(0), g_(0), b_(0)
{
  pinMode(board::LED.R, OUTPUT);
  digitalWrite(board::LED.R, HIGH);

  pinMode(board::LED.G, OUTPUT);
  digitalWrite(board::LED.G, HIGH);

  pinMode(board::LED.B, OUTPUT);
  digitalWrite(board::LED.B, HIGH);
}

Led::~Led()
{
  pinMode(board::LED.R, INPUT);
  pinMode(board::LED.G, INPUT);
  pinMode(board::LED.B, INPUT);
}

void Led::set(int red, int green, int blue)
{
  R(red);
  G(green);
  B(blue);
}

void Led::R(int red)
{
  r_ = red;
  digitalWrite(board::LED.R, !r_);
}

int Led::R()
{
  return r_;
}

void Led::G(int green)
{
  g_ = green;
  digitalWrite(board::LED.G, !g_);
}

int Led::G()
{
  return g_;
}

void Led::B(int blue)
{
  b_ = blue;
  digitalWrite(board::LED.B, !b_);
}

int Led::B()
{
  return b_;
}

Motor::Motor(int channel)
  : enable_(board::MOTOR[channel].ENABLE), enabled_(false), velocity_(0), servo_ctrl(board::MOTOR[channel].SERVO_CTRL)
{
  channel_ = channel;
  servo_.attach(board::MOTOR[channel_].SERVO);
  pinMode(enable_, OUTPUT);
  // Initialize with power output on
  digitalWrite(enable_, HIGH);
  // Initialize with ESC softswitch off
  pinMode(servo_ctrl,OUTPUT);
  digitalWrite(servo_ctrl,HIGH);
}

Motor::~Motor()
{
  pinMode(enable_, INPUT);
  digitalWrite(enable_, LOW);
  pinMode(servo_ctrl,INPUT);
  digitalWrite(servo_ctrl,HIGH);  
  servo_.detach();
}

void Motor::velocity(float velocity)
{
  //Cap motor signals to between -1.0 and 1.0
  if (velocity > 1.0) {
    velocity = 1.0;
  }
  else if (velocity < -1.0) {
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

void Motor::enablePower(bool enabled)
{
  digitalWrite(enable_, enabled);
}

// Deals with ESC softswitch exclusively
void Motor::enable(bool enabled)
{
  enabled_ = enabled;
  digitalWrite(servo_ctrl, !enabled_);

  if (!enabled_)
  {
    velocity(0.0);
  }
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
  //Will no longer work with battery directly powering esc?
  float vsense = analogRead(board::MOTOR[channel_].CURRENT);
  //V sense is measured across a 330 Ohm resistor, I = V/R
  //I sense is ~1/5000 of output current
  return vsense*5000.0/330.0;
}

bool Motor::set(const char *param, const char *value)
{
  // Set motor velocity.
  if (!strncmp("v", param, 2))
  {
    float v = atof(value);

    // Cap velocity command to between -1.0 and 1.0
    if (v > 1.0) {
      v = 1.0;
    }
    else if (v < -1.0) {
      v = -1.0;
    }

    desiredVelocity_ = v;
    
    return true;
  }
  // Return false for unknown command.
  else
  {
    return false;
  }
}

void Motor::loop()
{
  // At the desired velocity - Do nothing
  if (abs(desiredVelocity_ - velocity_) < VELOCITY_THRESHOLD){
    velocity(desiredVelocity_);
    return;
  }

  /*
  // Scale

  //New desired deadband around 0 - all commands under this magnitude map to 0
  double deadBandSize = 0.001;

  //Max safe reverse command
  double reverseCommandLowerBound = -1.0;
  //Min reverse command that will spin the motors
  double reverseCommandUpperBound = -0.1;

  //Min forward command that will spin the motors
  double forwardCommandLowerBound = 0.03;
  //Max safe forward command
  double forwardCommandUpperBound = 1.0;

  float v = desiredVelocity_;

  if (v < -deadBandSize){
    v = (v + 1.0) * (reverseCommandUpperBound - reverseCommandLowerBound) + reverseCommandLowerBound;
  } else if (v > deadBandSize){
    v = v * (forwardCommandUpperBound - forwardCommandLowerBound) + forwardCommandLowerBound;
  } else {
    v = 0.0;
  }*/

  float v = (1.0 - VELOCITY_ALPHA) * velocity_ + VELOCITY_ALPHA * desiredVelocity_;
  velocity(v);
  
}

void Motor::onLoop_(void *data)
{ 
  // Resolve self-reference and call member function.
  Motor *self = (Motor*)data;
  self->loop();
}

Sensor::Sensor(int channel) 
: channel_(channel)
{  
  // Disable RSxxx receiver
  pinMode(board::SENSOR[channel].RX_DISABLE, OUTPUT);
  digitalWrite(board::SENSOR[channel].RX_DISABLE, HIGH);

  // Disable RSxxx transmitter
  pinMode(board::SENSOR[channel].TX_ENABLE, OUTPUT);
  digitalWrite(board::SENSOR[channel].TX_ENABLE, LOW);

  // Disable RS485 termination resistor
  pinMode(board::SENSOR[channel].RS485_TE, OUTPUT);
  digitalWrite(board::SENSOR[channel].RS485_TE, LOW);

  // Select RS232 (deselect RS485)
  pinMode(board::SENSOR[channel].RS485_232, OUTPUT);
  digitalWrite(board::SENSOR[channel].RS485_232, LOW);

  // TODO: deconflict this!
  if (channel < 2)
  {
    // Disable half-duplex
    pinMode(board::HALF_DUPLEX01, OUTPUT);
    digitalWrite(board::HALF_DUPLEX01, LOW);
  } 
  else 
  {
    // Disable half-duplex
    pinMode(board::HALF_DUPLEX23, OUTPUT);
    digitalWrite(board::HALF_DUPLEX23, LOW);
  }
  
  // Disable loopback test
  pinMode(board::LOOPBACK, OUTPUT);
  digitalWrite(board::LOOPBACK, LOW);
  
  // Disable 12V output
  pinMode(board::SENSOR[channel].PWR_ENABLE, OUTPUT);
  digitalWrite(board::SENSOR[channel].PWR_ENABLE, LOW);
  
  // Register serial event handler
  SerialHandler_t handler = {Sensor::onSerial_, this}; 
  SERIAL_HANDLERS[channel] = handler;
}

void Sensor::calibrate(int flag){
  
}

Sensor::~Sensor()
{
  // TODO: fill me in
}

bool Sensor::set(const char* param, const char* value)
{
  return false;
}

void Sensor::onSerial() 
{
  // Default to doing nothing on serial events. 
}

void Sensor::onSerial_(void *data)
{
  // Resolve self-reference and call member function.
  Sensor *self = (Sensor*)data;
  self->onSerial();
}

void Sensor::loop()
{
  // Do nothing.
}

void Sensor::onLoop_(void *data)
{ 
  // Resolve self-reference and call member function.
  Sensor *self = (Sensor*)data;
  self->loop();
}
