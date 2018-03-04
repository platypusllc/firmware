#include "Platypus.h"


#include <Adafruit_NeoPixel.h>
// LED serial controller.
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(board::NUM_LEDS, board::LED, NEO_GRB + NEO_KHZ800);

using namespace platypus;

// Velocity decay/ramping constants
constexpr float VELOCITY_ALPHA = 0.1;
constexpr float VELOCITY_THRESHOLD = 0.001;

// TODO: Correct default initialization of these sensors.
platypus::Peripheral *platypus::peripherals[board::NUM_PERIPHERALS];
platypus::Motor *platypus::motors[board::NUM_MOTORS];
platypus::Sensor *platypus::sensors[board::NUM_SENSORS];

// TODO: Switch to using HardwareSerial.
USARTClass *platypus::SERIAL_PORTS[4] = {
  &Serial1,
  &Serial2,
  &Serial3,
  nullptr
};

SerialHandler_t platypus::SERIAL_HANDLERS[4] = {
  {NULL, NULL},
  {NULL, NULL},
  {NULL, NULL},
  {NULL, NULL}
};


void serialEvent1()
{
  if (SERIAL_HANDLERS[0].handler != NULL)
  {
    (*SERIAL_HANDLERS[0].handler)(SERIAL_HANDLERS[0].data);
  }
}

void serialEvent2()
{
  if (SERIAL_HANDLERS[1].handler != NULL)
  {
    (*SERIAL_HANDLERS[1].handler)(SERIAL_HANDLERS[1].data);
  }
}

void serialEvent3()
{
  if (SERIAL_HANDLERS[2].handler != NULL)
  {
    (*SERIAL_HANDLERS[2].handler)(SERIAL_HANDLERS[2].data);
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
  //pixels.begin();
  //pixels.show();
}

Led::Led()
  : r_(0), g_(0), b_(0)
{
}

Led::~Led()
{

}

void Led::set(int red, int green, int blue)
{
  r_ = red;
  g_ = green;
  b_ = blue;

  /*
    while (!pixels.canShow());
    for (size_t pixel_idx = 0; pixel_idx < board::NUM_LEDS; ++pixel_idx)
    pixels.setPixelColor(pixel_idx, r_, g_, b_);
    pixels.show();*/
}

void Led::R(int red)
{
  set(red, g_, b_);
}

int Led::R()
{
  return r_;
}

void Led::G(int green)
{
  set(r_, green, b_);
}

int Led::G()
{
  return g_;
}

void Led::B(int blue)
{
  set(r_, g_, blue);
}

int Led::B()
{
  return b_;
}

Peripheral::Peripheral(int channel, bool enabled)
  : channel_(channel), enable_(board::PERIPHERAL[channel].ENABLE)
{
  pinMode(enable_, OUTPUT);
  enable(enabled);
}

Peripheral::~Peripheral()
{
  disable();
  pinMode(enable_, INPUT);
}

void Peripheral::enable(bool enabled)
{
  enabled_ = enabled;
  digitalWrite(enable_, enabled);
}

float Peripheral::current()
{
  float vsense = analogRead(board::PERIPHERAL[channel_].CURRENT);
  //V sense is measured across a 330 Ohm resistor, I = V/R
  //I sense is ~1/5000 of output current
  return vsense*5000.0/330.0;
}

Motor::Motor(int channel,int motorMin,int motorMax,int motorCenter,int motorFDB, int motorRDB)
  : channel_(channel), enable_(board::MOTOR[channel].ENABLE), enabled_(false), velocity_(0.0), 
    motorMax_(motorMax), motorMin_(motorMin), motorCenter_(motorCenter), motorFDB_(motorFDB), motorRDB_(motorRDB)
{
  // Initialize with ESC softswitch off
  pinMode(enable_, OUTPUT);
  disable();

  // Attach Servo object to signal pin
	/* This is commented out to support changes to for BR thrusters
		 Since velocity now sends 0 when !enabled, when the board boots 
		 the pin is attached yet and not enabled, which will send a 0 command
		 and then arm the motors. I think we can also just get rid of that !enabled
		 statement in velocity since now when its not enabled it detaches that servo pin
	*/
  //servo_.attach(board::MOTOR[channel_].SERVO);

}

Motor::~Motor()
{
  disable();
  pinMode(enable_, INPUT);
  servo_.detach();
}

void Motor::velocity(float velocity)
{
	if (!enabled()) //accidently arming boat on start
		{
			servo_.writeMicroseconds(motorCenter_);
			return;
		}
	
  if (velocity > 1.0) {
    velocity = 1.0;
  }
  else if (velocity < -1.0) {
    velocity = -1.0;
  }
  velocity_ = velocity;
  
  float command;
  if (velocity <= -VELOCITY_THRESHOLD)
  {
    command = motorMin_ + (motorCenter_ - motorMin_ + motorRDB_) * (velocity + 1.0) / (1.0 - VELOCITY_THRESHOLD);
  }
  else if (velocity >= VELOCITY_THRESHOLD)
  {
    command = motorCenter_ + motorFDB_ + (motorMax_ - motorCenter_ - motorFDB_) * (velocity - VELOCITY_THRESHOLD) / (1.0 - VELOCITY_THRESHOLD);
  }
  else
  {
    command = motorCenter_;
  }
  //float command = (velocity * 200) + 1500; //binding it from 1300 to 1700 because it sounds like there is damage being done at max 1900?
  servo_.writeMicroseconds(command);
	// printf("velocity is: %d \n",velocity);
	// printf("command is %d \n",command);
}

// Deals with ESC softswitch exclusively
void Motor::enable(bool enabled)
{
  enabled_ = enabled;
  digitalWrite(enable_, !enabled_);

  if (!enabled_)
  {
    velocity(0.0);
  }
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

  float v = (1.0 - VELOCITY_ALPHA) * velocity_ + VELOCITY_ALPHA * desiredVelocity_;
  velocity(v);

}

void Motor::onLoop_(void *data)
{
  // Resolve self-reference and call member function.
  Motor *self = (Motor*)data;
  self->loop();
}

Sensor::Sensor(int id) : id_(id)
{
  // Abstract Class - should not be instantiated
}

Sensor::~Sensor()
{
  // TODO: fill me in
}

bool Sensor::set(const char* param, const char* value)
{
  return false;
}

void Sensor::loop()
{
  // Default to doring nothing during loop function.
}

void Sensor::onLoop_(void *data)
{
  // Resolve self-reference and call member function.
  Sensor *self = (Sensor*)data;
  self->loop();
}

ExternalSensor::ExternalSensor(int id, int port) : Sensor(id), port_(port)
{
  //Unknown if following code is necessary...

  // Disable RSxxx receiver
  pinMode(board::SENSOR_PORT[port].RX_DISABLE, OUTPUT);
  digitalWrite(board::SENSOR_PORT[port].RX_DISABLE, HIGH);

  // Disable RSxxx transmitter
  pinMode(board::SENSOR_PORT[port].TX_ENABLE, OUTPUT);
  digitalWrite(board::SENSOR_PORT[port].TX_ENABLE, LOW);

  // Disable RS485 termination resistor
  pinMode(board::SENSOR_PORT[port].RS485_TE, OUTPUT);
  digitalWrite(board::SENSOR_PORT[port].RS485_TE, LOW);

  // Select RS232 (deselect RS485)
  pinMode(board::SENSOR_PORT[port].RS485_232, OUTPUT);
  digitalWrite(board::SENSOR_PORT[port].RS485_232, LOW);

  // TODO: deconflict this!
  if (port < 2)
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
  pinMode(board::SENSOR_PORT[port].PWR_ENABLE, OUTPUT);
  digitalWrite(board::SENSOR_PORT[port].PWR_ENABLE, LOW);
}


AnalogSensor::AnalogSensor(int id, int port)
  : ExternalSensor(id, port), scale_(1.0f), offset_(0.0f) {}

bool AnalogSensor::set(const char* param, const char* value)
{
  // Set analog scale.
  if (!strncmp("scale", param, 6))
  {
    float s = atof(value);
    scale(s);
    return true;
  }
  // Set analog offset.
  else if (!strncmp("offset", param, 7))
  {
    float o = atof(value);
    offset(o);
    return true;
  }
  // Return false for unknown command.
  else
  {
    return false;
  }
}

void AnalogSensor::scale(float scale)
{
  scale_ = scale;
}

void AnalogSensor::offset(float offset)
{
  offset_ = offset;
}

PoweredSensor::PoweredSensor(int id, int port, bool poweredOn)
  : ExternalSensor(id, port), state_(true)
{
  //PowerOff to be sure sensor is off
  powerOff();

  if (poweredOn){
    powerOn();
  } else {
    powerOff();
  }
}

// Turn 12v pin on. Returns true if successful, false if power was already on
bool PoweredSensor::powerOn(){
  if (state_){
    //Serial.println("Power On Failure, sensor already powered on");
    return false;
  } else {
    pinMode(board::SENSOR_PORT[port_].PWR_ENABLE, OUTPUT);
    digitalWrite(board::SENSOR_PORT[port_].PWR_ENABLE, HIGH);
    state_ = true;
    return true;
  }
}

// Turn 12v pin off. Returns true if successful, false if power was already off
bool PoweredSensor::powerOff(){
  if (state_){
    pinMode(board::SENSOR_PORT[port_].PWR_ENABLE, OUTPUT);
    digitalWrite(board::SENSOR_PORT[port_].PWR_ENABLE, LOW);
    state_ = false;
    return true;
  } else {
    //Serial.println("Power Off Failure, sensor already powered off");
    return false;
  }
}

SerialSensor::SerialSensor(int id, int port, int baud, int type, int dataLength)
  : ExternalSensor(id, port), recv_index_(0), baudRate_(baud), serialType_(type)
{
  minDataStringLength_ = dataLength;

  if (type == TTL)
  {
    pinMode(board::SENSOR_PORT[port].TX_ENABLE, OUTPUT);
    digitalWrite(board::SENSOR_PORT[port].TX_ENABLE, HIGH);

    pinMode(board::SENSOR_PORT[port].RS485_TE, OUTPUT);
    digitalWrite(board::SENSOR_PORT[port].RS485_TE, LOW);

    pinMode(board::SENSOR_PORT[port].RS485_232, OUTPUT);
    digitalWrite(board::SENSOR_PORT[port].RS485_232, HIGH);
  } 
  else if (type == RS485)
  {
    // Enable RSxxx receiver
    pinMode(board::SENSOR_PORT[port].RX_DISABLE, OUTPUT);
    digitalWrite(board::SENSOR_PORT[port].RX_DISABLE, LOW);

    // Enable RSxxx transmitter
    pinMode(board::SENSOR_PORT[port].TX_ENABLE, OUTPUT);
    digitalWrite(board::SENSOR_PORT[port].TX_ENABLE, HIGH);

    // Enable RS485 termination resistor
    pinMode(board::SENSOR_PORT[port].RS485_TE, OUTPUT);
    digitalWrite(board::SENSOR_PORT[port].RS485_TE, HIGH);

    // Select RS485 (deselect RS232)
    pinMode(board::SENSOR_PORT[port].RS485_232, OUTPUT);
    digitalWrite(board::SENSOR_PORT[port].RS485_232, HIGH);
  }

  if (SERIAL_PORTS[port] != nullptr)
  {
    // Register serial event handler
    SerialHandler_t handler = {SerialSensor::onSerial_, this};
    SERIAL_HANDLERS[port] = handler;

    SERIAL_PORTS[port]->begin(baud);
  }
}

void SerialSensor::onSerial(){
  char c = SERIAL_PORTS[port_]->read();

  // Ignore null and tab characters
  if (c == '\0' || c == '\t') {
    return;
  }
  if (c != '\r' && c != '\n' && recv_index_ < DEFAULT_BUFFER_SIZE)
  {
    recv_buffer_[recv_index_] = c;
    ++recv_index_;
  }
  else if (recv_index_ > 0)
  {
    recv_buffer_[recv_index_] = '\0';

    if (recv_index_ >  minDataStringLength_){
      char output_str[DEFAULT_BUFFER_SIZE + 3];
      snprintf(output_str, DEFAULT_BUFFER_SIZE,
               "{"
               "\"s%u\":{"
               "\"type\":\"%s\","
               "\"data\":\"%s\""
               "}"
               "}",
               id_,
               this->name(),
               recv_buffer_
        );
      send(output_str);
    }

    memset(recv_buffer_, 0, recv_index_);
    recv_index_ = 0;
  }
}

void SerialSensor::onSerial_(void *data)
{
  // Resolve self-reference and call member function.
  SerialSensor *self = (SerialSensor*)data;
  self->onSerial();
}
