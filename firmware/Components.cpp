#include "Components.h"

using namespace platypus;

#define WAIT_FOR_CONDITION(condition, timeout_ms) for (unsigned int j = 0; j < (timeout_ms) && !(condition); ++j) delay(1);

// TODO: move these somewhere reasonable
// Default gains for the Roboclaw.
#define Kp 0x00010000
#define Ki 0x00008000
#define Kd 0x00004000
#define Qpps 44000

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

void HobbyKingBoat::arm()
{
  disable();
  delay(1000);

  velocity(1.0);
  enable();
  delay(3000);

  velocity(0.0);
  delay(3000);
}

void Seaking::arm()
{
  disable();
  delay(500);

  velocity(1.0);
  enable();
  delay(3000);

  velocity(0.0);
  delay(2000);
}

void Swordfish::arm()
{
  disable();
  delay(500);
  
  velocity(1.0);
  enable();
  delay(5000);

  velocity(0.0);
  delay(3000);
}

AnalogSensor::AnalogSensor(int channel)
  : Sensor(channel), scale_(1.0f), offset_(0.0f) {}

bool AnalogSensor::set(char* param, char* value)
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

float AnalogSensor::scale()
{
  return scale_;
}

void AnalogSensor::offset(float offset)
{
  offset = offset_;
}

float AnalogSensor::offset()
{
  return offset_;
}

char* AnalogSensor::name()
{
  return "analog";
}

ServoSensor::ServoSensor(int channel)
  : Sensor(channel), position_(0.0)
{
  servo_.attach(board::SENSOR[channel].GPIO[board::TX_NEG]);
  servo_legacy_.attach(board::SENSOR[channel].GPIO[board::RX_POS]);
}

ServoSensor::~ServoSensor()
{
  servo_.detach(); 
  servo_legacy_.detach();
}

void ServoSensor::position(float position)
{
  if (position > 1.0) {
    position = 1.0;
  }
  if (position < -1.0) {
    position = -1.0;
  }
  position_ = position;

  float command = (position_ * 600) + 1500;
  servo_.writeMicroseconds(command);
  servo_legacy_.writeMicroseconds(command);
}

float ServoSensor::position()
{
  return position_;
}

bool ServoSensor::set(char *param, char *value)
{
  // Set motor velocity.
  if (!strncmp("p", param, 2))
  {
    float p = atof(value);
    position(p);
    return true;
  }
  // Return false for unknown command.
  else
  {
    return false;
  }
}

char* ServoSensor::name()
{
  return "servo";
}

PoweredSensor::PoweredSensor(int channel)
: Sensor(channel)
{
  pinMode(board::SENSOR[channel_].PWR_ENABLE, OUTPUT);
  digitalWrite(board::SENSOR[channel_].PWR_ENABLE, HIGH);
}

char* PoweredSensor::name()
{
  return "powered";
}

ES2::ES2(int channel)
: Sensor(channel), recv_index_(0)
{
  // Start up serial port
  SERIAL_PORTS[channel]->begin(1200);
}

char* ES2::name()
{
  return "es2";
}

void ES2::loop()
{
  // Enable +12V output.
  pinMode(board::SENSOR[channel_].PWR_ENABLE, OUTPUT);
  digitalWrite(board::SENSOR[channel_].PWR_ENABLE, HIGH);
  
  // Read response from sensor.
  delay(250);
  
  // Turn off +12V output.
  pinMode(board::SENSOR[channel_].PWR_ENABLE, OUTPUT);
  digitalWrite(board::SENSOR[channel_].PWR_ENABLE, LOW);
  
  // Wait a while for next sensor reading.
  delay(1750);
}

void ES2::onSerial()
{
  // TODO: verify checksum.
  char c = SERIAL_PORTS[channel_]->read();
  if (c == '\0') { 
    return;
  } 
  else if (c != '\r' && c != '\n' && recv_index_ < DEFAULT_BUFFER_SIZE)
  {
    recv_buffer_[recv_index_] = c;
    ++recv_index_;
  }
  else if (recv_index_ > 0)
  { 
    recv_buffer_[recv_index_] = '\0';
    
    if (recv_index_ > 6) // Only send data strings
    {
      char output_str[DEFAULT_BUFFER_SIZE+3];
      snprintf(output_str, DEFAULT_BUFFER_SIZE,
        "{"
         "\"s%u\":{"
           "\"type\":\"es2\","
           "\"data\":\"%s\""
         "}"
        "}",
        channel_,
        recv_buffer_
      );
      send(output_str);
    }

    memset(recv_buffer_, 0, recv_index_);
    recv_index_ = 0;
  }
}

AtlasSensor::AtlasSensor(int channel)
: Sensor(channel), recv_index_(0)
{
  // Start up serial port.
  SERIAL_PORTS[channel]->begin(38400);
  
  // Tell the sensor to output continuously.
  SERIAL_PORTS[channel_]->print("C\r");
}

char* AtlasSensor::name()
{
  return "atlas";
}

void AtlasSensor::onSerial()
{
  char c = SERIAL_PORTS[channel_]->read();
  if (c != '\r' && c != '\n' && recv_index_ < DEFAULT_BUFFER_SIZE)
  {
    recv_buffer_[recv_index_] = c;
    ++recv_index_;
  }
  else if (recv_index_ > 0)
  { 
    recv_buffer_[recv_index_] = '\0';
    
    char output_str[DEFAULT_BUFFER_SIZE+3];
    snprintf(output_str, DEFAULT_BUFFER_SIZE,
      "{"
       "\"s%u\":{"
         "\"type\":\"atlas\","
         "\"data\":\"%s\""
       "}"
      "}",
      channel_,
      recv_buffer_
    );  
    send(output_str);

    memset(recv_buffer_, 0, recv_index_);   
    recv_index_ = 0;
  }
}

Hdf5::Hdf5(int channel)
: Sensor(channel), recv_index_(0)
{
  // Enable +12V output
  pinMode(board::SENSOR[channel].PWR_ENABLE, OUTPUT);
  digitalWrite(board::SENSOR[channel].PWR_ENABLE, HIGH);

  // Enable RSxxx receiver
  pinMode(board::SENSOR[channel].RX_DISABLE, OUTPUT);
  digitalWrite(board::SENSOR[channel].RX_DISABLE, LOW);

  // Enable RSxxx transmitter
  pinMode(board::SENSOR[channel].TX_ENABLE, OUTPUT);
  digitalWrite(board::SENSOR[channel].TX_ENABLE, HIGH);

  // Enable RS485 termination resistor
  pinMode(board::SENSOR[channel].RS485_TE, OUTPUT);
  digitalWrite(board::SENSOR[channel].RS485_TE, HIGH);

  // Select RS485 (deselect RS232)
  pinMode(board::SENSOR[channel].RS485_232, OUTPUT);
  digitalWrite(board::SENSOR[channel].RS485_232, HIGH);
  
  // Start up serial port
  SERIAL_PORTS[channel]->begin(4800);
}

char* Hdf5::name()
{
  return "hdf5";
}

void Hdf5::onSerial()
{
  char c = SERIAL_PORTS[channel_]->read();
  if (c != '\r' && c != '\n' && recv_index_ < DEFAULT_BUFFER_SIZE)
  {
    recv_buffer_[recv_index_] = c;
    ++recv_index_;
  }
  else if (recv_index_ > 0)
  { 
    recv_buffer_[recv_index_] = '\0';
    
    char output_str[DEFAULT_BUFFER_SIZE+3];
    snprintf(output_str, DEFAULT_BUFFER_SIZE,
      "{"
       "\"s%u\":{"
         "\"type\":\"hdf5\","
         "\"nmea\":\"%s\""
       "}"
      "}",
      channel_,
      recv_buffer_
    );  
    send(output_str);
    
    memset(recv_buffer_, 0, recv_index_);
    recv_index_ = 0;
  }
}

Winch::Winch(int channel, uint8_t address)
: Sensor(channel)
, roboclaw_(platypus::SERIAL_PORTS[channel], 100)
, address_(address)
, desired_position_(0)
, desired_velocity_(0)
, desired_acceleration_(12000)
{  
  // Enable +12V output.
  pinMode(board::SENSOR[channel].PWR_ENABLE, OUTPUT);
  digitalWrite(board::SENSOR[channel].PWR_ENABLE, HIGH);
  
  // TODO: specifically enable e-stop line.
  // (Right now it is just pulled up by default.)
  
  // Start up Roboclaw.
  roboclaw_.begin(38400);
}

char* Winch::name()
{
  return "winch";
}

bool Winch::set(char* param, char* value)
{
  // Set winch position
  if (!strncmp("p", param, 2))
  {
    uint32_t pos = atol(value);
    position(pos);
    return true;
  }
  else if (!strncmp("v", param, 2))
  {
    int32_t vel = atol(value);
    velocity(vel);
    return true;    
  }
  else if (!strncmp("reset", param, 6))
  {
    reset();
    return true;
  }
  // Return false for unknown command.
  else
  {
    return false;
  }
}

void Winch::reset()
{
  roboclaw_.ResetEncoders(address_);
}

void Winch::position(uint32_t pos)
{
  desired_position_ = pos;
  roboclaw_.SetM1VelocityPID(address_, Kd, Kp, Ki, Qpps);
  roboclaw_.SpeedAccelDistanceM1(address_,
                                 desired_acceleration_,
                                 desired_velocity_,
                                 desired_position_);
}

void Winch::velocity(int32_t vel)
{
  desired_velocity_ = vel;
  roboclaw_.SetM1VelocityPID(address_, Kd, Kp, Ki, Qpps);
  roboclaw_.SpeedAccelDistanceM1(address_, 
                                 desired_acceleration_, 
                                 desired_velocity_,
                                 desired_position_);
}

uint32_t Winch::encoder(bool *valid)
{
  uint32_t enc1 = roboclaw_.ReadEncM1(address_, NULL, valid);
  return enc1;
}

