#include "Components.h"

using namespace platypus;

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

char* ES2::name()
{
  return "es2";
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
         "\"nmea\":\"%s\""
       "}"
      "}",
      channel_,
      recv_buffer_
    );  
    send(output_str);
    
    recv_index_ = 0;
  }
}

Winch::Winch(int channel)
: Sensor(channel) 
{
  // Enable +12V output
  pinMode(board::SENSOR[channel].PWR_ENABLE, OUTPUT);
  digitalWrite(board::SENSOR[channel].PWR_ENABLE, HIGH);
    
  // Start up serial port
  SERIAL_PORTS[channel]->begin(38400);
}

char* Winch::name()
{
  return "winch";
}

void Winch::send(uint8_t address, uint8_t command, uint8_t data)
{
  // Compute 7-bit checksum as per Roboclaw datasheet.
  uint8_t checksum = (address + command + data) & 0x7F;
  
  // TODO: don't hard code this
  SERIAL_PORTS[channel_]->write(address);
  SERIAL_PORTS[channel_]->write(command);
  SERIAL_PORTS[channel_]->write(data);
  SERIAL_PORTS[channel_]->write(checksum);
}
