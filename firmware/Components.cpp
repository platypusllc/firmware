#include "Components.h"

using namespace platypus;

#define WAIT_FOR_CONDITION(condition, timeout_ms) for (unsigned int j = 0; j < (timeout_ms) && !(condition); ++j) delay(1);

// TODO: move these somewhere reasonable
// Default gains for the Roboclaw.
/*#define Kp 0x00010000
#define Ki 0x00008000
#define Kd 0x00004000
#define Qpps 44000
*/
//Velocity PID coefficients
#define Kp 2
#define Ki 4
#define Kd 0
#define Qpps 44000 //3600

//Position PID coefficients
#define PosKp 2500
#define PosKi 0
#define PosKd 20000
#define KikMax 0
#define kDeadZone 5 //10
#define kMin 50
#define kMax 1950


#define addr 0x80

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

void Dynamite::arm()
{
  disable();
  delay(500);

  velocity(0.0);
  enable();
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

ServoSensor::ServoSensor(int channel)
  : Sensor(channel), position_(0.0)
{
  servo_.attach(board::SENSOR[channel].GPIO[board::TX_NEG]);
}

ServoSensor::~ServoSensor()
{
  servo_.detach();
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

PoweredSensor::PoweredSensor(int channel, bool poweredOn)
  : Sensor(channel), state_(true)
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
    pinMode(board::SENSOR[channel_].PWR_ENABLE, OUTPUT);
    digitalWrite(board::SENSOR[channel_].PWR_ENABLE, HIGH);
    state_ = true;
    return true;
  }
}

// Turn 12v pin off. Returns true if successful, false if power was already off
bool PoweredSensor::powerOff(){
  if (state_){
    pinMode(board::SENSOR[channel_].PWR_ENABLE, OUTPUT);
    digitalWrite(board::SENSOR[channel_].PWR_ENABLE, LOW);
    state_ = false;
    return true;
  } else {
    //Serial.println("Power Off Failure, sensor already powered off");
    return false;
  }
}

SerialSensor::SerialSensor(int channel, int baudRate, int serialType, int dataStringLength) 
  : Sensor(channel), recv_index_(0)
{
  baud_ = baudRate;
  serialType_ = serialType;
  minDataStringLength_ = dataStringLength;

  if (serialType == RS485){
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
  }
  
  SERIAL_PORTS[channel]->begin(baudRate);
}

void SerialSensor::onSerial(){
  char c = SERIAL_PORTS[channel_]->read();
  
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
               channel_,
               this->name(),
               recv_buffer_
              );
      send(output_str);  
    }
    
    memset(recv_buffer_, 0, recv_index_);
    recv_index_ = 0;
  }
}

ES2::ES2(int channel)
  : Sensor(channel), PoweredSensor(channel, false), SerialSensor(channel, 1200, RS232, 3) //minDataLength filters out "q>"
{
  
}

char* ES2::name()
{
  return "es2";
}

void ES2::loop()
{
  // Enable +12V output.
  powerOn();

  // Read response from sensor.
  delay(250);

  // Turn off +12V output.
  powerOff();

  // Wait a while for next sensor reading.
  delay(1750);
}

AtlasPH::AtlasPH(int channel) 
  : Sensor(channel), SerialSensor(channel, 115200)
{
  //SERIAL_PORTS[channel]->print("C,1,\r");
  //SERIAL_PORTS[channel]->print("SERIAL,115200\r");
}

char * AtlasPH::name(){
  return "atlas_ph";
}

bool AtlasPH::set(char* param, char* value){
  if (strncmp(param, "temp", 4) == 0){
    this->setTemp(atof(value));
    return true;
  }
  return false;
}

void AtlasPH::setTemp(double temp) {
  SERIAL_PORTS[channel_]->print("T,");
  SERIAL_PORTS[channel_]->print(temp);
  SERIAL_PORTS[channel_]->print("\r");
}

void AtlasPH::calibrate(){
  //todo: add calibration routine
}

AtlasDO::AtlasDO(int channel) 
  : Sensor(channel), SerialSensor(channel, 115200)
{
  //SERIAL_PORTS[channel]->print("C,1,\r");
  //SERIAL_PORTS[channel]->print("SERIAL,115200\r");
}


char * AtlasDO::name(){
  return "atlas_do";
}

bool AtlasDO::set(char* param, char* value){
  Serial.println("In AtlasDO set method");
  if (strncmp(param, "ec", 2) == 0){
    this->setEC(atof(value));
    return true;
  } else if (strncmp(param, "temp", 4) == 0){
    this->setTemp(atof(value));
    return true;  
  }
  return false;
}

void AtlasDO::setTemp(double temp) {
  Serial.println("Setting atlas do temp");
  Serial.println(temp);
  SERIAL_PORTS[channel_]->print("T,");
  SERIAL_PORTS[channel_]->print(temp);
  SERIAL_PORTS[channel_]->print("\r");
}

void AtlasDO::setEC(double ec) {
  //Check for salt water and set ec compensation if applicable
  if (ec >= 2500){
    SERIAL_PORTS[channel_]->print("S,");
    SERIAL_PORTS[channel_]->print(ec);
    SERIAL_PORTS[channel_]->print("\r");
  }
}

void AtlasDO::calibrate(){
  //todo: add calibration routine
}

HDS::HDS(int channel)
  : Sensor(channel), PoweredSensor(channel, true), SerialSensor(channel, 4800, RS485)
{

}

char* HDS::name()
{
  return "hds";
}

Winch::Winch(int channel, uint8_t address)
  : Sensor(channel)
  , roboclaw_(platypus::SERIAL_PORTS[channel], 10000)
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
  roboclaw_.SetM1PositionPID(addr, PosKd, PosKp, PosKi, KikMax, kDeadZone, kMin, kMax);
}

void Winch::velocity(int32_t vel)
{
  desired_velocity_ = vel;
  roboclaw_.SetM1VelocityPID(addr, Kd, Kp, Ki, Qpps);
  roboclaw_.SpeedAccelDistanceM1(addr,
                                 desired_acceleration_,
                                 desired_velocity_,
                                 desired_position_);
}

uint32_t Winch::encoder(bool *valid)
{
  uint32_t enc1 = roboclaw_.ReadEncM1(addr, NULL, valid);
  return enc1;
}

