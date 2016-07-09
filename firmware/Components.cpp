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

bool ServoSensor::set(const char *param, const char *value)
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

// Known working values: measurementInterval = 1500, minReadTime = 350 (min difference seems to be 1150)
ES2::ES2(int channel)
  : Sensor(channel), PoweredSensor(channel, false), SerialSensor(channel, 1200, RS232, 3), measurementInterval(1500), minReadTime(350)//minDataLength filters out "q>"
{
  lastMeasurementTime = 0;
  state = OFF;
}

char* ES2::name()
{
  return "es2";
}

void ES2::loop()
{

  switch (state){
    case IDLE:
    // Sensor should not enter this state
    case OFF:
      if (millis() - lastMeasurementTime > measurementInterval){
        // Take a measurement
        powerOn();
        state = WAITING;
        lastMeasurementTime = millis();
      }
      break;
    case WAITING:
      if (millis() - lastMeasurementTime > minReadTime){
        // Done taking measurement
        powerOff();
        state = OFF;  
      }
  }
  
}

AtlasPH::AtlasPH(int channel) 
  : Sensor(channel), SerialSensor(channel, 9600), measurementInterval(3000)
{
  // Initialize internal variables
  lastMeasurementTime = 0;
  lastCommand = NONE;
  initialized = false;
  calibrationStatus = -1; // -1 uninitialized, 0 not calibrated, 1 single point, 2 two point, 3 three point
  temperature = -1.0;

  state = INIT;
}

char * AtlasPH::name(){
  return "atlas_ph";
}

bool AtlasPH::set(const char* param, const char* value){
  if (strncmp(param, "t", 1) == 0){
    if ( value != NULL )
    {
      this->setTemp(atof(value));
      return true;
    }
    //Send temperature compensation value
    else 
    {
      char output_str[DEFAULT_BUFFER_SIZE + 3];
      snprintf(output_str, DEFAULT_BUFFER_SIZE,
               "{"
               "\"s%u\":{"
               "\"type\":\"%s\","
               "\"data\":\t,%f\""
               "}"
               "}",
               channel_,
               this->name(),
               temperature
              );
      send(output_str);
        
    }
  } else if (strncmp(param, "c", 1) == 0){
      //Get calibration status
      if (value == NULL)
      {
        char output_str[DEFAULT_BUFFER_SIZE + 3];
      snprintf(output_str, DEFAULT_BUFFER_SIZE,
               "{"
               "\"s%u\":{"
               "\"type\":\"%s\","
               "\"data\":\"c,%f\""
               "}"
               "}",
               channel_,
               this->name(),
               calibrationStatus
              );
      send(output_str);
         
      }
      //Calibrate PH lowpoint
      else if(strncmp(value, "low", 3) == 0)
      {
        this->calibrate(-1);
        return true; 
      }
      //Calibrate PH midpoint
      else if(strncmp(value, "mid", 3) == 0)
      {
        this->calibrate(0);
        return true;
      }
      //Calibrate PH highpoint
      else if(strncmp(value, "high", 4) == 0)
      {
        this->calibrate(1);
        return true;
      }
      //Serial.println("trigger calibrate method");
  }
  return false;
}

void AtlasPH::loop(){
  // Enter INIT state if sensor is not fully initialized
  if (state != WAITING && !initialized){
    state = INIT;
  }
  
  switch (state){
  // Initializing calibration status from sensor config
  case INIT:
    if (calibrationStatus < 0){
      lastCommand = GET_CALIB;
    } else if (temperature < 0.0){
      lastCommand = GET_TEMP;
    } else {
      Serial.println(F("Atlas pH Sensor Successfully Initialized!"));
      Serial.print("Calibration: "); Serial.println(calibrationStatus);
      Serial.print("Temperature(C): "); Serial.println(temperature);
      initialized = true;
      state = IDLE;
      lastCommand = NONE;
    }
    break;

  // Sensor Idle, waiting to poll
  case IDLE:
    if (millis() - lastMeasurementTime > measurementInterval){
      lastCommand = READING;
    }
  }


  if (lastCommand != NONE && state != WAITING){
    this->sendCommand();
  }
}

void AtlasPH::setTemp(double temp) {
  if (temp > 0.0){
    this->temperature = temp;
    lastCommand = SET_TEMP;
    this->sendCommand();
  }
}

void AtlasPH::calibrate(int flag){
  if (flag < 0){
    //calibrate lowpoint
    lastCommand = CALIB_LOW;
  } else if (flag > 0){
    //calibrate highpoint
    lastCommand = CALIB_HIGH;
  } else{
    //calibrate midpoint
    lastCommand = CALIB_MID;
  }

  this->sendCommand();
}

void AtlasPH::sendCommand(){
  state = WAITING;
  
  switch (lastCommand){
  case NONE:
    state = IDLE;
    break;
    
  case GET_CALIB:
    SERIAL_PORTS[channel_]->print("Cal,?\r");
    break;

  case GET_TEMP:
    SERIAL_PORTS[channel_]->print("T,?\r");
    break;
  
  case READING:
    SERIAL_PORTS[channel_]->print("R\r");
    break;

  case SET_TEMP:
    SERIAL_PORTS[channel_]->print("T,");
    SERIAL_PORTS[channel_]->print(temperature);
    SERIAL_PORTS[channel_]->print("\r");
    break;

  case CALIB_LOW:
    Serial.println(F("Calibrate pH probe lowpoint"));
    SERIAL_PORTS[channel_]->print("Cal,low,4.00\r");
    break;

  case CALIB_MID:
    Serial.println(F("Calibrate pH probe midpoint"));
    SERIAL_PORTS[channel_]->print("Cal,mid,7.00\r");
    break;
  
  case CALIB_HIGH:
    Serial.println(F("Calibrate pH probe highpoint"));
    SERIAL_PORTS[channel_]->print("Cal,high,10.00\r");
    break;
  }
}

void AtlasPH::onSerial(){
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

    //Serial.print("Raw Sensor Input: ");
    //Serial.println(recv_buffer_);

    // Trims first three characters off response (used to trim temp and ec responses)
    char *subString = recv_buffer_ + 3;

    switch (state){
    case WAITING:
      if (!strcmp(recv_buffer_, "*ER")){
        //Serial.println("Error Detected, resending last command");
        this->sendCommand();
      } else if (!strcmp(recv_buffer_, "*OK")){
        //Serial.println("OK Confirmation Response Received");
        
        if (lastCommand == CALIB_MID || lastCommand == CALIB_LOW || lastCommand == CALIB_HIGH){
          lastCommand = GET_CALIB;
          this->sendCommand();
          //lastCommand = NONE;
          //state = IDLE;
        }
        
      } else {
        switch (lastCommand){
        case READING:
          if (recv_index_ >  minDataStringLength_){
            lastMeasurementTime = millis();
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
          state = IDLE;
          lastCommand = NONE;
          break;

        case GET_CALIB:
          calibrationStatus = recv_buffer_[5] - '0';

          state = IDLE;
          lastCommand = NONE;
          break;

        case GET_TEMP:
          temperature = atof(subString);
          
          state = IDLE;
          lastCommand = NONE;
          break;

        }
      }
      
    }

    memset(recv_buffer_, 0, recv_index_);
    recv_index_ = 0;
  }
}

AtlasDO::AtlasDO(int channel) 
  : Sensor(channel), SerialSensor(channel, 9600), measurementInterval(3000)
{
  // Initialize internal variables
  lastMeasurementTime = 0;
  lastCommand = NONE;
  initialized = false;
  calibrationStatus = -1; // -1 uninitialized, 0 not calibrate, 1 single point, 2 two point
  temperature = -1.0;
  ec = -1.0;

  // Enter INIT state to read sensor info
  state = INIT;

  // Code to set BAUD rate - eventually implement check for incorrect baud rate
  //SERIAL_PORTS[channel]->print("SERIAL,115200\r");
  
  // Disable continous sensor polling
  //SERIAL_PORTS[channel]->print("C,0,\r");
  //SERIAL_PORTS[channel]->print("C,0,\r");
}


char * AtlasDO::name(){
  return "atlas_do";
}

bool AtlasDO::set(const char* param, const char* value){
  if (strncmp(param, "e", 1) == 0){
    if ( value != NULL )
    {
      this->setEC(atof(value));
      return true;
    }
    else
    {
      char output_str[DEFAULT_BUFFER_SIZE + 3];
      snprintf(output_str, DEFAULT_BUFFER_SIZE,
               "{"
               "\"s%u\":{"
               "\"type\":\"%s\","
               "\"data\":\e,%f\""
               "}"
               "}",
               channel_,
               this->name(),
               ec
              );
      send(output_str);
    }
  } else if (strncmp(param, "t", 1) == 0){
    if ( value != NULL )
    {
      this->setTemp(atof(value));
      return true;
    }
    //Send temperature compensation value
    else 
    {
      char output_str[DEFAULT_BUFFER_SIZE + 3];
      snprintf(output_str, DEFAULT_BUFFER_SIZE,
               "{"
               "\"s%u\":{"
               "\"type\":\"%s\","
               "\"data\":\t,%f\""
               "}"
               "}",
               channel_,
               this->name(),
               temperature
              );
      send(output_str);
        
    }
  } else if (strncmp(param, "c", 1) == 0){
      //Get calibration status
      if (value == NULL)
      {
        char output_str[DEFAULT_BUFFER_SIZE + 3];
      snprintf(output_str, DEFAULT_BUFFER_SIZE,
               "{"
               "\"s%u\":{"
               "\"type\":\"%s\","
               "\"data\":\"c,%f\""
               "}"
               "}",
               channel_,
               this->name(),
               calibrationStatus
              );
      send(output_str);
         
      }
      //Zero DO sensor
      else if(strncmp(value, "zero", 4) == 0)
      {
        this->calibrate(0);
        return true; 
      }
      //Calibrate DO sensor to atmospheric pressure
      else if(strncmp(value, "atm", 3) == 0)
      {
        this->calibrate(1);
        return true;
      }
     
      //Serial.println("trigger calibrate method");
  }
  return false;
}

//Clamp temperature to range 0,100.0
void AtlasDO::setTemp(double temp) {
  if (temp > 0.0){
    this->temperature = min(temp, 100.0);
    lastCommand = SET_TEMP;
    this->sendCommand();
  }
}

void AtlasDO::setEC(double ec) {
  //Check for salt water and set ec compensation if applicable
  if (ec >= 2500){
    this->ec = ec;
    lastCommand = SET_TEMP;
    this->sendCommand();
  } else if (this->ec > 0.0){
    this->ec = 0.0;
    lastCommand = SET_TEMP;
    this->sendCommand();
  }
}

void AtlasDO::calibrate(int flag){
  if (flag == 0){
    //calib 0 solution
    lastCommand = CALIB_ZERO;
  } else {
    lastCommand = CALIB_ATM;
  }

  this->sendCommand();
}

void AtlasDO::sendCommand(){
  state = WAITING;
  
  switch (lastCommand){
  case NONE:
    state = IDLE;
    break;
    
  case GET_CALIB:
    SERIAL_PORTS[channel_]->print("Cal,?\r");
    break;

  case GET_TEMP:
    SERIAL_PORTS[channel_]->print("T,?\r");
    break;

  case GET_EC:
    SERIAL_PORTS[channel_]->print("S,?\r");
    break;
  
  case READING:
    SERIAL_PORTS[channel_]->print("R\r");
    break;

  case SET_TEMP:
    SERIAL_PORTS[channel_]->print("T,");
    SERIAL_PORTS[channel_]->print(temperature);
    SERIAL_PORTS[channel_]->print("\r");
    break;

  case SET_EC:
    SERIAL_PORTS[channel_]->print("S,");
    SERIAL_PORTS[channel_]->print(ec);
    SERIAL_PORTS[channel_]->print("\r");
    break;

  case CALIB_ATM:
    Serial.println(F("Calibrate DO probe to atm"));
    SERIAL_PORTS[channel_]->print("Cal\r");
    break;

  case CALIB_ZERO:
    Serial.println(F("Calibrate DO probe to 0"));
    SERIAL_PORTS[channel_]->print("Cal,0\r");
    break;

  }
}

void AtlasDO::loop(){
  // Enter INIT state if sensor is not fully initialized
  if (state != WAITING && !initialized){
    state = INIT;
  }
  
  switch (state){
  // Initializing calibration status from sensor config
  case INIT:
    if (calibrationStatus < 0){
      lastCommand = GET_CALIB;
    } else if (temperature < 0.0){
      lastCommand = GET_TEMP;
    } else if (ec < 0.0){
      lastCommand = GET_EC;
    } else {
      Serial.println(F("Atlas DO Sensor Successfully Initialized!"));
      Serial.print("Calibration: "); Serial.println(calibrationStatus);
      Serial.print("Temperature(C): "); Serial.println(temperature);
      Serial.print("EC(uS): "); Serial.println(ec);
      initialized = true;
      state = IDLE;
      lastCommand = NONE;
    }
    break;

  // Sensor Idle, waiting to poll
  case IDLE:
    if (millis() - lastMeasurementTime > measurementInterval){
      lastCommand = READING;
    }
  }


  if (lastCommand != NONE && state != WAITING){
    this->sendCommand();
  }
}

void AtlasDO::onSerial(){
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

    //Serial.print("Raw Sensor Input: ");
    //Serial.println(recv_buffer_);

    // Trims first three characters off response (used to trim temp and ec responses)
    char *subString = recv_buffer_ + 3;

    switch (state){
    case WAITING:
      if (!strcmp(recv_buffer_, "*ER")){
        //Serial.println("Error Detected, resending last command");
        this->sendCommand();
      } else if (!strcmp(recv_buffer_, "*OK")){
        //Serial.println("OK Confirmation Response Received");
        
        if (lastCommand == CALIB_ATM || lastCommand == CALIB_ZERO){
          lastCommand = GET_CALIB;
          this->sendCommand();
          //state = IDLE;
          //lastCommand = NONE;
        }
        
      } else {
        switch (lastCommand){
        case READING:
          if (recv_index_ >  minDataStringLength_){
            lastMeasurementTime = millis();
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
          state = IDLE;
          lastCommand = NONE;
          break;

        case GET_CALIB:
          calibrationStatus = recv_buffer_[5] - '0';

          state = IDLE;
          lastCommand = NONE;
          break;

        case GET_TEMP:
          temperature = atof(subString);
          
          state = IDLE;
          lastCommand = NONE;
          break;

        case GET_EC:
          // Trim off ",uS" units 
          recv_buffer_[recv_index_-3] = '\0';
          
          ec = atof(subString);

          state = IDLE;
          lastCommand = NONE;
          break;
        }
      }
      
    }

    memset(recv_buffer_, 0, recv_index_);
    recv_index_ = 0;
  }
}

GY26Compass::GY26Compass(int channel) : Sensor(channel), SerialSensor(channel, 9600, 0), measurementInterval(5000)
{
  lastMeasurementTime = 0;
  declinationAngle = 93; 
}

char * GY26Compass::name(){
  return "GY26Compass";
}

void GY26Compass::loop(){
  if (millis() - lastMeasurementTime > measurementInterval){
    SERIAL_PORTS[channel_]->write(0x31);
    lastMeasurementTime = millis();
    //Serial.println("requested compass measurement");
  }
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

bool Winch::set(const char* param, const char* value)
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

