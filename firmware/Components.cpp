#include "Components.h"

using namespace platypus;

#define WAIT_FOR_CONDITION(condition, timeout_ms) for (unsigned int j = 0; j < (timeout_ms) && !(condition); ++j) delay(1);

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

BatterySensor::BatterySensor(int id) 
  : Sensor(id), measurementInterval(5000)
{
  lastMeasurementTime = 0;
  lastReading = 0.0;
}

char* BatterySensor::name()
{
  return "battery";
}

void BatterySensor::loop()
{
  if (millis() - lastMeasurementTime > measurementInterval){
    // Take a measurement
    int rawVoltage = analogRead(board::V_BATT);
    lastReading = board::V_SCALE * rawVoltage + board::V_OFFSET;
    lastMeasurementTime = millis();

    char output_str[DEFAULT_BUFFER_SIZE + 3];
    snprintf(output_str, DEFAULT_BUFFER_SIZE,
             "{"
             "\"s%u\":{"
             "\"type\":\"%s\","
             "\"data\":\"%.3f %f %d\""
             "}"
             "}",
             id_,
             this->name(),
             lastReading,
             0.0,
             0.0
            );
    send(output_str);  
  }
}

IMU::IMU(int id)
  : Sensor(id), measurementInterval(1000), bno(55, 0x29)
{
  lastMeasurementTime = 0;

  if (!bno.begin()){
    Serial.println(F("Error: Could not initialize IMU"));
  }

  bno.setExtCrystalUse(true);

  delay(500);

  bno.getCalibration(&sysCalib, &gyroCalib, &accelCalib, &magCalib);

  // Wait until calibration values are within limits
  while (sysCalib < 3 && magCalib < 3)
  {
    delay(100);
    Serial.println(F("Warning: Waiting for IMU to calibrate, please move Boat around"));
    bno.getCalibration(&sysCalib, &gyroCalib, &accelCalib, &magCalib);
  }
}

char* IMU::name()
{
  return "imu";
}

void IMU::loop()
{
  if (millis() - lastMeasurementTime > measurementInterval)
  {
    bno.getCalibration(&sysCalib, &gyroCalib, &accelCalib, &magCalib);

    sensors_event_t event;
    bno.getEvent(&event);

    if (sysCalib < 3)
    {
      Serial.println("Warning poor IMU calibration");
    }

    char output_str[DEFAULT_BUFFER_SIZE + 3];
    snprintf(output_str, DEFAULT_BUFFER_SIZE,
             "{"
             "\"s%u\":{"
             "\"type\":\"%s\","
             "\"data\":\"%.4f\""
             "}"
             "}",
             id_,
             this->name(),
             event.orientation.x
            );
    send(output_str); 
  }
}

ServoSensor::ServoSensor(int id, int port) 
  : ExternalSensor(id, port), position_(0.0)
{
  servo_.attach(board::SENSOR_PORT[port].GPIO[board::TX_NEG]);
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

// Known working values: measurementInterval = 1500, minReadTime = 350 (min difference seems to be 1150)
ES2::ES2(int id, int port)
  : ExternalSensor(id, port), PoweredSensor(id, port, false), SerialSensor(id, port, 1200, RS232, 3), measurementInterval(1500), minReadTime(350)//minDataLength filters out "q>"
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

AtlasPH::AtlasPH(int id, int port) 
  : ExternalSensor(id, port), SerialSensor(id, port, 9600), measurementInterval(3000)
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
  if (strncmp(param, "temp", 4) == 0){
    this->setTemp(atof(value));
    return true;
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
    SERIAL_PORTS[port_]->print("Cal,?\r");
    break;

  case GET_TEMP:
    SERIAL_PORTS[port_]->print("T,?\r");
    break;
  
  case READING:
    SERIAL_PORTS[port_]->print("R\r");
    break;

  case SET_TEMP:
    SERIAL_PORTS[port_]->print("T,");
    SERIAL_PORTS[port_]->print(temperature);
    SERIAL_PORTS[port_]->print("\r");
    break;

  case CALIB_LOW:
    Serial.println(F("Calibrate pH probe lowpoint"));
    SERIAL_PORTS[port_]->print("Cal,low,4.00\r");
    break;

  case CALIB_MID:
    Serial.println(F("Calibrate pH probe midpoint"));
    SERIAL_PORTS[port_]->print("Cal,mid,7.00\r");
    break;
  
  case CALIB_HIGH:
    Serial.println(F("Calibrate pH probe highpoint"));
    SERIAL_PORTS[port_]->print("Cal,high,10.00\r");
    break;
  }
}

void AtlasPH::onSerial(){
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
                     id_,
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

AtlasDO::AtlasDO(int id, int port) 
  : ExternalSensor(id, port), SerialSensor(id, port, 9600), measurementInterval(3000)
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
  if (strncmp(param, "ec", 2) == 0){
    this->setEC(atof(value));
    return true;
  } else if (strncmp(param, "temp", 4) == 0){
    this->setTemp(atof(value));
    return true;  
  } else if (strncmp(param, "cal", 3) == 0){
    //Serial.println("trigger calibrate method");
  }
  return false;
}

void AtlasDO::setTemp(double temp) {
  if (temp > 0.0){
    this->temperature = temp;
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
    SERIAL_PORTS[port_]->print("Cal,?\r");
    break;

  case GET_TEMP:
    SERIAL_PORTS[port_]->print("T,?\r");
    break;

  case GET_EC:
    SERIAL_PORTS[port_]->print("S,?\r");
    break;
  
  case READING:
    SERIAL_PORTS[port_]->print("R\r");
    break;

  case SET_TEMP:
    SERIAL_PORTS[port_]->print("T,");
    SERIAL_PORTS[port_]->print(temperature);
    SERIAL_PORTS[port_]->print("\r");
    break;

  case SET_EC:
    SERIAL_PORTS[port_]->print("S,");
    SERIAL_PORTS[port_]->print(ec);
    SERIAL_PORTS[port_]->print("\r");
    break;

  case CALIB_ATM:
    Serial.println(F("Calibrate DO probe to atm"));
    SERIAL_PORTS[port_]->print("Cal\r");
    break;

  case CALIB_ZERO:
    Serial.println(F("Calibrate DO probe to 0"));
    SERIAL_PORTS[port_]->print("Cal,0\r");
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
                     id_,
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

HDS::HDS(int id, int port)
  : ExternalSensor(id, port), PoweredSensor(id, port, true), SerialSensor(id, port, 4800, RS485)
{

}

char* HDS::name()
{
  return "hds";
}

