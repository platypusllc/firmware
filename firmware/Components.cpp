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

BatterySensor::BatterySensor(int id, int interval) 
  : Sensor(id), interval_(interval)
{
  lastMeasurementTime_ = 0;
  lastMeasurement_ = 0.0;
}

char* BatterySensor::name()
{
  return "battery";
}

void BatterySensor::loop()
{
  if (millis() - lastMeasurementTime_ > interval_){
    // Take a measurement
    int rawVoltage = analogRead(board::V_BATT);
    lastMeasurement_ = board::V_SCALE * rawVoltage + board::V_OFFSET;
    lastMeasurementTime_ = millis();

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
             lastMeasurement_,
             0.0,
             0.0
            );
    send(output_str);  
  }
}

IMU::IMU(int id, int interval)
  : Sensor(id), interval_(interval), available_(false), bno_(55, 0x29)
{
  lastMeasurementTime_ = 0;

  if (!bno_.begin()){
    Serial.println(F("Error: Could not initialize IMU"));
    return;
  } else {
    available_ = true;
  }

  bno_.setExtCrystalUse(true);

  delay(500);

  bno_.getCalibration(&sysCalib_, &gyroCalib_, &accelCalib_, &magCalib_);


  /* Want to move this to EEPROM and put int option to recalibrate */
  adafruit_bno055_offsets_t calib;
  calib.accel_offset_x = 0;
  calib.accel_offset_y = 35;
  calib.accel_offset_z = 24;
  calib.gyro_offset_x = 65533;
  calib.gyro_offset_y = 0;
  calib.gyro_offset_z = 65535;
  calib.mag_offset_x = 140;
  calib.mag_offset_y = 65208;
  calib.mag_offset_z = 173;
  calib.accel_radius = 1000;
  calib.mag_radius = 894;

  bno_.setSensorOffsets(calib);


  // Wait until calibration values are within limits
  while (!bno_.isFullyCalibrated())
  {
    delay(100);
    Serial.println(F("Warning: Waiting for IMU to calibrate, please move Boat around"));
    Serial.println(sysCalib_);
    Serial.println(magCalib_);
    Serial.println(accelCalib_);
    bno_.getCalibration(&sysCalib_, &gyroCalib_, &accelCalib_, &magCalib_);
  }


  /* Todo: Put in option to recalibrate
  adafruit_bno055_offsets_t newCalib;

  bno_.getSensorOffsets(newCalib);
  displaySensorOffsets(newCalib);
  */
}

char* IMU::name()
{
  return "imu";
}

void IMU::loop()
{
  if (isAvailable() && millis() - lastMeasurementTime_ > interval_)
  {
    bno_.getCalibration(&sysCalib_, &gyroCalib_, &accelCalib_, &magCalib_);

    sensors_event_t event;
    bno_.getEvent(&event);
    lastMeasurementTime_ = millis();


    if (sysCalib_ < 3)
    {
      Serial.println("Warning poor IMU calibration");
    }

    char output_str[DEFAULT_BUFFER_SIZE + 3];
    snprintf(output_str, DEFAULT_BUFFER_SIZE,
             "{"
             "\"s%u\":{"
             "\"type\":\"%s\","
             "\"data\":\"%.4f,%d,%d,%d,%d\""
             "}"
             "}",
             id_,
             this->name(),
             event.orientation.x,
             sysCalib_,
             magCalib_,
             gyroCalib_,
             accelCalib_
            );
    send(output_str); 
  }
}

AdafruitGPS::AdafruitGPS(int id, int port)
  : ExternalSensor(id, port), SerialSensor(id, port, 9600, RS232, 0)
{
  // Note: This currently does not work after SerialSensor Init!

  SERIAL_PORTS[port]->setTimeout(250);
  // Set output to RMC only
  SERIAL_PORTS[port]->println(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  SERIAL_PORTS[port]->flush();
  // Set output rate to 5Hz
  SERIAL_PORTS[port]->println(PMTK_API_SET_FIX_CTL_5HZ);
  // Set fix rate to 5Hz
  SERIAL_PORTS[port]->println(PMTK_SET_NMEA_UPDATE_5HZ);
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
ES2::ES2(int id, int port, int interval)
  : ExternalSensor(id, port), PoweredSensor(id, port, false), SerialSensor(id, port, 1200, RS232, 3), interval_(interval), minReadTime_(350)
{
  lastMeasurementTime_ = 0;
  state_ = OFF;
}

char* ES2::name()
{
  return "es2";
}

void ES2::loop()
{

  switch (state_){
    case IDLE:
    // Sensor should not enter this state
    case OFF:
      if (millis() - lastMeasurementTime_ > interval_){
        // Take a measurement
        powerOn();
        state_ = WAITING;
        lastMeasurementTime_ = millis();
      }
      break;
    case WAITING:
      if (millis() - lastMeasurementTime_ > minReadTime_){
        // Done taking measurement
        powerOff();
        state_ = OFF;  
      }
  }
  
}

AtlasPH::AtlasPH(int id, int port, int interval) 
  : ExternalSensor(id, port), SerialSensor(id, port, 9600), interval_(interval)
{
  // Initialize internal variables
  lastMeasurementTime_ = 0;
  lastCommand_ = NONE;
  initialized_ = false;
  calibrationStatus_ = -1; // -1 uninitialized, 0 not calibrated, 1 single point, 2 two point, 3 three point
  temperature_ = -1.0;

  state_ = INIT;
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
  if (state_ != WAITING && !initialized_){
    state_ = INIT;
  }
  
  switch (state_){
  // Initializing calibration status from sensor config
  case INIT:
    if (calibrationStatus_ < 0){
      lastCommand_ = GET_CALIB;
    } else if (temperature_ < 0.0){
      lastCommand_ = GET_TEMP;
    } else {
      Serial.println(F("Atlas pH Sensor Successfully Initialized!"));
      Serial.print("Calibration: "); Serial.println(calibrationStatus_);
      Serial.print("Temperature(C): "); Serial.println(temperature_);
      initialized_ = true;
      state_ = IDLE;
      lastCommand_ = NONE;
    }
    break;

  // Sensor Idle, waiting to poll
  case IDLE:
    if (millis() - lastMeasurementTime_ > interval_){
      lastCommand_ = READING;
    }
  }


  if (lastCommand_ != NONE && state_ != WAITING){
    this->sendCommand();
  }
}

void AtlasPH::setTemp(double temp) {
  if (temp > 0.0){
    temperature_ = temp;
    lastCommand_ = SET_TEMP;
    this->sendCommand();
  }
}

void AtlasPH::calibrate(int flag){
  if (flag < 0){
    //calibrate lowpoint
    lastCommand_ = CALIB_LOW;
  } else if (flag > 0){
    //calibrate highpoint
    lastCommand_ = CALIB_HIGH;
  } else{
    //calibrate midpoint
    lastCommand_ = CALIB_MID;
  }

  this->sendCommand();
}

void AtlasPH::sendCommand(){
  state_ = WAITING;
  
  switch (lastCommand_){
  case NONE:
    state_ = IDLE;
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
    SERIAL_PORTS[port_]->print(temperature_);
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

    switch (state_){
    case WAITING:
      if (!strcmp(recv_buffer_, "*ER")){
        //Serial.println("Error Detected, resending last command");
        this->sendCommand();
      } else if (!strcmp(recv_buffer_, "*OK")){
        //Serial.println("OK Confirmation Response Received");
        
        if (lastCommand_ == CALIB_MID || lastCommand_ == CALIB_LOW || lastCommand_ == CALIB_HIGH){
          lastCommand_ = GET_CALIB;
          this->sendCommand();
          //lastCommand = NONE;
          //state = IDLE;
        }
        
      } else {
        switch (lastCommand_){
        case READING:
          if (recv_index_ >  minDataStringLength_){
            lastMeasurementTime_ = millis();
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
          state_ = IDLE;
          lastCommand_ = NONE;
          break;

        case GET_CALIB:
          calibrationStatus_ = recv_buffer_[5] - '0';

          state_ = IDLE;
          lastCommand_ = NONE;
          break;

        case GET_TEMP:
          temperature_ = atof(subString);
          
          state_ = IDLE;
          lastCommand_ = NONE;
          break;

        }
      }
      
    }

    memset(recv_buffer_, 0, recv_index_);
    recv_index_ = 0;
  }
}

AtlasDO::AtlasDO(int id, int port, int interval) 
  : ExternalSensor(id, port), SerialSensor(id, port, 9600), interval_(interval)
{
  // Initialize internal variables
  lastMeasurementTime_ = 0;
  lastCommand_ = NONE;
  initialized_ = false;
  calibrationStatus_ = -1; // -1 uninitialized, 0 not calibrate, 1 single point, 2 two point
  temperature_ = -1.0;
  ec_ = -1.0;

  // Enter INIT state to read sensor info
  state_ = INIT;

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
    temperature_ = temp;
    lastCommand_ = SET_TEMP;
    this->sendCommand();
  }
}

void AtlasDO::setEC(double ec) {
  //Check for salt water and set ec compensation if applicable
  if (ec >= 2500){
    ec_ = ec;
    lastCommand_ = SET_TEMP;
    this->sendCommand();
  } else if (ec_ > 0.0){
    ec_ = 0.0;
    lastCommand_ = SET_TEMP;
    this->sendCommand();
  }
}

void AtlasDO::calibrate(int flag){
  if (flag == 0){
    //calib 0 solution
    lastCommand_ = CALIB_ZERO;
  } else {
    lastCommand_ = CALIB_ATM;
  }

  this->sendCommand();
}

void AtlasDO::sendCommand(){
  state_ = WAITING;
  
  switch (lastCommand_){
  case NONE:
    state_ = IDLE;
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
    SERIAL_PORTS[port_]->print(temperature_);
    SERIAL_PORTS[port_]->print("\r");
    break;

  case SET_EC:
    SERIAL_PORTS[port_]->print("S,");
    SERIAL_PORTS[port_]->print(ec_);
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
  if (state_ != WAITING && !initialized_){
    state_ = INIT;
  }
  
  switch (state_){
  // Initializing calibration status from sensor config
  case INIT:
    if (calibrationStatus_ < 0){
      lastCommand_ = GET_CALIB;
    } else if (temperature_ < 0.0){
      lastCommand_ = GET_TEMP;
    } else if (ec_ < 0.0){
      lastCommand_ = GET_EC;
    } else {
      Serial.println(F("Atlas DO Sensor Successfully Initialized!"));
      Serial.print("Calibration: "); Serial.println(calibrationStatus_);
      Serial.print("Temperature(C): "); Serial.println(temperature_);
      Serial.print("EC(uS): "); Serial.println(ec_);
      initialized_ = true;
      state_ = IDLE;
      lastCommand_ = NONE;
    }
    break;

  // Sensor Idle, waiting to poll
  case IDLE:
    if (millis() - lastMeasurementTime_ > interval_){
      lastCommand_ = READING;
    }
  }


  if (lastCommand_ != NONE && state_ != WAITING){
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

    switch (state_){
    case WAITING:
      if (!strcmp(recv_buffer_, "*ER")){
        //Serial.println("Error Detected, resending last command");
        this->sendCommand();
      } else if (!strcmp(recv_buffer_, "*OK")){
        //Serial.println("OK Confirmation Response Received");
        
        if (lastCommand_ == CALIB_ATM || lastCommand_ == CALIB_ZERO){
          lastCommand_ = GET_CALIB;
          this->sendCommand();
          //state = IDLE;
          //lastCommand = NONE;
        }
        
      } else {
        switch (lastCommand_){
        case READING:
          if (recv_index_ >  minDataStringLength_){
            lastMeasurementTime_ = millis();
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
          state_ = IDLE;
          lastCommand_ = NONE;
          break;

        case GET_CALIB:
          calibrationStatus_ = recv_buffer_[5] - '0';

          state_ = IDLE;
          lastCommand_ = NONE;
          break;

        case GET_TEMP:
          temperature_ = atof(subString);
          
          state_ = IDLE;
          lastCommand_ = NONE;
          break;

        case GET_EC:
          // Trim off ",uS" units 
          recv_buffer_[recv_index_-3] = '\0';
          
          ec_ = atof(subString);

          state_ = IDLE;
          lastCommand_ = NONE;
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

