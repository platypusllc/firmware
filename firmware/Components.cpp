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


// GPS Settings


// different commands to set the update rate from once a second (1 Hz) to 10 times a second (10Hz)
// Note that these only control the rate at which the position is echoed, to actually speed up the
// position fix you must also send one of the position fix rate commands below too.
#define PMTK_SET_NMEA_UPDATE_100_MILLIHERTZ  "$PMTK220,10000*2F" // Once every 10 seconds, 100 millihertz.
#define PMTK_SET_NMEA_UPDATE_200_MILLIHERTZ  "$PMTK220,5000*1B"  // Once every 5 seconds, 200 millihertz.
#define PMTK_SET_NMEA_UPDATE_1HZ  "$PMTK220,1000*1F"
#define PMTK_SET_NMEA_UPDATE_5HZ  "$PMTK220,200*2C"
#define PMTK_SET_NMEA_UPDATE_10HZ "$PMTK220,100*2F"
// Position fix update rate commands.
#define PMTK_API_SET_FIX_CTL_100_MILLIHERTZ  "$PMTK300,10000,0,0,0,0*2C" // Once every 10 seconds, 100 millihertz.
#define PMTK_API_SET_FIX_CTL_200_MILLIHERTZ  "$PMTK300,5000,0,0,0,0*18"  // Once every 5 seconds, 200 millihertz.
#define PMTK_API_SET_FIX_CTL_1HZ  "$PMTK300,1000,0,0,0,0*1C"
#define PMTK_API_SET_FIX_CTL_5HZ  "$PMTK300,200,0,0,0,0*2F"
// Can't fix position faster than 5 times a second!


#define PMTK_SET_BAUD_57600 "$PMTK251,57600*2C"
#define PMTK_SET_BAUD_9600 "$PMTK251,9600*17"

// turn on only the second sentence (GPRMC)
#define PMTK_SET_NMEA_OUTPUT_RMCONLY "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"
// turn on GPRMC and GGA
#define PMTK_SET_NMEA_OUTPUT_RMCGGA "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
// turn on ALL THE DATA
#define PMTK_SET_NMEA_OUTPUT_ALLDATA "$PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
// turn off output
#define PMTK_SET_NMEA_OUTPUT_OFF "$PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"

#define RMC_MAX_CHAR_LENGTH   71  // the exact proper number of characters for a RMC sentence 
#define RMC_WORD_COUNT 9 // after the 9th is not useful



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
  else if (serialType == DIRECT)
  {
    //TX signal unable to pass through bypass resistor
    //Need to enable RS485 driver to succecssfully transmit
    digitalWrite(board::SENSOR[channel].TX_ENABLE, HIGH);
    digitalWrite(board::SENSOR[channel].RS485_232, HIGH);
  }
  //Short delay allows hardware switching to take place
  //before configuration signals are sent
  delay(1); 
  
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

AHRS::AHRS(int channel): Sensor(channel), SerialSensor(channel, 9600, DIRECT, 0){
  
 
}

char * AHRS::name(){
  return "AHRS";
}

void AHRS::loop(){
  
}

AdafruitGPS::AdafruitGPS(int channel): Sensor(channel), SerialSensor(channel, 9600, DIRECT, 0){
  
 
  SERIAL_PORTS[channel]->setTimeout(250);
  // Set output to RMC only
  SERIAL_PORTS[channel]->println(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  SERIAL_PORTS[channel]->flush();
  // Set output rate to 5Hz
  SERIAL_PORTS[channel]->println(PMTK_API_SET_FIX_CTL_5HZ);
  // Set fix rate to 5Hz
  SERIAL_PORTS[channel]->println(PMTK_SET_NMEA_UPDATE_5HZ);

}

char * AdafruitGPS::name(){
  return "AdafruitGPS";
}

void AdafruitGPS::loop(){
  //SERIAL_PORTS[channel_]->println(PMTK_SET_NMEA_OUTPUT_RMCONLY);
}

float AdafruitGPS::RMC_to_Deg(float value) {
   float fullDegrees = floor(value/100.0);
   float minutes = value - fullDegrees*100.0;
   return (fullDegrees + minutes/60.0);
}

/*void AdafruitGPS::onSerial(){
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

    buf = String(recv_buffer_);
    String leader = buf.substring(0,6);
    if (leader.compareTo("$GPRMC") == 0) { 
      int start = 0;
      for (int i = 0; i < RMC_WORD_COUNT; i++) { // split by commas and place each substring into the vector
        int finish = buf.indexOf(',',start);
        RMCStrings[i] = buf.substring(start,finish); // don't include the trailing comma
        start = finish + 1;
      }
    }
  
    latDeg = 0.0;
    lonDeg = 0.0;
    // extract the useful information from the RMC sentence after it is parsed
    if (RMCStrings[STATUS].compareTo(active) != 0) { // there is not a fix
      fixed = false;
      return;
    }
    fixed = true;  
    timeStamp = atof(RMCStrings[TIME].c_str());
    latDeg = RMC_to_Deg(atof(RMCStrings[LATITUDE_RAW].c_str()))*((RMCStrings[LAT_CARDINAL].compareTo(north) == 0) ? 1 : -1);
    lonDeg = RMC_to_Deg(atof(RMCStrings[LONGITUDE_RAW].c_str()))*((RMCStrings[LON_CARDINAL].compareTo(east) == 0) ? 1 : -1);  
    //Serial.print("Latitude = "); Serial.print(latDeg,10);
    //Serial.print("   Longitude = "); Serial.println(lonDeg,10);  
    if (abs(latDeg) < 1e-6 || abs(lonDeg) < 1e-6) {
      valid = false;
    }
    valid = true;  

    if (recv_index_ >  minDataStringLength_ && valid == true){
      char output_str[DEFAULT_BUFFER_SIZE + 3];
      /*snprintf(output_str, DEFAULT_BUFFER_SIZE,
               "{"
               "\"s%u\":{"
               "\"type\":\"%s\","
               "\"data\":\"%s\""
               "}"
               "}",
               channel_,
               this->name(),
               recv_buffer_
              );*/
        /*snprintf(output_str,DEFAULT_BUFFER_SIZE,
        "{"
        "\"g1\":{"
        "\"lati\":%.5f,"
        "\"longi\":%.5f,"
        "\"time\":%.1f"
        "}"
        "}",
        latDeg,
        lonDeg,
        timeStamp
        );
      send(output_str);  
    }
    
    memset(recv_buffer_, 0, recv_index_);
    recv_index_ = 0;
  }
}*/

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
  : Sensor(channel), SerialSensor(channel, 9600, RS232), measurementInterval(3000)
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
  //this->setTemp(27.0);

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
      //Serial.println(F("Atlas pH Sensor Successfully Initialized!"));
      //Serial.print("Calibration: "); Serial.println(calibrationStatus);
      //Serial.print("Temperature(C): "); Serial.println(temperature);
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
    lastCommand = SET_EC;
    this->sendCommand();
  } else if (this->ec > 0.0){
    this->ec = 0.0;
    lastCommand = SET_EC;
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
  //this->setTemp(27.0);
  //this->setEC(10000);
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
      //Serial.println(F("Atlas DO Sensor Successfully Initialized!"));
      //Serial.print("Calibration: "); Serial.println(calibrationStatus);
      //Serial.print("Temperature(C): "); Serial.println(temperature);
      //Serial.print("EC(uS): "); Serial.println(ec);
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

