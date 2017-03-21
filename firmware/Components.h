#ifndef COMPONENTS_H
#define COMPONENTS_H

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

#include "Platypus.h"

namespace platypus 
{

  typedef enum 
  {
    OFF,
    INIT,
    IDLE,
    WAITING // Awaiting response from sensor
  } SensorState;

  typedef enum
  {
    NONE,
    STOP_CONTINUOUS, // Stop continuous measurement mode
    READING, // Take a reading
    GET_CALIB, // Get calibration status
    CALIB_ATM, // Atlas DO: Calibrate to atmospheric oxygen levels
    CALIB_ZERO, // Atlas DO: Calibrate to 0 dissolved oxygen
    CALIB_LOW, // Atlas pH: Lowpoint Calibration
    CALIB_MID, // Atlas pH: Midpoint Calibration
    CALIB_HIGH,// Atlas pH: Highpoint Calibration
    GET_TEMP, // Get temperature compensation value
    SET_TEMP, // Set temperature compensation value
    GET_EC, // Get EC compensation value
    SET_EC // Set EC compensation value
  } AtlasCommand;

  // ESCs //
  class VaporPro : public Motor 
  {
  public:
    VaporPro(int channel) : Motor(channel) {}
    void arm();
  };

  class HobbyKingBoat : public Motor 
  {
  public:
    HobbyKingBoat(int channel) : Motor(channel) {}
    void arm();
  };
  
  class Seaking : public Motor 
  {
  public:
    Seaking(int channel) : Motor(channel) {}
    void arm();
  };

  class Swordfish : public Motor 
  {
  public:
    Swordfish(int channel) : Motor(channel) {}
    void arm();
  };

    class Dynamite : public Motor 
  {
  public:
    Dynamite(int channel) : Motor(channel) {}
    void arm();
  };

  // Internal Sensors //
  class BatterySensor : public Sensor
  {
  public:
    BatterySensor(int channel);
    virtual char* name();
    void loop();

  private:
    const int measurementInterval;
    int lastMeasurementTime;
    double lastReading;
  };

  class IMU : public Sensor
  {
  public:
    IMU(int channel);
    virtual char* name();
    void loop();

  private:
    const int measurementInterval;
    int lastMeasurementTime;
    Adafruit_BNO055 bno;

    uint8_t sysCalib;
    uint8_t gyroCalib;
    uint8_t accelCalib;
    uint8_t magCalib;
  };
  
  // External Sensors //
  class ServoSensor : public Sensor 
  {
  public:
    ServoSensor(int channel);
    ~ServoSensor();

    bool set(const char* param, const char* value);
    virtual char *name();
    
    void position(float velocity);
    float position(){ return position_; };
    
  private:
    Servo servo_;
    float position_;
  };

  class ES2 : public PoweredSensor, public SerialSensor
  {
  private:
    SensorState state;
    int lastMeasurementTime;
    const int measurementInterval;
    const int minReadTime;
    
  public:
    ES2(int channel);
    virtual char *name();
    void loop();
  };
  
  class AtlasPH : public SerialSensor
  {
  private:
    const int measurementInterval;
    int lastMeasurementTime;
    SensorState state;
    bool initialized;
    int calibrationStatus;
    float temperature;
    AtlasCommand lastCommand;

    void sendCommand();
    
  public:
    AtlasPH(int channel);
    bool set(const char * param, const char * value);
    virtual char * name();
    void setTemp(double temp);
    void calibrate(int flag);
    void loop();
    void onSerial();
  };

  class AtlasDO : public SerialSensor
  {
  private:
    const int measurementInterval;
    int lastMeasurementTime;
    SensorState state;
    AtlasCommand lastCommand;
    bool initialized;
    int calibrationStatus;
    float temperature;
    float ec;

    //void updateCalibrationStatus();
    void sendCommand();
    
  public:
    AtlasDO(int channel);
    bool set(const char * param, const char * value);
    virtual char * name();
    void setTemp(double temp);
    void setEC(double EC);
    void calibrate(int flag);
    void loop();
    void onSerial();
  };
  
  class HDS : public PoweredSensor, public SerialSensor
  {
  public:
    HDS(int channel);
    virtual char *name();
    //void onSerial();
  };
  
}

#endif //COMPONENTS_H
