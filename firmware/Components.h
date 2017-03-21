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
    BatterySensor(int id);
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
    IMU(int id);
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
  class EmptySensor :  public ExternalSensor
  {
  // Empty Sensor placeholder representing a non-connected port
  public:
    EmptySensor(int id) : EmptySensor(id, id){};
    EmptySensor(int id, int port) : ExternalSensor(id, port){};

    virtual char *name(){ return "empty"; };
  };

  class ServoSensor : public ExternalSensor 
  {
  public:
    ServoSensor(int id) : ServoSensor(id, id){};
    ServoSensor(int id, int port);
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
  public:
    ES2(int id) : ES2(id, id){};
    ES2(int id, int port);
    virtual char *name();
    void loop();

  private:
    SensorState state;
    int lastMeasurementTime;
    const int measurementInterval;
    const int minReadTime;
  };
  
  class AtlasPH : public SerialSensor
  {
  public:
    AtlasPH(int id) : AtlasPH(id, id){};
    AtlasPH(int id, int port);
    bool set(const char * param, const char * value);
    virtual char * name();
    void setTemp(double temp);
    void calibrate(int flag);
    void loop();
    void onSerial();

  private:
    const int measurementInterval;
    int lastMeasurementTime;
    SensorState state;
    bool initialized;
    int calibrationStatus;
    float temperature;
    AtlasCommand lastCommand;

    void sendCommand();
  };

  class AtlasDO : public SerialSensor
  {    
  public:
    AtlasDO(int id) : AtlasDO(id, id){};
    AtlasDO(int id, int port);
    bool set(const char * param, const char * value);
    virtual char * name();
    void setTemp(double temp);
    void setEC(double EC);
    void calibrate(int flag);
    void loop();
    void onSerial();

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

  };
  
  class HDS : public PoweredSensor, public SerialSensor
  {
  public:
    HDS(int id) : HDS(id, id){};
    HDS(int id, int port);
    virtual char *name();
    //void onSerial();
  };
  
}

#endif //COMPONENTS_H
