#ifndef COMPONENTS_H
#define COMPONENTS_H

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

#include "Platypus.h"

#define DEFAULT_BATTERY_INTERVAL 5000
#define DEFAULT_ATLAS_INTERVAL 3000
#define DEFAULT_ES2_INTERVAL 1500
#define DEFAULT_IMU_INTERVAL 200

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
    BatterySensor(int id, int interval = DEFAULT_BATTERY_INTERVAL);
    virtual char* name();
    void loop();

  private:
    const int interval_;
    int lastMeasurementTime_;
    double lastMeasurement_;
  };

  class IMU : public Sensor
  {
  public:
    IMU(int id, int interval = DEFAULT_IMU_INTERVAL);
    virtual char* name();
    void loop();

    bool isAvailable(){ return available_; };

  private:
    const int interval_;
    bool available_;
    int lastMeasurementTime_;
    Adafruit_BNO055 bno_;

    uint8_t sysCalib_;
    uint8_t gyroCalib_;
    uint8_t accelCalib_;
    uint8_t magCalib_;
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
    ES2(int id, int port, int inteval = DEFAULT_ES2_INTERVAL);
    virtual char *name();
    void loop();

  private:
    SensorState state_;
    int lastMeasurementTime_;
    const int interval_;
    const int minReadTime_;
  };
  
  class AtlasPH : public SerialSensor
  {
  public:
    AtlasPH(int id) : AtlasPH(id, id){};
    AtlasPH(int id, int port, int interval = DEFAULT_ATLAS_INTERVAL);
    bool set(const char * param, const char * value);
    virtual char * name();
    void setTemp(double temp);
    void calibrate(int flag);
    void loop();
    void onSerial();

  private:
    const int interval_;
    int lastMeasurementTime_;
    SensorState state_;
    bool initialized_;
    int calibrationStatus_;
    float temperature_;
    AtlasCommand lastCommand_;

    void sendCommand();
  };

  class AtlasDO : public SerialSensor
  {    
  public:
    AtlasDO(int id) : AtlasDO(id, id){};
    AtlasDO(int id, int port, int interval = DEFAULT_ATLAS_INTERVAL);
    bool set(const char * param, const char * value);
    virtual char * name();
    void setTemp(double temp);
    void setEC(double EC);
    void calibrate(int flag);
    void loop();
    void onSerial();

  private:
    const int interval_;
    int lastMeasurementTime_;
    SensorState state_;
    AtlasCommand lastCommand_;
    bool initialized_;
    int calibrationStatus_;
    float temperature_;
    float ec_;

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
