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

// Adafruit GPS Setting Strings
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
  Dynamite(int channel) : Motor(channel,1000,2000,1500,30,-75) {} //dead range is 1425-1530~
    void arm();
  };
    class AfroESC : public Motor
  {
  public:
    AfroESC(int channel) : Motor(channel,1100,1900,1500,25,-25) {}
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

  class AdafruitGPS : public SerialSensor
  {
  public:
    AdafruitGPS(int id, int port);
    virtual char *name(){ return "gps"; };
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
