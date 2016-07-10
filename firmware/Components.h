#ifndef COMPONENTS_H
#define COMPONENTS_H

#include "Platypus.h"
#include "RoboClaw.h"

namespace platypus 
{
  const int DEFAULT_BUFFER_SIZE = 128;

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

  enum RMCString_t {
    SENTENCE_TYPE,
    TIME,
    STATUS,
    LATITUDE_RAW,
    LAT_CARDINAL,
    LONGITUDE_RAW,
    LON_CARDINAL,
    SPEED,
    TRACK_ANGLE
  };

  #define RMC_MAX_CHAR_LENGTH   71  // the exact proper number of characters for a RMC sentence 
  #define RMC_WORD_COUNT 9 // after the 9th is not useful


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

  // Sensors //
  class AnalogSensor : public Sensor 
  {
  public:
    AnalogSensor(int channel);

    bool set(const char* param, const char* value);
    virtual char *name() = 0;
    
    void scale(float scale);
    float scale();
    
    void offset(float offset);
    float offset();
    
  private:
    float scale_;
    float offset_;
  };
  
  class ServoSensor : public Sensor 
  {
  public:
    ServoSensor(int channel);
    ~ServoSensor();

    bool set(const char* param, const char* value);
    virtual char *name();
    
    void position(float velocity);
    float position();
    
  private:
    Servo servo_;
    float position_;
  };

  class PoweredSensor : virtual public Sensor 
  {
  public:
    PoweredSensor(int channel, bool poweredOn=true);
    virtual char *name() = 0;
    bool powerOn();
    bool powerOff();

  private:
    bool state_;
  };

  class SerialSensor : virtual public Sensor
  {
  public:
    SerialSensor(int channel, int baudRate, int serialType = DIRECT, int dataStringLength = 0);
    virtual char * name() = 0;
    void onSerial();

    enum SERIAL_TYPE{
      RS232,
      RS485,
      DIRECT
    };

  protected:
    int baud_;
    int serialType_;
    int minDataStringLength_;
    char recv_buffer_[DEFAULT_BUFFER_SIZE];
    unsigned int recv_index_;
  };

  class AHRS : public SerialSensor
  {
  public:
    AHRS(int channel);
    virtual char *name();
    void loop();
   };

  
  class AdafruitGPS : public SerialSensor
  {
  private:
    float RMC_to_Deg(float value);
    String buf;
    String RMCStrings[RMC_WORD_COUNT];
    float latDeg;
    float lonDeg;
    float timeStamp;
    bool valid;
    bool fixed;
    String north = String('N');
    String east = String('E');
    String active = String('A');

  public:
    AdafruitGPS(int channel);
    virtual char *name();
    void loop();
    //void onSerial();
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

  class GY26Compass : public SerialSensor
  {
  private:
    const int measurementInterval;
    int lastMeasurementTime;
    int declinationAngle;
    
  public:
    GY26Compass(int channel);
    virtual char * name();  
    void loop();
    //void onSerial();
  };
  
  class HDS : public PoweredSensor, public SerialSensor
  {
  public:
    HDS(int channel);
    virtual char *name();
    //void onSerial();
  };
  
  class Winch : public Sensor 
  {
  public:
    Winch(int channel, uint8_t address);
    virtual char *name();
    bool set(const char* param, const char* value);
    
    void reset();

    void velocity(int32_t pos);
    void position(uint32_t pos);
    uint32_t encoder(bool *valid = NULL);

  private:
    RoboClaw roboclaw_;
    uint8_t address_;
    uint32_t desired_position_;
    int32_t desired_velocity_;
    uint32_t desired_acceleration_;
  };
}

#endif //COMPONENTS_H
