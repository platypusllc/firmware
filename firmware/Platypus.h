#ifndef PLATYPUS_H
#define PLATYPUS_H

// TODO: Move these to subdirectory or something!
#include "Board.h"
#include <Servo.h>
#include <Scheduler.h>

// TODO: move all ADK stuff into this class
/**
 * Wrapper for ADK send command that copies data to debug port.
 * Requires a null-terminated char* pointer.
 */
extern void send(char *str);

namespace platypus
{
  const int DEFAULT_BUFFER_SIZE = 128;

  // Main library initialization function.
  void init();

  class Configurable
  {
  public:
    virtual bool set(const char *param, const char *value);
  };
  class Led
  {
  public:
    Led();
    virtual ~Led();
    void set(int red, int green, int blue);
    void R(int red);
    int R();
    void G(int green);
    int G();
    void B(int blue);
    int B();

  private:
    Led(const Led&);
    Led& operator=(const Led&);

    int r_, g_, b_;
  };

  class Peripheral
  {
  public:
    // Initialize with power off by default
    Peripheral(int channel, bool enabled = false);
    virtual ~Peripheral();

    // Functions to turn peripheral power on/off
    void enable(bool enabled);
    bool enabled(){ return enabled_; };

    void enable(){ enable(true); };
    void disable(){ enable(false); };

    // Get peripheral current use
    float current();

  private:
    const int channel_;
    const int enable_;
    bool enabled_;
  };

  class Motor : public Configurable
  {
  public:
    Motor(int channel, int motorMin = 1000, int motorMax = 2000, int motorCenter = 1500, int motorFDB = 25, int motorRDB = -25);
    virtual ~Motor();

    virtual void arm() = 0;
    virtual bool set(const char *param, const char *value);
    virtual void loop();

    void velocity(float velocity);
    float velocity(){ return velocity_; };

    // Enable ESCs (softswitch)
    void enable(bool enabled);
    bool enabled(){ return enabled_; };

    void enable(){ enable(true); servo_.attach(board::MOTOR[channel_].SERVO);};
    void disable(){ enable(false); servo_.detach();};

  private:
    Servo servo_;
    const int channel_;
    const int enable_;
    bool enabled_;
    float velocity_;
    float desiredVelocity_;

    int motorMax_; 
    int motorMin_; 
    int motorCenter_;
    int motorFDB_; //motor forward deadband (positive val)
    int motorRDB_; //motor reverse deadband (negative val)

  public:
    static void onLoop_(void *data);
  };

  class Sensor : public Configurable
  {
  public:
    Sensor(int id);
    virtual ~Sensor();

    virtual bool set(const char* param, const char* value);
    virtual char *name() = 0;
    //virtual void onSerial();
    virtual void loop();

  protected:
    const int id_;

  public:
    //static void onSerial_(void *data);
    static void onLoop_(void *data);
    virtual void calibrate(int flag){};
  };

  class ExternalSensor : public Sensor
  {
  public:
    ExternalSensor(int id, int port);
    virtual char  *name() = 0;

  protected:
    const int port_;

  };

  class AnalogSensor : public ExternalSensor
  {
  public:
    AnalogSensor(int id, int port);

    bool set(const char* param, const char* value);
    virtual char *name() = 0;

    void scale(float scale);
    float scale(){ return scale_; };

    void offset(float offset);
    float offset(){ return offset_; };

  private:
    float scale_;
    float offset_;
  };

  class PoweredSensor : virtual public ExternalSensor
  {
  public:
    PoweredSensor(int id, int port, bool poweredOn=true);
    virtual char *name() = 0;
    bool powerOn();
    bool powerOff();

  private:
    bool state_;
  };

  class SerialSensor : virtual public ExternalSensor
  {
  public:
    SerialSensor(int id,  int port, int baud, int type = RS232, int dataLength = 0);
    virtual char * name() = 0;
    static void onSerial_(void *data);
    void onSerial();

    enum SERIAL_TYPE{
      RS232,
      RS485,
			TTL
    };

  protected:
    int baudRate_;
    int serialType_;
    int minDataStringLength_;
    char recv_buffer_[DEFAULT_BUFFER_SIZE];
    unsigned int recv_index_;
  };


  extern platypus::Motor *motors[board::NUM_MOTORS];
  extern platypus::Sensor *sensors[board::NUM_SENSORS];
  extern platypus::Peripheral *peripherals[board::NUM_PERIPHERALS];

  // Callbacks structure for serial events
  typedef struct {
    void (*handler)(void *arg);
    void *data;
  } SerialHandler_t;

  // Callbacks for serial events
  extern SerialHandler_t SERIAL_HANDLERS[4];

  // Array of available serial ports
  extern USARTClass *SERIAL_PORTS[4];

  // Helper function to do endian conversion
  uint32_t swap(uint32_t bytes);
}

#endif //PLATYPUS_H
