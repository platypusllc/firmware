#ifndef PLATYPUS_H
#define PLATYPUS_H

// TODO: Move these to subdirectory or something!
#include "Board.h"
#include <Servo.h>
#include <Scheduler.h>

namespace platypus 
{  
  // Callbacks structure for serial events
  typedef struct {
    void (*handler)(void *arg);
    void *data;
  } SerialHandler_t;
  
  // Callbacks for serial events
  extern SerialHandler_t SERIAL_HANDLERS[4];
  
  // Array of available serial ports
  extern USARTClass *SERIAL_PORTS[4];
  
  class Configurable
  {
  public:     
    virtual bool set(char *param, char *value);
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
  
  class Motor : public Configurable
  { 
  public:
    Motor(int channel);
    virtual ~Motor();

    virtual void arm() = 0;
    virtual bool set(char *param, char *value);
    
    void velocity(float velocity);
    float velocity();
    
    void enable(bool enabled);
    bool enabled();
    
    void enable();
    void disable();
    
    float current();
    
  private:
    Servo servo_;
    int enable_;
    bool enabled_;
    float velocity_;
  };
  
  class Sensor : public Configurable
  {
  public:
    Sensor(int channel);
    virtual ~Sensor();
    
    virtual bool set(char* param, char* value);
    virtual char *name() = 0;
    virtual void onSerial();
    
  private:
    board::Sensor_t sensor_;
    static void onSerial_(void *data);
  };
}

#endif //PLATYPUS_H
