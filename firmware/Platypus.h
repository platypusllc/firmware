#ifndef PLATYPUS_H
#define PLATYPUS_H

// TODO: Move these to subdirectory or something!
#include "Board.h"
#include <Servo.h>

namespace platypus 
{
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
  
  class Motor 
  { 
  public:
    Motor(int channel);
    virtual ~Motor();

    virtual void arm() = 0;
    
    void velocity(float velocity);
    float velocity();
    
    void enable(bool isOn);
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
  
  class Sensor
  {
  public:
    Sensor(int channel);
    virtual ~Sensor();
    
    virtual void set(char* param, char* value) = 0;
    
  private:
    board::Sensor_t sensor_;
  };
  
  class VaporPro : public Motor {
  public:
    VaporPro(int channel) : Motor(channel) {}
    void arm();
  };
  
  class AnalogSensor : public Sensor {
  public:
    AnalogSensor(int channel) : Sensor(channel) {}
    void set(char* param, char* value);
  };
  
  class ES2 : public Sensor {
  public:
    ES2(int channel) : Sensor(channel) {}
    void set(char* param, char* value);
  };
}

#endif //PLATYPUS_H
