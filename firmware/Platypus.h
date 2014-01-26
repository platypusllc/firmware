#ifndef PLATYPUS_H
#define PLATYPUS_H

// TODO: Move these to subdirectory or something!
#include "Board.h"
#include <Servo.h>

namespace platypus 
{
  class LED 
  { 
  public:
    LED();
    ~LED();
    void set(int red, int green, int blue);
    void R(int red);
    int R();
    void G(int green);
    int G();
    void B(int blue);
    int B();
    
  private:
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
  
  class VaporPro : public Motor {
  public:
    VaporPro(int channel) : Motor(channel) {}
    void arm();
  };
}

#endif //PLATYPUS_H
