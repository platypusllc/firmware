#ifndef PLATYPUS_H
#define PLATYPUS_H

// TODO: Move these to subdirectory or something!
#include "Board.h"

namespace platypus 
{
  class LED 
  { 
  public:
    LED();
    ~LED();
    void set(uint8_t red, uint8_t green, uint8_t blue);
    void R(uint8_t red);
    const uint8_t R();
    void G(uint8_t green);
    const uint8_t G();
    void B(uint8_t blue);
    const uint8_t B();
    
  private:
    uint8_t r_, g_, b_;
  };
}

#endif //PLATYPUS_H
