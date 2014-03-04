#ifndef COMPONENTS_H
#define COMPONENTS_H

#include "Platypus.h"

namespace platypus 
{
  class VaporPro : public Motor {
  public:
    VaporPro(int channel) : Motor(channel) {}
    void arm();
  };

  class HobbyKingBoat : public Motor {
  public:
    HobbyKingBoat(int channel) : Motor(channel) {}
    void arm();
  };
  
  class AnalogSensor : public Sensor {
  public:
    AnalogSensor(int channel);

    bool set(char* param, char* value);
    char *name();
    
    void scale(float scale);
    float scale();
    
    void offset(float offset);
    float offset();
    
  private:
    float scale_;
    float offset_;
  };
  
  class ES2 : public Sensor {
  public:
    ES2(int channel) : Sensor(channel) {}
    char *name();
  };
  
  class Hdf5 : public Sensor {
  public:
    Hdf5(int channel);
    char *name();
    void onSerial();
  };
  
  class Winch : public Sensor {
  public:
    Winch(int channel);
    char *name();
    void send(uint8_t address, uint8_t command, uint8_t data);
  };
}

#endif //COMPONENTS_H
