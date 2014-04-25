#ifndef COMPONENTS_H
#define COMPONENTS_H

#include "Platypus.h"
#include "RoboClaw.h"

namespace platypus 
{
  const int DEFAULT_BUFFER_SIZE = 128;
  
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
  
  class Seaking : public Motor {
  public:
    Seaking(int channel) : Motor(channel) {}
    void arm();
  };

  class Swordfish : public Motor {
  public:
    Swordfish(int channel) : Motor(channel) {}
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
    
  private:
    char recv_buffer_[DEFAULT_BUFFER_SIZE];
    unsigned int recv_index_;
  };
  
  class Winch : public Sensor {
  public:
    Winch(int channel, uint8_t address);
    char *name();
    bool set(char* param, char* value);
    
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
