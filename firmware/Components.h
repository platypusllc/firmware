#ifndef COMPONENTS_H
#define COMPONENTS_H

#include "Platypus.h"

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
    Winch(int channel);
    char *name();

    bool read(uint8_t address, uint8_t command, uint8_t *response, unsigned int response_len);

    void send(uint8_t address, uint8_t command, uint8_t data);
    void send(uint8_t address, uint8_t command, uint8_t *data, unsigned int data_len);
    
    typedef struct {
      uint32_t accel;
      uint32_t speed;
      uint32_t decel;
      int32_t position;
      unsigned char is_buffered;
    } PositionCommand;
    
    typedef struct {
      uint32_t P;
      uint32_t I;
      uint32_t D;
      uint32_t MaxI;
      uint32_t Deadzone;
      uint32_t MinPos;
      uint32_t MaxPos;
    } PidCommand;
    
    typedef struct {
      uint32_t ticks;
      uint8_t status;
    } QuadratureResponse;
  };
}

#endif //COMPONENTS_H
