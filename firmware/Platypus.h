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
    Motor(int channel);
    virtual ~Motor();

    virtual void arm() = 0;
    virtual bool set(const char *param, const char *value);
    virtual void loop();
    
    void velocity(float velocity);
    float velocity(){ return velocity_; };

    // Enable ESCs (softswitch)
    void enable(bool enabled);
    bool enabled(){ return enabled_; };
    
    void enable(){ enable(true); };
    void disable(){ enable(false); };
    
  private:
    Servo servo_;
    const int channel_;
    const int enable_;
    bool enabled_;
    float velocity_;
    float desiredVelocity_;
    
  public:
    static void onLoop_(void *data);
  };
  
  class Sensor : public Configurable
  {
  public:
    Sensor(int channel);
    virtual ~Sensor();
    
    virtual bool set(const char* param, const char* value);
    virtual char *name() = 0;
    virtual void onSerial();
    virtual void loop();

  protected:
    // TODO: Change from channel to struct reference?
    const int channel_;
    
  public:
    static void onSerial_(void *data);
    static void onLoop_(void *data);
    virtual void calibrate(int flag){};
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
