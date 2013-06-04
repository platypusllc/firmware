/**
 * Airboat Control Firmware - Monitor Instruments Spectrometer
 *
 * Contains control and update code interfacing with a
 * Monitor Instruments mass spectrometer. The data is directly
 * forwarded over serial.
 */
#ifndef MONITOR_SENSOR_H
#define MONITOR_SENSOR_H

#include "sensor.h"
#include "serial.h" 

#include <stdio.h>
#include <string.h>

// Define the char code for the Amarino callback
#define RECV_MONITOR_FN 'm'

#define SERIAL_BUFFER_SIZE 1024

struct MonitorConfig
{
  PORT_t * const pwr_port;
  const uint8_t pwr_pin;
};

template<const MonitorConfig &_monitorConfig, const SerialConfig &_serialConfig>
class MonitorSensor : public Sensor
{
 public:
 MonitorSensor(MeetAndroid * const a) 
   : serial(BAUD_38400), stream(serial.stream()), 
    amarino(a), serialIndex(0) { 

    // Power up the sensor                                                     
    //_monitorConfig.pwr_port->OUTSET = _BV(_monitorConfig.pwr_pin);
    //_monitorConfig.pwr_port->DIRSET = _BV(_monitorConfig.pwr_pin);

    // Wait until it has powered up                                            
    //_delay_ms(1000);
  }

  ~MonitorSensor() { 

    // Power down the sensor                                                   
    //_monitorConfig.pwr_port->OUTCLR = _BV(_monitorConfig.pwr_pin);
    //_monitorConfig.pwr_port->DIRCLR = _BV(_monitorConfig.pwr_pin);
  }

  void loop() {

    // Get bytes from serial buffer
    while (serial.available()) {
      char c = fgetc(stream);
      
      // Parse if we receive end-of-line characters
      if (c == '\r' || c == '\n' || serialIndex >= SERIAL_BUFFER_SIZE) {
        
        // Check if it is a valid reading
        if ((serialIndex > 3) && (!strncmp(serialBuffer, "59,", 3))) {
          
          // Send parsed reading to server
          amarino->send(RECV_MONITOR_FN);
          amarino->send(serialBuffer);
          amarino->sendln();
        } 

        // Clear out existing buffer
        clearSerial();
      } else {
        serialBuffer[serialIndex++] = c;
      }       
    }
  }

  void update() {
    // Do nothing
    //    fputs("59,1,2,3,4,5,6,7,8,9\r\n", stream);
  }

 private:
  SerialHW<_serialConfig> serial;
  FILE * const stream;
  MeetAndroid * const amarino;

  char serialBuffer[SERIAL_BUFFER_SIZE+1];
  uint16_t serialIndex;
  
  void clearSerial(void)
  {
    memset(serialBuffer, 0, SERIAL_BUFFER_SIZE);
    serialIndex = 0;
  }
};

#endif /* MONITOR_SENSOR_H */
