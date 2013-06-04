#ifndef DO_SENSOR_H
#define DO_SENSOR_H

#include "sensor.h"
#include "serial.h"
#include <string.h>

#define RECV_DO_FN 'o'
#define DO_BUFFER_SIZE 20

template<const SerialConfig &_config>
class DOSensor : public Sensor {
public:

  DOSensor(MeetAndroid *a)
  : serial(BAUD_38400), stream(serial.stream()),
  amarino(a), doIndex(0) {

    // The command "C" will tell the stamp to take continuous readings
    fputs("\rC\r", stream);
  }

  ~DOSensor() {
  }

  void loop() {
    while (serial.available()) {
      char c = fgetc(stream); // Get the char we just received
      doReading[doIndex++] = c; // Add it to the inputString

      // if Atlas Scientific reading has been received in its entirety      
      if (c == '\r' || doIndex >= DO_BUFFER_SIZE) {

        // Null-terminate current reading 
        doReading[doIndex] = '\0';
        
        float v = atof(doReading);
        
        // Send to server
        amarino->send(RECV_DO_FN);        
        amarino->send(v);
        amarino->sendln();

        // Move to beginning of buffer
        doIndex = 0;
      }
    }
  }

  void update() {
  }

private:
  SerialHW<_config> serial;
  FILE *stream;
  MeetAndroid *amarino;

  char doReading[DO_BUFFER_SIZE + 1];
  uint8_t doIndex;
};

#endif /* DO_SENSOR_H */
