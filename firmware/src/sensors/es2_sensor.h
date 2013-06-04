/**
 * ES2 Sensor - EC, Water, Temperature
 *
 * Contains control and update code interfacing with an ES2 sensor
 *
 */
#ifndef ES2_SENSOR_H
#define ES2_SENSOR_H

#include "sensor.h"
#include "meet_android.h"
#include <stdio.h>
#include <util/delay.h>

// Define the char code for the Amarino callback
#define RECV_ES_FN 'e'

#define ES_BUFFER_SIZE (32)

// Take measurements every n-th time update is called
#define ES_SENSOR_INTERVAL (10)

// Define the pins used to interface with the sensor

struct ES2Config {
  PORT_t * const pwr_port;
  const uint8_t pwr_pin;
};

template<const ES2Config &_esConfig, const SerialConfig &_serialConfig>
class ES2Sensor : public Sensor {
private:
  SerialHW<_serialConfig> serial;
  FILE *stream;
  MeetAndroid *amarino;

  char esReading[ES_BUFFER_SIZE + 1];
  uint8_t esIndex;
  uint8_t crc;
  int esCount;

  volatile bool isReading;
  bool wasInitialized;
  bool messageIsDone;

public:

  ES2Sensor(MeetAndroid * const a)
  : serial(BAUD_1200), stream(serial.stream()), amarino(a), esCount(0) {

    // Turn off sensor
    _serialConfig.port->DIRCLR = _BV(_serialConfig.rxPin);
    _esConfig.pwr_port->OUTCLR = _BV(_esConfig.pwr_pin);
    _esConfig.pwr_port->DIRSET = _BV(_esConfig.pwr_pin);
  }

  // Converts from raw ES2 sensor value to a floating point in Celsius.

  float toTemp(const int &rawTemp) {
    return (float) rawTemp; // (rawTemp - 400) / 10.0;
  }

  // Converts from raw ES2 sensor value to a floating point mS/cm (dS/m).

  float toConductivity(const int &rawCond) {
    return ((float) rawCond); // / 100.0;
  }

  // Converts from raw ES2 sensor value to dieletric spec in datasheet.

  float toDielectric(const int &rawDielectric) {
    return ((float) rawDielectric) / 50.0;
  }

  void loop() {
    // Do nothing if we aren't reading sensor
    if (!isReading) {
      return;
    }


    // Did the initialization sequence start yet?
    if (!wasInitialized) {
      wasInitialized = !(_serialConfig.port->IN & _BV(_serialConfig.rxPin));
      while (serial.available()) {
        fgetc(stream);
      }

      return;
    }

    // Did the initialization sequence end yet?
    if (_serialConfig.port->IN & _BV(_serialConfig.rxPin)) {
      return;
    }

    // If all this is satisfied, read the serial port
    while (serial.available()) {

      // Read in next available character
      char c = fgetc(stream);
      esReading[esIndex++] = c;      

      // If this indicates sensor type, the next byte is checksum
      // So if we see a sensor type, activate a flag saying the next
      // byte is the last one.

      // Empirical evidence suggests 13 ... 
      if (!messageIsDone) {
        messageIsDone = (c == 'z' || c == 'q' || esIndex == 13 || esIndex >= ES_BUFFER_SIZE);        
        crc += c;

        if (messageIsDone) {
          // uint16_t dielectric = 0;
          uint16_t conductivity = 0;
          uint16_t temperature = 0;
          char type = '\0';

          // Turn off sensor (we are done reading)
          _esConfig.pwr_port->OUTCLR = _BV(_esConfig.pwr_pin);
          isReading = false;

          // TODO: Normally we would verify sensor checksum here

          // Null-terminate the string 
          esReading[esIndex] = '\0';

          // Look for ES2 reading
          //sscanf(esReading, "%d %d %d %c",
          //        &dielectric, &conductivity, &temperature, &type);

          sscanf(esReading, "%d %d %c",
                  &conductivity, &temperature, &type);
                  
          // Convert and output the returned values
          amarino->send(RECV_ES_FN);
          // amarino->send(toDielectric(dielectric));
          amarino->send(toConductivity(conductivity));
          amarino->send(toTemp(temperature));          
          amarino->sendln();

          // Clear the buffer
          esIndex = 0;
        }
      }
    }
  }

  // Powers up and reads the sensor values from a ES2 environmental sensor, 
  // and checks the resultiFng checksum for validity.  If data is invalid,
  // all values will return zero.

  void update() {
    // Only take measurements every ES_SENSOR_INTERVAL times
    if (esCount < ES_SENSOR_INTERVAL) {
      esCount++;
      return;
    } else {
      esCount = 0;
    }

    // Turn off sensor (if it was on this whole time)
    if (_esConfig.pwr_port->OUT & _BV(_esConfig.pwr_pin)) {
      _esConfig.pwr_port->OUTCLR = _BV(_esConfig.pwr_pin);
      _delay_ms(20); // TODO: how long do we wait?
    }

    // Turn on sensor
    _esConfig.pwr_port->OUTSET = _BV(_esConfig.pwr_pin);

    // Wait for power-up sequence (15ms high)  
    _delay_ms(1);

    // Begin reading in sensor data
    crc = 0;
    esIndex = 0;
    wasInitialized = false;
    messageIsDone = false;
    isReading = true;
  }
};

#endif /* ES2_SENSOR_H */
