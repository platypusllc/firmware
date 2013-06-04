/**
 * Airboat Control Firmware - Depth
 *
 * Contains control and update code interfacing with an ATU200S
 * depth sonar.  The data is directly forwarded over serial.
 */
#ifndef DEPTH_SENSOR_H
#define DEPTH_SENSOR_H

#include "sensor.h"
#include "serial.h" 

#include <stdio.h>
#include <string.h>

// Define the char code for the Amarino callback
#define RECV_DEPTH_FN 'd'

#define NMEA_BUFFER_SIZE 40

struct DepthConfig
{
  PORT_t * const pwr_port;
  const uint8_t pwr_pin;
};

template<const DepthConfig &_depthConfig, const SerialConfig &_serialConfig>
class DepthSensor : public Sensor
{
 public:
 DepthSensor(MeetAndroid * const a) 
   : serial(BAUD_4800), stream(serial.stream()), 
    amarino(a), nmeaIndex(0) { 

    // Power up the sensor                                                     
    _depthConfig.pwr_port->OUTSET = _BV(_depthConfig.pwr_pin);
    _depthConfig.pwr_port->DIRSET = _BV(_depthConfig.pwr_pin);

    // Wait until it has powered up                                            
    _delay_ms(1000);
  }

  ~DepthSensor() { 

    // Power down the sensor                                                   
    _depthConfig.pwr_port->OUTCLR = _BV(_depthConfig.pwr_pin);
    _depthConfig.pwr_port->DIRCLR = _BV(_depthConfig.pwr_pin);
  }

  void loop() {
    
    // Get bytes from serial buffer
    while (serial.available()) {
      char c = fgetc(stream);
      nmeaSample[nmeaIndex++] = c;
      
      // Parse if we receive end-of-line characters
      if (c == '\r' || c == '\n' || nmeaIndex >= NMEA_BUFFER_SIZE) {

	// Check if it is a valid reading
	if ((nmeaIndex > 6) && (!strncmp(nmeaSample, "$SDDBT", 6)) && (nmeaChecksum(nmeaSample))) {

	  // Send parsed reading to server
	  amarino->send(RECV_DEPTH_FN);
	  amarino->send(parseNMEA(nmeaSample));
	  amarino->sendln();
	} 

	// Clear out existing buffer
	clearNMEA();
      }
    }
  }

  void update() { }

 private:
  SerialHW<_serialConfig> serial;
  FILE * const stream;
  MeetAndroid * const amarino;

  char nmeaSample[NMEA_BUFFER_SIZE+1];
  uint8_t nmeaIndex;
  
  // Parses the NMEA string to just the depth in meters.
  static char* parseNMEA(char* depth) {

    // Null-terminate the string after the depth reading
    depth[22] = '\0';
    return &depth[16];
  }

  // Calculate nmea checksum and return if it is correct or not
  static bool nmeaChecksum(const char *depth) {
    char checksum = 0;
    char cs[3];

    const char *indx = depth + 1;
    for (int i = 1; i < 33; i++, indx++)
      checksum ^= *indx;
    
    sprintf(cs, "%02X", checksum);
    return (!strncmp(cs, &depth[34], 2));
  }

  void clearNMEA(void)
  {
    memset(nmeaSample, 0, NMEA_BUFFER_SIZE);
    nmeaIndex = 0;
  }
};

#endif /* DEPTH_SENSOR_H */
