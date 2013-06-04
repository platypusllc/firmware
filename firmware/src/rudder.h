/**
 * Airboat Control Firmware - Rudder
 *
 * Contains control and update code interfacing with a servo-driven
 * mechanical rudder.  This code runs a PID loop attempting to match
 * a desired yaw velocity using feedback from a gyro.
 */

#ifndef RUDDER_H
#define RUDDER_H

#include "servo.h"
#include "meet_android.h"

#define RBUFSIZE  100
#define RMIN   -32000
#define RMAX    32000

#define RECV_RUDDER_POS 'r'
#define RUDDER_UPDATE_INTERVAL_MS (100)
#define RUDDER_UPDATE_COUNT (10)

class Rudder
{
 public:
  Rudder(MeetAndroid * const a, Servo * const s);
  ~Rudder();
  
  void update(void);

 private:
  Servo * const servo;
  MeetAndroid * const amarino;

  int pos;
  float prevError;
  float bufferSum;
  float buffer[RBUFSIZE];
  int bufferIdx;
  
  int sendCounter;
};

#endif /* RUDDER_H */

