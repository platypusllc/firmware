#include "rudder.h"

struct pidConstants_t { float Kp[6], Ki[6], Kd[6]; };

extern float desiredVelocity[];
extern float actualVelocity[];
extern pidConstants_t pid;

Rudder::Rudder(MeetAndroid * const a, Servo * const s) 
  : servo(s), amarino(a), prevError(0), bufferSum(0), bufferIdx(0), sendCounter(5)
{
  for (int i = 0; i < 100; i++)
    buffer[i] = 0;
}

Rudder::~Rudder() { }

void Rudder::update(void)
{
  float error = desiredVelocity[5] - actualVelocity[5];
  
  bufferIdx++;
  if (bufferIdx == RBUFSIZE)
    bufferIdx = 0;
  
  bufferSum -= buffer[bufferIdx];
  bufferSum += error;
  buffer[bufferIdx] = error;
  
  float output = (pid.Kp[5] * error) + (pid.Kd[5] * ((error - prevError) / (RUDDER_UPDATE_INTERVAL_MS))) + (pid.Ki[5] * bufferSum);
  prevError = error;
  
  if (output < RMIN)
    output = RMIN;
  if (output > RMAX)
    output = RMAX;
  servo->set(output);
  
  sendCounter++;
  if (sendCounter > RUDDER_UPDATE_COUNT) {
    amarino->send(RECV_RUDDER_POS);
    amarino->send(output);
    amarino->sendln();
    
    sendCounter = 0;
  }
}
