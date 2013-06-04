#include "thruster.h"

struct pidConstants_t { float Kp[6], Ki[6], Kd[6]; };

extern float desiredVelocity[];
extern float actualVelocity[];
extern pidConstants_t pid;

Thruster::Thruster(MeetAndroid * const a, Servo * const s)
  : servo(s), amarino(a), prevError(0), bufferSum(0), bufferIdx(0), sendCounter(0)
{
  for (int i = 0; i < 100; i++)
    buffer[i] = 0;
}

Thruster::~Thruster() { }

void Thruster::arm(void)
{
  servo->set(32000);
  _delay_ms(5500);

  servo->set(-32000);
  _delay_ms(3500);

  servo->set(0);
  _delay_ms(8500);
}

void Thruster::update(void)
{
  float error = desiredVelocity[0] - actualVelocity[0];

  bufferIdx++;
  if (bufferIdx >= TBUFSIZE)
    bufferIdx = 0;

  bufferSum -= buffer[bufferIdx];
  bufferSum += error;
  buffer[bufferIdx] = error;

  float output = (pid.Kp[0] * error) + (pid.Kd[0] * ((error - prevError)/(THRUSTER_UPDATE_INTERVAL_MS))) + (pid.Ki[0] * bufferSum);
  prevError = error;

  if (output < TMIN)
    output = TMIN;
  if (output > TMAX)
    output = TMAX;

  servo->set(output);

  sendCounter++;
  if (sendCounter > THRUSTER_UPDATE_COUNT) {
    amarino->send(RECV_THRUSTER_DEG);
    amarino->send(output);
    amarino->sendln();

    sendCounter = 0;
  }
}

