/**
 * Airboat Control Firmware - Sampler
 *
 * Contains control and update code interfacing with a servo-driven
 * water-sampler.  This code runs a simple state machine allowing the
 * sampler to cycle between an array of predefined configurations.
 */

#include <Servo.h> 

#define NUM_SAMPLER_POS 5

Servo sampler;

int samplerPos[NUM_SAMPLER_POS] = {1450, 1175, 1800};
int samplerPosIdx = 0;

void initSampler()
{
  sampler.attach(39);
}

void updateSampler()
{
  sampler.write(samplerPos[samplerPosIdx]);
}

/**
 * Receives a water sampling command from Amarino.
 */
void setSampler(byte flag, byte numOfValues)
{
  // Ignore if wrong number of arguments
  if (numOfValues != 1) return;

  // Increment the sampler to the next position 
  if (samplerPosIdx < NUM_SAMPLER_POS - 1) ++samplerPosIdx;
}
