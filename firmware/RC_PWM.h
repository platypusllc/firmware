#ifndef RC_PWM_H
#define RC_PWM_H

#include <Arduino.h>

namespace rc { // extern declarations (had to be done to avoid multiple definitions)
  
  extern volatile int OVERRIDE_PIN;
  extern volatile int THRUST_SCALE_PIN;
  extern volatile int THRUST_FRACTION_PIN;
  extern volatile int HEADING_FRACTION_PIN;
  
  extern volatile uint32_t override_pwm;
  extern volatile uint32_t thrust_scale_pwm;
  extern volatile uint32_t thrust_fraction_pwm;
  extern volatile uint32_t heading_fraction_pwm;
  
  extern void overrideInterrupt();  
  extern void thrustScaleInterrupt();  
  extern void thrustFractionInterrupt();  
  extern void headingFractionInterrupt();  
}

#endif // RC_PWM_H
