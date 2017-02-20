#include "RC_PWM.h"

namespace rc { // definitions for the externs in the header (had to be done to avoid multiple definitions)
  volatile int OVERRIDE_PIN;
  volatile int THRUST_SCALE_PIN;
  volatile int THRUST_FRACTION_PIN;
  volatile int HEADING_FRACTION_PIN;
  
  volatile uint32_t override_scale_pwm;
  volatile uint32_t thrust_scale_pwm;
  volatile uint32_t thrust_fraction_pwm;
  volatile uint32_t heading_fraction_pwm;  
  
  //override pin listener
  void overrideInterrupt(  )
  {
    static uint32_t overrideStartTime;
    //Start timer on rising edge
    if(digitalRead(rc::OVERRIDE_PIN))
    {
      overrideStartTime = micros();
    }
    //Stop timer on falling edge
    else
    {
      //Compute pulse duration
      rc::override_pwm = (uint32_t)(micros() - overrideStartTime);
    }
  }

  //Thrust scale pin listener
  void thrustScaleInterrupt(  )
  {
    static uint32_t thrustStartTime;
  
    if(digitalRead(rc::THRUST_SCALE_PIN))
    {
      thrustStartTime = micros();
    }
    else
    {
      rc::thrust_scale_pwm = (uint32_t)(micros() - thrustStartTime);
    }
  }  
  
  //Thrust fraction pin listener
  void thrustFractionInterrupt(  )
  {
    static uint32_t thrustStartTime;
  
    if(digitalRead(rc::THRUST_FRACTION_PIN))
    {
      thrustStartTime = micros();
    }
    else
    {
      rc::thrust_fraction_pwm = (uint32_t)(micros() - thrustStartTime);
    }
  }
  
  //Heading fraction pin listener
  void headingFractionInterrupt(  )
  {
    static uint32_t headingStartTime;
  
    if(digitalRead(rc::HEADING_FRACTION_PIN))
    {
      headingStartTime = micros();
    }
    else
    {
      rc::heading_fraction_pwm = (uint32_t)(micros() - headingStartTime);
    }
  }  
}
