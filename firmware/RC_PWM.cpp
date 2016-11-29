#include "RC_PWM.h"

namespace rc { // definitions for the externs in the header (had to be done to avoid multiple definitions)
  
  volatile int OVERRIDE_PIN;
  volatile int THRUST_PIN;
  volatile int RUDDER_PIN;
  volatile uint32_t thrust_pwm;
  volatile uint32_t rudder_pwm;
  volatile uint32_t override_pwm;
  volatile bool RCoverride;
  

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
  
  //Throttle pin listener
  void throttleInterrupt(  )
  {
    static uint32_t throttleStartTime;
  
    if(digitalRead(rc::THRUST_PIN))
    {
      throttleStartTime = micros();
    }
    else
    {
      rc::thrust_pwm = (uint32_t)(micros() - throttleStartTime);
    }
  }
  
  //Rudder pin listener
  void rudderInterrupt(  )
  {
    static uint32_t rudderStartTime;
  
    if(digitalRead(rc::RUDDER_PIN))
    {
      rudderStartTime = micros();
    }
    else
    {
      rc::rudder_pwm = (uint32_t)(micros() - rudderStartTime);
    }
  }  
}

// RC_PWM::RC_PWM() {}

/*
RC_PWM::RC_PWM(int thrust_pin_, int rudder_pin_, int override_pin_)
{
  //Assign global pins for interrupts
  rc::THRUST_PIN = thrust_pin_;
  rc::RUDDER_PIN = rudder_pin_;
  rc::OVERRIDE_PIN = override_pin_;
  pinMode(thrust_pin_, INPUT); digitalWrite(thrust_pin_, LOW);
  pinMode(rudder_pin_, INPUT);   digitalWrite(rudder_pin_, LOW);
  pinMode(override_pin_, INPUT); digitalWrite(override_pin_, LOW);
  //attachInterrupt(rudder_pin_, rc::rudderInterrupt, CHANGE);
  //attachInterrupt(thrust_pin_, rc::throttleInterrupt, CHANGE);
  //attachInterrupt(override_pin_, rc::overrideInterrupt, CHANGE);
}
*/
/*
//Velocity mixers
//Implemented as simple linear mixers. Turn speed is controlled by
//the position of the throttle stick. Turning angle is linearly proportional
//to the position of the rudder

//Returns the velocity for the right motor
float RC_PWM::rightVelocity()
{
    float l_throttle_val = channel_values[rc::THRUST];
    if(control_state){     l_throttle_val = control_velocity;}
      
     if (velocityMode == 1)
    {
      
    }
    else
    {
        //Turning right -> reduce left motor speed
      if (channel_values[rc::RUDDER] > 0 && l_throttle_val > 0)
      {
        return l_throttle_val;
      }
      else if(l_throttle_val > 0)
      {
        return l_throttle_val*(1+channel_values[rc::RUDDER]);
      }
      else if(channel_values[rc::RUDDER] < 0)
      {
        return l_throttle_val;
      }
      else
      {
        return l_throttle_val*(1-channel_values[rc::RUDDER]);
      }
    }
}

//Returns the velocity for the right motor
float RC_PWM::leftVelocity()
{
    float l_throttle_val = channel_values[rc::THRUST];
    if(control_state){     l_throttle_val = control_velocity;}
  
    if (velocityMode == 1)
    {
      
    }
    else
    {
      //Turning right -> reduce left motor speed
      if (channel_values[rc::RUDDER] > 0 && l_throttle_val > 0)
      {
        return l_throttle_val*(1-channel_values[rc::RUDDER]);
      }
      else if(l_throttle_val > 0)
      {
        return l_throttle_val;
      }
      else if(channel_values[rc::RUDDER] < 0)
      {
        return l_throttle_val*(1+channel_values[rc::RUDDER]);
      }
      else
      {
        return l_throttle_val;
      }
    }
}

float RC_PWM::rightFan()
{
 
   // return 90.0 + rudder_val*30.0;
   return channel_values[rc::RUDDER];
}


float RC_PWM::leftFan()
{
  return channel_values[rc::RUDDER];
   // return 90.0 + rudder_val*30.0;
  
}


//RC Update loop
//Reads channel inputs if available
//Sets Override flag and arming + calibration routine
void RC_PWM::update()
{
     

      //Turn off interrupts to update local variables
      noInterrupts();

      //Update local variables
      channel_values[rc::OVERRIDE] = rc::override_pwm;
      
      //override value is below threshold or outside of valid window
      //set override to false
      if(channel_values[rc::OVERRIDE] < override_threshold_l || channel_values[rc::OVERRIDE] > override_high)
      {
        //If override was previously enabled, then
        //stop listening to throttle and rudder pins
        if(overrideEnabled)
        {
          channel_values[rc::THRUST] = 0;
          channel_values[rc::RUDDER] = 0;
        }

        overrideEnabled = false;
        control_velocity = 0.1;
      }
      //override pin is above threshold
      else if(channel_values[rc::OVERRIDE] > override_threshold_h)
      {
        overrideEnabled = true;
        
        //Zero throttle and rudder values to prevent false readings
        channel_values[rc::THRUST] = 0;
        channel_values[rc::RUDDER] = 0;
      }
      else
      {
              interrupts();
              return;
      }
      

      //Read throttle and rudder values if override pin was high
      if(overrideEnabled )
      {
        channel_values[rc::THRUST] = (float)map(rc::thrust_pwm, min_throttle, max_throttle, -100, 100)/100.0;
        channel_values[rc::RUDDER]   = (float)map(rc::rudder_pwm,left_rudder, right_rudder, -100, 100)/100.0;
        if(abs(channel_values[rc::THRUST]) < 0.01) channel_values[rc::THRUST] = 0;
        if(abs(channel_values[rc::RUDDER]) < 0.01) channel_values[rc::RUDDER] = 0;
      }
      

      //Local update complete - turn on interrupts
       interrupts();


      //RC commands are being received - deal with calibration and arming
      if(overrideEnabled)
      {         
           // Arming and calibration           
       }

}
*/
