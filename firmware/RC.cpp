#include "RC.h"

//Rx input pins
int THROTTLE_PIN = 3;
int RUDDER_PIN = 2;
int AUX_PIN = 20;
int ARMING_PIN = 21;

//Throttle and Rudder Values as sent by RC
volatile uint32_t throttle_pwm = 0;
volatile uint32_t rudder_pwm = 0;
volatile uint32_t aux_pwm = 0;
volatile uint32_t arming_pwm = 0;

volatile bool RCoverride = false;



//auxiliary pin listener
void auxInterrupt(  )
{
  static uint32_t auxStartTime;
  //Start timer on rising edge
  if(digitalRead(AUX_PIN))
  {
    auxStartTime = micros();
  }
  //Stop timer on falling edge
  else
  {
    //Compute pulse duration
    aux_pwm = (uint32_t)(micros() - auxStartTime);

  }
}

//Throttle pin listener
void throttleInterrupt(  )
{
  static uint32_t throttleStartTime;

  if(digitalRead(THROTTLE_PIN))
  {
    throttleStartTime = micros();
  }
  else
  {
    throttle_pwm = (uint32_t)(micros() - throttleStartTime);
  }

}

//Rudder pin listener
void rudderInterrupt(  )
{
  static uint32_t rudderStartTime;

  if(digitalRead(RUDDER_PIN))
  {
    rudderStartTime = micros();
  }
  else
  {
    rudder_pwm = (uint32_t)(micros() - rudderStartTime);
  }

}

//Rudder pin listener
void armedInterrupt(  )
{
  static uint32_t armedStartTime;

  if(digitalRead(ARMING_PIN))
  {
    armedStartTime = micros();
  }
  else
  {
    arming_pwm = (uint32_t)(micros() - armedStartTime);
  }

}

/**
 * Rc_Controller default constructor
 * Enable aux, throttle and rudder pins to be used for interrupts
 * Attach interrupt to auxiliary pin to listen for manual override
 *
 * Default pins are used as defined in header file unless custom
 * constructor was called
 */
RC_Controller::RC_Controller()
{
  Serial.println("Constructor Started");
  pinMode(THROTTLE_PIN, INPUT); digitalWrite(THROTTLE_PIN, LOW);
  pinMode(RUDDER_PIN, INPUT);   digitalWrite(RUDDER_PIN, LOW);
  pinMode(AUX_PIN, INPUT);    digitalWrite(AUX_PIN, LOW);
  pinMode(ARMING_PIN, INPUT); digitalWrite(ARMING_PIN, LOW);
  attachInterrupt(AUX_PIN, auxInterrupt, CHANGE);
  attachInterrupt(RUDDER_PIN, rudderInterrupt, CHANGE);
  attachInterrupt(THROTTLE_PIN, throttleInterrupt, CHANGE);
  attachInterrupt(ARMING_PIN, armedInterrupt, CHANGE);
  Serial.println("Constructor Completed");

}

/**
 * Custom Constructor to attach Rx module to given pins
 * @param aux_pin: Input pin for auxiliary channel
 * @param throttle_pin: Input pin for throttle channel
 * @param rudder_pin: Input pin for rudder channel
 */
RC_Controller::RC_Controller(int aux_pin, int throttle_pin, int rudder_pin, int arming_pin)
{
  //Assign pins
  AUX_PIN = aux_pin;
  THROTTLE_PIN = throttle_pin;
  RUDDER_PIN = rudder_pin;
  ARMING_PIN = arming_pin;
  //Call default constructor to initialise pins
  RC_Controller();
}


//Accessor Methods
bool  RC_Controller::isOverrideEnabled() {return overrideEnabled;}
bool  RC_Controller::isArmed(){ return armed;}
float RC_Controller::throttleVal() { return throttle_val; }
float RC_Controller::rudderVal() { return rudder_val; }
float RC_Controller::auxVal() { return aux_val; }
float RC_Controller::armingVal() { return arming_val; }


bool RC_Controller::isMotorUpdateBlocked() { return motorUpdateBlocked; }

void RC_Controller::setMotorUpdateBlocked(bool state) { motorUpdateBlocked = state; }



//Velocity mixers
//Implemented as simple linear mixers. Turn speed is controlled by
//the position of the throttle stick. Turning angle is linearly proportional
//to the position of the rudder

//Returns the velocity for the right motor
float RC_Controller::rightVelocity()
{
    float l_throttle_val = throttle_val;
    if(control_state){     l_throttle_val = control_velocity;}
      
     if (velocityMode == 1)
    {
      
    }
    else
    {
        //Turning right -> reduce left motor speed
      if (rudder_val > 0 && l_throttle_val > 0)
      {
        return l_throttle_val;
      }
      else if(l_throttle_val > 0)
      {
        return l_throttle_val*(1+rudder_val);
      }
      else if(rudder_val < 0)
      {
        return l_throttle_val;
      }
      else
      {
        return l_throttle_val*(1-rudder_val);
      }
    }
}

//Returns the velocity for the right motor
float RC_Controller::leftVelocity()
{
    float l_throttle_val = throttle_val;
    if(control_state){     l_throttle_val = control_velocity;}
  
    if (velocityMode == 1)
    {
      
    }
    else
    {
      //Turning right -> reduce left motor speed
      if (rudder_val > 0 && l_throttle_val > 0)
      {
        return l_throttle_val*(1-rudder_val);
      }
      else if(l_throttle_val > 0)
      {
        return l_throttle_val;
      }
      else if(rudder_val < 0)
      {
        return l_throttle_val*(1+rudder_val);
      }
      else
      {
        return l_throttle_val;
      }
    }
}

float RC_Controller::rightFan()
{
 
   // return 90.0 + rudder_val*30.0;
   return rudder_val;
}


float RC_Controller::leftFan()
{
  return rudder_val;
   // return 90.0 + rudder_val*30.0;
  
}


//RC Update loop
//Reads channel inputs if available
//Sets Override flag and arming + calibration routine
void RC_Controller::update()
{
     

      //Turn off interrupts to update local variables
      noInterrupts();

      //Update local variables
      aux_val = aux_pwm;
      
      //AUX value is below threshold or outside of valid window
      //set override to false
      if(aux_val < aux_threshold_l || aux_val > aux_high)
      {
        //If override was previously enabled, then
        //stop listening to throttle and rudder pins
        if(overrideEnabled)
        {
          //detachInterrupt(THROTTLE_PIN);
          //detachInterrupt(RUDDER_PIN);
          //detachInterrupt(arming_PIN);
          arming_val = arming_low;
          throttle_val = 0;
          rudder_val = 0;

        }

        overrideEnabled = false;
        control_velocity = 0.1;
       // armed = false;

      }
      //AUX pin is above threshold
      else if(aux_val > aux_threshold_h)
      {
        //Attach throttle and rudder listeners
        //if override was previously off
        if(!overrideEnabled)
        {
         //attachInterrupt(RUDDER_PIN, rudderInterrupt, CHANGE);
         //attachInterrupt(THROTTLE_PIN, throttleInterrupt, CHANGE);
         //attachInterrupt(arming_PIN, armedInterrupt, CHANGE);
        }
        overrideEnabled = true;
        
        //Zero throttle and rudder values to prevent false readings
        throttle_val = 0;
        rudder_val = 0;
        arming_val = arming_low;
        

      }
      else
      {
              interrupts();
              return;
      }
      

      //Read throttle and rudder values if auxiliary pin was high
      if(overrideEnabled )
      {
        arming_val = arming_pwm;
        throttle_val = (float)map(throttle_pwm, min_throttle, max_throttle, -100, 100)/100.0;
        rudder_val   = (float)map(rudder_pwm,left_rudder, right_rudder, -100, 100)/100.0;
        if(abs(throttle_val) < 0.01) throttle_val = 0;
        if(abs(rudder_val) < 0.01) rudder_val = 0;
      }
      

      //Local update complete - turn on interrupts
       interrupts();


      //RC commands are being received - deal with calibration and arming
      if(overrideEnabled)
      {
          //Arming switch is active - enable escs
          if(arming_val > arming_threshold_h && arming_val < arming_high)
          {
              //control_state = true;
              //Record start time for controlled state
              controlled_timer = millis();
              armed = true;
          }
          else if(arming_val < arming_threshold_l || arming_val > arming_high )
          {
              armed = false;
          }
          else if(!armed)
          {
            throttle_val = 0;
            rudder_val = 0;
          }

       }

}
