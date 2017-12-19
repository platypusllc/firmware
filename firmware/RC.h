#ifndef RC_H
#define RC_H

#include <Arduino.h>

/**
 * RC_Controller: Class used to read and handle commands sent by an attached RC controller
 *                The class reads auxiliary, throttle and rudder channels and converts from
 *                PPM into boolean, float and float respectively. These values can be read
 *                using the appropriate accessor methods. The class assumes the following
 *                functionality:
 *                AUX Pin is low:  Ignore throttle and rudder input
 *                AUX Pin is high: Read throttle and rudder input. Throttle and rudder
 *                                 output have no effect until the controller is armed using
 *                                 the command: throttle down, rudder left. After 5 seconds
 *                                 the same sequence will disarm the controller.
 *                                 A throttle down and rudder right command will force the
 *                                 esc into calibration mode. This process will block for
 *                                 10 seconds.
 *
 *                                 Once armed the throttle and rudder commands will be mixed
 *                                 to give a velocity for each motor
 */
class RC_Controller{
  public:
  
    RC_Controller();
    RC_Controller(int aux_pin, int throttle_pin, int rudder_pin, int arming_pin );

    bool  isOverrideEnabled();
    bool  isArmed();
    bool  isMotorUpdateBlocked();
    void  setMotorUpdateBlocked(bool);
    
    float throttleVal();
    float rudderVal();
    float auxVal();
    float armingVal();
    float rightVelocity();
    float leftVelocity();
    float rightFan();
    float leftFan();
    void  update();
    void  configUpdate();

    //Variables for fixed velocity testing
    bool control_state = false;
    float control_velocity = 0.1;

  private:
      //RC Controller Ouptut Values
    int min_throttle = 1000;
    int max_throttle = 1975;
    int mid_throttle = 1471;

    int left_rudder = 1000;
    int right_rudder = 2000;
    
    int aux_low = 980*0.95;
    int aux_high = 1966*1.05;
    int aux_threshold_l = 1500*0.9;
    int aux_threshold_h = 1500*1.1;
    
    int arming_low = 980*0.95;
    int arming_high = 1966*1.05;
    int arming_threshold_l = 1500*0.9;
    int arming_threshold_h = 1500*1.1;
    
    bool  overrideEnabled = false;
    bool  armed = true;
    bool  motorUpdateBlocked = false;
    int velocityMode = 0;
    
    float throttle_val = 0;
    float rudder_val = 0;
    int   aux_val;
    int arming_val;
    long controlled_timer;


};



#endif
