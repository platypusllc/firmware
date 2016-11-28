#ifndef RC_PWM_H
#define RC_PWM_H

#include "RC.h"

/**
 * RC_PWM: Class used to read and handle commands sent by an attached RC controller
 *                The class reads override, throttle and rudder channels and converts from
 *                PPM into boolean, float and float respectively. These values can be read
 *                using the appropriate accessor methods. The class assumes the following
 *                functionality:
 *                override Pin is low:  Ignore throttle and rudder input
 *                override Pin is high: Read throttle and rudder input. Throttle and rudder
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
class RC_PWM : public RC_Controller {
  public:
  
    RC_PWM();
    RC_PWM(int thrust_pin_, int rudder_pin_, int override_pin_);
    
    float rightVelocity();
    float leftVelocity();
    float rightFan();
    float leftFan();
    void  update(); // override the RC_Controller class update()
    void  configUpdate();

    //Variables for fixed velocity testing
    bool control_state = false;
    float control_velocity = 0.1;

  private:
      //RC Controller Output Values
    int min_throttle = 1000;
    int max_throttle = 1975;
    int mid_throttle = 1471;

    int left_rudder = 1000;
    int right_rudder = 2000;
    
    int override_low = 980*0.95;
    int override_high = 1966*1.05;
    int override_threshold_l = 1500*0.9;
    int override_threshold_h = 1500*1.1;
    
    int arming_low = 980*0.95;
    int arming_high = 1966*1.05;
    int arming_threshold_l = 1500*0.9;
    int arming_threshold_h = 1500*1.1;
    
    int velocityMode = 0;
    
    long controlled_timer;


};



#endif // RC_PWM_H
