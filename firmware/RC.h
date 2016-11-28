#ifndef RC_H
#define RC_H

#include <Arduino.h>
#include "Board.h"

namespace rc {
  enum RC_CHANNEL // note the relation to the board::GPIO enum!!!
  {
    THRUST = 0,
    RUDDER = 1,
    OVERRIDE = 2,
    THRUST_SCALE = 3,
    RUDDER_SCALE = 4
  };
}


/**
 * RC: Class used to read and handle commands sent by an attached RC controller
 *                This is an interface class. More specific RC child classes are
 *                used elsewhere in the code.
 */
class RC_Controller {
  public:

    RC_Controller();
    RC_Controller(int thrust_pin_, int rudder_pin_, int override_pin_);

    bool  isOverrideEnabled();
    void  setOverrideEnabled(bool state);
    
    float getRCChannelValue(int channel);
    float getThrust();
    float getRudder();
    float getOverride();
    virtual void  update(); // instead of using Sensor::loop we will call this in its own parallel thread

  protected:
    bool  overrideEnabled;
    float channel_values[16];
    float thrust_scale;
    float rudder_scale;
    int thrust_pin;
    int rudder_pin;
    int override_pin;
};



#endif // RC_H
