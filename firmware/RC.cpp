#include "RC.h"

/**
 * RC_Controller default constructor
 *
 */
RC_Controller::RC_Controller()
{
  overrideEnabled = false;
  thrust_scale = 1.0;
  rudder_scale = 1.0;
}

/**
 * Custom Constructor to attach Rx module to given pins
 */
RC_Controller::RC_Controller(int thrust_pin_, int rudder_pin_, int override_pin_)
{
  RC_Controller();
  thrust_pin = thrust_pin_;
  rudder_pin = rudder_pin_;
  override_pin = override_pin_;
}

bool  RC_Controller::isOverrideEnabled() {return overrideEnabled;}

void  RC_Controller::setOverrideEnabled(bool state) { overrideEnabled = state; }

float RC_Controller::getRCChannelValue(int channel) { return channel_values[channel]; }
float RC_Controller::getThrust() { return getRCChannelValue(rc::THRUST); }
float RC_Controller::getRudder() { return getRCChannelValue(rc::RUDDER); }
float RC_Controller::getOverride() { return getRCChannelValue(rc::OVERRIDE); }

void  RC_Controller::update() {/*I'm virtual!*/};

