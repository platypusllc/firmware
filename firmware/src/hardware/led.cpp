#include "led.h"

Led::Led() { } 

Led::~Led() { }

void Led::on(void)
{
  set(true);
}

void Led::off(void)
{
  set(false);
}

void Led::toggle(void)
{
  set(!get());
}
