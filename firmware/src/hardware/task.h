/*
 * Simple periodic task scheduler for XMega.
 *
 * author: Pras Velagapudi
 */

#ifndef TASK_H
#define TASK_H

#include <avr/io.h>
#include <avr/interrupt.h> 

#if (F_CPU != 32000000UL)
#error Task timer settings do not match CPU frequency!
#endif

// Scale factor converting from milliseconds to timer ticks
#define TASK_SCALE_FACTOR ((F_CPU / 1024) / 1000)

struct TaskConfig
{
  TC0_t *timer;
};

// TODO: NO GLOBAL STUFF!
// Store the current task (only handles one right now)
void (*TASK_Function)(void *);
void *TASK_Args;

// TODO: handle more than one task!
ISR (TCC0_OVF_vect) // TODO: Make me parameterizable!
{ 
  TASK_Function(TASK_Args);
}

template <const TaskConfig &_config>
class Task
{
 public:
  Task(void (*task)(void*), void* args, uint16_t interval_ms) {

    // Store the reference to the callback function
    TASK_Function = task;
    TASK_Args = args;

    // Set up the timer to fire periodically
    _config.timer->PER = interval_ms * TASK_SCALE_FACTOR;
    _config.timer->INTCTRLA = TC_OVFINTLVL_HI_gc; // Enable overflow interrupt
    _config.timer->CTRLB = TC_WGMODE_NORMAL_gc;

    // Start the timer running
    _config.timer->CTRLA = TC_CLKSEL_DIV1024_gc; // 32Mhz / 1024 = 31.25kHz

    // Enable medium level interrupts
    PMIC.CTRL |= PMIC_HILVLEN_bm;
    sei();
  }

  ~Task() {
    // TODO: remove callback function from the timer handler

    // Disable the timer
    _config.timer->CTRLA = 0;
  }
};

#endif
