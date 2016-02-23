#ifndef BOARD_H
#define BOARD_H

#include <Arduino.h>

namespace board 
{
  
  // Board type definitions
  struct LED_t
  { 
    int R;
    int G;
    int B;
  };
  
  struct Motor_t
  { 
    int ENABLE;
    int SERVO;
    int SERVO_CTRL;
    int CURRENT;
  };
  
  struct Sensor_t
  {
    int RX_DISABLE;
    int TX_ENABLE;
    int RS485_TE;
    int RS485_232;
    int GPIO[4];
    int PWR_ENABLE;
    int PWR_CURRENT;
    int ANALOG;
  };

  enum GPIO
  {
    RX_POS = 0,
    RX_NEG = 1,
    TX_POS = 2,
    TX_NEG = 3
  };
  
  // Pin definitions
  const LED_t LED = { 54, 55, 56 };

  const size_t NUM_MOTORS = 2;

  const Motor_t MOTOR[NUM_MOTORS] = { 
    { 50, 11, 43, A8 }, // Motor 0 
    { 48, 12, 45, A9 }  // Motor 1
  };

  const size_t NUM_SENSORS = 4;

  const Sensor_t SENSOR[NUM_SENSORS] = {
    { 32, 30, 36, 34, { 21, 2, 20, 3 }, 66,  A8, A7 }, // Sensor 0
    { 33, 31, 37, 35, { 19, 4, 18, 5 }, 67,  A9, A6 }, // Sensor 1
    { 24, 22, 28, 26, { 17, 6, 16, 7 }, 68, A10, A5 }, // Sensor 2
    { 25, 23, 29, 27, { 15, 8, 14, 9 }, 69, A11, A4 }  // Sensor 3
  };

  const int V_BATT = A3;
  const int CHG_CTRL = 51;
  
  const int HALF_DUPLEX01 = 38;
  const int HALF_DUPLEX23 = 39;
  const int LOOPBACK = 40;
  
  const int PWR_INT = 44;
  const int PWR_KILL = 46;
}
#endif //BOARD_H
