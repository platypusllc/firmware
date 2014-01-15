#ifndef BOARD_H
#define BOARD_H

#include <Arduino.h>

namespace board 
{
  
  // Board type definitions
  typedef struct LED_t 
  { 
    int R;
    int G;
    int B;
  };
  
// Pin definitions
// Left side of Arduino
const LED_t LED = { 54, 55, 56 };

const int V_BATT = A3;

const int S4_ANALOG = A4;
const int S3_ANALOG = A5;
const int S2_ANALOG = A6;
const int S1_ANALOG = A7;

const int S1_CURRENT = A8;
const int S2_CURRENT = A9;
const int S3_CURRENT = A10;
const int S4_CURRENT = A11;

const int S1_PWR = 66;
const int S2_PWR = 67;
const int S3_PWR = 68;
const int S4_PWR = 69;

const int M2_PWR = 48;
const int M1_PWR = 50;

const int CHG_CTRL = 51;

// Right side of Arduino
const int M2_SERVO = 12;
const int M1_SERVO = 11;

const int S4_B = 9;
const int S4_A = 8;
const int S3_B = 7;
const int S3_A = 6;
const int S2_B = 5;
const int S2_A = 4;
const int S1_B = 3;
const int S1_A = 2;

}
#endif //BOARD_H
