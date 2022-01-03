
#include "speed_stepper.h"
#define BAUD_RATE 500000
#define SPR 200
#define PPR 1000

union open_float
{
  char bytes[4];
  float value = 0;
};

open_float index_cnt[6];
open_float pulse_cnt[6];

#define M1_DIR_PIN 10
#define M1_PULSE_PIN 11

#define M2_DIR_PIN 12
#define M2_PULSE_PIN 24

#define M3_DIR_PIN 25
#define M3_PULSE_PIN 26

#define M4_DIR_PIN 27
#define M4_PULSE_PIN 28

#define M5_DIR_PIN 29
#define M5_PULSE_PIN 30

#define M6_DIR_PIN 31
#define M6_PULSE_PIN 32


// Define a stepper and the pins it will use
SpeedStepper stepper1(M1_PULSE_PIN, M1_DIR_PIN);
SpeedStepper stepper2(M2_PULSE_PIN, M2_DIR_PIN);
SpeedStepper stepper3(M3_PULSE_PIN, M3_DIR_PIN);
SpeedStepper stepper4(M4_PULSE_PIN, M4_DIR_PIN);
SpeedStepper stepper5(M5_PULSE_PIN, M5_DIR_PIN);
SpeedStepper stepper6(M6_PULSE_PIN, M6_DIR_PIN);
