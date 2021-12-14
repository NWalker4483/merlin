#include <AccelStepper.h>


#define NUM_STEPPERS 6

#define M1_DIR_PIN 11
#define M1_PULSE_PIN 12

#define M2_DIR_PIN 9
#define M2_PULSE_PIN 10

#define M3_DIR_PIN 7
#define M3_PULSE_PIN 8

#define M4_DIR_PIN 5
#define M4_PULSE_PIN 6

#define M5_DIR_PIN 3
#define M5_PULSE_PIN 4

#define M6_DIR_PIN 14
#define M6_PULSE_PIN 2

// Define a stepper and the pins it will use
AccelStepper stepper1(AccelStepper::DRIVER, M1_PULSE_PIN,    M1_DIR_PIN); 
AccelStepper stepper2(AccelStepper::DRIVER, M2_PULSE_PIN,    M2_DIR_PIN);
AccelStepper stepper3(AccelStepper::DRIVER, M3_PULSE_PIN,    M3_DIR_PIN); 
AccelStepper stepper4(AccelStepper::DRIVER, M4_PULSE_PIN,    M4_DIR_PIN); 
AccelStepper stepper5(AccelStepper::DRIVER, M5_PULSE_PIN,    M5_DIR_PIN); 
AccelStepper stepper6(AccelStepper::DRIVER, M6_PULSE_PIN,    M6_DIR_PIN);

AccelStepper *stepper[6];

int a = 250;
int b = 250;

int cmd_buffer[6][2][6];
int cmd_idx = -1;

bool debug = 1;
int cmd_len = 0;
bool restart = false;
union open_float
{
  char bytes[4];
  float value = 0;
};

open_float index_cnt[6];
open_float pulse_cnt[6];
