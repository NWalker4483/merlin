#include <BasicLinearAlgebra.h>
#include <ElementStorage.h>
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

#define BIG_NUMBER 10000000
#define FULL_ROTATION 360
#define BAUD_RATE 500000
#define SPR 200
#define PPR 1000
#define MAX_CMD_LENGTH 6

using namespace BLA;

Matrix<6, 6> motor_reductions = {
  1. / 48., 0, 0,         0, 0, 0,
  0, 1. / 48., 0,         0, 0, 0,
  0, -1. / 48., 1. / 48., 0, 0, 0,
  0, 0, 0,                 1. / 24., 0, 0,
  0, 0, 0,                -1. / 28.8, 1. / 28.8, 0,
  0, 0, 0,                -1. / 12., 1. / 24., 1. / 24.
};

Matrix<6, 6> degrees_per_step;
Matrix<6, 6> degrees_per_step_inv;

Matrix<6, 1> target_speeds;
Matrix<6, 1>  target_poses;

Matrix<6, 1>  current_poses;

// Define a stepper and the pins it will use
AccelStepper stepper1(AccelStepper::DRIVER, M1_PULSE_PIN,    M1_DIR_PIN); 
AccelStepper stepper2(AccelStepper::DRIVER, M2_PULSE_PIN,    M2_DIR_PIN);
AccelStepper stepper3(AccelStepper::DRIVER, M3_PULSE_PIN,    M3_DIR_PIN); 
AccelStepper stepper4(AccelStepper::DRIVER, M4_PULSE_PIN,    M4_DIR_PIN); 
AccelStepper stepper5(AccelStepper::DRIVER, M5_PULSE_PIN,    M5_DIR_PIN); 
AccelStepper stepper6(AccelStepper::DRIVER, M6_PULSE_PIN,    M6_DIR_PIN);

AccelStepper *stepper[NUM_STEPPERS];

int cmd_buffer[MAX_CMD_LENGTH][2][NUM_STEPPERS];
int cmd_idx = -1;

int cmd_len = 0;
bool interrupt = false;

union open_float
{
  char bytes[4];
  float value = 0;
};

open_float index_cnt[NUM_STEPPERS];
open_float pulse_cnt[NUM_STEPPERS];
