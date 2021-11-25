#include <AccelStepper.h>

#define WAIST_DIR_PIN 11
#define WAIST_PULSE_PIN 12

#define SHOULDER_DIR_PIN 9
#define SHOULDER_PULSE_PIN 10

#define ELBOW_DIR_PIN 7
#define ELBOW_PULSE_PIN 8

#define AXIS_4_DIR_PIN 5
#define AXIS_4_PULSE_PIN 6

#define AXIS_5_DIR_PIN 3
#define AXIS_5_PULSE_PIN 4

#define AXIS_6_DIR_PIN 14
#define AXIS_6_PULSE_PIN 2

const int pulse_pins[6] = {WAIST_PULSE_PIN, SHOULDER_PULSE_PIN, ELBOW_PULSE_PIN, AXIS_4_PULSE_PIN, AXIS_5_PULSE_PIN, AXIS_6_PULSE_PIN};
const int dir_pins[6] =   {WAIST_DIR_PIN, SHOULDER_DIR_PIN, ELBOW_DIR_PIN, AXIS_4_DIR_PIN, AXIS_5_DIR_PIN, AXIS_6_DIR_PIN};

AccelStepper axis_1(AccelStepper::DRIVER, WAIST_PULSE_PIN, WAIST_DIR_PIN);
AccelStepper axis_2(AccelStepper::DRIVER, SHOULDER_PULSE_PIN, SHOULDER_DIR_PIN);
AccelStepper axis_3(AccelStepper::DRIVER, ELBOW_PULSE_PIN, ELBOW_DIR_PIN);
AccelStepper axis_4(AccelStepper::DRIVER, AXIS_4_PULSE_PIN, AXIS_4_DIR_PIN);
AccelStepper axis_5(AccelStepper::DRIVER, AXIS_5_PULSE_PIN, AXIS_5_DIR_PIN);
AccelStepper axis_6(AccelStepper::DRIVER, AXIS_6_PULSE_PIN, AXIS_6_DIR_PIN);

union open_float
{
  char bytes[4];
  float value = 0;
};

float joint_states[6];
float joint_speeds[6];
float joint_targets[6];
bool forward_solve = false;

const int min_motor_pulse_time[6] = {2000, 2000, 2000, 1000, 1000, 1000}; // microseconds
const int motor_sprs[6] = {200, 200, 200, 200, 200, 200}; // Should all be the same. The code can't handle non-uniform micro stepping right now but should be possible 

// Motor:y , Axis:x  Waist, Shoulder, Elbow, Wrist Roll, Wrist Flex, Hand Roll
const float motor_reductions[6][6] = {{1/48., 0,     0,      0,          0,      0}, 
                                      {0,     1/48., -1/48., 0,          0,      0}, 
                                      {0,     0,     1/48.,  0,          0,      0}, 
                                      {0,     0,     0,      1/24., -1/28.8,     1/24.}, 
                                      {0,     0,     0,      0,      1/28.8,     -1/48.}, 
                                      {0,     0,     0,      0,          0,      1/24.}};
                 
unsigned int joint_step_time[6];
unsigned int motor_step_time[6];

int min_joint_step_time[6];// =  {2000, 2000, 2000, 1000, 1208, 1000}; // Fastest 
