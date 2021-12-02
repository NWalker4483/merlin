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

union open_float
{
  char bytes[4];
  float value = 0;
};

union open_int
{
  char bytes[4];
  int value = 0;
};

float joint_states[6];
float joint_targets[6];

const int motor_sprs[6] = {200, 200, 200, 200, 200, 200}; // Should all be the same. The code can't handle non-uniform micro stepping right now but should be possible 

// Motor:y , Joint:x  Waist, Shoulder, Elbow, Wrist Roll, Wrist Flex, Hand Roll
const float motor_reductions[6][6] = {{1/48., 0,     0,      0,          0,      0}, 
                                      {0,     1/48., -1/48., 0,          0,      0}, 
                                      {0,     0,     1/48.,  0,          0,      0}, 
                                      {0,     0,     0,      1/24., -1/28.8,     1/24.}, 
                                      {0,     0,     0,      0,      1/28.8,     -1/48.}, 
                                      {0,     0,     0,      0,          0,      1/24.}};
                
