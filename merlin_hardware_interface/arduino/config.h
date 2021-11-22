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

union open_float
{
  char bytes[4];
  float value = 0;
};

float joint_states[6];
float joint_targets[6];

const int pulse_pins[6] = {WAIST_PULSE_PIN, SHOULDER_PULSE_PIN, ELBOW_PULSE_PIN, AXIS_4_PULSE_PIN, AXIS_5_PULSE_PIN, AXIS_6_PULSE_PIN};
const int dir_pins[6] =   {WAIST_DIR_PIN, SHOULDER_DIR_PIN, ELBOW_DIR_PIN, AXIS_4_DIR_PIN, AXIS_5_DIR_PIN, AXIS_6_DIR_PIN};

bool forward_solve = false;

// Motor:y , Axis:x  Waist, Shoulder, Elbow, Wrist Roll, Wrist Flex, Hand Roll
const int motor_sprs[6] = {200, 200, 200, 200, 200, 200};
const float motor_reductions[6][6] = {{1/48., 0,     0,      0,          0,      0}, 
                                      {0,     1/48., -1/48., 0,          0,      0}, 
                                      {0,     0,     1/48.,  0,          0,      0}, 
                                      {0,     0,     0,      1/24., -1/29.,     1/24.}, 
                                      {0,     0,     0,      0,      1/29.,     -1/48.}, 
                                      {0,     0,     0,      0,          0,      1/24.}};

const int joint_step_rules[6][6] = {{1,     0,     0,      0,          0,      0}, 
                                    {0,     1,     0,      0,          0,      0}, 
                                    {0,     1,     1,      0,          0,      0}, 
                                    {0,     0,     0,     -1,          0,      0}, 
                                    {0,     0,     0,     -1,          1,      0}, 
                                    {0,     0,     0,     -1,         -1,      1}};
                                    
unsigned int joint_step_time[6];
unsigned int motor_step_time[6];

unsigned int min_motor_pulse_time[6] = {2, 2, 2, 1, 1, 1};
float min_joint_step_time[6] = {4, 4, 4, 2, 2, 2};