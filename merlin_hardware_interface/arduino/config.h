#include "ABIEncoder.h"
#define HEDS6310_PPR

#define WAIST_PULSE_PIN 7
#define WAIST_DIR_PIN 6
#define WAIST_BRAKE_PIN 8
#define WAIST_ENCODER_SELECT 20
#define WAIST_SPR 200

#define SHOULDER_PULSE_PIN 5
#define SHOULDER_DIR_PIN 4
#define SHOULDER_BRAKE_PIN 9
#define SHOULDER_ENCODER_SELECT 20
#define SHOULDER_SPR 200

#define ELBOW_PULSE_PIN 3
#define ELBOW_DIR_PIN 2
#define ELBOW_BRAKE_PIN 10
#define ELBOW_ENCODER_SELECT 20
#define ELBOW_SPR 200

#define AXIS_4_PULSE_PIN 11
#define AXIS_4_DIR_PIN 12
#define AXIS_4_ENCODER_SELECT 20

#define AXIS_5_PULSE_PIN 13
#define AXIS_5_DIR_PIN 14
#define AXIS_5_ENCODER_SELECT 20

#define AXIS_6_PULSE_PIN 15
#define AXIS_6_DIR_PIN 16
#define AXIS_6_ENCODER_SELECT 20

#define ONE_DEGREE 0.0175
#define PI 3.14

ABIEncoder waist_encoder(WAIST_ENCODER_SELECT);
ABIEncoder shoulder_encoder(WAIST_ENCODER_SELECT);
ABIEncoder elbow_encoder(WAIST_ENCODER_SELECT);
ABIEncoder axis_4_encoder(WAIST_ENCODER_SELECT);
ABIEncoder axis_5_encoder(WAIST_ENCODER_SELECT);
ABIEncoder axis_6_encoder(WAIST_ENCODER_SELECT);

union open_float
{
  char bytes[4];
  float value = 0;
};

open_float waist, shoulder, elbow, wrist_roll, wrist_flex, hand_roll;
open_float waist_target, shoulder_target, elbow_target, wrist_roll_target, wrist_flex_target, hand_roll_target;

// int joint_states;
// int joint_targets;
ABIEncoder encoders[6] = {waist_encoder, shoulder_encoder, elbow_encoder, axis_4_encoder, axis_5_encoder, axis_6_encoder};
int pulse_pins[6] = {WAIST_PULSE_PIN, SHOULDER_PULSE_PIN, ELBOW_PULSE_PIN, AXIS_4_PULSE_PIN, AXIS_5_PULSE_PIN, AXIS_6_PULSE_PIN};
int dir_pins[6] = {WAIST_DIR_PIN, SHOULDER_DIR_PIN, ELBOW_DIR_PIN, AXIS_4_DIR_PIN, AXIS_5_DIR_PIN, AXIS_6_DIR_PIN};

int last_encoder_val[6];
bool wrist_first = false;

// Motor:y , Axis:x  Waist, Shoulder, Elbow, Wrist Roll, Wrist Flex, Hand Roll

float motor_reductions[6][6] = {{48, 0, 0, 0, 0, 0}, 
                                {0, 48, -48, 0, 0, 0}, 
                                {0, 0, 48, 0, 0, 0}, 
                                {0, 0, 0, 1, 1, 0}, 
                                {0, 0, 0, 0, 1, 1}, 
                                {0, 0, 0, 0, 0, 1}};
