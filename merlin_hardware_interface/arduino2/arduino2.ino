#include <BasicLinearAlgebra.h>
#include <ElementStorage.h>
#include "config.h"

#define SPR 200
#define PPR 1000

BLA::Matrix<6, 6> motor_reductions = {
  1. / 48., 0, 0,         0, 0, 0,
  0, 1. / 48., 0,         0, 0, 0,
  0, -1. / 48., 1. / 48., 0, 0, 0,
  0, 0, 0,               1. / 24., 0, 0,
  0, 0, 0,              -1. / 28.8, 1. / 28.8, 0,
  0, 0, 0,              -1. / 12., 1. / 24., 1. / 24.
};

BLA::Matrix<6, 6> radians_per_step;
BLA::Matrix<6, 6> radians_per_step_inv;

BLA::Matrix<6, 1> target_speeds;
BLA::Matrix<6, 1>  target_poses;

BLA::Matrix<6, 1>  current_poses;
BLA::Matrix<6, 1>  min_joint_travel;

void setup_steppers()
{
  stepper[0] = &stepper1;
  stepper[1] = &stepper2;
  stepper[2] = &stepper3;
  stepper[3] = &stepper4;
  stepper[4] = &stepper5;
  stepper[5] = &stepper6;
  for (int i = 0; i < 6; i++){
    stepper[i]->setMinPulseWidth(2000);
    stepper[i]->setAcceleration(4000);
  }
}
void update_encode_states(){
  }
void update_arm_state()
{
  BLA::Matrix<6, 1> steps_taken;
  for (int i = 0; i < 6; i++) steps_taken(i) = stepper[i]->currentPosition();
  current_poses = radians_per_step * steps_taken;
}

void solve_motor_speeds()
{
  BLA::Matrix<6, 1> delta_theta = target_poses - current_poses;
  // Compute Errors
  bool changed = false;
  for (int joint = 0; joint < 6; joint++)
  {
    if (abs(delta_theta(joint)) >= radians_per_step(joint, joint)) {
      changed = true;
      if (delta_theta(joint) > 0){
        target_poses(joint) *= -1;
      }
    } else {delta_theta(joint) = 0;}
  }
  if (not changed) return;
  else {
  cmd_idx = -1;
  restart = true;
  }
  for (int stage = 0; stage < 6; stage++)
  {
    double min_travel_time = 10000000;
    bool change = false;
    for (int joint = 0; joint < 6; joint++)
    {
      if ((abs(delta_theta(joint)) > radians_per_step(joint, joint)) and (target_speeds(joint) != 0))
      {
        change = true;
        float travel_time = delta_theta(joint) / target_speeds(joint);
        if (travel_time < min_travel_time)
          min_travel_time = travel_time;
      } else {
        delta_theta(joint) = 0;
        }
    }
    if (not change)
      break;
    BLA::Matrix<6, 1> steps_per_second = radians_per_step_inv * target_speeds;
    BLA::Matrix<6, 1, BLA::Array<6,1,float>> steps_to_take_float = steps_per_second * min_travel_time;
    BLA::Matrix<6, 1, BLA::Array<6,1,int>> steps_to_take = steps_to_take_float;
    BLA::Matrix<6, 1> sub_delta_theta = radians_per_step * steps_to_take;
    delta_theta -= sub_delta_theta;
    
    cmd_len = stage + 1;
    for (int i = 0; i < 6; i++)
      cmd_buffer[stage][0][i] = steps_to_take(i);
    for (int i = 0; i < 6; i++)
      cmd_buffer[stage][1][i] = steps_per_second(i);
  }
}

void handle_commands()
{
  if (Serial.available() > 0)
  {
    int cmd = Serial.read();
    open_float temp;
    if (cmd == 'R')
    {
      open_float temp;
      for (int joint = 0; joint < 6; joint++)
      {
        temp.value = current_poses(joint);
        Serial.write(temp.bytes, 4);
      }
    }

    if (cmd == 'T')
    {
      while (not Serial.available() > 0);
      open_float temp;
      for (int i = 0; i < 6; i++) {
        Serial.readBytes(temp.bytes, 4);
        target_poses(i) = temp.value;
      }
      for (int i = 0; i < 6; i++) {
        Serial.readBytes(temp.bytes, 4);
        target_speeds(i) = abs(temp.value);
      }
      solve_motor_speeds();
    }
  }
}

void setup()
{
  Serial.begin(115200);
  setup_steppers();
  // load_saved_state();
  // Pre Compute Useful Variables
  radians_per_step = motor_reductions * (double)(360. / 200);
  radians_per_step_inv = radians_per_step;

  if (not Invert(radians_per_step_inv)) {
//    Serial.println("\nInversion Failed");
  }
  else {
//    Serial.println("\nInversion Good\nRadian Per Step Inv \n");
  }
}

bool run_commands()
{
  bool done = true;
  for (int i = 0; i < 6; i++) if (stepper[i]->distanceToGo() != 0) done = false;

  if (done or restart)
  {
    restart = false;
    if (cmd_idx == (cmd_len - 1))
      return false;
    else
      cmd_idx += 1;
      
    for (int i = 0; i < 6; i++)
    {
      stepper[i]->setMaxSpeed(cmd_buffer[cmd_idx][1][i]);
      stepper[i]->setSpeed(cmd_buffer[cmd_idx][1][i]);
      stepper[i]->move(cmd_buffer[cmd_idx][0][i]);
      Serial.print(cmd_buffer[cmd_idx][0][i]);
      Serial.print(" ");
      Serial.println(cmd_buffer[cmd_idx][1][i]);
    }
  }
    for (int i = 0; i < 6; i++) {
      stepper[i]->run();
    }
  
  return true;
}
bool toggle = false;
void loop()
{
  update_encode_states();
  update_arm_state();
  handle_commands();
  bool running_ = run_commands();
if (not running_){
delay(500);
  target_speeds.Fill(toggle? 9.: 9.);
   target_poses.Fill(toggle? 5. : 0.);
    toggle = not toggle;
   solve_motor_speeds();
  }
}
