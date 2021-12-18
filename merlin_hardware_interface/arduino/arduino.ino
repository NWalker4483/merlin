#include "config.h"
void setup_steppers()
{
  stepper[0] = &stepper1;
  stepper[1] = &stepper2;
  stepper[2] = &stepper3;
  stepper[3] = &stepper4;
  stepper[4] = &stepper5;
  stepper[5] = &stepper6;
  for (int i = 0; i < 6; i++) {
//    stepper[i]->setMinPulseWidth(1500);
    stepper[i]->setMaxSpeed(1000);
   stepper[i]->setAcceleration(1000);
  }
}

void update_encode_states() {

}

void update_arm_state()
{
  Matrix<6, 1> steps_taken;
  for (int i = 0; i < 6; i++) steps_taken(i) = stepper[i]->currentPosition();
  current_poses = degrees_per_step * steps_taken;
}

float round_to_multiple(float num, float multiple){
//   multiple = abs(multiple);
   float result = abs(num) + (multiple / 2.);
   result -= fmod(result, multiple);
   result *= num > 0 ? 1.: -1.;
   return result; 
  }

void solve_motor_speeds()
{
  // Saves a little bit of time 
  static Matrix<6, 1> steps_per_second; 
  static Matrix<6, 1, Array<6,1,int>> steps_to_take;
  static Matrix<6, 1> sub_delta_theta;
  static Matrix<6, 1> delta_theta;
  update_arm_state();
  delta_theta = target_poses - current_poses;
  // Compute Appropriate Errors
  for (int joint = 0; joint < 6; joint++)
  {    
    target_speeds(joint) = abs(target_speeds(joint)) * ( delta_theta(joint) > 0 ? 1 : -1);
  }
  
  for (int stage = 0; stage < MAX_CMD_LENGTH; stage++)
  {
    float min_travel_time = BIG_NUMBER;
    for (int joint = 0; joint < 6; joint++)
    {
      delta_theta(joint) = round_to_multiple(delta_theta(joint), degrees_per_step(joint, joint));
      if ((abs(delta_theta(joint)) > 0) and target_speeds(joint) != 0)
      {
        float travel_time = delta_theta(joint) / target_speeds(joint);
        if (travel_time < min_travel_time)
          min_travel_time = travel_time;
      } 
      else {
        target_speeds(joint) = 0;
        }
    }
    // All joints are within their acceptable range
    if (min_travel_time == BIG_NUMBER) break;


    steps_per_second = degrees_per_step_inv * target_speeds;
    steps_to_take = steps_per_second * abs(min_travel_time);
    
    sub_delta_theta = degrees_per_step * steps_to_take;
    delta_theta -= sub_delta_theta;

    if(0){
    Serial.print("\nStage: ");
    Serial.print(stage + 1);
    Serial.print("\nSub Delta Theta: ");
    Serial << sub_delta_theta;
    Serial.print("\nSteps to Take: ");
    Serial << steps_to_take;
    Serial.print("\nSteps per Second: ");
    Serial << steps_per_second;
    Serial.print("\nCurrent Poses: ");
    Serial << current_poses;
    Serial.println(" ");
    }
    
    for (int i = 0; i < 6; i++)
      cmd_buffer[stage][0][i] = steps_to_take(i);
    for (int i = 0; i < 6; i++)
      cmd_buffer[stage][1][i] = steps_per_second(i);

    cmd_len = stage + 1;
    interrupt = true;
  }
  cmd_idx = -1;
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
      update_arm_state();
      for (int joint = 0; joint < 6; joint++)
      {
        temp.value = current_poses(joint);
        Serial.write(temp.bytes, 4);
      }
    }

    if (cmd == 'T')
    {
      while (not Serial.available() > 0);
      bool changed = false;
      open_float temp;
      for (int i = 0; i < 6; i++) {
        Serial.readBytes(temp.bytes, 4);
        if (target_poses(i) != temp.value) {
          target_poses(i) = temp.value;
          changed = true;
        }
      }
      for (int i = 0; i < 6; i++) {
        Serial.readBytes(temp.bytes, 4);
        if (target_speeds(i) != temp.value) {
          target_speeds(i) = temp.value;
          changed = true;
        }
      }
      if (changed)
        solve_motor_speeds();
    }
  }
}
void load_saved_state(){};
void setup()
{
  Serial.begin(BAUD_RATE);
  setup_steppers();
  load_saved_state();
  // Pre Compute Useful Variables
  degrees_per_step = motor_reductions * ((float)FULL_ROTATION / (float)SPR);
  degrees_per_step_inv = degrees_per_step;
  Invert(degrees_per_step_inv);
//  Serial<< degrees_per_step;
//  while(1);
}

bool run_commands()
{
  bool done = true;
  for (int i = 0; i < 6; i++) done &= (stepper[i]->distanceToGo() == 0);

  if (done or interrupt)
  {
    interrupt = false;
    if (cmd_idx == (cmd_len - 1))
      return false;
    else
      cmd_idx += 1;
    for (int i = 0; i < 6; i++)
    {
      stepper[i]->move(cmd_buffer[cmd_idx][0][i]);
      stepper[i]->setSpeed(cmd_buffer[cmd_idx][1][i]);
    }
  }
 for (int i = 0; i < 6; i++) stepper[i]->runSpeedToPosition();
  return true;
}

bool toggle = false;
bool demo = 1;
void loop()
{ 
  update_encode_states();
  handle_commands();
  bool running_ = run_commands();
  if ((not running_) and demo) {
    delay(250);
    
    Serial.print("Command Length: ");
    Serial.println(cmd_len);
    target_speeds.Fill(9);

    target_poses(0) = (toggle ? 1 : 0);
    target_poses(1) = (toggle ? 2 : 0);
    target_poses(2) = (toggle ? 3 : 0);
    target_poses(3) = (toggle ? 4 : 0);
    target_poses(4) = (toggle ? 5 : 0);
    target_poses(5) = (toggle ? 6 : 0);
    
    toggle = not toggle;
    unsigned long start_time;
    unsigned long stop_time;
    start_time = millis();
    for (int i = 0; i < 100;i++){
      target_speeds.Fill(9);
      solve_motor_speeds();
      }
      stop_time = millis();
      Serial.println((stop_time - start_time)/100);
  }
}
