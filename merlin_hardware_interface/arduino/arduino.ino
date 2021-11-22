#include "config.h"
void setup_pins()
{
  for (int m = 0; m < 6; m++)
  {
    pinMode(pulse_pins[m], OUTPUT);
    pinMode(dir_pins[m], OUTPUT);
  }
}

void setDPS(float target_dps)
{
  // Calculate new min_joint_step_time array
}

void setup()
{
  Serial.begin(9600);
  setup_pins();
  //Serial.println("MERLIN MR6500 Controller Started");
}

bool update_arm_controls()
{
  int axis_steps[6] = {0, 0, 0, 0, 0, 0};
  unsigned int curr_time = millis();
  bool updated = false;

  // Update 6 Joints
  for (int j = 0; j < 6; j++)
  {
    if (!forward_solve)
      j %= 5;
    forward_solve = !forward_solve;
    float step_distance = (360.0 / motor_sprs[j]) * motor_reductions[j][j];
    if (abs(joint_states[j] - joint_targets[j]) >= abs(step_distance)
    {
      int step_direction = (joint_states[j] > joint_targets[j]) ? -1 : 1;

      bool overflow = false;
      overflow |= ((curr_time - joint_step_time[j]) < min_joint_step_time[j]);
      for (int m = 0; m < 6; m++)
      {
        overflow |= abs(axis_steps[m] + (step_direction * joint_step_rules[m][j])) > 1;
        overflow |= ((curr_time - motor_step_time[m]) < min_motor_pulse_time[m]);
      }
      if (overflow)
        continue;
      for (int m = 0; m < 6; m++)
      {
        axis_steps[m] += step_direction * joint_step_rules[m][j];
      }
      joint_states[j] += step_direction * step_distance;
      joint_step_time[j] = curr_time;
      updated = true;
    }
  }

  // Step all motors at once
  for (int m = 0; m < 6; m++)
  {
    if (axis_steps[m] != 0)
    {
      digitalWrite(dir_pins[m], (axis_steps[m] > 0) ? LOW : HIGH);
      digitalWrite(pulse_pins[m], HIGH);
      motor_step_time[m] = curr_time;
    }
  }

  for (int m = 0; m < 6; m++)
  {
    if (curr_time - motor_step_time[m] >= min_motor_pulse_time[m])
    {
      digitalWrite(pulse_pins[m], LOW);
    }
  }

  return updated;
}

void handle_commands()
{
  if (Serial.available() > 0)
  {
    char cmd = Serial.read();
    if (cmd == 'R')
    {
      open_float temp;
      for (int i = 0; i < 6; i++)
      {
        temp.value = joint_states[i];
        Serial.write(temp.bytes, 4);
      }
    }

    if (cmd == 'W')
    {
      open_float temp;
      for (int i = 0; i < 6; i++)
      {
        Serial.readBytes(temp.bytes, 4);
        joint_targets[i] = temp.value;
      }
    }

    if (cmd == 'C')
    {
      for (int i = 0; i < 6; i++)
      {
        Serial.print(joint_states[i]);
        Serial.print(" ");
      }
      Serial.println("");
    }

    if (cmd == 'M')
    {
      open_float temp;
      for (int i = 0; i < 6; i++)
      {
        Serial.readBytes(temp.bytes, 4);
        joint_targets[i] = temp.value;
      }
      bool done = false;
      while (not done)
      {
        done = not update_arm_controls();
      }
      Serial.println("DONE");
    }

    if (cmd == 'H')
    {
      for (int j = 0; j < 6; j++)
        joint_targets[j] = 0;
      bool done = false;
      while (not done)
      {
        done = not update_arm_controls();
      }
      Serial.println("DONE");
    }

    if (cmd == 'T')
    {
      // joint_targets[1] = 20;
      // joint_targets[0] = 20;
    }
  }
}

void loop()
{
  handle_commands();
  update_arm_controls();
  // update_arm_state();
}
