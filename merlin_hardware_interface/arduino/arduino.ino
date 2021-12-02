#include "config.h"

int a = 250;
int b = 500;

int cmd_buffer[6][2][6] ={{{a,a,a,a,a,a},{b,b,b,b,b,b}},
{{-a,-a,-a,-a,-a,-a},{b,b,b,b,b,b}}} ;

int cmd_idx = -1;

bool debug = 0;
int cmd_len = 0;

void setup_steppers()
{
  stepper[0] = &stepper1;
  stepper[1] = &stepper2;
  stepper[2] = &stepper3;
  stepper[3] = &stepper4;
  stepper[4] = &stepper5;
  stepper[5] = &stepper6;
  for (int i = 0; i < 6; i++)
    stepper[i]->setAcceleration(1000);
}

void setup()
{
  Serial.begin(115200);
  setup_steppers();
}

void handle_commands()
{
  if (Serial.available() > 2)
  {
    int cmd = Serial.read();
    open_float temp;
    if (cmd == 'R')
    {
      for (int j = 0; j < 6; j++)
      {
        Serial.write(stepper[j]->currentPosition());
        Serial.write(1);
      }
    }
    if (cmd == 'W')
    {
      open_float temp;
     cmd_len = Serial.read() - '0';
    Serial.println(cmd_len);
      // Read upto the next 96 bytes
      for (int step_ = 0; step_ < cmd_len; step_++)
      {
        for (int mode = 0; mode < 2; mode++)
        {
          for (int motor = 0; motor < 6; motor++)
          {
            Serial.readBytes(temp.bytes, 4);
            cmd_buffer[step_][mode][motor] = temp.value;
          }
        }
      }
      cmd_idx = -1;
    }
  }
}



void update_arm_controls()
{
  if (cmd_len == 0) return;
  for (int i = 0; i < 6; i++)
    stepper[i]->run();
    
  bool done = true;
  for (int i = 0; i < 6; i++)
    if (stepper[i]->isRunning())
      done = false;
  
  if (done)
  {
    cmd_idx = constrain(cmd_idx, -1, cmd_len) + 1;
    if (cmd_idx >= cmd_len){
      if (debug)
      cmd_idx = 0;
      else{
      return;
      }
    }
    //  
    for (int i = 0; i < 6; i++)
    {
      stepper[i]->setMaxSpeed(cmd_buffer[cmd_idx][1][i]);
      stepper[i]->setSpeed(cmd_buffer[cmd_idx][1][i]);
      stepper[i]->move(cmd_buffer[cmd_idx][0][i]);
    }
  };
 
}

void loop()
{
  handle_commands();
  update_arm_controls();
  //update_arm_state();
}

//void update_arm_state()
//{
//  // WARN: This is placeholder code for before the encoders are installed
//  for (int i = 0; i < 6; i++){
//  index_cnt[i].value = stepper[i]->currentPosition() / 200;
//  pulse_cnt[i].value = stepper[i]->currentPosition() % 200;
//  }
//}
