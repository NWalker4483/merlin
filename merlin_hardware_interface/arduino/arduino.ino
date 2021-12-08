#include "config.h"

void setup_steppers() {
  stepper[0] = &stepper1;
  stepper[1] = &stepper2;
  stepper[2] = &stepper3;
  stepper[3] = &stepper4;
  stepper[4] = &stepper5;
  stepper[5] = &stepper6;
  for (int i = 0; i < 6; i++)
    stepper[i]->setAcceleration(2000);
}

void load_saved_state() {}

void setup() {
  Serial.begin(115200);

  setup_steppers();
  load_saved_state();
}

void handle_commands() {
  if (Serial.available() > 0) {
    int cmd = Serial.read();
    open_float temp;
    if (cmd == 'R') {
      for (int motor = 0; motor < 6; motor++) {
        Serial.write(index_cnt[motor].bytes, 4);
        Serial.write(pulse_cnt[motor].bytes, 4);
      }
    }

    if (cmd == 'W') {
      for (int motor = 0; motor < 6; motor++) {
        Serial.readBytes(temp.bytes, 4);
        stepper[motor]->setSpeed(temp.value);
        
        stepper[motor]->move(temp.value > 0 ? 100000: -100000);
      if (temp.value == 0) stepper[motor]->move(0);
      }
    }

//    if (cmd == 'T') {
//      while (not Serial.available());
//      cmd_len = Serial.read() - '0';
//      // Read upto the next 96 bytes
//      for (int step_ = 0; step_ < cmd_len; step_++) {
//        for (int mode = 0; mode < 2; mode++) {
//          for (int motor = 0; motor < 6; motor++) {
//            Serial.readBytes(temp.bytes, 4);
//            cmd_buffer[step_][mode][motor] = temp.value;
//          }
//        }
//      }
//      cmd_idx = -1;
//    }
  }
}

void update_arm_state() {
  // WARN: This is placeholder code for before the encoders are installed
  for (int i = 0; i < 6; i++) {
    index_cnt[i].value = (int)stepper[i]->currentPosition() / 200;
    pulse_cnt[i].value = (int)stepper[i]->currentPosition() % 200;
  }
}

//void update_arm_controls() {
//  if (cmd_len == 0)
//    return;
//  for (int i = 0; i < 6; i++)
//    stepper[i]->run();
//
//  bool done = true;
//  for (int i = 0; i < 6; i++)
//    if (stepper[i]->isRunning())
//      done = false;
//
//  if (done) {
//    cmd_idx = constrain(cmd_idx, -1, cmd_len) + 1;
//    if (cmd_idx >= cmd_len) {
//      if (debug)
//        cmd_idx = 0;
//      else {
//        return;
//      }
//    }
//    for (int i = 0; i < 6; i++) {
//      stepper[i]->setMaxSpeed(cmd_buffer[cmd_idx][1][i]);
//      stepper[i]->setSpeed(cmd_buffer[cmd_idx][1][i]);
//      stepper[i]->move(cmd_buffer[cmd_idx][0][i]);
//    }
//  };
//}

void loop() {
  handle_commands();
  for (int i = 0; i < 6; i++)
    stepper[i]->runSpeed();
  //   update_arm_controls();
  update_arm_state();
}
