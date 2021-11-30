#include "config.h"

void setup() { Serial.begin(115200); }

void handle_commands() {
  if (Serial.available() > 0) {
    int cmd = Serial.read();
    open_float temp;

    if (cmd == 'R') {
      open_float temp;
      for (int i = 0; i < 6; i++) {
        temp.value = joint_states[i];
        Serial.write(temp.bytes, 4);
      }
    }

    if (cmd == 'W') {
      for (int i = 0; i < 6; i++) {
        Serial.readBytes(temp.bytes, 4);
        joint_targets[i] = temp.value;
      }
    }

    if (cmd == 'V') { // Accept degree per second values
      for (int i = 0; i < 6; i++) {
        Serial.readBytes(temp.bytes, 4);
        // joint_targets[i] = temp.value;
      }
    }

    if (cmd == 'C') { // TODO: Udpate Merlin RoboDK Driver to accept bytes from
                      // 'R' command
      for (int i = 0; i < 6; i++) {
        Serial.print(joint_states[i]);
        Serial.print(" ");
      }
      Serial.println("");
    }

    if (cmd == 'M') {
      // todo calculate joint steps
      for (int i = 0; i < 6; i++) {
        Serial.readBytes(temp.bytes, 4);
        joint_targets[i] = temp.value;
      }
      bool done = false;
      while (not done) {
        done = not update_arm_controls();
      }
      Serial.println("DONE");
    }
    if (cmd == 'T') {
      for (int i = 0; i < 6; i++)
        joint_targets[i] = 45;
    }
    if (cmd == 'Z') {
      for (int i = 0; i < 6; i++)
        joint_targets[i] = 0;
    }
  }
}

void loop() {
  handle_commands();
  update_arm_controls();
  // update_arm_state();
}
