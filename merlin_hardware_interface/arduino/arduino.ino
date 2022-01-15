#include "config.h"
#include "robot_arm.h"

RobotArm merlin;

  double speeds[6] = {18, 18, 18, 18, 18, 18};
void setup()
{
  merlin.assignStepper(0, stepper1);
  merlin.assignStepper(1, stepper2);
  merlin.assignStepper(2, stepper3);
  merlin.assignStepper(3, stepper4);
  merlin.assignStepper(4, stepper5);
  merlin.assignStepper(5, stepper6);

  double reductions[6][6] = {
      {1. / 48., 0, 0, 0, 0, 0},
      {0, 1. / 48., 0, 0, 0, 0},
      {0, -1. / 48., 1. / 48., 0, 0, 0},
      {0, 0, 0,                1. / 24.,   0, 0},
      {0, 0, 0,               -1. / 28.8, 1. / 28.8, 0},
      {0, 0, 0,               -1. / 12.,  1. / 24., 1. / 24.}};

  merlin.setMotorReductions(reductions);
  merlin.setTargetSpeeds(speeds);
  double accel[6] = {9, 9, 9, 9, 9, 9};
  merlin.setAccelerations(accel);
  merlin.setFrequency(120.);
  Serial.begin(BAUD_RATE);
}
void handle_commands();
void loop()
{
  handle_commands();
 merlin.runSpeedToPositions();
// merlin.setTargetSpeeds(speeds);
//
//for (int i = 0; i < 6; i++){
//  merlin.moveTo(i, 25);
//  merlin.runToPositions();
//  delay(500);
//  }
//  
//  for (int i = 0; i < 6; i++){
//    merlin.moveTo(i, 0);
//  }
//  merlin.runToPositions();
//  delay(2000);
}

void handle_commands()
{
  if (Serial.available() > 0)
  {
    int cmd = Serial.read();
    open_float temp;
    if (cmd == 'R') // Read
    {
      for (int joint = 0; joint < 6; joint++)
      {
        temp.value = merlin.currentPosition(joint);
        Serial.write(temp.bytes, 4);
      }
    }
    if (cmd == 'P') // Pose Command
    {
      while (not Serial.available() > 0)
        ; // (4 * 6 * 2)

      for (int i = 0; i < 6; i++)
      {
        Serial.readBytes(temp.bytes, 4);
        merlin.moveTo(i, temp.value);
      }
    }
    if (cmd == 'S') // Set Speed Command
    {
      while (not Serial.available() > 0)
        ; // (4 * 6 * 2)
      for (int i = 0; i < 6; i++)
      {
        Serial.readBytes(temp.bytes, 4);
        merlin.setTargetSpeed(i, temp.value);
      }
    }
    if (cmd == 'T') // Trajectory Point Command
    {
      while (not Serial.available() > 0)
        ; // (4 * 6 * 2)
      for (int i = 0; i < 6; i++)
      {
        Serial.readBytes(temp.bytes, 4);
        merlin.moveTo(i, temp.value);
      }
      for (int i = 0; i < 6; i++)
      {
        Serial.readBytes(temp.bytes, 4);
        merlin.setTargetSpeed(i, temp.value);
      }
    }
  }
}
