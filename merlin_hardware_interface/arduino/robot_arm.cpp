#include "robot_arm.h"

RobotArm::RobotArm() {}

void RobotArm::setMotorReductions(double reductions[6][6])
{
  for (int row = 0; row < 6; row++)
  {
    for (int col = 0; col < 6; col++)
    {
      motor_reductions(row, col) = reductions[row][col];
    }
  }
  // Pre Compute Useful Variables
  degrees_per_step = motor_reductions * (360.L / 200.L);
  Invert(degrees_per_step, degrees_per_step_inv);
}

void RobotArm::assignStepper(int motor_num, SpeedStepper &new_stepper)
{
  stepper[motor_num] = &new_stepper;
}

void RobotArm::moveTo(int joint, double absolute)
{
  if (target_poses[joint] != absolute)
  {
    _target_changed = true;
    target_poses[joint] = absolute;
  }
}

void RobotArm::move(int joint, double relative)
{
  moveTo(joint, (current_poses(joint) + relative));
}

void RobotArm::moveAllTo(double absolute[6])
{
  for (int i = 0; i < 6; i++)
    moveTo(i, absolute[i]);
}
void RobotArm::moveAll(double relative[6])
{
  for (int i = 0; i < 6; i++)
    move(i, relative[i]);
}

void RobotArm::setTargetSpeeds(double speed[6])
{
  for (int i = 0; i < 6; i++)
    setTargetSpeed(i, speed[i]);
}
void RobotArm::setTargetAccelerations(double acceleration[6])
{
  for (int i = 0; i < 6; i++)
    setTargetAcceleration(i, acceleration[i]);
}

void RobotArm::setSpeeds(double speed[6])
{
  for (int i = 0; i < 6; i++)
    setSpeed(i, speed[i]);
}
void RobotArm::setAccelerations(double acceleration[6])
{
  for (int i = 0; i < 6; i++)
    setAcceleration(i, acceleration[i]);
}

void RobotArm::setTargetSpeed(int joint, double speed)
{
  target_speeds[joint] = speed;
}

void RobotArm::setSpeed(int joint, double speed)
{
  if (_speeds[joint] != speed)
  {
    _speeds[joint] = speed;
    _speeds_changed = true;
  }
}
void RobotArm::setTargetAcceleration(int joint, double acceleration)
{
  acceleration = fabs(acceleration);
  target_accel[joint] = acceleration;
}

void RobotArm::setAcceleration(int joint, double acceleration)
{
  if (_accelerations[joint] != acceleration)
  {
    _accelerations[joint] = acceleration;
    updateIncrementsAndDelays(joint);
    _accels_changed = true;
  }
}

void RobotArm::computeNewSpeed(int joint)
{
  long _time = micros();
  double distance_left = distanceToGo(joint);

  if (fabs(distance_left) < degrees_per_step(joint, joint)) // Need to stop
  {
    setSpeed(joint, 0);
    return;
  }

  if ((_time - last_increment_time[joint]) >= delays[joint]) // Time for update
  {
    //      Serial.print("AAAABBBBCCCCDDDDEEEEFFFFAAAABBBBCCCCDDDDEEEEFFFFAAAABBBBCCCCDDDDEEEEFFFF");
    //
    //      Serial.print("AAAABBBBCCCCDDDDEEEEFFFFAAAABBBBCCCCDDDDEEEEFFFFAAAABBBBCCCCDDDDEEEEFFFF");
    //
    //      Serial.print("AAAABBBBCCCCDDDDEEEEFFFFAAAABBBBCCCCDDDDEEEEFFFFAAAABBBBCCCCDDDDEEEEFFFF");
    double temp;
    if (((_speeds[joint] * _speeds[joint]) / (2. * _accelerations[joint])) >= abs(distance_left)) // Need to decelerate to stop
    {
      temp = _speeds[joint] - ((_speeds[joint] > 0) ? increments[joint] : -increments[joint]);
    }
    else if (abs(_speeds[joint] - target_speeds[joint]) >= increments[joint]) // Need to approach target speed
    {
      temp = _speeds[joint] + ((_speeds[joint] < target_speeds[joint]) ? increments[joint] : -increments[joint]);
    } else {
      temp = target_speeds[joint];
    }

    setSpeed(joint, temp);
    last_increment_time[joint] = _time;
  }
}
void RobotArm::computeNewSpeeds()
{
  for (int i = 0; i < 6; i++)
  {
    computeNewSpeed(i);
  }
}

void RobotArm::computeJointPositions()
{
  Matrix<6, 1, Array<6, 1, int>> steps_taken;
  for (int i = 0; i < 6; i++) {
    steps_taken(i) = stepper[i]->currentPosition();
  }
  current_poses = degrees_per_step * steps_taken;
}

void RobotArm::computeMotorSpeeds()
{
  Matrix<6, 1, Array<6, 1, double>> speeds_holder;
  Matrix<6, 1, Array<6, 1, double>> steps_per_second;

  for (int i = 0; i < 6; i++)
    speeds_holder(i) = _speeds[i];

  steps_per_second = degrees_per_step_inv * speeds_holder;

  for (int i = 0; i < 6; i++) {
    stepper[i]->setSpeed(steps_per_second(i));
  }
  _speeds_changed = false;
}

double RobotArm::distanceToGo(int joint)
{
  return target_poses[joint] - current_poses(joint);
}

double RobotArm::targetPosition(int joint)
{
  return target_poses[joint];
}

double RobotArm::currentPosition(int joint)
{
  return current_poses(joint);
}

bool RobotArm::run()
{
  if (_target_changed or _speeds_changed) {
    checkSpeedsForDirection();
  }
  runSpeed();
  bool done = true;
  for (int i = 0; i < 6; i++) done = done and (abs(distanceToGo(i)) < degrees_per_step(i, i));
  computeNewSpeeds();
  return not done;
}
bool RobotArm::runSpeed()
{
  if (_speeds_changed)
    computeMotorSpeeds();
  bool has_stepped = false;
  for (int i = 0; i < 6; i++)
  {
    bool stepped = stepper[i]->runSpeed();
    if (stepped)
      has_stepped = true;
  }
  computeJointPositions();
  return has_stepped;
}

void RobotArm::runToPositions()
{
  while (run());
}

void RobotArm::checkSpeedsForDirection() {
  for (int i = 0; i < 6; i++) {
    _speeds[i] = fabs(_speeds[i]) * ((target_poses[i] > current_poses(i)) ? 1 : -1 );
  }
  _target_changed = false;
}

bool RobotArm::runSpeedToPositions()
{
  if (_target_changed or _speeds_changed) {
    checkSpeedsForDirection();
  }
  runSpeed();
  bool done = true;
  for (int joint = 0; joint < 6; joint++)
  {
    if (fabs(distanceToGo(joint)) < degrees_per_step(joint, joint))
    {
      setSpeed(joint, 0);
    }
    else {
      setSpeed(joint, target_speeds[joint]);
      done = false;
    }
  }
  return not done;
}
void RobotArm::runToNewPositions(double positions[6])
{
  moveAllTo(positions);
  runToPositions();
}

void RobotArm::stop()
{
  // if (_speed != 0.0)
  // {
  // long stepsToStop = (long)((_speed * _speed) / (2.0 * _acceleration)) + 1; // Equation 16 (+integer rounding)
  // if (_speed > 0)
  //     move(stepsToStop);
  // else
  //     move(-stepsToStop);
  // }
}

void RobotArm::setFrequency(double freq) {
  _freq = fabs(freq);
  for (int i = 0; i < 6; i++) updateIncrementsAndDelays(i);
}
void RobotArm::updateIncrementsAndDelays(int joint)
{
  increments[joint] = fabs(_accelerations[joint]) / _freq;
  delays[joint] = 1000000. / _freq;
  _accels_changed = false;
}
