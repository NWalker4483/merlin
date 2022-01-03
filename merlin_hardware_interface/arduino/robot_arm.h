#ifndef Robot_Arm_h
#define Robot_Arm_h

#include <BasicLinearAlgebra.h>
#include "speed_stepper.h"
using namespace BLA;

class RobotArm
{
public:
  RobotArm();
  void setMotorReductions(double reductions[6][6]);
  void assignStepper(int motor_num, SpeedStepper &stepper);

  void moveTo(int joint, double absolute);
  void move(int joint, double relative);
  void moveAllTo(double absolute[6]);
  void moveAll(double relative[6]);

  void setTargetSpeeds(double speed[6]);
  void setSpeeds(double speed[6]);
  void computeNewSpeeds();

  void setTargetAccelerations(double acceleration[6]);
  void setAccelerations(double acceleration[6]);
  void computeNewAccelerations();

  void checkSpeedsForDirection();

  // Todo this should really just be one function
  void setFrequency(double freq);
  void setTargetSpeed(int joint, double speed);
  void setSpeed(int joint, double speed);
  void computeNewSpeed(int joint);

  void setTargetAcceleration(int joint, double acceleration);
  void setAcceleration(int joint, double acceleration);
  void computeNewAcceleration(int joint);
  
  /*virtual*/ void computeJointPositions();
  /*virtual*/ void computeMotorSpeeds();
  /*virtual*/ void setMotorSpeeds();

  double distanceToGo(int joint);

  double targetPosition(int joint);
  double currentPosition(int joint);

  bool run();
  bool runSpeed();

  void runToPositions();
  bool runSpeedToPositions();
  void runToNewPositions(double position[6]);

  void stop();

private:
  double target_poses[6];
  double target_speeds[6];
  double target_accel[6];

  double _motor_speeds[6];
  double _speeds[6];
  double _accelerations[6];

  double _freq = 30.;

  unsigned long last_increment_time[6];
  double increments[6];
  double delays[6];

  SpeedStepper *stepper[6];

  void updateIncrementsAndDelays(int joint);

  Matrix<6, 1, Array<6, 1, double>> current_poses;

  Matrix<6, 6, Array<6, 6, double>> motor_reductions;

  Matrix<6, 6, Array<6, 6, double>> degrees_per_step;
  Matrix<6, 6, Array<6, 6, double>> degrees_per_step_inv;

  bool _target_changed; //
  bool _speeds_changed; //
  bool _accels_changed; //
  bool _is_running;
};
#endif
