#ifndef Robot_Am_h
#define Robot_Am_h
#include "Arduino.h"
class SpeedStepper{
  public:
  SpeedStepper(int p, int d);
  bool runSpeed();
  void setSpeed(float speed);
  int currentPosition();
  private:
  int pulse_pin;
  int dir_pin;
  double _speed;
  bool _direction;
  int _currentPos;
  bool pulse_on; 
  unsigned long _lastStepTime;
  unsigned int _stepInterval;
  };
  #endif
