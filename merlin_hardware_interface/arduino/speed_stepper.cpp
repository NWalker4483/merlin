#include "speed_stepper.h"

SpeedStepper::SpeedStepper(int p, int d)
{
  pulse_pin = p;
  dir_pin = d;
  pinMode(p, OUTPUT);
  pinMode(d, OUTPUT);
  digitalWrite(d,LOW);
}

bool SpeedStepper::runSpeed()
{
  if (!_stepInterval or _speed == 0)
  {
    return false;
  }
  
  unsigned long time = micros();
  if (time - _lastStepTime >= (_stepInterval / 2.))
  {
    if (not pulse_on) {
      pulse_on = true;
      digitalWrite(pulse_pin, HIGH);
      
      _lastStepTime = time;
      if (_direction == 1)
      {
        // Clockwise
        _currentPos += 1;
      }
      else
      {
        // Anticlockwise
        _currentPos -= 1;
      }
      return false;
    } else {
      pulse_on = false;
      digitalWrite(pulse_pin, LOW);
      
      _lastStepTime = time;      
      return true;
    }
  }
  else
  {
    return false;
  }
}

void SpeedStepper::setSpeed(float speed)
{
  if (speed == _speed)
    return;
  if (speed == 0.0)
    _stepInterval = 0;
  else
  {
    _stepInterval = fabs(1000000.0 / speed);
    _direction = (speed > 0.0) ? 1 : 0;
    digitalWrite(dir_pin, _direction);
  }
  _speed = speed;
}

int SpeedStepper::currentPosition()
{
  return _currentPos;
}
