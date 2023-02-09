#include "merlin_bot.cpp"
#include "iostream"
 
Lloyd lloyd;

int main()
{
  lloyd.delay(2000);
  lloyd.setLimitMode(0);
  lloyd.setTargetSpeed(1);
  lloyd.setTargetAcceleration(.25);
  lloyd.axis[0].moveTo(0);
  lloyd.runToPositions();

  for (int i = 0; i < 2;i++){
    lloyd.axis[0].move(45);
    lloyd.runToPositions();
    lloyd.delay(2000);
    lloyd.axis[0].move(-45);
    lloyd.runToPositions();
    lloyd.delay(2000);
  }
}

