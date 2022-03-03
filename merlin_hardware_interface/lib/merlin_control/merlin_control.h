
#include <multi_axis.h>
#include <iostream>
#include <Eigen/Dense>
#include <pubSysCls.h>

// Use the Teknic library's namespace
using namespace sFnd;

class MR6200: MultiAxis<6>{
  private:
  INode* motors[6];
  SysManager& myMgr;
public:
MR6200();
~MR6200();

  double reductions[6][6] = {
      {1. / 48., 0, 0, 0, 0, 0},
      {0, 1. / 48., 0, 0, 0, 0},
      {0, -1. / 48., 1. / 48., 0, 0, 0},
      {0, 0, 0,                1. / 24.,   0, 0},
      {0, 0, 0,               -1. / 28.8, 1. / 28.8, 0},
      {0, 0, 0,               -1. / 12.,  1. / 24., 1. / 24.}};

void setHomeOffsets();

void setMotorReductions(double reductions[6][6]);

void updateMotorSpeeds(double *speeds) override;

void computeAxisPositions(double *poses) override;

unsigned int getTime() override;
};