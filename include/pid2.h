#include "vex.h"

using namespace vex;

class pid2 {
public:
  void drive(double target, double velocity, double slowDistance = 16, double dir = 999);
  void turn(double target);
};