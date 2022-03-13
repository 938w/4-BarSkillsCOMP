#include "vex.h"

using namespace vex;

class pid {
private:
  void linedrive(double distance, double dir, double velocity,
                 double porportion, int errorcorrect);

public:
  void drive(double distance, double dir, double velocity, double porportion, int errorcorrect);
  void drive(double distance, double dir, double velocity, double porportion);
};