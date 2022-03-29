#include "pid2.h"
#include <cmath>

using namespace std;
double getposition() {
  return ((LeftDriveSmart.rotation(rotationUnits::deg) +
           RightDriveSmart.rotation(rotationUnits::deg)) /
          2) *
         12.57 / 360;
}

double signnum_c(double x) {
  if (x > 0.0)
    return 1.0;
  if (x < 0.0)
    return -1.0;
  return x;
}

// drive straight is PID but slow down is not
void pid2::drive(double target, double velocity, double slowDistance,
                 double dir) {
  Inertial.resetRotation();
  LeftDriveSmart.resetRotation();
  RightDriveSmart.resetRotation();
  double yaw = dir;
  if (dir == 999) {
    yaw = Inertial.yaw();
  }
  double error;
  double lasterror = 0;
  double derivative;
  double velocityFromTheError;
  double integral = 0;
  double p = 1;
  double i = 0;
  double d = 0.005;
  double accelerationStep = 1.6;
  int notMovingCount = 0;
  double lastPosition = 0;

  if (target > 0) {

    // going forward
    while (getposition() < target) {

      double position = getposition();

      if (lastPosition == position) {
        notMovingCount++;
      }

      lastPosition = position;

      // if stuck for more than 1sec
      if (notMovingCount > 100) {
        break;
      }

      error = Inertial.yaw() - yaw;
      derivative = error - lasterror;
      lasterror = error;
      integral += error;
      velocityFromTheError = (error * p + derivative * d + integral * i);

      if (getposition() > target - slowDistance) {
        velocity = velocity - accelerationStep;
      }
      if (velocity < 10)
        velocity = 10;
      Mobile.Screen.newLine();
      Mobile.Screen.print(velocityFromTheError);

      LeftDriveSmart.spin(fwd, velocity - velocityFromTheError, percent);
      RightDriveSmart.spin(fwd, velocity + velocityFromTheError, percent);

      wait(10, msec);
    }
  } else {
    // going backward
    while (getposition() > target) {

      double position = getposition();
      if (lastPosition == position) {
        notMovingCount++;
      }

      lastPosition = position;

      // if stuck for more than 1sec
      if (notMovingCount > 100) {
        break;
      }

      error = Inertial.yaw() - yaw;
      derivative = error - lasterror;
      lasterror = error;
      integral += error;
      velocityFromTheError = (error * p + derivative * d + integral * i);

      if (getposition() < target + slowDistance) {
        velocity = velocity - accelerationStep;
      }
      if (velocity < 10)
        velocity = 10;
      LeftDriveSmart.spin(reverse, velocity + velocityFromTheError, percent);
      RightDriveSmart.spin(reverse, velocity - velocityFromTheError, percent);

      wait(10, msec);
    }
  }

  LeftDriveSmart.stop(brakeType::brake);
  RightDriveSmart.stop(brakeType::brake);
};

void pid2::turn(double target) {
  //
  double error;
  double lasterror = 0;
  double derivative = 0;
  double velocityFromTheError;
  double integral = 0;
  int maxIntegral = 300;
  double p = 0.012;
  double i = 0.0001;
  double d = 0.01;

  double velocity = 12; // voltage 12v is max
  if (abs(target) <= 100 && abs(target) > 60) {
    velocity = abs(target) / 100 * 12 + 1;
  } else if (abs(target) <= 60 && abs(target) > 30) {
    velocity = abs(target) / 100 * 11 + 1;
  } else if (abs(target) <= 30) {
    velocity = abs(target) / 100 * 10 + 1;
  }
  int turnMode = 0; // 1 = right turn -1 = left turn

  if (target > 0) {
    turnMode = 1;
  } else if (target < 0) {
    turnMode = -1;
  } else {
    // do nothing
    Mobile.Screen.print("target is zero");
    return;
  }

  while (true) {
    error = Inertial.rotation() - target;
    if (lasterror != 0) {
      derivative = error - lasterror;
    }

    integral += error;

    // max
    if (abs(integral) > maxIntegral) {
      integral = signnum_c(integral) * maxIntegral;
    }

    lasterror = error;

    velocityFromTheError = error * p + derivative * d + integral * i;

    if (turnMode == 1) {
      // right turn
      if (Inertial.rotation() >= target - 1) {
        break;
      }
      velocity = velocity + velocityFromTheError;
      if (velocity < 1.45) {
        velocity = 1.45;
      }

      Mobile.Screen.newLine();
      Mobile.Screen.print(velocityFromTheError);

      LeftDriveSmart.spin(fwd, velocity, voltageUnits::volt);
      RightDriveSmart.spin(reverse, velocity, voltageUnits::volt);

    } else if (turnMode == -1) {
      // left turn
      if (Inertial.rotation() <= target + 1) {
        break;
      }

      velocity = velocity - velocityFromTheError;
      if (velocity < 1.45)
        velocity = 1.45;

      Mobile.Screen.newLine();
      Mobile.Screen.print(velocity);

      LeftDriveSmart.spin(reverse, velocity, voltageUnits::volt);
      RightDriveSmart.spin(fwd, velocity, voltageUnits::volt);
    }

    wait(10, msec);
  }
  Inertial.resetRotation();
  LeftDriveSmart.stop(brakeType::brake);
  RightDriveSmart.stop(brakeType::brake);
};