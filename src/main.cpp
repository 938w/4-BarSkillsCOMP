/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "pid.h"
#include "pid2.h"
#include "vex.h"
using namespace vex;

double angle = 110;
// A global instance of competition
competition Competition;
/*
  void eztune () {
  bool last = false;
  bool loop = true;

  while (loop) {
    Mobile.Screen.newLine();
    Mobile.Screen.print(angle);
    Mobile.Screen.print(":first neutral | [A] to exit");
    if (Mobile.ButtonUp.pressing()&&!last){
      angle += 0.5;
      last = true;
    } else if(!Mobile.ButtonUp.pressing()) {
      last = false;
    }
    if (Mobile.ButtonA.pressing()) {
      loop = false;
    }
    wait(20, msec);


  }
}
*/

// define your global instances of motors and other devices here

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {

  // ROUTE2
  Mobile.Screen.print(Inertial.yaw());
  
  timer t1;
  timer t2;
  // Logger logger("route2");
  // logger.log("start the run");
  pid2 PID;
  t2.reset();
  Brain.Screen.print("route 2");

  // Set Velocity
  leftMotorA.setVelocity(100, percent);
  leftMotorB.setVelocity(100, percent);
  rightMotorA.setVelocity(100, percent);
  rightMotorB.setVelocity(100, percent);
  // Lower fork lift

  ForkLift.spinFor(reverse, 530, degrees, 100, rpm);
  ForkLift.spinFor(forward, 10, degrees, false);
  // Drive to blue goal
  Drivetrain.driveFor(forward, 16, inches, 140, rpm);
  Drivetrain.stop();
  // Pick up
  ForkLift.spinFor(forward, 350, degrees, 100, rpm);
  // drive backwards a little bit
  // Drivetrain.driveFor(reverse, 16, inches, 120, rpm);
  PID.drive(-16.5, 100, 16, 0);
  // Turn 90 degrees to face neutral goal
  Drivetrain.turnToHeading(109, degrees, 200, rpm);

  // Activate clamp
  Clamp.set(true);

  // Drive to neutral goal
  // Drivetrain.driveFor(reverse, 50, inches, 80, rpm);
  PID.drive(-50, 100);

  Drivetrain.stop(brake);

  // Clamp neutral
  Clamp.set(false);
  wait(0.8, sec);

  // go to platform
  FourBar.spinFor(forward, 770, degrees, 100, rpm, false);
  Drivetrain.turnToHeading(123, degrees);
  wait(0.2, sec);
  // Drivetrain.driveFor(reverse, 54, inches, 100,rpm);
  PID.drive(-54, 100);
  FourBar.spinFor(reverse, 75, degrees, false);
  // Turn so goal is closer to the platforms center of gravity
  // Drivetrain.turnFor(right, 25, degrees);
  // Lower fourbar so goal doesn't tip over while dropping

  FourBar.spinFor(reverse, 240, degrees);
  // faster

  LeftDriveSmart.rotateFor(reverse, 40, degrees, 160, rpm, false);
  RightDriveSmart.rotateFor(forward, 40, degrees, 160, rpm);
  Clamp.set(true);
  Drivetrain.turnToHeading(123, degrees);

  // Release
  // Clamp.set(true);
  // getbluegoal
  // Drivetrain.turnFor(right, 13, degrees);
  t1.reset();
  // placebluegoal and move forward while loweringfourbar
  ForkLift.spinFor(reverse, 360, degrees, 70, rpm, false);
  FourBar.spinFor(reverse, 520, degrees, false);
  Drivetrain.driveFor(forward, 27, inches, 120, rpm);

  Drivetrain.stop(brake);
  wait(0.4, sec);
  // moveback
  //Drivetrain.driveFor(reverse, 16, inches, 175, rpm);
  PID.drive(-16, 100, 12);
  Drivetrain.turnToHeading(303, degrees, 100, rpm);
  Mobile.Screen.newLine();
  Mobile.Screen.print(Inertial.heading());
  
  // moveforward

  PID.drive(-21, 80);
  Mobile.Screen.newLine();
  Mobile.Screen.print(t1.time(sec));
  // clamp
  Clamp.set(false);
  FourBar.spinFor(forward, 200, degrees);
  // turn to red
  Drivetrain.turnToHeading(-132, degrees, 150, rpm);
  Mobile.Screen.newLine();
  Mobile.Screen.print(Inertial.yaw());
  // drive to red
  ForkLift.spinFor(reverse, 15, degrees, false);
  // Drivetrain.driveFor(forward, 34.5, inches, 100, rpm);  
  PID.drive(31, 100);
  // pick up red
  ForkLift.spinFor(forward, 350, degrees, 100, rpm, false);
  wait(0.5, sec);
  FourBar.spinFor(forward, 460, degrees, 100, rpm, false);
  // Drivetrain.driveFor(reverse, 26, inches, 100, rpm);
  PID.drive(-27.5, 100);
  // turn to ramp

  Drivetrain.turnToHeading(140, degrees, 200, rpm);
  // go to ramp
  // Drivetrain.driveFor(reverse, 50, inches, 100, rpm);
  PID.drive(-39, 75);
  FourBar.spinFor(reverse, 90, degrees, 100, rpm);
  // turn to ramp
  LeftDriveSmart.rotateFor(reverse, 82, degrees, 160, rpm, false);
  RightDriveSmart.rotateFor(forward, 82, degrees, 160, rpm);
  Drivetrain.driveFor(reverse, 4, inches);
  Clamp.set(true);
  wait(0.2, sec);

  // drive out

  Drivetrain.turnToHeading(90, degrees, 100, rpm);
  FourBar.spinTo(0, degrees, 100, rpm, false);

  
  // Drivetrain.driveFor(forward, 10, inches, 180, rpm);

  // push middle goal
  //Drivetrain.driveFor(forward, 58, inches, 120, rpm);
  PID.drive(58, 100, 16, 90);
  // turn to neutral

  Drivetrain.turnToHeading(148, degrees, 160, rpm);

  // go to neutral
  // Drivetrain.driveFor(reverse, 40, inches, 160, rpm);
  PID.drive(-40, 100);
  // p
  Clamp.set(false);
  // turnaround
  wait(0.4, sec);
  FourBar.spinFor(forward, 670, degrees, false);

  Drivetrain.turnToHeading(-60, degrees, 140, rpm);

  PID.drive(-44, 100);
  FourBar.spinFor(reverse, 150, degrees, 100, rpm, false);
  // Drivetrain.turnFor(right, 15, degrees);
  LeftDriveSmart.rotateFor(forward, 95, degrees, 160, rpm, false);
  RightDriveSmart.rotateFor(reverse, 95, degrees, 160, rpm);
  // Lower fourbar

  Drivetrain.driveFor(reverse, 5.5, inches, 200, rpm);
  // release
  Clamp.set(true);
  wait(0.5, sec);
  Drivetrain.driveFor(forward, 4, inches, 200, rpm);
  LeftDriveSmart.rotateFor(forward, 120, degrees, 160, rpm, false);
  RightDriveSmart.rotateFor(reverse, 120, degrees, 160, rpm);
  /*
  // drive

  ForkLift.spinFor(reverse, 360, degrees, 70, rpm, false);
  Drivetrain.driveFor(forward, 10, inches, 120, rpm);
  FourBar.spinTo(0, degrees, false);
  Drivetrain.driveFor(forward, 16, inches, 120, rpm);
  Drivetrain.stop(brake);
  wait(0.3, sec);
  // moveback
  Drivetrain.driveFor(reverse, 12, inches, 175, rpm);
  Drivetrain.turnFor(180, degrees, 100, rpm);
  // moveforward

  PID.drive(-17, Inertial.yaw(), -80, 1);
  Mobile.Screen.newLine();
  Mobile.Screen.print(t1.time(sec));
  Clamp.set(false);
  // liftup and turnaround
  FourBar.spinFor(forward, 520, degrees, false);
  wait(0.4, sec);
  Drivetrain.turnToHeading(-60, degrees);
  Drivetrain.driveFor(reverse, 33, inches, 200, rpm);
  Clamp.set(true);
  */

  Mobile.Screen.newLine();
  Mobile.Screen.print("L");
  Mobile.Screen.print(t2.time(sec));
  Mobile.rumble(".-.-");

  // logger.log("end the run");
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  // User control code here, inside the loop
  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
