using namespace vex;

extern brain Brain;

// VEXcode devices
extern smartdrive Drivetrain;
extern motor FourBar;
extern motor ForkLift;
extern digital_out Clamp;
extern controller Mobile;
extern motor_group LeftDriveSmart; 
extern motor_group RightDriveSmart;
extern inertial Inertial; 
extern motor leftMotorA; 
extern motor leftMotorB; 
extern motor rightMotorA; 
extern motor rightMotorB; 


/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );