using namespace vex;

extern brain Brain;

// VEXcode devices
extern motor RightMiddle;
extern motor LeftMiddle;
extern motor RightBack;
extern motor RightFront;
extern motor LeftFront;
extern rotation horizEnc;
extern motor LeftBack;
extern controller Controller1;
extern digital_out Pneumatic;
extern inertial Inertial;
extern digital_out Arm;
extern digital_out IntakeLift;
extern digital_out hood;
extern limit IntakeSwitch;
extern motor Intake;
extern motor_group ladybrown;
extern rotation vertEncoder;
extern rotation wallrotational;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );