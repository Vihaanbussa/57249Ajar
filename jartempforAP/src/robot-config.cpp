#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor RightMiddle = motor(PORT12, ratio6_1, true);
motor LeftMiddle = motor(PORT17, ratio6_1, false);
motor RightBack = motor(PORT14, ratio6_1, false);
motor RightFront = motor(PORT3, ratio6_1, false);
motor LeftFront = motor(PORT1, ratio6_1, true);
rotation horizEnc = rotation(PORT7, true);
motor LeftBack = motor(PORT16, ratio6_1, true);
controller Controller1 = controller(primary);
digital_out Pneumatic = digital_out(Brain.ThreeWirePort.A);
inertial Inertial = inertial(PORT11);
digital_out Arm = digital_out(Brain.ThreeWirePort.B);
digital_out IntakeLift = digital_out(Brain.ThreeWirePort.H);
digital_out hood = digital_out(Brain.ThreeWirePort.F);
limit IntakeSwitch = limit(Brain.ThreeWirePort.G);
motor Intake = motor(PORT21, ratio6_1, true);
motor ladybrownMotorA = motor(PORT20, ratio18_1, true);
motor ladybrownMotorB = motor(PORT10, ratio18_1, false);
motor_group ladybrown = motor_group(ladybrownMotorA, ladybrownMotorB);
rotation vertEncoder = rotation(PORT19, true);
rotation wallrotational = rotation(PORT6, false);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}