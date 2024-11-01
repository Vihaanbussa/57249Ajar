#include "vex.h"
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// RightMiddle          motor         12              
// LeftMiddle           motor         17              
// RightBack            motor         14              
// RightFront           motor         3               
// LeftFront            motor         1               
// horizEnc             rotation      7               
// LeftBack             motor         16              
// Controller1          controller                    
// Pneumatic            digital_out   A               
// Inertial             inertial      18              
// Arm                  digital_out   B               
// IntakeLift           digital_out   H               
// hood                 digital_out   F               
// IntakeSwitch         limit         G               
// Intake               motor         21              
// ladybrown            motor_group   20, 10          
// vertEncoder          rotation      19              
// wallrotational       rotation      6               
// ---- END VEXCODE CONFIGURED DEVICES ----

using namespace vex;
competition Competition;
bool turn;
bool cata;



/*---------------------------------------------------------------------------*/
/*                             VEXcode Config                                */
/*                                                                           */
/*  Before you do anything else, start by configuring your motors and        */
/*  sensors using the V5 port icon in the top right of the screen. Doing     */
/*  so will update robot-config.cpp and robot-config.h automatically, so     */
/*  you don't have to.                                                       */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/*                             JAR-Template Config                           */
/*                                                                           */
/*  Where all the magic happens. Follow the instructions below to input      */
/*  all the physical constants and values for your robot. You should         */
/*  already have configured your robot manually with the sidebar configurer. */
/*---------------------------------------------------------------------------*/

Drive chassis(

//Specify your drive setup below. There are seven options:
//ZERO_TRACKER, TANK_ONE_ENCODER, TANK_ONE_ROTATION, TANK_TWO_ENCODER, TANK_TWO_ROTATION, HOLONOMIC_TWO_ENCODER, and HOLONOMIC_TWO_ROTATION
//For example, if you are not using odometry, put ZERO_TRACKER below:
ZERO_TRACKER,

//Add the names of your Drive motors into the motor groups below, separated by commas, i.e. motor_group(Motor1,Motor2,Motor3).
//You will input whatever motor names you chose when you configured your robot using the sidebar configurer, they don't have to be "Motor1" and "Motor2".

//Left Motors:
motor_group(LeftFront, LeftMiddle, LeftBack),

//Right Motors:
motor_group(RightFront, RightMiddle, RightBack),

//Specify the PORT NUMBER of your inertial sensor, in PORT format (i.e. "PORT1", not simply "1"):
PORT18,

//Input your wheel diameter. (4" omnis are actually closer to 4.125"):
3.25,

//External ratio, must be in decimal, in the format of input teeth/output teeth.
//If your motor has an 84-tooth gear and your wheel has a 60-tooth gear, this value will be 1.4.
//If the motor drives the wheel directly, this value is 1:
0.6,

//Gyro scale, this is what your gyro reads when you spin the robot 360 degrees.
//For most cases 360 will do fine here, but this scale factor can be very helpful when precision is necessary.
360,

/*---------------------------------------------------------------------------*/
/*                                  PAUSE!                                   */
/*                                                                           */
/*  The rest of the drive constructor is for robots using POSITION TRACKING. */
/*  If you are not using position tracking, leave the rest of the values as  */
/*  they are.                                                                */
/*---------------------------------------------------------------------------*/

//PAUSE! The rest of the drive constructor is for robot using POSITION TRACKING.
//If you are not using position tracking, leave the rest of the values as they are.

//Input your drive motors by position. This is only necessary for holonomic drives, otherwise this section can be left alone.
//LF:      //RF:    
PORT1,     -PORT2,

//LB:      //RB: 
PORT3,     -PORT4,

//If you are using position tracking, this is the Forward Tracker port (the tracker which runs parallel to the direction of the chassis).
//If this is a rotation sensor, leave it in "PORT1" format, inputting the port below.
//If this is an encoder, enter the port as an integer. Triport A will be a "1", Triport B will be a "2", etc.
3,

//Input the Forward Tracker diameter (reverse it to make the direction switch):
2.75,

//Input Forward Tracker center distance (a positive distance corresponds to a tracker on the right side of the robot, negative is left.)
//This distance is in inches:
-2,

//Input the Sideways Tracker Port, following the same steps as the Forward Tracker Port:
1,

//Sideways tracker diameter (reverse to make the direction switch):
-2.75,

//Sideways tracker center distance (positive distance is behind the center of the robot, negative is in front):
5.5

);


bool vertwingposition = false;
bool armposition = false;
bool intakelift = false;
bool hoodposition = false;
bool armbelow = true;

void vertwings() {
  vertwingposition = !vertwingposition;
  Pneumatic.set(vertwingposition);
}

void driveForward(float x) {
  LeftFront.spinFor(forward, x, turns, false);
  RightFront.spinFor(forward, x, turns, false);
  LeftBack.spinFor(forward, x, turns, false);
  RightBack.spinFor(forward, x, turns, false);
  LeftMiddle.spinFor(forward, x, turns, false); 
  RightMiddle.spinFor(forward, x, turns, true);
}
void driveReverse(float x) {
  LeftFront.spinFor(reverse, x, turns, true);
  RightFront.spinFor(reverse, x, turns, true);
  LeftBack.spinFor(reverse, x, turns, true);
  RightBack.spinFor(reverse, x, turns, true);
  LeftMiddle.spinFor(reverse, x, turns, true);
  RightMiddle.spinFor(reverse, x, turns, true);
}
void turnRight(float x){
  LeftFront.spinFor(reverse, x, degrees, false);
  RightFront.spinFor(forward, x, degrees, false);
  LeftMiddle.spinFor(reverse, x, degrees, false);
  RightMiddle.spinFor(forward, x, degrees, false);
  LeftBack.spinFor(reverse, x, degrees, false);
  RightBack.spinFor(forward, x, degrees, true);
}
void turnLeft(float x){
  LeftFront.spinFor(reverse, x, degrees, true);
  RightFront.spinFor(reverse, x, degrees, true);
  LeftMiddle.spinFor(reverse, x, degrees, true);
  RightMiddle.spinFor(reverse, x, degrees, true);
  LeftBack.spinFor(reverse, x, degrees, true);
  RightBack.spinFor(reverse, x, degrees, true);
}

void resetarm(){
  cata = true;
  while (cata == true) {
    ladybrown.spin(forward);
      if (ladybrown.position(rev) >= 0.565  && cata == true){
        ladybrown.stop(); 
        cata = false;
      }
  }
}
event Event = event(resetarm);
void clamp(){
  vertwingposition = !vertwingposition;
  wait(0.2, sec);
  Pneumatic.set(vertwingposition);
}
void arm(){
  armposition = !armposition;
  wait(0.2, sec);
  Arm.set(armposition);
}
void intlift(){
  intakelift = !intakelift;
  wait(0.2, sec);
  IntakeLift.set(intakelift);
}
void hoodlift(){
  hoodposition = !hoodposition;
  wait(0.2, sec);
  hood.set(hoodposition);
}



/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

int current_auton_selection = 0;
bool auto_started = false;

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  default_constants();
  //sets motor velocities and stopping
  RightFront.setStopping(coast);
  RightMiddle.setStopping(coast);
  RightBack.setStopping(coast);
  LeftFront.setStopping(coast);
  LeftMiddle.setStopping(coast);
  LeftBack.setStopping(coast);
  Intake.setVelocity(100, percent);
  ladybrown.setStopping(hold);
  RightFront.setVelocity(100, percent);
  RightMiddle.setVelocity(100, percent);
  RightBack.setVelocity(100, percent);
  LeftFront.setVelocity(100, percent);
  LeftMiddle.setVelocity(100, percent);
  LeftBack.setVelocity(100, percent);
  
  //switch function for auton selection
  while(auto_started == false){
    Brain.Screen.clearScreen();
    switch(current_auton_selection){
      case 0:
        Brain.Screen.printAt(50, 50, "OffRoller");
        break;
      case 1:
        Brain.Screen.printAt(50, 50, "OnRoller");
        break;
      case 2:
        Brain.Screen.printAt(50, 50, "93 point Skills");
        break;
      case 3:
        Brain.Screen.printAt(50, 50, "Full Skills");
        break;
    }
    if(Brain.Screen.pressing()){
      while(Brain.Screen.pressing()) {}
      current_auton_selection ++;
    } else if (current_auton_selection == 7){
      current_auton_selection = 0;
    }
    task::sleep(10);
  }
}

void autonomous(void) {
  // 1 tile for drive_distance should have value of 75
  // 90 degree rotation (clockwise with intake facing 12-o'clock) for turn_to_angle should have a value of 100
  auto_started = true;
  switch(current_auton_selection){  
    case 0: //redpos (forward = 30)
      chassis.set_drive_exit_conditions(9, 200, 1750);
      chassis.set_turn_exit_conditions(1, 300, 1250);
      chassis.set_swing_exit_conditions(1, 300, 1000);
      chassis.drive_distance(-30, 0, 5, 9);
      Pneumatic.set(true);
      Intake.setVelocity(100, percent);
      Intake.spin(forward);
      chassis.turn_to_angle(300);
      chassis.drive_distance(20);
      chassis.drive_distance(-20);
      chassis.turn_to_angle(75);
      Intake.spin(forward);
      Pneumatic.set(false);
      IntakeLift.set(true);
      chassis.drive_distance(28);
      IntakeLift.set(false);
      chassis.drive_distance(-10);
      Intake.stop();
      chassis.turn_to_angle(20);
      chassis.drive_distance(13);
      chassis.turn_to_angle(120);
      chassis.drive_distance(14.75, 120, 6, 9);
      chassis.set_turn_exit_conditions(1, 300, 1750);
      chassis.turn_to_angle(210, 7);
      break;
    case 1: //blueneg
      chassis.set_drive_exit_conditions(9, 200, 1000);
      chassis.set_turn_exit_conditions(1, 300, 1000);
      chassis.set_swing_exit_conditions(1, 300, 1000);
      chassis.drive_distance(-9);
      chassis.left_swing_to_angle(270);
      Intake.spin(forward);
      wait(1.25, sec);
      Pneumatic.set(true);
      chassis.drive_distance(-10);
      wait(1, sec);
      Intake.spin(reverse);
      chassis.drive_distance(20);
      chassis.turn_to_angle(-222);
      Intake.stop();
      Pneumatic.set(false);
      chassis.set_drive_exit_conditions(9, 200, 1500);
      chassis.drive_distance(-32);
      Pneumatic.set(true);
      Intake.spin(forward);
      chassis.turn_to_angle(-340);
      Intake.spin(forward);
      chassis.drive_distance(20);
      chassis.drive_distance(-2);
      chassis.turn_to_angle(-90);
      chassis.drive_distance(16);
      chassis.drive_distance(-18);
      chassis.turn_to_angle(-72);
      chassis.drive_distance(20);
      wait(0.75, sec);
      chassis.drive_distance(-18);
      chassis.turn_to_angle(-180);
      chassis.drive_distance(35);
      break;
    case 2: //redwin
      chassis.set_drive_exit_conditions(9, 200, 1000);
      chassis.set_turn_exit_conditions(1, 300, 1000);
      chassis.drive_distance(-19);
      Intake.spin(reverse, 70, percent);
      wait(1, sec);
      chassis.drive_distance(20);
      chassis.turn_to_angle(17);
      chassis.set_drive_exit_conditions(9, 200, 1500);
      Intake.spin(forward, 60, percent);
      chassis.drive_distance(45);
      chassis.turn_to_angle(140);
      Intake.stop();
      chassis.drive_distance(-19);
      Pneumatic.set(true);
      Intake.spin(forward, 100, percent);
      chassis.drive_distance(18);
      wait(1.2, sec);
      Pneumatic.set(false);
      chassis.drive_distance(12);
      chassis.turn_to_angle(60);
      chassis.drive_distance(-35);
      Pneumatic.set(true);
      chassis.turn_to_angle(179);
      IntakeLift.set(true);
      chassis.drive_distance(26);
      IntakeLift.set(false);
      wait(0.3, sec);
      chassis.drive_distance(-15);
      wait(1, sec);
      Pneumatic.set(false);
      chassis.turn_to_angle(270);
      chassis.drive_distance(14);
      break;
    case 3: //bluewin
      chassis.set_drive_exit_conditions(9, 200, 1000);
      chassis.set_turn_exit_conditions(1, 300, 1000);
      chassis.drive_distance(-19);
      Intake.spin(reverse, 70, percent);
      wait(1, sec);
      chassis.drive_distance(20);
      chassis.turn_to_angle(-17);
      chassis.set_drive_exit_conditions(9, 200, 1500);
      Intake.spin(forward, 60, percent);
      chassis.drive_distance(45);
      chassis.turn_to_angle(-140);
      Intake.stop();
      chassis.drive_distance(-19);
      Pneumatic.set(true);
      Intake.spin(forward, 100, percent);
      chassis.drive_distance(18);
      wait(1.2, sec);
      Pneumatic.set(false);
      chassis.drive_distance(12);
      chassis.turn_to_angle(-60);
      chassis.drive_distance(-35);
      Pneumatic.set(true);
      chassis.turn_to_angle(-179);
      IntakeLift.set(true);
      chassis.drive_distance(26);
      IntakeLift.set(false);
      wait(0.5, sec);
      chassis.drive_distance(-15);
      wait(1, sec);
      Pneumatic.set(false);
      chassis.turn_to_angle(-270);
      chassis.drive_distance(14);
      break;
    case 4: //redpos
      chassis.set_drive_exit_conditions(4, 200, 1500);
      chassis.set_turn_exit_conditions(1, 300, 1500);
      chassis.drive_distance(-30);
      chassis.turn_to_angle(-25);
      chassis.drive_distance(-17.5);
      Pneumatic.set(true);
      Intake.spin(forward);
      chassis.drive_distance(3);
      chassis.turn_to_angle(-350);
      chassis.drive_distance(15);
      chassis.turn_to_angle(-310);
      Intake.spin(reverse);
      Pneumatic.set(false);                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      
      chassis.set_turn_exit_conditions(1, 300, 1500);
      chassis.turn_to_angle(-94);
      chassis.drive_distance(-19);
      Pneumatic.set(true);
      chassis.turn_to_angle(-315);
      Intake.spin(forward);
      intlift();
      chassis.drive_distance(26);
      intlift();                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           chassis.drive_distance(-10);
      wait(2, sec);
      chassis.drive_distance(-10);
      wait(0.5, sec);
      chassis.turn_to_angle(-233);
      chassis.drive_distance(17);
      break;
    case 5: //bluepos
      chassis.set_drive_exit_conditions(4, 200, 1500);
      chassis.set_turn_exit_conditions(1, 300, 1500);
      chassis.drive_distance(-30);
      chassis.turn_to_angle(25);
      chassis.drive_distance(-17.5);
      Pneumatic.set(true);
      Intake.spin(forward);
      chassis.drive_distance(3);
      chassis.turn_to_angle(348);
      chassis.drive_distance(15);
      chassis.turn_to_angle(310);
      Intake.spin(reverse);
      Pneumatic.set(false);                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      
      chassis.set_turn_exit_conditions(1, 300, 1500);
      chassis.turn_to_angle(90);
      chassis.drive_distance(-19);
      Pneumatic.set(true);
      chassis.turn_to_angle(309);
      Intake.spin(forward);
      intlift();
      chassis.drive_distance(26);
      intlift();                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           chassis.drive_distance(-10);
      wait(1.5, sec);
      chassis.drive_distance(-10);
      wait(0.5, sec);
      chassis.turn_to_angle(233);
      chassis.drive_distance(17);
      break;
    case 6:
      chassis.set_drive_exit_conditions(9, 200, 1250);
      chassis.set_turn_exit_conditions(1, 300, 1000);
      chassis.set_swing_exit_conditions(1, 300, 1000);
      chassis.drive_distance(-35);
      Pneumatic.set(true);
      Intake.spin(forward);
      wait(0.5, sec);
      chassis.turn_to_angle(87);
      chassis.drive_distance(25);
      chassis.turn_to_angle(135);
      chassis.set_drive_exit_conditions(9, 200, 1500);
      chassis.drive_distance(-30);
      chassis.turn_to_angle(270);
      Pneumatic.set(false);
      intlift();
      chassis.drive_distance(25);
      intlift();
      wait(0.1, sec);
      chassis.drive_distance(-20);
      Intake.stop();
      chassis.turn_to_angle(0);
      chassis.drive_distance(10);
      chassis.turn_to_angle(90);
      chassis.drive_distance(-17.5);
      chassis.right_swing_to_angle(180);
      chassis.drive_distance(-10);
      Intake.spin(forward);
      wait(1, sec);
      Pneumatic.set(true);
      chassis.drive_distance(-5);
      break;
  }
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
  Event(resetarm);
  while (1) {
    
    //sets motor presets needed
    RightFront.setStopping(coast);
    RightMiddle.setStopping(coast);
    RightBack.setStopping(coast);
    LeftFront.setStopping(coast);
    LeftMiddle.setStopping(coast);
    LeftBack.setStopping(coast);
    Intake.setVelocity(100, percent);
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................

    // Intake (spins the intake | NOT TOGGLE, MUST HOLD)
    if(Controller1.ButtonL2.pressing()){
      Intake.setVelocity(100, percent);
      Intake.spin(reverse);
    }
    else if(Controller1.ButtonL1.pressing()){
      Intake.setVelocity(100, percent);
      Intake.spin(forward);
    }
    else{
      Intake.stop();
    }

    if(Controller1.ButtonDown.pressing()){
      ladybrown.setVelocity(100, percent);
      ladybrown.spin(reverse);
    }
    else if(Controller1.ButtonUp.pressing()){
      ladybrown.setVelocity(100, percent);
      ladybrown.spin(forward);
    }
    else{
      ladybrown.stop();
    }

    //expansion (makes it so that both ButtonB and ButtonDown must be pressed in order to launch expansion. Used as safety mechanism)
    if (Controller1.ButtonR2.pressing()) {
      clamp();
    }

    if(Controller1.ButtonR1.pressing()) {
      arm();
    }

    if(Controller1.ButtonB.pressing()) {
      ladybrown.setVelocity(50, percent);
    }

    if(Controller1.ButtonA.pressing()){
      ladybrown.setVelocity(50, percent);
      Event.broadcast();
    }

    if(Controller1.ButtonX.pressing()){
      intlift();
    }



    //expansion (makes it so that both ButtonB and ButtonDown must be pressed in order to launch expansion. Used as safety mechanism)

    
    //Drivetrain
    //Replace this line with chassis.control_tank(); for tank drive.
    chassis.control_tank();

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