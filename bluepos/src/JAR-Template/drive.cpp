#include "vex.h"
controller Controller = controller();

Drive::Drive(enum::drive_setup drive_setup, motor_group DriveL, motor_group DriveR, int gyro_port, float wheel_diameter, float wheel_ratio, float gyro_scale, int DriveLF_port, int DriveRF_port, int DriveLB_port, int DriveRB_port, int ForwardTracker_port, float ForwardTracker_diameter, float ForwardTracker_center_distance, int SidewaysTracker_port, float SidewaysTracker_diameter, float SidewaysTracker_center_distance) :
  wheel_diameter(wheel_diameter),
  wheel_ratio(wheel_ratio),
  gyro_scale(gyro_scale),
  drive_in_to_deg_ratio(wheel_ratio/360.0*M_PI*wheel_diameter),
  ForwardTracker_center_distance(ForwardTracker_center_distance),
  ForwardTracker_diameter(ForwardTracker_diameter),
  ForwardTracker_in_to_deg_ratio(M_PI*ForwardTracker_diameter/360.0),
  SidewaysTracker_center_distance(SidewaysTracker_center_distance),
  SidewaysTracker_diameter(SidewaysTracker_diameter),
  SidewaysTracker_in_to_deg_ratio(M_PI*SidewaysTracker_diameter/360.0),
  drive_setup(drive_setup),
  DriveL(DriveL),
  DriveR(DriveR),
  Gyro(inertial(gyro_port)),
  DriveLF(DriveLF_port, is_reversed(DriveLF_port)),
  DriveRF(DriveRF_port, is_reversed(DriveRF_port)),
  DriveLB(DriveLB_port, is_reversed(DriveLB_port)),
  DriveRB(DriveRB_port, is_reversed(DriveRB_port)),
  R_ForwardTracker(ForwardTracker_port),
  R_SidewaysTracker(SidewaysTracker_port),
  E_ForwardTracker(ThreeWire.Port[ForwardTracker_port-1]),
  E_SidewaysTracker(ThreeWire.Port[SidewaysTracker_port-1])
{
  if (drive_setup != ZERO_TRACKER){
    if (drive_setup == TANK_ONE_ENCODER || drive_setup == TANK_ONE_ROTATION){
      odom.set_physical_distances(ForwardTracker_center_distance, 0);
    } else {
      odom.set_physical_distances(ForwardTracker_center_distance, SidewaysTracker_center_distance);
    }
    odom_task = task(position_track_task);
    set_coordinates(0,0,0);
  }
}

void Drive::drive_with_voltage(float leftVoltage, float rightVoltage){
  DriveL.spin(fwd, leftVoltage, volt);
  DriveR.spin(fwd, rightVoltage,volt);
}

void Drive::set_turn_constants(float turn_max_voltage, float turn_kp, float turn_ki, float turn_kd, float turn_starti){
  this->turn_max_voltage = turn_max_voltage;
  this->turn_kp = turn_kp;
  this->turn_ki = turn_ki;
  this->turn_kd = turn_kd;
  this->turn_starti = turn_starti;
} 

void Drive::set_drive_constants(float drive_max_voltage, float drive_kp, float drive_ki, float drive_kd, float drive_starti){
  this->drive_max_voltage = drive_max_voltage;
  this->drive_kp = drive_kp;
  this->drive_ki = drive_ki;
  this->drive_kd = drive_kd;
  this->drive_starti = drive_starti;
} 

void Drive::set_heading_constants(float heading_max_voltage, float heading_kp, float heading_ki, float heading_kd, float heading_starti){
  this->heading_max_voltage = heading_max_voltage;
  this->heading_kp = heading_kp;
  this->heading_ki = heading_ki;
  this->heading_kd = heading_kd;
  this->heading_starti = heading_starti;
}

void Drive::set_swing_constants(float swing_max_voltage, float swing_kp, float swing_ki, float swing_kd, float swing_starti){
  this->swing_max_voltage = swing_max_voltage;
  this->swing_kp = swing_kp;
  this->swing_ki = swing_ki;
  this->swing_kd = swing_kd;
  this->swing_starti = swing_starti;
} 

void Drive::set_turn_exit_conditions(float turn_settle_error, float turn_settle_time, float turn_timeout){
  this->turn_settle_error = turn_settle_error;
  this->turn_settle_time = turn_settle_time;
  this->turn_timeout = turn_timeout;
}

void Drive::set_drive_exit_conditions(float drive_settle_error, float drive_settle_time, float drive_timeout){
  this->drive_settle_error = drive_settle_error;
  this->drive_settle_time = drive_settle_time;
  this->drive_timeout = drive_timeout;
}

void Drive::set_swing_exit_conditions(float swing_settle_error, float swing_settle_time, float swing_timeout){
  this->swing_settle_error = swing_settle_error;
  this->swing_settle_time = swing_settle_time;
  this->swing_timeout = swing_timeout;
}

float Drive::get_absolute_heading(){ 
  return( reduce_0_to_360( Gyro.rotation()*360.0/gyro_scale ) ); 
}

float Drive::get_left_position_in(){
  return( DriveL.position(deg)*drive_in_to_deg_ratio );
}

float Drive::get_right_position_in(){
  return( DriveL.position(deg)*drive_in_to_deg_ratio );
}

void Drive::turn_to_angle(float angle){
  turn_to_angle(angle, turn_max_voltage, turn_settle_error, turn_settle_time, turn_timeout, turn_kp, turn_ki, turn_kd, turn_starti);
}

void Drive::turn_to_angle(float angle, float turn_max_voltage){
  turn_to_angle(angle, turn_max_voltage, turn_settle_error, turn_settle_time, turn_timeout, turn_kp, turn_ki, turn_kd, turn_starti);
}

void Drive::turn_to_angle(float angle, float turn_max_voltage, float turn_settle_error, float turn_settle_time, float turn_timeout){
  turn_to_angle(angle, turn_max_voltage, turn_settle_error, turn_settle_time, turn_timeout, turn_kp, turn_ki, turn_kd, turn_starti);
}

void Drive::turn_to_angle(float angle, float turn_max_voltage, float turn_settle_error, float turn_settle_time, float turn_timeout, float turn_kp, float turn_ki, float turn_kd, float turn_starti){
  desired_heading = angle;
  PID turnPID(reduce_negative_180_to_180(angle - get_absolute_heading()), turn_kp, turn_ki, turn_kd, turn_starti, turn_settle_error, turn_settle_time, turn_timeout);
  while(turnPID.is_settled() == false){
    float error = reduce_negative_180_to_180(angle - get_absolute_heading());
    float output = turnPID.compute(error);
    output = clamp(output, -turn_max_voltage, turn_max_voltage);
    drive_with_voltage(output, -output);
    task::sleep(9);
  }
  DriveL.stop(hold);
  DriveR.stop(hold);
}

void Drive::drive_distance(float distance){
  drive_distance(distance, desired_heading, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_distance(float distance, float heading){
  drive_distance(distance, heading, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_distance(float distance, float heading, float drive_max_voltage, float heading_max_voltage){
  drive_distance(distance, heading, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_distance(float distance, float heading, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout){
  drive_distance(distance, heading, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_distance(float distance, float heading, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout, float drive_kp, float drive_ki, float drive_kd, float drive_starti, float heading_kp, float heading_ki, float heading_kd, float heading_starti){
  desired_heading = heading;
  PID drivePID(distance, drive_kp, drive_ki, drive_kd, drive_starti, drive_settle_error, drive_settle_time, drive_timeout);
  PID headingPID(reduce_negative_180_to_180(heading - get_absolute_heading()), heading_kp, heading_ki, heading_kd, heading_starti);
  float start_average_position = (get_left_position_in()+get_right_position_in())/2.0;
  float average_position = start_average_position;
  while(drivePID.is_settled() == false){
    average_position = (get_left_position_in()+get_right_position_in())/2.0;
    float drive_error = distance+start_average_position-average_position;
    float heading_error = reduce_negative_180_to_180(heading - get_absolute_heading());
    float drive_output = drivePID.compute(drive_error);
    float heading_output = headingPID.compute(heading_error);

    drive_output = clamp(drive_output, -drive_max_voltage, drive_max_voltage);
    heading_output = clamp(heading_output, -heading_max_voltage, heading_max_voltage);

    drive_with_voltage(drive_output+heading_output, drive_output-heading_output);
    task::sleep(8);
  }
  DriveL.stop(hold);
  DriveR.stop(hold);
}

void Drive::left_swing_to_angle(float angle){
  left_swing_to_angle(angle, swing_max_voltage, swing_settle_error, swing_settle_time, swing_timeout, swing_kp, swing_ki, swing_kd, swing_starti);
}

void Drive::left_swing_to_angle(float angle, float swing_max_voltage, float swing_settle_error, float swing_settle_time, float swing_timeout, float swing_kp, float swing_ki, float swing_kd, float swing_starti){
  desired_heading = angle;
  PID swingPID(reduce_negative_180_to_180(angle - get_absolute_heading()), swing_kp, swing_ki, swing_kd, swing_starti, swing_settle_error, swing_settle_time, swing_timeout);
  while(swingPID.is_settled() == false){
    float error = reduce_negative_180_to_180(angle - get_absolute_heading());
    float output = swingPID.compute(error);
    output = clamp(output, -turn_max_voltage, turn_max_voltage);
    DriveL.spin(fwd, output, volt);
    DriveR.stop(hold);
    task::sleep(10);
  }
  DriveL.stop(hold);
  DriveR.stop(hold);
}

void Drive::right_swing_to_angle(float angle){
  right_swing_to_angle(angle, swing_max_voltage, swing_settle_error, swing_settle_time, swing_timeout, swing_kp, swing_ki, swing_kd, swing_starti);
}

void Drive::right_swing_to_angle(float angle, float swing_max_voltage, float swing_settle_error, float swing_settle_time, float swing_timeout, float swing_kp, float swing_ki, float swing_kd, float swing_starti){
  desired_heading = angle;
  PID swingPID(reduce_negative_180_to_180(angle - get_absolute_heading()), swing_kp, swing_ki, swing_kd, swing_starti, swing_settle_error, swing_settle_time, swing_timeout);
  while(swingPID.is_settled() == false){
    float error = reduce_negative_180_to_180(angle - get_absolute_heading());
    float output = swingPID.compute(error);
    output = clamp(output, -turn_max_voltage, turn_max_voltage);
    DriveR.spin(reverse, output, volt);
    DriveL.stop(hold);
    task::sleep(10);
  }
  DriveL.stop(hold);
  DriveR.stop(hold);
}

float Drive::get_ForwardTracker_position(){
  if (drive_setup==TANK_ONE_ENCODER || drive_setup == TANK_TWO_ENCODER || drive_setup == HOLONOMIC_TWO_ENCODER){
    return(E_ForwardTracker.position(deg)*ForwardTracker_in_to_deg_ratio);
  }else{
    return(R_ForwardTracker.position(deg)*ForwardTracker_in_to_deg_ratio);
  }
}

float Drive::get_SidewaysTracker_position(){
  if (drive_setup==TANK_ONE_ENCODER || drive_setup == TANK_ONE_ROTATION){
    return(0);
  }else if (drive_setup == TANK_TWO_ENCODER || drive_setup == HOLONOMIC_TWO_ENCODER){
    return(E_SidewaysTracker.position(deg)*SidewaysTracker_in_to_deg_ratio);
  }else{
    return(R_SidewaysTracker.position(deg)*SidewaysTracker_in_to_deg_ratio);
  }
}

void Drive::position_track(){
  while(1){
    odom.update_position(get_ForwardTracker_position(), get_SidewaysTracker_position(), get_absolute_heading());
    task::sleep(5);
  }
}

void Drive::set_coordinates(float X_position, float Y_position, float orientation_deg){
  odom.set_position(X_position, Y_position, orientation_deg, get_ForwardTracker_position(), get_SidewaysTracker_position());
}

int Drive::position_track_task(){
  chassis.position_track();
  return(0);
}

float Drive::get_X_position(){
  return(odom.X_position);
}

float Drive::get_Y_position(){
  return(odom.Y_position);
}

void Drive::drive_to_point(float X_position, float Y_position){
  drive_to_point(X_position, Y_position, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_to_point(float X_position, float Y_position, float drive_max_voltage, float heading_max_voltage){
  drive_to_point(X_position, Y_position, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_to_point(float X_position, float Y_position, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout){
  drive_to_point(X_position, Y_position, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_to_point(float X_position, float Y_position, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout, float drive_kp, float drive_ki, float drive_kd, float drive_starti, float heading_kp, float heading_ki, float heading_kd, float heading_starti){
  PID drivePID(hypot(X_position-get_X_position(),Y_position-get_Y_position()), drive_kp, drive_ki, drive_kd, drive_starti, drive_settle_error, drive_settle_time, drive_timeout);
  PID headingPID(reduce_negative_180_to_180(to_deg(atan2(X_position-get_X_position(),Y_position-get_Y_position()))-get_absolute_heading()), heading_kp, heading_ki, heading_kd, heading_starti);
  while(drivePID.is_settled() == false){
    float drive_error = hypot(X_position-get_X_position(),Y_position-get_Y_position());
    float heading_error = reduce_negative_180_to_180(to_deg(atan2(X_position-get_X_position(),Y_position-get_Y_position()))-get_absolute_heading());
    float drive_output = drivePID.compute(drive_error);

    float heading_scale_factor = cos(to_rad(heading_error));
    drive_output*=heading_scale_factor;
    heading_error = reduce_negative_90_to_90(heading_error);
    float heading_output = headingPID.compute(heading_error);
    
    if (drive_error<drive_settle_error) { heading_output = 0; }

    drive_output = clamp(drive_output, -fabs(heading_scale_factor)*drive_max_voltage, fabs(heading_scale_factor)*drive_max_voltage);
    heading_output = clamp(heading_output, -heading_max_voltage, heading_max_voltage);

    drive_with_voltage(drive_output+heading_output, drive_output-heading_output);
    task::sleep(10);
  }
  DriveL.stop(hold);
  DriveR.stop(hold);
}

void Drive::turn_to_point(float X_position, float Y_position){
  turn_to_point(X_position, Y_position, 0, turn_max_voltage, turn_settle_error, turn_settle_time, turn_timeout, turn_kp, turn_ki, turn_kd, turn_starti);
}

void Drive::turn_to_point(float X_position, float Y_position, float extra_angle_deg){
  turn_to_point(X_position, Y_position, extra_angle_deg, turn_max_voltage, turn_settle_error, turn_settle_time, turn_timeout, turn_kp, turn_ki, turn_kd, turn_starti);
}

void Drive::turn_to_point(float X_position, float Y_position, float extra_angle_deg, float turn_max_voltage, float turn_settle_error, float turn_settle_time, float turn_timeout){
  turn_to_point(X_position, Y_position, extra_angle_deg, turn_max_voltage, turn_settle_error, turn_settle_time, turn_timeout, turn_kp, turn_ki, turn_kd, turn_starti);
}

void Drive::turn_to_point(float X_position, float Y_position, float extra_angle_deg, float turn_max_voltage, float turn_settle_error, float turn_settle_time, float turn_timeout, float turn_kp, float turn_ki, float turn_kd, float turn_starti){
  PID turnPID(reduce_negative_180_to_180(to_deg(atan2(X_position-get_X_position(),Y_position-get_Y_position())) - get_absolute_heading()), turn_kp, turn_ki, turn_kd, turn_starti, turn_settle_error, turn_settle_time, turn_timeout);
  while(turnPID.is_settled() == false){
    float error = reduce_negative_180_to_180(to_deg(atan2(X_position-get_X_position(),Y_position-get_Y_position())) - get_absolute_heading() + extra_angle_deg);
    float output = turnPID.compute(error);
    output = clamp(output, -turn_max_voltage, turn_max_voltage);
    drive_with_voltage(output, -output);
    task::sleep(10);
  }
  DriveL.stop(hold);
  DriveR.stop(hold);
}

void Drive::holonomic_drive_to_point(float X_position, float Y_position){
  holonomic_drive_to_point(X_position, Y_position, get_absolute_heading(), drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::holonomic_drive_to_point(float X_position, float Y_position, float angle){
  holonomic_drive_to_point(X_position, Y_position, angle, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::holonomic_drive_to_point(float X_position, float Y_position, float angle, float drive_max_voltage, float heading_max_voltage){
  holonomic_drive_to_point(X_position, Y_position, angle, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::holonomic_drive_to_point(float X_position, float Y_position, float angle, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout){
  holonomic_drive_to_point(X_position, Y_position, angle, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::holonomic_drive_to_point(float X_position, float Y_position, float angle, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout, float drive_kp, float drive_ki, float drive_kd, float drive_starti, float heading_kp, float heading_ki, float heading_kd, float heading_starti){
  PID drivePID(hypot(X_position-get_X_position(),Y_position-get_Y_position()), drive_kp, drive_ki, drive_kd, drive_starti, drive_settle_error, drive_settle_time, drive_timeout);
  PID turnPID(reduce_negative_180_to_180(to_deg(atan2(X_position-get_X_position(),Y_position-get_Y_position()))-get_absolute_heading()), heading_kp, heading_ki, heading_kd, heading_starti);
  while(!(drivePID.is_settled() && turnPID.is_settled() ) ){
    float drive_error = hypot(X_position-get_X_position(),Y_position-get_Y_position());
    float turn_error = reduce_negative_180_to_180(to_deg(atan2(X_position-get_X_position(),Y_position-get_Y_position()))-get_absolute_heading());

    float drive_output = drivePID.compute(drive_error);
    float turn_output = turnPID.compute(turn_error);

    drive_output = clamp(drive_output, drive_max_voltage, drive_max_voltage);
    turn_output = clamp(turn_output, -heading_max_voltage, heading_max_voltage);

    float heading_error = atan2(Y_position-get_Y_position(), X_position-get_X_position());

    DriveLF.spin(fwd, drive_output*cos(to_rad(get_absolute_heading()) + heading_error - M_PI/4) + turn_output, volt);
    DriveLB.spin(fwd, drive_output*cos(-to_rad(get_absolute_heading()) - heading_error + 3*M_PI/4) + turn_output, volt);
    DriveRB.spin(fwd, drive_output*cos(to_rad(get_absolute_heading()) + heading_error - M_PI/4) - turn_output, volt);
    DriveRF.spin(fwd, drive_output*cos(-to_rad(get_absolute_heading()) - heading_error + 3*M_PI/4) - turn_output, volt);
    task::sleep(10);
  }
  DriveLF.stop(hold);
  DriveLB.stop(hold);
  DriveRB.stop(hold);
  DriveRF.stop(hold);
}

void setLeftDriveExpo(vex::directionType type, int percentage){
  if(percentage>=0){
    percentage = 1.2*pow(1.043, percentage) - 1.2 + 0.2*percentage;
  }
  else{
    percentage = -percentage;
    percentage = 1.2*pow(1.043,percentage) - 1.2 + 0.2*percentage;
    percentage = -percentage;
  }
  LeftBack.spin(type, percentage, vex::velocityUnits::pct);
  LeftMiddle.spin(type, percentage, vex::velocityUnits::pct);
  LeftFront.spin(type, percentage, vex::velocityUnits::pct);
}

// void setLeftDriveExpo(vex::directionType type, int percentage){
//   double pwrF = percentage;
//   if(pwrF>0){
//     double pwrF2 = pwrF*pwrF;
//     pwrF = 9.821e-07*pwrF2*pwrF2 - 6.986e-05*pwrF2*pwrF + 0.006751*pwrF2  + 0.03879*pwrF;
//     double baseF = 6;
//     pwrF*=(100-baseF)/100;
//     pwrF+=baseF;
//     if(percentage<0) pwrF *=-1;
//   }
//   percentage=pwrF;
//   LeftBack.spin(type, percentage, vex::velocityUnits::pct);
//   LeftMiddle.spin(type, percentage, vex::velocityUnits::pct);
//   LeftFront.spin(type, percentage, vex::velocityUnits::pct);
// }

// void setRightDriveExpo(vex::directionType type, int percentage){
//   double pwrF = percentage;
//   if(pwrF>0){
//     double pwrF2 = pwrF*pwrF;
//     pwrF = 9.821e-07*pwrF2*pwrF2 - 6.986e-05*pwrF2*pwrF + 0.006751*pwrF2  + 0.03879*pwrF;
//     double baseF = 6;
//     pwrF*=(100-baseF)/100;
//     pwrF+=baseF;
//     if(percentage<0) pwrF *=-1;
//   }
//   percentage=pwrF;
//   RightFront.spin(type, percentage, vex::velocityUnits::pct);
//   RightMiddle.spin(type, percentage, vex::velocityUnits::pct);
//   RightFront.spin(type, percentage, vex::velocityUnits::pct);
//}
void setRightDriveExpo(vex::directionType type, int percentage){
  if(percentage>=0){
    percentage = 1.2*pow(1.043, percentage) - 1.2 + 0.2*percentage;
  }
  else{
    percentage = -percentage;
    percentage = 1.2*pow(1.043,percentage) - 1.2 + 0.2*percentage;
    percentage = -percentage;
  }
  RightBack.spin(type, percentage, vex::velocityUnits::pct);
  RightMiddle.spin(type, percentage, vex::velocityUnits::pct);
  RightFront.spin(type, percentage, vex::velocityUnits::pct);
}





void Drive::control_arcade(){
  //setRightDriveExpo(directionType::rev, Controller.Axis1.value() - Controller.Axis3.value()); 
  //setLeftDriveExpo(directionType::fwd, Controller.Axis1.value() + Controller.Axis3.value()); 
  double ctrX = Controller1.Axis1.value();
  double ctrY = Controller1.Axis3.value();

  // if(input[0] == 0 && input[1]==0) return std::vector<double> {0, 0};
  double pwrF = fabs(ctrY);
  // double actPwrT = ctrX;
  double pwrT = fabs(ctrX);

  // if(pwrF+pwrT>100) pwrF = 100-pwrT;
  if(pwrF>0){
    double pwrF2 = pwrF*pwrF;
    pwrF = 9.821e-07*pwrF2*pwrF2 - 6.986e-05*pwrF2*pwrF + 0.006751*pwrF2  + 0.03879*pwrF;
    double baseF = 6;
    pwrF*=(100-baseF)/100;
    pwrF+=baseF;
    if(ctrY<0) pwrF *=-1;
  }

  if(pwrT>0){
    double pwrT2 = pwrT*pwrT;
    pwrT = 9.821e-07*pwrT2*pwrT2 - 6.986e-05*pwrT2*pwrT + 0.006751*pwrT2  + 0.03879*pwrT;
    double baseT = 8;
    pwrT*=(100-baseT)/100;
    pwrT+=baseT;
    pwrT*=.8;
    if(ctrX<0)pwrT*=-1;
  }
  
  double pwrL = pwrF+pwrT;
  double pwrR = pwrF-pwrT;
  DriveL.spin(fwd, to_volt(pwrL), volt);
  DriveR.spin(fwd, to_volt(pwrR), volt);
}

void Drive::control_tank(){
  //setRightDriveExpo(directionType::rev, Controller.Axis1.value() - Controller.Axis3.value()); 
  //setLeftDriveExpo(directionType::fwd, Controller.Axis1.value() + Controller.Axis3.value()); 
  double ctrX = Controller1.Axis1.value();
  double ctrY = Controller1.Axis3.value();

  // if(input[0] == 0 && input[1]==0) return std::vector<double> {0, 0};
  double pwrF = ctrY;
  // double actPwrT = ctrX;
  double pwrT = ctrX;

  // if(pwrF+pwrT>100) pwrF = 100-pwrT;
  if(1){
    double pwrF3 = pwrF*pwrF*pwrF;
    pwrF3 = pwrF3/10000;
  }

  if(1){
    double pwrT3 = pwrT*pwrT*pwrT;
    pwrT3 = pwrT3/10000;
    pwrT*=.925;
  }

  double pwrL = pwrF+pwrT;
  double pwrR = pwrF-pwrT;
  DriveL.spin(fwd, to_volt(pwrL), volt);
  DriveR.spin(fwd, to_volt(pwrR), volt);
}