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

#include "vex.h"
#include "auton_voids.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here
motor frontRight(PORT18, gearSetting::ratio18_1, false);
motor frontLeft(PORT19, gearSetting::ratio18_1, true);
motor backLeft(PORT17, gearSetting::ratio18_1, true);
motor backRight(PORT15, gearSetting::ratio18_1, false);

motor_group leftGroup(frontLeft, backLeft);
motor_group rightGroup(frontRight, backRight);
drivetrain driveTrain(leftGroup, rightGroup);

controller Controller1;

inertial Inertial(PORT20); // change port

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/
void pTurn(double revs) //P loop turn code
{
  if(Inertial.installed())
  {
    int dt = 20; // Wait time in milliseconds
    double target = revs; // In revolutions
    double error = target - Inertial.rotation();
    double kP = .6;
    while (abs(error) > 1) // Allows +- 1 degree variance, don't reduce unless you know what you are doing
    {
      error = target - Inertial.rotation();
      double percent = kP * error + 20 * error / abs(error);
      leftGroup.spin(directionType::fwd, percent, pct);
      rightGroup.spin(directionType::rev, percent, pct);
      vex::task::sleep(dt);
    }
    leftGroup.stop();
    rightGroup.stop();
  }
  else
  {
    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1,1);
    Controller1.Screen.print("No Inertial Sensor Installed");
  }
}

/*
void pdTurn(double degrees) //PD loop turn code (better than the smartdrive and P loop methods once kP and kD are tuned properly)
{
  if(Inertial.installed())
  {
    int dt = 20;  // Recommended wait time in milliseconds
    double target = degrees; // In revolutions
    double error = target - Inertial.rotation();
    double kP = .7;
    double kD = .1;
    double prevError = error;
    while (abs(error) > 1) // Allows +- 1 degree variance, don't reduce unless you know what you are doing
    {
      error = target - Inertial.rotation();
      double derivative = (error - prevError)/dt;
      double percent = kP * error + kD * derivative;
      leftGroup.spin(directionType::fwd, percent, pct);
      rightGroup.spin(directionType::rev, percent, pct);
      vex::task::sleep(dt);
      prevError = error;
    }
    leftGroup.stop();
    rightGroup.stop();
  }
  else
  {
    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1,1);
    Controller1.Screen.print("No Inertial Sensor Installed");
  }
}*/

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  if(Inertial.installed())
  {
      Inertial.calibrate(2000);
      vex::task::sleep(2000);
      Controller1.Screen.clearScreen();
      Controller1.Screen.setCursor(1,1);
      Controller1.Screen.print("Inertial Sensor Calibrating");
  }
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
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
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
    leftGroup.spin(vex::directionType::fwd, (Controller1.Axis3.value() + Controller1.Axis1.value())*0.8, vex::velocityUnits::pct);
    rightGroup.spin(vex::directionType::fwd, (Controller1.Axis3.value() - Controller1.Axis1.value())*0.8, vex::velocityUnits::pct);
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
