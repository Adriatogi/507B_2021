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
motor FR(PORT18, gearSetting::ratio18_1, false);
motor FL(PORT19, gearSetting::ratio18_1, true);
motor BL(PORT17, gearSetting::ratio18_1, true);
motor BR(PORT15, gearSetting::ratio18_1, false);
motor roller1(PORT14, gearSetting::ratio6_1, true);
motor roller2(PORT20,gearSetting::ratio6_1, false);
motor intake1(PORT13, gearSetting::ratio6_1, true);
motor intake2 (PORT2, gearSetting::ratio6_1, true);


motor_group leftGroup(FL, BL);
motor_group rightGroup(FR, BR);
motor_group rollers(roller1, roller2);
drivetrain driveTrain(leftGroup, rightGroup);

controller Controller1;

inertial Inertial(PORT7);
optical Optical = optical(PORT10);


/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void drive(double distanceInches, int speed, bool wait)
{
  driveTrain.driveFor(directionType::fwd, distanceInches, distanceUnits::in, speed, velocityUnits::pct, wait); // wait to complete
}

void moveRobotNoWait(float rotationLeft, float rotationRight, int speed) {
  FL.resetRotation();
  BL.resetRotation();
  FR.resetRotation();
  BR.resetRotation();

  FL.rotateTo(rotationLeft, rotationUnits::rev, speed, velocityUnits::pct,
  false);
  BL.rotateTo(rotationLeft, rotationUnits::rev, speed, velocityUnits::pct,
  false);
  FR.rotateTo(rotationRight, rotationUnits::rev, speed, velocityUnits::pct,
  false);
  BR.rotateTo(rotationRight, rotationUnits::rev, speed, velocityUnits::pct,
  false);
}

void moveRobotWait(float rotationLeft, float rotationRight, int speed) {
FL.resetRotation();
BL.resetRotation();
FR.resetRotation();
BR.resetRotation();

FL.rotateTo(rotationLeft, rotationUnits::rev, speed, velocityUnits::pct,
false);
BL.rotateTo(rotationLeft, rotationUnits::rev, speed, velocityUnits::pct,
false);
FR.rotateTo(rotationRight, rotationUnits::rev, speed, velocityUnits::pct,
false);
BR.rotateTo(rotationRight, rotationUnits::rev, speed, velocityUnits::pct,
true);

FL.stop();
BL.stop();
FR.stop();
BR.stop();

}

void moveRobotTimer(int speed, int ms) {
  FL.spin(directionType::fwd, speed, velocityUnits::pct);
  BL.spin(directionType::fwd, speed, velocityUnits::pct);
  FR.spin(directionType::fwd, speed, velocityUnits::pct);
  BR.spin(directionType::fwd, speed, velocityUnits::pct);

  task::sleep(ms);
  FL.spin(directionType::rev, speed * 0.1, velocityUnits::pct);
  BL.spin(directionType::rev, speed * 0.1, velocityUnits::pct);
  FR.spin(directionType::rev, speed * 0.1, velocityUnits::pct);
  BR.spin(directionType::rev, speed * 0.1, velocityUnits::pct);
  FL.stop();
  BL.stop();
  FR.stop();
  BR.stop();
}

void loadSingleBall(float distanceTravel, int speed)
{
  rollers.spin(directionType::fwd, 100, velocityUnits::pct);
  intake1.spin(directionType::fwd, 100, velocityUnits::pct);
  intake2.stop(brakeType::hold);

  drive(distanceTravel, speed, true);

  rollers.stop();
  intake1.stop();
  intake2.stop();
}

void scoreBallsDeScoreTower() 
{
  /*
  TODO Add a timer that if no red ball is detected after 2(prone to change) seconds
  stops the top intake
  Lower optical sensor below the current c-channel
  */
  rollers.spin(directionType::fwd, 80, velocityUnits::pct);
  intake1.spin(directionType::fwd, 80, velocityUnits::pct);
  intake2.spin(directionType::fwd, 80, velocityUnits::pct);
  
  while(Optical.color() != blue)
  {

  }

  rollers.stop();
  intake1.stop(brakeType::hold);
  intake2.stop(brakeType::hold);
  moveRobotWait(-2, -2, 50);
  intake1.stop();
  intake2.stop();
}

void pdTurn(double degrees) //PD loop turn code (better than the smartdrive and P loop methods once kP and kD are tuned properly)
{
  if(Inertial.installed())
  {
    int dt = 20;  // Recommended wait time in milliseconds
    double target = degrees; // In revolutions
    double error = target - Inertial.rotation();
    double kP = .21;
    double kD = .90;
    double prevError = error;
    printf("Inertial: %f\n", Inertial.rotation());
    //printf("Error: %f\n", error);
    while (abs(error) > 0) // Allows +- 1 degree variance, don't reduce unless you know what you are doing
    {
      error = target - Inertial.rotation();
      //printf("Error: %f\n", error);
      printf("KPError: %f\n", kP*error);
      double derivative = (error - prevError)/dt;
      printf("derivative: %f\n", kD*derivative);
      double percent = (kP * error + kD * derivative)+0.5;
      printf("Percent: %f\n", percent);
      prevError = error;
      leftGroup.spin(directionType::rev, percent, pct);
      rightGroup.spin(directionType::fwd, percent, pct);
      task::sleep(dt);
      printf("Inertial: %f\n", Inertial.rotation());
      printf("Error: %f\n", error);
    }
    printf("done\n");
    printf("Inertial: %f\n", Inertial.rotation());
    printf("Error: %f\n", error);
    leftGroup.stop();
    rightGroup.stop();
    task::sleep(500);
  }
  else
  {
    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1,1);
    Controller1.Screen.print("No Inertial Sensor Installed");
  }
}

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  if(Inertial.installed())
  {
      Controller1.Screen.clearScreen();
      Controller1.Screen.setCursor(1,1);
      Controller1.Screen.print("Inertial Sensor Calibrating");
      Inertial.calibrate(2000);
      vex::task::sleep(2000);
  }

  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1,1);
  Controller1.Screen.print("Ready to go");
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
  //pTurn(90);
  //pdTurn(-90);
  //drive(2, 10, false);
  Controller1.Screen.setCursor(1,1);
  Controller1.Screen.clearLine();
  Controller1.Screen.print("Finished");
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
  Optical.setLight(ledState::off);

  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.
    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1, 1);
    Controller1.Screen.print("Temperature FR: %f",
    FR.temperature(percentUnits::pct));


    leftGroup.spin(vex::directionType::fwd, ((Controller1.Axis3.value()*0.75) + (Controller1.Axis1.value()*0.5)), vex::velocityUnits::pct);
    rightGroup.spin(vex::directionType::fwd, ((Controller1.Axis3.value()*0.75) - (Controller1.Axis1.value()*0.5)), vex::velocityUnits::pct);
   
    if(Controller1.ButtonR1.pressing())
    {
     roller1.spin(directionType::fwd, 100, velocityUnits::pct);
     roller2.spin(directionType::fwd, 100, velocityUnits::pct);
    }
    else if(Controller1.ButtonR2.pressing())
    {
     roller1.spin(directionType::rev, 100, velocityUnits::pct);
     roller2.spin(directionType::rev, 100, velocityUnits::pct);
    }
    else
    {
     roller1.stop(brakeType::brake);
     roller2.stop(brakeType::brake);
    }

    if(Controller1.ButtonL1.pressing())
    {
     intake1.spin(directionType::fwd, 100, velocityUnits::pct);
     intake2.spin(directionType::fwd, 100, velocityUnits::pct);
    }
    else if(Controller1.ButtonL2.pressing())
    {
     intake1.spin(directionType::rev, 100, velocityUnits::pct);
     intake2.spin(directionType::rev, 100, velocityUnits::pct);
    }
    else if(Controller1.ButtonY.pressing()) // Eject ball when there is one on top
    {
     intake1.spin(directionType::fwd, 100, velocityUnits::pct);
     intake2.stop(brakeType::hold);
    }
    else if(Controller1.ButtonUp.pressing()) // eject ball when no ball on top
    {
     intake1.spin(directionType::fwd, 100, velocityUnits::pct);
     intake2.spin(directionType::rev, 100, velocityUnits::pct);
    }
    else
    {
     intake1.stop(brakeType::brake);
     intake2.stop(brakeType::brake);
    }
  
    if(Controller1.ButtonA.pressing())
    {
      scoreBallsDeScoreTower();
    }
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
