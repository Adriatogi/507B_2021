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

timer timer1;


/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

int pickUpRedTask() // pick up red ball task
{

  //while the red ball isnt picked up, do nothing
  while(Optical.color() != red) 
  {
  }

  //Stop rollers when ball is picked up
  rollers.stop(brakeType::hold);
  intake1.stop(brakeType::hold);
  return(0);
}

int pickUpBlueTask() // pick up blue ball task
{

  //while the blue isnt picked, do nothing
  while(Optical.color() != blue) 
  {
  }

  //stop rollers when ball is picked up
  rollers.stop(brakeType::hold);
  intake1.stop(brakeType::hold);
  return(0);
}

void drive(double distanceInches, int speed, bool wait)
{
  driveTrain.driveFor(directionType::fwd, distanceInches, distanceUnits::in, speed, velocityUnits::pct, wait); // wait to complete
}

void driveRobot(float totalDistance, int speed, bool wait) // Slows it down towards the end to avoid overshooting
{
  float initialDistance = totalDistance *0.9;
  float finalDistance =  totalDistance;
  int finalSpeed = 20;

  FL.resetRotation();
  BL.resetRotation();
  FR.resetRotation();
  BR.resetRotation();

  FL.rotateTo(initialDistance, rotationUnits::rev, speed, velocityUnits::pct,
  false);
  BL.rotateTo(initialDistance, rotationUnits::rev, speed, velocityUnits::pct,
  false);
  FR.rotateTo(initialDistance, rotationUnits::rev, speed, velocityUnits::pct,
  false);
  BR.rotateTo(initialDistance, rotationUnits::rev, speed, velocityUnits::pct,
  true);
  FL.rotateTo(finalDistance, rotationUnits::rev, finalSpeed, velocityUnits::pct,
  false);
  BL.rotateTo(finalDistance, rotationUnits::rev, finalSpeed, velocityUnits::pct,
  false);
  FR.rotateTo(finalDistance, rotationUnits::rev, finalSpeed, velocityUnits::pct,
  false);
  BR.rotateTo(finalDistance, rotationUnits::rev, finalSpeed, velocityUnits::pct,
  wait);

  printf("Done\n");
  FL.stop();
  BL.stop();
  FR.stop();
  BR.stop();
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

void moveRobotTimer(int speed, int ms) { // moves on a timer
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

void moveRobotSpin(int speed) // just spin motors
{
  FL.spin(directionType::fwd, speed, velocityUnits::pct);
  BL.spin(directionType::fwd, speed, velocityUnits::pct);
  FR.spin(directionType::fwd, speed, velocityUnits::pct);
  BR.spin(directionType::fwd, speed, velocityUnits::pct);
}

void loadSingleBall(float revs, int speed)// stop spinning if it also detects a blue ball in the inside, but keep driving
{
  rollers.spin(directionType::fwd, 100, velocityUnits::pct);
  intake1.spin(directionType::fwd, 80, velocityUnits::pct);
  intake2.stop(brakeType::hold);

  task myTask = task(pickUpRedTask);

  driveRobot(revs, speed, true);

  myTask.stop();
  
  rollers.stop();
  intake1.stop();
  intake2.stop();
}

void scoreBallsDeScoreTower() //TODO maybe delay the start of the task even more 
//so it makes sure the red ball atleast had the power of the bottom intake? Idk 
//seems to work most of the time still
{

  timer1.clear();

  //move forward and spin rolers 
  rollers.spin(directionType::fwd, 100, velocityUnits::pct);
  moveRobotSpin(20);
  task::sleep(500);

  //start scoring
  intake1.spin(directionType::fwd, 100, velocityUnits::pct);
  intake2.spin(directionType::fwd, 100, velocityUnits::pct);
  task::sleep(500);

  //stop moving robot and if blue ball detected, stop rollers and bottom intake
  task myTask = task(pickUpBlueTask);
  moveRobotSpin(0);

  //top intake will keep going for one second to score red ball
  task::sleep(1000);
  myTask.stop();
  
  //while((Optical.color() != blue)&&(timer1<4000))
  //{ 
  //}

  rollers.stop();
  intake1.stop(brakeType::hold);
  intake2.stop(brakeType::hold);
  task::sleep(100); // to stop momentum of intakes
  intake1.stop();
  intake2.stop();
}

void outtakeBalls()
{
  timer1.clear();

  //start shooting them out
  rollers.spin(directionType::fwd, 100, velocityUnits::pct);
  intake1.spin(directionType::fwd, 100, velocityUnits::pct);
  intake2.spin(directionType::rev, 100, velocityUnits::pct);

  //run for atleast three seconds and make sure there is no detected blue ball
  while(timer1<3000 || (Optical.color() == blue))
  { 
  }

  rollers.stop();
  intake1.stop();
  intake2.stop();

}
void pdTurn(double degrees) //PD loop turn code (better than the smartdrive and P loop methods once kP and kD are tuned properly)
{
  if(Inertial.installed())
  {
    printf("---------------------------------------------------");
    int dt = 20;  // Recommended wait time in milliseconds
    double target = degrees; // In revolutions
    double error = target - Inertial.rotation();
    double kP = .21;
    double kD = .90;
    double prevError = error;
    double backUpSpeed = 1.3;

    if(error<0)
    {
      backUpSpeed*=-1;
    }

    printf("Inertial: %f\n", Inertial.rotation());
    //printf("Error: %f\n", error);
    while (abs(error) > 0) // Allows +- 1 degree variance, don't reduce unless you know what you are doing
    {
      error = target - Inertial.rotation();
      //printf("Error: %f\n", error);
      printf("KPError: %f\n", kP*error);
      double derivative = (error - prevError)/dt;
      printf("derivative: %f\n", kD*derivative);
      double percent = (kP * error + kD * derivative)+ backUpSpeed;
      printf("Percent: %f\n", percent);
      prevError = error;
      leftGroup.spin(directionType::fwd, percent, pct);
      rightGroup.spin(directionType::rev, percent, pct);
      task::sleep(dt);
      printf("Inertial: %f\n", Inertial.rotation());
      printf("Error: %f\n", error);
    }
    printf("done\n");
    printf("Inertial: %f\n", Inertial.rotation());
    printf("Error: %f\n", error);
    leftGroup.stop(brakeType::hold);
    rightGroup.stop(brakeType::hold);
    task::sleep(250); // to stop momentum
  }
  else
  {
    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1,1);
    Controller1.Screen.print("No Inertial Sensor Installed");
  }
}

void startUp()
{
  intake2.spin(directionType::fwd, 50, velocityUnits::pct);
  roller1.spin(directionType::fwd, 50, velocityUnits::pct);
  roller2.spin(directionType::fwd, 50, velocityUnits::pct);
  task::sleep(1000);
  roller1.stop();
  roller2.stop();
  intake2.stop();
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
  
  //drive forward
  startUp();
  loadSingleBall(2.2, 50);
  pdTurn(80);
  driveRobot(0.77, 35, true);
  scoreBallsDeScoreTower();
  driveRobot(-2.75, 35, true);
  pdTurn(203);
  outtakeBalls();
  loadSingleBall(3.25, 50);
  pdTurn(160);
  driveRobot(2.3, 35, true);
  scoreBallsDeScoreTower();



  //go forward pick up second ball
  //turn to corner 
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
  Controller1.Screen.clearScreen();

  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.
    Controller1.Screen.setCursor(1, 1);
    Controller1.Screen.print("Temperature FR: %f",
    FR.temperature(percentUnits::pct));
    printf("Inertial: %f\n", Inertial.rotation());


    leftGroup.spin(vex::directionType::fwd, ((Controller1.Axis3.value()*0.75) + (Controller1.Axis1.value()*0.2)), vex::velocityUnits::pct);
    rightGroup.spin(vex::directionType::fwd, ((Controller1.Axis3.value()*0.75) - (Controller1.Axis1.value()*0.2)), vex::velocityUnits::pct);
   
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
      //driveRobot(2, 50, true);
      //loadSingleBall(5, 35);
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
