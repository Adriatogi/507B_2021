//auton using encoders, referenced from VEX forums, used last year 507E
const float WHEEL_DIAMETER = 4.125; // IN INCHES/ remeasure
const float WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * 3.1416; //remeasure
const float GEAR_RATIO = 6; //remeasure
const float AUTON_DRIVE_PCT = 50; // remeasure
const float TURNING_DIAMETER = 13; // distance (in inches) from top-left wheel to bottom-right wheel/ remeasure

void move(float inches) {
  driveTrain.resetRotation();
  float inchesPerDegree = WHEEL_CIRCUMFERENCE / 360;
  float degrees = inches / inchesPerDegree;
  ALL.rotateFor(degrees * GEAR_RATIO, vex::rotationUnits::deg, AUTON_DRIVE_PCT, vex::velocityUnits::pct);
}

/*void lift(float inches) {
  L.resetRotation();
  float inchesPerDegree = WHEEL_CIRCUMFERENCE / 360;
  float degrees = inches / inchesPerDegree / 10;
  L.rotateFor(degrees * GEAR_RATIO, vex::rotationUnits::deg, AUTON_DRIVE_PCT * 6, vex::velocityUnits::pct);
}

void tray(float inches) {
  T.resetRotation();
  float inchesPerDegree = WHEEL_CIRCUMFERENCE / 360;
  float degrees = inches / inchesPerDegree / 10;
  T.rotateFor(degrees * GEAR_RATIO, vex::rotationUnits::deg, AUTON_DRIVE_PCT*2, vex::velocityUnits::pct);
}

void innout(int s, int t) {
  T.spin(directionType::fwd, s, velocityUnits::rpm);
  task::sleep(t);
}

void roller(int s) { R.spin(directionType::fwd, s, velocityUnits::rpm); }

void rollertray(int s, int t) {
  R.spin(directionType::fwd, s, velocityUnits::rpm);
  T.spin(directionType::fwd, s, velocityUnits::rpm);
  task::sleep(t);
}*/

void turn(float degrees) {
  //+90 degrees is a right turn, -90 degrees is a left turn
  float turningRatio = TURNING_DIAMETER / WHEEL_DIAMETER;
  float wheelDegrees = turningRatio * degrees;
  rightGroup.rotateFor(wheelDegrees * GEAR_RATIO / 2, vex::rotationUnits::deg AUTON_DRIVE_PCT, vex::velocityUnits::pct);
  leftGroup.rotateFor(wheelDegrees * GEAR_RATIO / -2, vex::rotationUnits::deg AUTON_DRIVE_PCT, vex::velocityUnits::pct);
} //if code doesn't work use following:

/*void LT(float degrees) {
  //+90 degrees is a left turn
  float turningRatio = TURNING_DIAMETER / WHEEL_DIAMETER;
  float wheelDegrees = turningRatio * degrees;
  leftGroup.rotateFor(wheelDegrees * GEAR_RATIO / 2, vex::rotationUnits::deg AUTON_DRIVE_PCT, vex::velocityUnits::pct);
}

void RT(float degrees) {
  //+90 degrees is a right turn
  float turningRatio = TURNING_DIAMETER / WHEEL_DIAMETER;
  float wheelDegrees = turningRatio * degrees;
  rightGroup.rotateFor(wheelDegrees * GEAR_RATIO / 2, vex::rotationUnits::deg AUTON_DRIVE_PCT, vex::velocityUnits::pct);
}*/