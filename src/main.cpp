#include "vex.h"

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// LeftFront            motor         3               
// LeftBack             motor         14              
// RightFront           motor         4               
// RightBack            motor         10              
// RightLift            motor         15              
// Clamp                motor         12              
// Inertial             inertial      1               
// Controller1          controller                    
// OldbackPiston        digital_out   D               
// LeftMiddle           motor         9               
// RightMiddle          motor         6               
// ---- END VEXCODE CONFIGURED DEVICES ----
using namespace vex;
// A global instance of competition
competition Competition;
//Function for determining whether input is positive, negative, or 0

int autonselect = 1;
int numOfAutons = 7;

int getSign (double inputValue) {
  if (inputValue > 0){
    return 1;
  }
  else if (inputValue < 0){
    return -1;
  }
  else return 0;
}

/*Our code uses PID, a control loop used to help the robot move efficiently and accurately
without overshooting its target position. PID takes in input based on the sensors
in the V5 Motors and uses a function to output the target speed for the motors. The "P" in PID
stands for proportional. It makes the motors move based on the distance to the target value.
The "I" in PID stands for integral. It calculates how far the motors have already moved to
give it a little push when proportional control cannot get the robot to its final destination.
The "D" in PID stands for derivative. The derivative calculates how fast the robot has been
accelerating and slows it down if it has been accelerating too rapidly. Combined, these
features create a powerful control loop that keeps our robot's performance consistently high.*/
 
//PID to make the robot drive a certain distance during the autonomous period
void PID (double kP, double kI, double kD, double maxIntegral, double tolerance, double maximumSpeed, double minimumSpeed, double target){
  double error = target;
  double derivative = 0;
  double integral = 0;
  double LastError=error;
  double total = 0;
  LeftBack.setPosition(0, turns);
  Inertial.setRotation(0, degrees);
  while(fabs(tolerance)<fabs(error)){
    LeftBack.spin(forward);
    RightBack.spin(forward);
    LeftFront.spin(forward);
    RightFront.spin(forward);
    LeftMiddle.spin(forward);
    RightMiddle.spin(forward);
    double SensorValue = LeftBack.position(turns)*3.25*5/3*M_PI;
    error = target - SensorValue;
    integral = integral + error;
    if(fabs(integral)>fabs(maxIntegral)){
      integral=getSign(integral)*maxIntegral;
    }
    derivative = error-LastError;
    LastError = error;
    total = kP*error + kI*integral + kD*derivative;
    double amountOff = Inertial.rotation(degrees);
    if(-1 < amountOff < 1){
      amountOff = 0;
    }
    if(fabs(total) > fabs(maximumSpeed)){
      LeftBack.setVelocity(getSign(total)*maximumSpeed - 0.5*amountOff, percent);
      RightBack.setVelocity(getSign(total)*maximumSpeed + 0.5*amountOff, percent);
      LeftFront.setVelocity(getSign(total)*maximumSpeed - 0.5*amountOff, percent);
      RightFront.setVelocity(getSign(total)*maximumSpeed + 0.5*amountOff, percent);
      LeftMiddle.setVelocity(getSign(total)*maximumSpeed - 0.5*amountOff, percent);
      RightMiddle.setVelocity(getSign(total)*maximumSpeed + 0.5*amountOff, percent);
    }
    else if(fabs(total) < fabs(minimumSpeed)){
      LeftBack.setVelocity(getSign(total)*minimumSpeed - 0.5*amountOff, percent);
      RightBack.setVelocity(getSign(total)*minimumSpeed + 0.5*amountOff, percent);
      LeftFront.setVelocity(getSign(total)*minimumSpeed - 0.5*amountOff, percent);
      RightFront.setVelocity(getSign(total)*minimumSpeed + 0.5*amountOff, percent);
      LeftMiddle.setVelocity(getSign(total)*minimumSpeed - 0.5*amountOff, percent);
      RightMiddle.setVelocity(getSign(total)*minimumSpeed + 0.5*amountOff, percent);
    }
    else{
      LeftBack.setVelocity(total - 0.5*amountOff, percent);
      RightBack.setVelocity(total + 0.5*amountOff, percent);
      LeftFront.setVelocity(total - 0.5*amountOff, percent);
      RightFront.setVelocity(total + 0.5*amountOff,percent);
      LeftMiddle.setVelocity(total - 0.5*amountOff, percent);
      RightMiddle.setVelocity(total + 0.5*amountOff,percent);
    }
  }
  LeftBack.stop(brake);
  RightBack.stop(brake);
  RightFront.stop(brake);
  LeftFront.stop(brake);
  LeftMiddle.stop(brake);
  RightMiddle.stop(brake);
}
//Void that controls the drivetrain based on inputs from the joysticks

int speedFactor = 1;

void platformMode() {
  if(Controller1.ButtonX.pressing()){
    speedFactor = 2;
  }
  else if(Controller1.ButtonB.pressing()){
    speedFactor = 1;
  }
}

void goSlow(){
  if(Controller1.ButtonX.pressing()){
    LeftBack.spin(forward, 50, percent);
    RightBack.spin(forward, 50, percent);
    LeftFront.spin(forward, 50, percent);
    RightFront.spin(forward, 50, percent);
    LeftMiddle.spin(forward, 50, percent);
    RightMiddle.spin(forward, 50, percent);
  }
  else if(Controller1.ButtonB.pressing()){
    LeftBack.spin(reverse, 50, percent);
    RightBack.spin(reverse, 50, percent);
    LeftFront.spin(reverse, 50, percent);
    RightFront.spin(reverse, 50, percent);
    LeftMiddle.spin(reverse, 50, percent);
    RightMiddle.spin(reverse, 50, percent);
  }
  else if(Controller1.ButtonY.pressing()){
    LeftBack.spin(reverse, 50, percent);
    RightBack.spin(forward, 50, percent);
    LeftFront.spin(reverse, 50, percent);
    RightFront.spin(forward, 50, percent);
    LeftMiddle.spin(reverse, 50, percent);
    RightMiddle.spin(forward, 50, percent);
  }
  else if(Controller1.ButtonA.pressing()){
    LeftBack.spin(forward, 50, percent);
    RightBack.spin(reverse, 50, percent);
    LeftFront.spin(forward, 50, percent);
    RightFront.spin(reverse, 50, percent);
    LeftMiddle.spin(forward, 50, percent);
    RightMiddle.spin(reverse, 50, percent);
  }
  else{
    RightBack.stop(hold);
    RightFront.stop(hold);
    LeftBack.stop(hold);
    LeftFront.stop(hold);
    RightMiddle.stop(hold);
    LeftMiddle.stop(hold);
  }
}

void simpleDrive(){
  double forwardAmount = Controller1.Axis3.position();
  double turnAmount = Controller1.Axis1.position();
  RightFront.spin(forward, forwardAmount-turnAmount, percent);
  RightBack.spin(forward, forwardAmount-turnAmount, percent);
  RightMiddle.spin(forward, forwardAmount-turnAmount, percent);
  LeftFront.spin(forward, forwardAmount+turnAmount, percent);
  LeftBack.spin(forward, forwardAmount+turnAmount, percent);
  LeftMiddle.spin(forward, forwardAmount+turnAmount, percent);

}
//Void that controls the movement of the 4-bar lift
void armLift(){
  if (Controller1.ButtonL2.pressing()) {
    RightLift.setVelocity(90, percent);
    RightLift.spin(forward);
  }
  else if (Controller1.ButtonL1.pressing()){
    RightLift.setVelocity(90, percent);
    RightLift.spin(reverse);
  }
  else{
    RightLift.setStopping(hold);
    RightLift.stop();
  }
}

void clampMovement() {
  if(Controller1.ButtonR2.pressing()){
    Clamp.setVelocity(50,percent);
    Clamp.spin(forward);
  }
  else if(Controller1.ButtonR1.pressing()){
    Clamp.setVelocity(50, percent);
    Clamp.spin(reverse);
  }
  else{
    Clamp.setStopping(hold);
    Clamp.stop();
  }
}

void turnCounterClockwise(double amount){
  Inertial.setRotation(0, degrees);
  while(fabs(Inertial.rotation(degrees)) < amount){
    double error = amount - fabs(Inertial.rotation(degrees));
    LeftBack.spin(reverse, 0.2*error + 5, percent);
    RightBack.spin(forward, 0.2*error + 5, percent);
    LeftFront.spin(reverse, 0.2*error + 5, percent);
    RightFront.spin(forward, 0.2*error + 5, percent);
    LeftMiddle.spin(reverse, 0.2*error + 5, percent);
    RightMiddle.spin(forward, 0.2*error + 5, percent);    
    wait(5, msec);
  }
  LeftBack.setStopping(hold);
  RightBack.setStopping(hold);
  RightFront.setStopping(hold);
  LeftFront.setStopping(hold);
  LeftMiddle.setStopping(hold);
  RightMiddle.setStopping(hold);
  LeftBack.stop();
  RightBack.stop();
  RightFront.stop();
  LeftFront.stop();
  LeftMiddle.stop();
  RightMiddle.stop();
  wait(0.5, sec);
}

void turnClockwise(double amount){
  Inertial.setRotation(0, degrees);
  while(fabs(Inertial.rotation(degrees))< amount){
    double error = amount - fabs(Inertial.rotation(degrees));
    LeftBack.spin(forward, 0.2*error + 5, percent);
    RightBack.spin(reverse, 0.2*error + 5, percent);
    LeftFront.spin(forward, 0.2*error + 5, percent);
    RightFront.spin(reverse, 0.2*error + 5, percent);
    LeftMiddle.spin(forward, 0.2*error + 5, percent);
    RightMiddle.spin(reverse, 0.2*error + 5, percent);
    wait(5, msec);
  }
  LeftBack.setStopping(hold);
  RightBack.setStopping(hold);
  RightFront.setStopping(hold);
  LeftFront.setStopping(hold);
  LeftMiddle.setStopping(hold);
  RightMiddle.setStopping(hold);
  LeftBack.stop();
  RightBack.stop();
  RightFront.stop();
  LeftFront.stop();
  LeftMiddle.stop();
  RightMiddle.stop();
  wait(0.5, sec);
}


int selected = 0;
std::string autons[4] = {"Disabled", "Left 1 Neutral", "AWP Left", "AWP Right"};
int size = 4;

void autonSelector(){
  Controller1.Screen.clearScreen();
  task::sleep(100);
  while(true){
    Controller1.Screen.clearScreen();
    task::sleep(100);
    Controller1.Screen.clearLine(2);
    Controller1.Screen.setCursor(2,1);
    Controller1.Screen.print(autons[selected].c_str());
    task::sleep(100);
     if(Controller1.ButtonRight.pressing()){
       selected = (selected + 1 + size) % size;
     }else if(Controller1.ButtonLeft.pressing()){
       selected = (selected - 1 + size) % size;
     }else if(Controller1.ButtonA.pressing()){
       task::sleep(100);
       if(Controller1.ButtonA.pressing()){
         goto slctEnd;
       }
     }
   }
   slctEnd:
   Controller1.rumble("..");
}

void pre_auton(void) {
 // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  RightLift.stop(hold);
  Inertial.calibrate();
  wait(3, sec);
  autonSelector();

}
/*---------------------------------------------------------------------------*/
/*                              Autonomous Task                              */
/*  This task is used to control the robot during the autonomous phase of    */
/*  a VEX Competition.                                                       */
/*---------------------------------------------------------------------------*/
void autonomous(void) {
  switch(selected){
    case 0:{
      break;
    }
    case 1:{
      int x = 980;
      LeftFront.setStopping(coast);
      LeftBack.setStopping(coast);
      LeftMiddle.setStopping(coast);
      RightBack.setStopping(coast);
      RightFront.setStopping(coast);
      RightMiddle.setStopping(coast);
      LeftFront.setVelocity(200, rpm);
      LeftMiddle.setVelocity(200, rpm);
      LeftBack.setVelocity(200, rpm);
      RightFront.setVelocity(200, rpm);
      RightMiddle.setVelocity(200, rpm);
      RightBack.setVelocity(200, rpm);
      LeftFront.setPosition(0, degrees);
      Clamp.setVelocity(200, rpm); //BEN TEST EDIT - ADDED
      LeftFront.spin(forward);
      LeftBack.spin(forward);
      RightFront.spin(forward);
      RightBack.spin(forward);
      LeftMiddle.spin(forward);
      RightMiddle.spin(forward);
      while(LeftFront.position(degrees)< x){
        wait(10, msec);
      }
      LeftFront.stop();
      LeftMiddle.stop();
      LeftBack.stop();
      RightFront.stop();
      RightBack.stop();
      RightMiddle.stop();
      Clamp.spinFor(forward, 140, degrees); // TEST EDIT - CHANGED 130 to 140

      LeftFront.setVelocity(100, rpm);
      LeftMiddle.setVelocity(100, rpm);
      LeftBack.setVelocity(100, rpm);

      LeftFront.spinFor(reverse, x, degrees, false);
      LeftBack.spinFor(reverse, x, degrees, false);
      RightFront.spinFor(reverse, x, degrees, false);
      RightBack.spinFor(reverse, x, degrees, false);
      LeftMiddle.spinFor(reverse, x, degrees, false);
      RightMiddle.spinFor(reverse, x, degrees, true);
      break; 
    }
    case 2: {
      LeftFront.setStopping(coast);
      LeftBack.setStopping(coast);
      LeftMiddle.setStopping(coast);
      RightBack.setStopping(coast);
      RightFront.setStopping(coast);
      RightMiddle.setStopping(coast);
      break;
    }
    case 3: {
      LeftFront.setStopping(coast);
      LeftBack.setStopping(coast);
      LeftMiddle.setStopping(coast);
      RightBack.setStopping(coast);
      RightFront.setStopping(coast);
      RightMiddle.setStopping(coast);
      break;
    }
  }
}
/*---------------------------------------------------------------------------*/
/*                              User Control Task                            */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*---------------------------------------------------------------------------*/
void usercontrol(void) {
 // User control code here, inside the loop
  while (1) {
    Clamp.setStopping(hold);
    simpleDrive();
    armLift();
    clampMovement();
    platformMode();
    if(Controller1.ButtonLeft.pressing() && Controller1.ButtonRight.pressing()){
      RightLift.stop(hold);
      Clamp.stop(hold);
      while((Controller1.ButtonY.pressing() && Controller1.ButtonA.pressing()) == false){
        goSlow();
        wait(10, msec);
      }
    }
    wait(15, msec);
  } // Sleep the task for a short amount of time to prevent wasted resources.
}

int main() {
 // Set up callbacks for autonomous and driver control periods.
 Competition.autonomous(autonomous);
 Competition.drivercontrol(usercontrol);
 pre_auton();
 // Prevent main from exiting with an infinite loop.
 while (true) {
   wait(100, msec);
 }
}
