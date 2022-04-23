// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// LeftFront            motor         1               
// LeftBack             motor         5               
// RightFront           motor         2               
// RightBack            motor         6               
// RightLift            motor         10              
// Clamp                motor         20              
// Inertial             inertial      21              
// Controller1          controller                    
// OldbackPiston        digital_out   D               
// Sporklift            motor         9               
// Clamp2               motor         16              
// RightMiddle          motor         4               
// LeftMiddle           motor         3               
// ClampSolenoid        digital_out   A               
// ---- END VEXCODE CONFIGURED DEVICES ----
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// LeftFront            motor         1               
// LeftBack             motor         5               
// RightFront           motor         2               
// RightBack            motor         6               
// RightLift            motor         10              
// Clamp                motor         20              
// Inertial             inertial      21              
// Controller1          controller                    
// OldbackPiston        digital_out   D               
// Sporklift            motor         9               
// Clamp2               motor         16              
// RightMiddle          motor         4               
// LeftMiddle           motor         3               
// ClampSolenoid        digital_out   A               
// ---- END VEXCODE CONFIGURED DEVICES ----
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// LeftFront            motor         1               
// LeftBack             motor         5               
// RightFront           motor         2               
// RightBack            motor         6               
// RightLift            motor         10              
// Clamp                motor         20              
// Inertial             inertial      21              
// Controller1          controller                    
// OldbackPiston        digital_out   D               
// Sporklift            motor         9               
// Clamp2               motor         16              
// RightMiddle          motor         4               
// LeftMiddle           motor         3               
// ClampSolenoid        digital_out   A               
// ---- END VEXCODE CONFIGURED DEVICES ----
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// LeftFront            motor         1               
// LeftBack             motor         5               
// RightFront           motor         2               
// RightBack            motor         6               
// RightLift            motor         10              
// Clamp                motor         20              
// Inertial             inertial      21              
// Controller1          controller                    
// OldbackPiston        digital_out   D               
// Sporklift            motor         9               
// Clamp2               motor         16              
// RightMiddle          motor         4               
// LeftMiddle           motor         3               
// ClampSolenoid        digital_out   A               
// ---- END VEXCODE CONFIGURED DEVICES ----
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// LeftFront            motor         1               
// LeftBack             motor         5               
// RightFront           motor         2               
// RightBack            motor         6               
// RightLift            motor         10              
// Clamp                motor         20              
// Inertial             inertial      21              
// Controller1          controller                    
// OldbackPiston        digital_out   D               
// Sporklift            motor         8               
// Clamp2               motor         16              
// RightMiddle          motor         4               
// LeftMiddle           motor         3               
// ClampSolenoid        digital_out   A               
// ---- END VEXCODE CONFIGURED DEVICES ----
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// LeftFront            motor         1               
// LeftBack             motor         5               
// RightFront           motor         2               
// RightBack            motor         6               
// RightLift            motor         7               
// Clamp                motor         20              
// Inertial             inertial      21              
// Controller1          controller                    
// OldbackPiston        digital_out   D               
// Sporklift            motor         8               
// Clamp2               motor         16              
// RightMiddle          motor         4               
// LeftMiddle           motor         3               
// ClampSolenoid        digital_out   A               
// ---- END VEXCODE CONFIGURED DEVICES ----


#include "vex.h"
#include <cmath>

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
      LeftMiddle.setVelocity(getSign(total)*maximumSpeed + 0.5*amountOff, percent);
      RightMiddle.setVelocity(getSign(total)*maximumSpeed + 0.5*amountOff, percent);
    }
    else if(fabs(total) < fabs(minimumSpeed)){
      LeftBack.setVelocity(getSign(total)*minimumSpeed - 0.5*amountOff, percent);
      RightBack.setVelocity(getSign(total)*minimumSpeed + 0.5*amountOff, percent);
      LeftFront.setVelocity(getSign(total)*minimumSpeed - 0.5*amountOff, percent);
      RightFront.setVelocity(getSign(total)*minimumSpeed + 0.5*amountOff, percent);
      LeftMiddle.setVelocity(getSign(total)*minimumSpeed + 0.5*amountOff, percent);
      RightMiddle.setVelocity(getSign(total)*minimumSpeed + 0.5*amountOff, percent);
    }
    else{
      LeftBack.setVelocity(total - 0.5*amountOff, percent);
      RightBack.setVelocity(total + 0.5*amountOff, percent);
      LeftFront.setVelocity(total - 0.5*amountOff, percent);
      RightFront.setVelocity(total + 0.5*amountOff,percent);
      LeftMiddle.setVelocity(total + 0.5*amountOff,percent);
      RightMiddle.setVelocity(total + 0.5*amountOff,percent);
    }
  }
  LeftBack.stop(brake);
  RightBack.stop(brake);
  RightFront.stop(brake);
  LeftFront.stop(brake);
  RightMiddle.stop(brake);
  LeftMiddle.stop(brake);
}
//Void that controls the drivetrain based on inputs from the joysticks

int speedFactor = 1;

void platformMode() {
  if(Controller1.ButtonX.pressing()){
    speedFactor = 6;
    LeftFront.setStopping(hold);
    LeftBack.setStopping(hold);
    RightFront.setStopping(hold);
    RightBack.setStopping(hold);
    RightMiddle.setStopping(hold);
    LeftMiddle.setStopping(hold);
  }
  else {
    speedFactor = 1;
    LeftFront.setStopping(coast);
    LeftBack.setStopping(coast);
    RightFront.setStopping(coast);
    RightBack.setStopping(coast);
    RightMiddle.setStopping(coast);
    LeftMiddle.setStopping(coast);
  }
}

void goSlow(){
  if(Controller1.ButtonX.pressing()){
    LeftBack.spin(forward, 50, percent);
    RightBack.spin(forward, 50, percent);
    LeftFront.spin(forward, 50, percent);
    RightFront.spin(forward, 50, percent);
    RightMiddle.spin(forward, 50, percent);
    LeftMiddle.spin(forward, 50, percent);
  }
  else if(Controller1.ButtonB.pressing()){
    LeftBack.spin(reverse, 50, percent);
    RightBack.spin(reverse, 50, percent);
    LeftFront.spin(reverse, 50, percent);
    RightFront.spin(reverse, 50, percent);
    RightMiddle.spin(reverse, 50, percent);
    LeftMiddle.spin(reverse, 50, percent);
  }
  else if(Controller1.ButtonY.pressing()){
    LeftBack.spin(reverse, 50, percent);
    RightBack.spin(forward, 50, percent);
    LeftFront.spin(reverse, 50, percent);
    RightFront.spin(forward, 50, percent);
    RightMiddle.spin(forward, 50, percent);
    LeftMiddle.spin(forward, 50, percent);
  }
  else if(Controller1.ButtonA.pressing()){
    LeftBack.spin(forward, 50, percent);
    RightBack.spin(reverse, 50, percent);
    LeftFront.spin(forward, 50, percent);
    RightFront.spin(reverse, 50, percent);
    RightMiddle.spin(reverse, 50, percent);
    LeftMiddle.spin(reverse, 50, percent);
  }
  else{
    RightBack.stop(hold);
    RightFront.stop(hold);
    LeftBack.stop(hold);
    LeftFront.stop(hold);
    LeftMiddle.stop(hold);
    RightMiddle.stop(hold);
  }
}

void simpleDrive(){
  double forwardAmount = Controller1.Axis3.position();
  double turnAmount = Controller1.Axis1.position(); //Axis 4 for unified joystick
  
  RightFront.spin(forward, (forwardAmount-turnAmount) / speedFactor, percent);
  RightBack.spin(forward, (forwardAmount-turnAmount) / speedFactor, percent);
  LeftFront.spin(forward, (forwardAmount+turnAmount) / speedFactor, percent);
  LeftBack.spin(forward, (forwardAmount+turnAmount) / speedFactor, percent);
  RightMiddle.spin(forward, (forwardAmount-turnAmount) / speedFactor, percent);
  LeftMiddle.spin(forward, (forwardAmount+turnAmount) / speedFactor, percent);
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
    /*Clamp.setVelocity(200,percent);
    Clamp.spin(forward);*/
    ClampSolenoid.set(false);
  }
  else if(Controller1.ButtonR1.pressing()){
    /*Clamp.setVelocity(200, percent);
    Clamp.spin(reverse);*/
    ClampSolenoid.set(true);
  }
  else{
    /*Clamp.setStopping(hold);
    Clamp.stop();*/
  }
}

void clamp2Movement() {
  if(Controller1.ButtonR2.pressing()){
    Clamp2.setVelocity(200,percent);
    Clamp2.spin(forward);
  }
  else if(Controller1.ButtonR1.pressing()){
    Clamp2.setVelocity(200,percent);
    Clamp2.spin(reverse);
  }
  else{
    Clamp2.setStopping(hold);
    Clamp2.stop();
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
    RightMiddle.spin(forward, 0.2*error + 5, percent);
    LeftMiddle.spin(forward, 0.2*error + 5, percent);
    wait(5, msec);
  }
  LeftBack.setStopping(hold);
  RightBack.setStopping(hold);
  RightFront.setStopping(hold);
  LeftFront.setStopping(hold);
  RightMiddle.setStopping(hold);
  LeftMiddle.setStopping(hold);
  LeftBack.stop();
  RightBack.stop();
  RightFront.stop();
  LeftFront.stop();
  RightMiddle.stop();
  LeftMiddle.stop();
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
    RightMiddle.spin(reverse, 0.2*error + 5, percent);
    LeftMiddle.spin(reverse, 0.2*error + 5, percent);
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
//BEN'S HELPER FUNCTIONS------------------------------------------------------------

void moveDrivetrain(float vel, int dist, bool smooth, bool sync) {
  LeftFront.setStopping(coast);
  LeftBack.setStopping(coast);
  RightFront.setStopping(coast);
  RightBack.setStopping(coast);
  RightMiddle.setStopping(coast);
  LeftMiddle.setStopping(coast); 
  LeftFront.setVelocity(vel, percent);
  LeftBack.setVelocity(vel, percent);
  RightFront.setVelocity(vel, percent);
  RightBack.setVelocity(vel, percent);
  RightMiddle.setVelocity(vel, percent);
  LeftMiddle.setVelocity(vel, percent);

  if (smooth) {
    LeftBack.setPosition(0, degrees);

    LeftFront.spin(forward);
    LeftBack.spin(forward);
    RightFront.spin(forward);
    RightBack.spin(forward);
    RightMiddle.spin(forward);
    LeftMiddle.spin(forward);

    while (std::abs(LeftBack.position(degrees)) < std::abs(dist)) {
      wait(10, msec);
    }

    LeftFront.stop();
    LeftBack.stop();
    RightFront.stop();
    RightBack.stop();
    RightMiddle.stop();
    LeftMiddle.stop();

  } else {
    LeftFront.spinFor(forward, dist, degrees, false);
    LeftBack.spinFor(forward, dist, degrees, false);
    RightFront.spinFor(forward, dist, degrees, false);
    RightBack.spinFor(forward, dist, degrees, false);
    RightMiddle.spinFor(forward, dist, degrees, false);
    LeftMiddle.spinFor(forward, dist, degrees, sync);
  }
}

//----------------------------------------------------------------------------------

int selected = 0;
std::string autons[8] = {"Disabled", "Left Neutral", "AWP Left", "AWP Right", "Right Neutral", "Right Neutral AWP", "Right Mid", "AWP2 from Left"};
int size = sizeof(autons);

bool elevated = false;

void autonSelector(){
  Controller1.Screen.clearScreen();
  task::sleep(100);
  while(true){
    Controller1.Screen.clearScreen();
    task::sleep(100);
    Controller1.Screen.clearLine(2);
    Controller1.Screen.setCursor(2,1);
    Controller1.Screen.print((autons[selected] + ",").c_str());
    Controller1.Screen.newLine();
    Controller1.Screen.print((elevated ? "Elevated" : "Default"));
    task::sleep(100);
     if(Controller1.ButtonRight.pressing()){
      elevated = !elevated;
        if (!elevated) {
          selected = (selected + 1 + size) % size;
        }
     }else if(Controller1.ButtonLeft.pressing()){
       elevated = !elevated;
       if (elevated) {
        selected = (selected - 1 + size) % size;
       }
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
  int x = 980; // Degrees for moving forward to the neutral goal
  switch(selected){
    case 0:{ //Disabled
      break;
    }
    case 1:{ //Left Neutral
      LeftFront.setStopping(coast);
      LeftBack.setStopping(coast);
      RightBack.setStopping(coast);
      RightFront.setStopping(coast);
      LeftMiddle.setStopping(coast);
      RightMiddle.setStopping(coast);
      LeftFront.setVelocity(100, percent);
      LeftBack.setVelocity(100, percent);
      RightFront.setVelocity(100, percent);
      RightBack.setVelocity(100, percent);
      RightMiddle.setVelocity(100, percent);
      LeftBack.setVelocity(100, percent);
      LeftFront.setPosition(0, degrees);
      Clamp.setVelocity(100, percent);
      //Clamp.spinFor(forward, -45, degrees, false);
      ClampSolenoid.set(true);
      RightLift.spinFor(reverse, 50, degrees, false);
      LeftFront.spin(forward);
      LeftBack.spin(forward);
      RightFront.spin(forward);
      RightBack.spin(forward);
      RightMiddle.spin(forward);
      LeftMiddle.spin(forward);
      while(LeftFront.position(degrees)< x + 40 - (elevated ? 20 : 0)){
        wait(10, msec);
      }
      LeftFront.stop();
      LeftBack.stop();
      RightFront.stop();
      RightBack.stop();
      RightMiddle.stop();
      LeftMiddle.stop();

      wait(100, msec);

      /* Clamp.setVelocity(100, percent);

      Clamp.setPosition(0, degrees);
      Clamp.spin(forward);
      while(Clamp.position(degrees) < 40){
        wait(10, msec);
      }

      Clamp.stop();
      */

      ClampSolenoid.set(false);

      //LeftFront.setVelocity(100, rpm);
      //LeftBack.setVelocity(100, rpm);

      LeftFront.setVelocity(100, percent);
      LeftBack.setVelocity(100, percent);
      RightFront.setVelocity(100, percent);
      RightBack.setVelocity(100, percent);
      RightMiddle.setVelocity(100, percent);
      LeftMiddle.setVelocity(100, percent);  

      LeftFront.spinFor(reverse, x, degrees, false);
      LeftBack.spinFor(reverse, x, degrees, false);
      RightFront.spinFor(reverse, x, degrees, false);
      RightBack.spinFor(reverse, x, degrees, false);
      RightMiddle.spinFor(reverse, x, degrees, false);
      LeftMiddle.spinFor(reverse, x, degrees, false);
      break; 
    }
    case 2: { //AWP Left
      /* Clamp.spinFor(forward, 40, degrees, true);
      Clamp.spinFor(forward, -40, degrees, true); */
      ClampSolenoid.set(false);
      break;
    }
    case 3: { //AWP Right
      LeftFront.setStopping(coast);
      LeftBack.setStopping(coast);
      RightBack.setStopping(coast);
      RightFront.setStopping(coast);
      RightMiddle.setStopping(coast);
      LeftMiddle.setStopping(coast);
      break;
    }
    case 4: { //Right Neutral
      int y = 500;
      int z = 1230;

      LeftFront.setStopping(coast);
      LeftBack.setStopping(coast);
      RightBack.setStopping(coast);
      RightFront.setStopping(coast);
      LeftMiddle.setStopping(coast);
      RightMiddle.setStopping(coast);
      LeftFront.setVelocity(100, percent);
      LeftBack.setVelocity(100, percent);
      RightFront.setVelocity(100, percent);
      RightBack.setVelocity(100, percent);
      LeftMiddle.setVelocity(100, percent);
      RightMiddle.setVelocity(100, percent);
      LeftFront.setPosition(0, degrees);
      Clamp.setVelocity(100, percent);
      LeftFront.spin(forward);
      LeftBack.spin(forward);
      RightFront.spin(forward);
      RightBack.spin(forward);
      RightMiddle.spin(forward);
      LeftMiddle.spin(forward);
      while(LeftFront.position(degrees)< x){
        wait(10, msec);
      }
      LeftFront.stop();
      LeftBack.stop();
      RightFront.stop();
      RightBack.stop();
      RightMiddle.stop();
      LeftMiddle.stop();
      Clamp.spinFor(forward, 40, degrees);

      //LeftFront.setVelocity(100, rpm);
      //LeftBack.setVelocity(100, rpm);

      LeftFront.setVelocity(100, percent);
      LeftBack.setVelocity(100, percent);
      RightFront.setVelocity(100, percent);
      RightBack.setVelocity(100, percent);
      LeftMiddle.setVelocity(100, percent);
      RightMiddle.setVelocity(100, percent);

      Controller1.rumble("...");

      LeftFront.spinFor(forward, y, degrees, false);
      LeftBack.spinFor(forward, y, degrees, false);
      RightFront.spinFor(reverse, y, degrees, false);
      RightBack.spinFor(reverse, y, degrees, true);
      LeftMiddle.spinFor(reverse, y, degrees, true);
      RightMiddle.spinFor(reverse, y, degrees, true);

      Controller1.rumble("------------------");

      Sporklift.spinFor(forward, 500, degrees, true);

      LeftFront.spinFor(forward, -z, degrees, false);
      LeftBack.spinFor(forward, -z, degrees, false);
      RightFront.spinFor(forward, -z, degrees, false);
      RightBack.spinFor(forward, -z, degrees, true);
      LeftMiddle.spinFor(forward, -z, degrees, true);
      RightMiddle.spinFor(forward, -z, degrees, true);

      Sporklift.spinFor(forward, -500, degrees, true);

      LeftFront.spinFor(forward, z, degrees, false);
      LeftBack.spinFor(forward, z, degrees, false);
      RightFront.spinFor(forward, z, degrees, false);
      RightBack.spinFor(forward, z, degrees, false);
      RightMiddle.spinFor(forward, z, degrees, false);
      LeftMiddle.spinFor(forward, z, degrees, false);

      break;
    }
    case 5: { //Right Neutral AWP
      LeftFront.setStopping(coast);
      LeftBack.setStopping(coast);
      RightBack.setStopping(coast);
      RightFront.setStopping(coast);
      RightMiddle.setStopping(coast);
      LeftMiddle.setStopping(coast);
      LeftFront.setVelocity(100, percent);
      LeftBack.setVelocity(100, percent);
      RightFront.setVelocity(100, percent);
      RightBack.setVelocity(100, percent);
      RightMiddle.setVelocity(100, percent);
      LeftMiddle.setVelocity(100, percent);
      LeftFront.setPosition(0, degrees);
      Clamp.setVelocity(100, percent);
      
      // setting up for auton

      Clamp.spinFor(forward, -45, degrees, false);
      RightLift.spinFor(reverse, 50, degrees, false);
      
      // spinning forward towards the goal
      
      LeftFront.spin(forward);
      LeftBack.spin(forward);
      RightFront.spin(forward);
      RightBack.spin(forward);
      RightMiddle.spin(forward);
      LeftMiddle.spin(forward);
      while(LeftFront.position(degrees)< x){
        wait(10, msec);
      }
      LeftFront.stop();
      LeftBack.stop();
      RightFront.stop();
      RightBack.stop();
      RightMiddle.stop();
      LeftMiddle.stop();

      wait(100, msec);

      // clamp down
      
      Clamp.setVelocity(100, percent);

      /*Clamp.setPosition(0, degrees);
      Clamp.spin(forward);
      while(Clamp.position(degrees) < 40){
        wait(10, msec);
      }

      Clamp.stop();*/

      Clamp.spinFor(forward, 45, degrees, false);

      //LeftFront.setVelocity(100, rpm);
      //LeftBack.setVelocity(100, rpm);

      // moving backwards with the goal

      LeftFront.setVelocity(100, percent);
      LeftBack.setVelocity(100, percent);
      RightFront.setVelocity(100, percent);
      RightBack.setVelocity(100, percent);
      LeftMiddle.setVelocity(100, percent);
      RightMiddle.setVelocity(100, percent);  

      LeftFront.spinFor(reverse, x, degrees, false);
      LeftBack.spinFor(reverse, x, degrees, false);
      RightFront.spinFor(reverse, x, degrees, false);
      RightBack.spinFor(reverse, x, degrees, true);
      LeftMiddle.spinFor(reverse, x, degrees, true);
      RightMiddle.spinFor(reverse, x, degrees, true);

      // turning 90°
      
      LeftFront.spinFor(reverse, 250, degrees, false);
      LeftBack.spinFor(reverse, 250, degrees, false);
      RightFront.spinFor(forward, 250, degrees, false);
      RightBack.spinFor(forward, 250, degrees, true);
      RightMiddle.spinFor(forward, 250, degrees, true);
      LeftMiddle.spinFor(forward, 250, degrees, true);

      // moving backwards to place ring in alliance goal
      
      LeftFront.spinFor(reverse, 100, degrees, false);
      LeftBack.spinFor(reverse, 100, degrees, false);
      RightFront.spinFor(reverse, 100, degrees, false);
      RightBack.spinFor(reverse, 100, degrees, true);
      RightMiddle.spinFor(reverse, 100, degrees, true);
      LeftMiddle.spinFor(reverse, 100, degrees, true);

      // placing ring in alliance goal
      
      Sporklift.spinFor(forward, 50, degrees, true);

      // setting up to pickup alliance goal by moving forward
      
      LeftFront.spinFor(forward, 135, degrees, false);
      LeftBack.spinFor(forward, 135, degrees, false);
      RightFront.spinFor(forward, 135, degrees, false);
      RightBack.spinFor(forward, 135, degrees, true);
      LeftMiddle.spinFor(forward, 135, degrees, true);
      RightMiddle.spinFor(forward, 135, degrees, true);

      // 2nd part of setting up to pick up alliance goal; moving forklift down
      
      Sporklift.spinFor(forward, 50, degrees, true);
      
      // 3rd part of setting up; moving backwards with forklift down
      
      LeftFront.spinFor(reverse, 150, degrees, false);
      LeftBack.spinFor(reverse, 150, degrees, false);
      RightFront.spinFor(reverse, 150, degrees, false);
      RightBack.spinFor(reverse, 150, degrees, true);
      RightMiddle.spinFor(reverse, 150, degrees, true);
      LeftMiddle.spinFor(reverse, 150, degrees, true);

      // 4th part of getting AWP; forklifting up to pick up goal
      
      Sporklift.spinFor(reverse, 100, degrees, true);
      
      // moving forward to get the AWP
      
      LeftFront.spinFor(forward, 145, degrees, false);
      LeftBack.spinFor(forward, 145, degrees, false);
      RightFront.spinFor(forward, 145, degrees, false);
      RightBack.spinFor(forward, 145, degrees, true);
      LeftMiddle.spinFor(forward, 145, degrees, true);
      RightMiddle.spinFor(forward, 145, degrees, true);
      
      break; 
    }
    case 6: { //Right Mid
      

      LeftFront.setStopping(coast);
      LeftBack.setStopping(coast);
      RightBack.setStopping(coast);
      RightFront.setStopping(coast);
      RightMiddle.setStopping(coast);
      LeftMiddle.setStopping(coast);
      LeftFront.setVelocity(100, percent);
      LeftBack.setVelocity(100, percent);
      RightFront.setVelocity(100, percent);
      RightBack.setVelocity(100, percent);
      RightMiddle.setVelocity(100, percent);
      LeftMiddle.setVelocity(100, percent);
      LeftFront.setPosition(0, degrees);
      Clamp.setVelocity(100, percent);
      Clamp.spinFor(forward, -45, degrees, false);
      RightLift.spinFor(reverse, 50, degrees, false);
      LeftFront.spin(forward);
      LeftBack.spin(forward);
      RightFront.spin(forward);
      RightBack.spin(forward);
      RightMiddle.spin(forward);
      LeftMiddle.spin(forward);
      while(LeftFront.position(degrees)< x + 430 - (elevated ? 20 : 0)){
        wait(10, msec);
      }
      LeftFront.stop();
      LeftBack.stop();
      RightFront.stop();
      RightBack.stop();
      LeftMiddle.stop();
      RightMiddle.stop();

      wait(100, msec);

      Clamp.setVelocity(100, percent);

      Clamp.setPosition(0, degrees);
      Clamp.spin(forward);
      while(Clamp.position(degrees) < 40){
        wait(10, msec);
      }

      Clamp.stop();

      //LeftFront.setVelocity(100, rpm);
      //LeftBack.setVelocity(100, rpm);

      LeftFront.setVelocity(100, percent);
      LeftBack.setVelocity(100, percent);
      RightFront.setVelocity(100, percent);
      RightBack.setVelocity(100, percent); 
      LeftMiddle.setVelocity(100, percent); 
      RightMiddle.setVelocity(100, percent);  

      LeftFront.spinFor(reverse, x + 230, degrees, false);
      LeftBack.spinFor(reverse, x + 230, degrees, false);
      RightFront.spinFor(reverse, x + 230, degrees, false);
      RightBack.spinFor(reverse, x + 230, degrees, false);
      RightMiddle.spinFor(reverse, x + 230, degrees, false);
      LeftMiddle.spinFor(reverse, x + 230, degrees, false);
      break;
    }
    case 7: { //AWP Carry from Left
      moveDrivetrain(100, 200, true, true);
      ClampSolenoid.set(true);
    }
  }
}

/*---------------------------------------------------------------------------*/
/*                                    Sporklift Code                         */
/*---------------------------------------------------------------------------*/

void sporkliftMovement() {
  if(Controller1.ButtonDown.pressing()){
    Sporklift.setVelocity(100, percent);
    Sporklift.spin(forward);
  }
  else if(Controller1.ButtonUp.pressing()){
    Sporklift.setVelocity(100, percent);
    Sporklift.spin(reverse);
  }
  else{
    Sporklift.setStopping(hold);
    Sporklift.stop();
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
    sporkliftMovement();
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
