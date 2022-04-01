// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// LeftFront            motor         3               
// LeftBack             motor         14              
// RightFront           motor         4               
// RightBack            motor         10              
// RightLift            motor         15              
// Clamp                motor         5               
// Inertial             inertial      1               
// Controller1          controller                    
// OldbackPiston        digital_out   D               
// Sporklift            motor         7               
// Clamp2               motor         16              
// ---- END VEXCODE CONFIGURED DEVICES ----
#include "vex.h"
#include <string>

using namespace vex;
// A global instance of competition
competition Competition;
//Function for determining whether input is positive, negative, or 0

bool heatedBools[7] = {false, false, false, false, false, false, false};
motor heatedMotors[7] {LeftFront, LeftBack, RightFront, RightBack, Clamp, RightLift, Sporklift}; 

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
    }
    else if(fabs(total) < fabs(minimumSpeed)){
      LeftBack.setVelocity(getSign(total)*minimumSpeed - 0.5*amountOff, percent);
      RightBack.setVelocity(getSign(total)*minimumSpeed + 0.5*amountOff, percent);
      LeftFront.setVelocity(getSign(total)*minimumSpeed - 0.5*amountOff, percent);
      RightFront.setVelocity(getSign(total)*minimumSpeed + 0.5*amountOff, percent);
    }
    else{
      LeftBack.setVelocity(total - 0.5*amountOff, percent);
      RightBack.setVelocity(total + 0.5*amountOff, percent);
      LeftFront.setVelocity(total - 0.5*amountOff, percent);
      RightFront.setVelocity(total + 0.5*amountOff,percent);
    }
  }
  LeftBack.stop(brake);
  RightBack.stop(brake);
  RightFront.stop(brake);
  LeftFront.stop(brake);
}
//Void that controls the drivetrain based on inputs from the joysticks

int speedFactor = 1;

void platformMode() {
  if(Controller1.ButtonX.pressing()){
    speedFactor = 4;
    LeftFront.setStopping(hold);
    LeftBack.setStopping(hold);
    RightFront.setStopping(hold);
    RightBack.setStopping(hold);
  }
  else {
    speedFactor = 1;
    LeftFront.setStopping(coast);
    LeftBack.setStopping(coast);
    RightFront.setStopping(coast);
    RightBack.setStopping(coast);
  }
}

void goSlow(){
  if(Controller1.ButtonX.pressing()){
    LeftBack.spin(forward, 50, percent);
    RightBack.spin(forward, 50, percent);
    LeftFront.spin(forward, 50, percent);
    RightFront.spin(forward, 50, percent);
  }
  else if(Controller1.ButtonB.pressing()){
    LeftBack.spin(reverse, 50, percent);
    RightBack.spin(reverse, 50, percent);
    LeftFront.spin(reverse, 50, percent);
    RightFront.spin(reverse, 50, percent);
  }
  else if(Controller1.ButtonY.pressing()){
    LeftBack.spin(reverse, 50, percent);
    RightBack.spin(forward, 50, percent);
    LeftFront.spin(reverse, 50, percent);
    RightFront.spin(forward, 50, percent);
  }
  else if(Controller1.ButtonA.pressing()){
    LeftBack.spin(forward, 50, percent);
    RightBack.spin(reverse, 50, percent);
    LeftFront.spin(forward, 50, percent);
    RightFront.spin(reverse, 50, percent);
  }
  else{
    RightBack.stop(hold);
    RightFront.stop(hold);
    LeftBack.stop(hold);
    LeftFront.stop(hold);
  }
}

void simpleDrive(){
  double forwardAmount = Controller1.Axis3.position();
  double turnAmount = Controller1.Axis1.position(); //Axis 4 for unified joystick
  
  RightFront.spin(forward, (forwardAmount-turnAmount) / speedFactor, percent);
  RightBack.spin(forward, (forwardAmount-turnAmount) / speedFactor, percent);
  LeftFront.spin(forward, (forwardAmount+turnAmount) / speedFactor, percent);
  LeftBack.spin(forward, (forwardAmount+turnAmount) / speedFactor, percent);
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
    Clamp.setVelocity(200,percent);
    Clamp.spin(forward);
  }
  else if(Controller1.ButtonR1.pressing()){
    Clamp.setVelocity(200, percent);
    Clamp.spin(reverse);
  }
  else{
    Clamp.setStopping(hold);
    Clamp.stop();
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
    wait(5, msec);
  }
  LeftBack.setStopping(hold);
  RightBack.setStopping(hold);
  RightFront.setStopping(hold);
  LeftFront.setStopping(hold);
  LeftBack.stop();
  RightBack.stop();
  RightFront.stop();
  LeftFront.stop();
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
    wait(5, msec);
  }
  LeftBack.setStopping(hold);
  RightBack.setStopping(hold);
  RightFront.setStopping(hold);
  LeftFront.setStopping(hold);
  LeftBack.stop();
  RightBack.stop();
  RightFront.stop();
  LeftFront.stop();
  wait(0.5, sec);
}

//BENS HELPER FUNCTIONS------------------------------------------------------------

void blank() {}

void DriveForward(int amt, int turning = 0, bool smooth = true, int vel = 200) {
  
  if (turning > 0) {
    RightFront.setVelocity(vel - 2 * turning, rpm);
    RightBack.setVelocity(vel - 2 * turning, rpm);
    LeftFront.setVelocity(vel, rpm);
    LeftBack.setVelocity(vel, rpm);
  } else if (turning < 0) {
    LeftFront.setVelocity(vel - 2 * turning, rpm);
    LeftBack.setVelocity(vel - 2 * turning, rpm);
    RightFront.setVelocity(vel, rpm);
    RightBack.setVelocity(vel, rpm);
  }
  
  if (smooth) {
    LeftFront.setPosition(0, degrees);

    LeftFront.spin(forward);
    LeftBack.spin(forward);
    RightFront.spin(forward);
    RightBack.spin(forward);
    if (amt > 0) {
      while (LeftFront.position(degrees) < turning + amt) {
        wait(10, msec);
      }
    } else if (amt < 0) {
      while (LeftFront.position(degrees) > - turning - amt) {
        wait(10, msec);
      }
    }
    LeftFront.stop();
    LeftBack.stop();
    RightFront.stop();
    RightBack.stop();
  } else {
    LeftFront.spinFor(forward, amt + turning, degrees, false);
    LeftBack.spinFor(forward, amt + turning, degrees, false);
    RightFront.spinFor(forward, amt - turning, degrees, false);
    RightBack.spinFor(forward, amt - turning, degrees, true);
  }
}

void ClampDown(int rev = false) {
  Clamp.spinFor(forward, rev ? -40: 40, degrees);
}

void ForkliftDown(int rev = false) {
  Sporklift.spinFor(forward, rev ? -100: 100, degrees);
}

void UnstableFwd(int amt) {
  /*
  bad thing

  error = amt
  vel = 0
  Drivetrain.go();
  while (error >= 0) {
    vel += inertial.acc(xaxis) * 0.02
    error -= vel * 0.02
    Drivetrain.setVelocity((amt-error)*100/amt, percent);
    wait(0.02)
  }
  Drivetrain.stop();
  */
}

//----------------------------------------------------------------------------------

int selected = 0;
std::string autons[9] = {"Disabled", "Left 1 Neutral", "AWP Left", "AWP Right", "Right 2 Neutral", "Skills", "Auton with 95070G", "Right 1 Neutral AWP", "Right Mid Neutral"};
int size = 9;

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
    case 0:{
      break;
    }
    case 1:{
      LeftFront.setStopping(coast);
      LeftBack.setStopping(coast);
      RightBack.setStopping(coast);
      RightFront.setStopping(coast);
      LeftFront.setVelocity(100, percent);
      LeftBack.setVelocity(100, percent);
      RightFront.setVelocity(100, percent);
      RightBack.setVelocity(100, percent);
      LeftFront.setPosition(0, degrees);
      Clamp.setVelocity(100, percent);
      Clamp.spinFor(forward, -45, degrees, false);
      RightLift.spinFor(reverse, 50, degrees, false);
      LeftFront.spin(forward);
      LeftBack.spin(forward);
      RightFront.spin(forward);
      RightBack.spin(forward);
      while(LeftFront.position(degrees)< x + 40 - (elevated ? 20 : 0)){
        wait(10, msec);
      }
      LeftFront.stop();
      LeftBack.stop();
      RightFront.stop();
      RightBack.stop();

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

      LeftFront.spinFor(reverse, x, degrees, false);
      LeftBack.spinFor(reverse, x, degrees, false);
      RightFront.spinFor(reverse, x, degrees, false);
      RightBack.spinFor(reverse, x, degrees, false);
      break; 
    }
    case 2: {
      Clamp.spinFor(forward, 40, degrees, true);
      Clamp.spinFor(forward, -40, degrees, true);
      break;
    }
    case 3: {
      LeftFront.setStopping(coast);
      LeftBack.setStopping(coast);
      RightBack.setStopping(coast);
      RightFront.setStopping(coast);
      break;
    }
    case 4: {
      int y = 500;
      int z = 1230;

      LeftFront.setStopping(coast);
      LeftBack.setStopping(coast);
      RightBack.setStopping(coast);
      RightFront.setStopping(coast);
      LeftFront.setVelocity(100, percent);
      LeftBack.setVelocity(100, percent);
      RightFront.setVelocity(100, percent);
      RightBack.setVelocity(100, percent);
      LeftFront.setPosition(0, degrees);
      Clamp.setVelocity(100, percent);
      LeftFront.spin(forward);
      LeftBack.spin(forward);
      RightFront.spin(forward);
      RightBack.spin(forward);
      while(LeftFront.position(degrees)< x){
        wait(10, msec);
      }
      LeftFront.stop();
      LeftBack.stop();
      RightFront.stop();
      RightBack.stop();
      Clamp.spinFor(forward, 40, degrees);

      //LeftFront.setVelocity(100, rpm);
      //LeftBack.setVelocity(100, rpm);

      LeftFront.setVelocity(100, percent);
      LeftBack.setVelocity(100, percent);
      RightFront.setVelocity(100, percent);
      RightBack.setVelocity(100, percent);

      Controller1.rumble("...");

      LeftFront.spinFor(forward, y, degrees, false);
      LeftBack.spinFor(forward, y, degrees, false);
      RightFront.spinFor(reverse, y, degrees, false);
      RightBack.spinFor(reverse, y, degrees, true);

      Controller1.rumble("------------------");

      Sporklift.spinFor(forward, 500, degrees, true);

      LeftFront.spinFor(forward, -z, degrees, false);
      LeftBack.spinFor(forward, -z, degrees, false);
      RightFront.spinFor(forward, -z, degrees, false);
      RightBack.spinFor(forward, -z, degrees, true);

      Sporklift.spinFor(forward, -500, degrees, true);

      LeftFront.spinFor(forward, z, degrees, false);
      LeftFront.spinFor(forward, z, degrees, false);
      LeftFront.spinFor(forward, z, degrees, false);
      LeftFront.spinFor(forward, z, degrees, false);

      break;
    }
    case 5: {
      //SKILLS
      //EXTREMELY ROUGH. NO WAY IN HECK THIS IS GONNA WORK.

      DriveForward(980);
      ClampDown();
      ForkliftDown();
      wait(1, sec);
      DriveForward(-600, -200, false);
      wait(1, sec);
      ForkliftDown(false);
      wait(1, sec);
      DriveForward(1000, -300, false);

      RightLift.setVelocity(100, percent);
      RightLift.spinFor(reverse, 100, degrees);
      DriveForward(50);
      RightLift.spinFor(forward, 20, degrees);
      ClampDown(true);
      ForkliftDown(true);
      DriveForward(-50);
      RightLift.spinFor(forward, 80, degrees);

      DriveForward(-100);
      DriveForward(100);
      DriveForward(-400, -30);
      DriveForward(400);
      ClampDown();
      DriveForward(-400, 30);
      DriveForward(400);

      RightLift.setVelocity(100, percent);
      RightLift.spinFor(reverse, 100, degrees);
      DriveForward(50);
      ClampDown(true);
      RightLift.spinFor(forward, 100, degrees);
      DriveForward(-50);

      DriveForward(0, 70);
      DriveForward(100, 30);
      ClampDown();
      DriveForward(-100, -30);
      DriveForward(0, -70);

      RightLift.setVelocity(100, percent);
      RightLift.spinFor(reverse, 100, degrees);
      DriveForward(50);
      ClampDown(true);
      RightLift.spinFor(forward, 100, degrees);
      DriveForward(-50);
      break;
    }
    case 6: {
      LeftFront.setStopping(coast);
      LeftBack.setStopping(coast);
      RightBack.setStopping(coast);
      RightFront.setStopping(coast);
      LeftFront.setVelocity(100, percent);
      LeftBack.setVelocity(100, percent);
      RightFront.setVelocity(100, percent);
      RightBack.setVelocity(100, percent);
      LeftFront.setPosition(0, degrees);
      Clamp.setVelocity(100, percent);
      Clamp.spinFor(forward, -45, degrees, false);
      RightLift.spinFor(reverse, 50, degrees, false);
      LeftFront.spin(forward);
      LeftBack.spin(forward);
      RightFront.spin(forward);
      RightBack.spin(forward);
      while(LeftFront.position(degrees)< x){
        wait(10, msec);
      }
      LeftFront.stop();
      LeftBack.stop();
      RightFront.stop();
      RightBack.stop();

      wait(100, msec);

      Clamp.setVelocity(100, percent);

      Clamp.setPosition(0, degrees);
      Clamp.spin(forward);
      while(Clamp.position(degrees) < 40){
        wait(10, msec);
      }

      Clamp.stop();

      LeftFront.setVelocity(100, percent);
      LeftBack.setVelocity(100, percent);
      RightFront.setVelocity(100, percent);
      RightBack.setVelocity(100, percent);

      LeftFront.spinFor(forward, 250, degrees, false);
      LeftBack.spinFor(forward, 250, degrees, false);
      RightFront.spinFor(reverse, 250, degrees, false);
      RightBack.spinFor(reverse, 250, degrees, false);
      break;
    }
    case 7: {
      LeftFront.setStopping(coast);
      LeftBack.setStopping(coast);
      RightBack.setStopping(coast);
      RightFront.setStopping(coast);
      LeftFront.setVelocity(100, percent);
      LeftBack.setVelocity(100, percent);
      RightFront.setVelocity(100, percent);
      RightBack.setVelocity(100, percent);
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
      while(LeftFront.position(degrees)< x){
        wait(10, msec);
      }
      LeftFront.stop();
      LeftBack.stop();
      RightFront.stop();
      RightBack.stop();

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

      LeftFront.spinFor(reverse, x, degrees, false);
      LeftBack.spinFor(reverse, x, degrees, false);
      RightFront.spinFor(reverse, x, degrees, false);
      RightBack.spinFor(reverse, x, degrees, true);

      // turning 90°
      
      LeftFront.spinFor(reverse, 250, degrees, false);
      LeftBack.spinFor(reverse, 250, degrees, false);
      RightFront.spinFor(forward, 250, degrees, false);
      RightBack.spinFor(forward, 250, degrees, true);

      // moving backwards to place ring in alliance goal
      
      LeftFront.spinFor(reverse, 100, degrees, false);
      LeftBack.spinFor(reverse, 100, degrees, false);
      RightFront.spinFor(reverse, 100, degrees, false);
      RightBack.spinFor(reverse, 100, degrees, true);

      // placing ring in alliance goal
      
      Sporklift.spinFor(forward, 50, degrees, true);

      // setting up to pickup alliance goal by moving forward
      
      LeftFront.spinFor(forward, 135, degrees, false);
      LeftBack.spinFor(forward, 135, degrees, false);
      RightFront.spinFor(forward, 135, degrees, false);
      RightBack.spinFor(forward, 135, degrees, true);

      // 2nd part of setting up to pick up alliance goal; moving forklift down
      
      Sporklift.spinFor(forward, 50, degrees, true);
      
      // 3rd part of setting up; moving backwards with forklift down
      
      LeftFront.spinFor(reverse, 150, degrees, false);
      LeftBack.spinFor(reverse, 150, degrees, false);
      RightFront.spinFor(reverse, 150, degrees, false);
      RightBack.spinFor(reverse, 150, degrees, true);

      // 4th part of getting AWP; forklifting up to pick up goal
      
      Sporklift.spinFor(reverse, 100, degrees, true);
      
      // moving forward to get the AWP
      
      LeftFront.spinFor(forward, 145, degrees, false);
      LeftBack.spinFor(forward, 145, degrees, false);
      RightFront.spinFor(forward, 145, degrees, false);
      RightBack.spinFor(forward, 145, degrees, true);
      
      break; 
    }
    case 8: {
      

      LeftFront.setStopping(coast);
      LeftBack.setStopping(coast);
      RightBack.setStopping(coast);
      RightFront.setStopping(coast);
      LeftFront.setVelocity(100, percent);
      LeftBack.setVelocity(100, percent);
      RightFront.setVelocity(100, percent);
      RightBack.setVelocity(100, percent);
      LeftFront.setPosition(0, degrees);
      Clamp.setVelocity(100, percent);
      Clamp.spinFor(forward, -45, degrees, false);
      RightLift.spinFor(reverse, 50, degrees, false);
      LeftFront.spin(forward);
      LeftBack.spin(forward);
      RightFront.spin(forward);
      RightBack.spin(forward);
      while(LeftFront.position(degrees)< x + 430 - (elevated ? 20 : 0)){
        wait(10, msec);
      }
      LeftFront.stop();
      LeftBack.stop();
      RightFront.stop();
      RightBack.stop();

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

      LeftFront.spinFor(reverse, x + 230, degrees, false);
      LeftBack.spinFor(reverse, x + 230, degrees, false);
      RightFront.spinFor(reverse, x + 230, degrees, false);
      RightBack.spinFor(reverse, x + 230, degrees, false);
      break;
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

int triggerPercent = 30;

void testHeat() {
  for (int i = 0; i < sizeof(heatedBools); i++) {
    if (!heatedBools[i] && heatedMotors[i].temperature() > triggerPercent) {
      Controller1.rumble("-.-");
    }
    heatedBools[i] = (heatedMotors[i].temperature() > triggerPercent);
  } //extras to prevent controller spamming
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
    testHeat();
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
