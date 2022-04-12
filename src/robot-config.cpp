#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor LeftFront = motor(PORT3, ratio18_1, true);
motor LeftBack = motor(PORT14, ratio18_1, true);
motor RightFront = motor(PORT8, ratio18_1, false);
motor RightBack = motor(PORT10, ratio18_1, false);
motor RightLift = motor(PORT15, ratio36_1, true);
motor Clamp = motor(PORT1, ratio36_1, false);
inertial Inertial = inertial(PORT21);
controller Controller1 = controller(primary);
digital_out OldbackPiston = digital_out(Brain.ThreeWirePort.D);
motor Sporklift = motor(PORT7, ratio36_1, false);
motor Clamp2 = motor(PORT16, ratio18_1, true);
motor RightMiddle = motor(PORT18, ratio18_1, true);
motor LeftMiddle = motor(PORT17, ratio18_1, false);

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