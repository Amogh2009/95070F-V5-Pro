using namespace vex;

extern brain Brain;

// VEXcode devices
extern motor LeftFront;
extern motor LeftBack;
extern motor RightFront;
extern motor RightBack;
extern motor RightLift;
extern motor Clamp;
extern inertial Inertial;
extern controller Controller1;
extern digital_out OldbackPiston;
extern motor Sporklift;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );