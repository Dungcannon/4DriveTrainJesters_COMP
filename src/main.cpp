/* PORT USAGE
leftMotorA          1
leftMotorB          2
rightMotorA         3
rightMotorB         4
*LeftDriveSmart     1,2
*RightDriveSmart    3,4
DrivetrainInertial  5
*Drivetrain         1,2, 3,4, 5
Collector           6
Catapult            8
DoubleReverseMotorA 9
DoubleReverseMotorB 10
*DoubleReverse      9,10
ThreeWirePort       22
SolenoidA           A
SolenoidB           B
*/

#include <math.h>
#include "vex.h"

using namespace vex;
brain Brain;


// Robot configuration code.
motor leftMotorA = motor(PORT1, ratio18_1, true);
motor leftMotorB = motor(PORT2, ratio18_1, true);
motor_group LeftDriveSmart = motor_group(leftMotorA, leftMotorB);
motor rightMotorA = motor(PORT3, ratio18_1, false);
motor rightMotorB = motor(PORT4, ratio18_1, false);
motor_group RightDriveSmart = motor_group(rightMotorA, rightMotorB);
inertial DrivetrainInertial = inertial(PORT5);
smartdrive Drivetrain = smartdrive(RightDriveSmart, LeftDriveSmart, DrivetrainInertial, 319.19, 320, 40, mm, 1);

controller Controller1 = controller(primary);
motor Collector = motor(PORT6, ratio18_1, false);

triport ThreeWirePort = triport(PORT22);
digital_out SolenoidA = digital_out(ThreeWirePort.A);
digital_out SolenoidB = digital_out(ThreeWirePort.B);
motor Catapult = motor(PORT8, ratio18_1, false);

motor DoubleReverseMotorA = motor(PORT8, ratio36_1, false);
motor DoubleReverseMotorB = motor(PORT9, ratio36_1, false);
motor_group DoubleReverse = motor_group(DoubleReverseMotorA, DoubleReverseMotorB);

digital_in HomeJumper = digital_in(ThreeWirePort.G);
digital_in EnemyJumper = digital_in(ThreeWirePort.H);


void calibrateDrivetrain() {
  wait(200, msec);
  Brain.Screen.print("Calibrating");
  Brain.Screen.newLine();
  Brain.Screen.print("Inertial");
  DrivetrainInertial.calibrate();
  while (DrivetrainInertial.isCalibrating()) {
    wait(100, msec);
  }

  // Clears the screen and returns the cursor to row 1, column 1.
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
}// define variable for remote controller enable/disable

bool RemoteControlCodeEnabled = true;
bool Controller1LeftShoulderControlMotorsStopped = true;
bool Controller1RightShoulderControlMotorsStopped = true;
bool Controller1YAButtonsControlMotorsStopped = true;
bool DrivetrainLNeedsToBeStopped_Controller1 = true;
bool DrivetrainRNeedsToBeStopped_Controller1 = true;

bool DrivetrainToggle = false;
bool SolenoidToggle = false;
bool solToggleL = false;
bool solToggleR = false;
bool isJoystickSwapped = false;


competition Competition;



/* FUNCTIONS */



void LockIt(){
  LeftDriveSmart.setStopping(hold);
  RightDriveSmart.setStopping(hold);
}
void UnlockIt(){
  LeftDriveSmart.setStopping(coast);
  RightDriveSmart.setStopping(coast);
}

void MotorDrive(double x, double y){
  LeftDriveSmart.spinFor(x, degrees, false);
  RightDriveSmart.spinFor(y, degrees, true);
}

void TriggerHappy(int timems) 
{
  Catapult.spin(reverse);
  wait(timems, msec);
  Catapult.stop();
  return;
}

void DoubleSolenoid(bool isExtended){
  SolenoidA.set(isExtended);
  SolenoidB.set(isExtended);
  wait(100, msec);
}


/* - PREAUTON / AUTON / COMP - */
void EnemySideStart(){ // When Jumper is on port H, run this code
  DrivetrainInertial.calibrate();
  Drivetrain.setHeading(0, degrees);
  Collector.spin(reverse); // Take in Load Triball
  Drivetrain.driveFor(-60, inches);
  Drivetrain.turnToHeading(269, degrees);
  Drivetrain.driveFor(-6, inches);
  Drivetrain.driveFor(6, inches);
  Drivetrain.turnToHeading(90, degrees);
  Collector.spin(forward);
  DoubleSolenoid(true);

  Drivetrain.drive(reverse);
  wait(1.5, seconds);
  Drivetrain.stop();
  
  Drivetrain.turnToHeading(42, degrees);
  Drivetrain.driveFor(69, inches);
  Drivetrain.turnToHeading(90, degrees);
  Drivetrain.driveFor(-15, inches);
}


// NUMBERS AS PLACEHOLDER
void HomeSideStart(){ // When Jumper is on port G, run this code
  DrivetrainInertial.calibrate();
  Drivetrain.setHeading(0, degrees);
  Collector.spin(reverse);
  Drivetrain.turnToHeading(15, degrees);
  Drivetrain.driveFor(-15, inches);
  Drivetrain.driveFor(10, inches);
  Drivetrain.turnToHeading(15, degrees);
  Drivetrain.driveFor(10, inches);
  SolenoidB.set(true);
  Drivetrain.turnToHeading(0, degrees);
  Drivetrain.driveFor(15, inches);

}



void pre_auton(void) {
  calibrateDrivetrain();
  Catapult.setVelocity(75, percent); // catapult shoot speed
  Catapult.setMaxTorque(100, percent); // catapult torque
  Collector.setVelocity(100, percent);
  Catapult.setStopping(hold);
  DoubleSolenoid(false);
}

void autonomous(void) {
  Drivetrain.setDriveVelocity(200, rpm);
  if(EnemyJumper.value() == 0){
    Brain.Screen.print("This is on port H, ENEMY SIDE");
    EnemySideStart();
  }
  else if (HomeJumper.value() == 0){
    Brain.Screen.print("This is on Port G, HOME SIDE");
    HomeSideStart();
  }
  else {
    Brain.Screen.print("ERROR: Jumper not detected. Using Fallback code.");
    DrivetrainInertial.calibrate();
    Drivetrain.setDriveVelocity(200, rpm);
    Drivetrain.setHeading(0, degrees);
    Drivetrain.drive(forward);
    wait(1.5, seconds);
    Drivetrain.stop();
    Drivetrain.drive(reverse);
    wait(400, msec);
    Drivetrain.stop();
    Drivetrain.drive(forward);
    wait(600, msec);
    Drivetrain.stop();

    Drivetrain.drive(reverse);
    wait(500, msec);
    Drivetrain.stop(); 

    Drivetrain.turnToHeading(180, degrees);
  }
}

void usercontrol(void) {
  // User control code here, inside the loop
  while (1) {
    if(RemoteControlCodeEnabled) {
      // calculate the drivetrain motor velocities from the controller joystick axies
      // left = Axis3
      // right = Axis2
      int drivetrainLeftSideSpeed = Controller1.Axis3.position();
      int drivetrainRightSideSpeed = Controller1.Axis2.position();
      
      // check if the value is inside of the deadband range
      if (drivetrainLeftSideSpeed < 5 && drivetrainLeftSideSpeed > -5) {
        // check if the left motor has already been stopped
        if (DrivetrainLNeedsToBeStopped_Controller1) {
          // stop the left drive motor
          LeftDriveSmart.stop();
          // tell the code that the left motor has been stopped
          DrivetrainLNeedsToBeStopped_Controller1 = false;
        }
      } else {
        // reset the toggle so that the deadband code knows to stop the left motor nexttime the input is in the deadband range
        DrivetrainLNeedsToBeStopped_Controller1 = true;
      }
      // check if the value is inside of the deadband range
      if (drivetrainRightSideSpeed < 5 && drivetrainRightSideSpeed > -5) {
        // check if the right motor has already been stopped
        if (DrivetrainRNeedsToBeStopped_Controller1) {
          // stop the right drive motor
          RightDriveSmart.stop();
          // tell the code that the right motor has been stopped
          DrivetrainRNeedsToBeStopped_Controller1 = false;
        }
      } else {
        // reset the toggle so that the deadband code knows to stop the right motor next time the input is in the deadband range
        DrivetrainRNeedsToBeStopped_Controller1 = true;
      }
      
      // only tell the left drive motor to spin if the values are not in the deadband range
      if (DrivetrainLNeedsToBeStopped_Controller1) {
        LeftDriveSmart.setVelocity(drivetrainLeftSideSpeed, percent);
        LeftDriveSmart.spin(forward);
      }
      // only tell the right drive motor to spin if the values are not in the deadband range
      if (DrivetrainRNeedsToBeStopped_Controller1) {
        RightDriveSmart.setVelocity(drivetrainRightSideSpeed, percent);
        RightDriveSmart.spin(forward);
      }
      // check the ButtonL1/ButtonL2 status to control Collector
      if (Controller1.ButtonL1.pressing()) {
        Collector.spin(forward);
        Controller1LeftShoulderControlMotorsStopped = false;
      } else if (Controller1.ButtonL2.pressing()) {
        Collector.spin(reverse);
        Controller1LeftShoulderControlMotorsStopped = false;
      } else if (!Controller1LeftShoulderControlMotorsStopped) {
        Collector.stop();
        // set the toggle so that we don't constantly tell the motor to stop when the buttons are released
        Controller1LeftShoulderControlMotorsStopped = true;
      }
      // check the ButtonR1/ButtonR2 status to control Catapult
      if (Controller1.ButtonR1.pressing()) {
        Catapult.spin(forward);
        Controller1RightShoulderControlMotorsStopped = false;
      } else if (Controller1.ButtonR2.pressing()) {
        Catapult.spin(reverse);
        Controller1RightShoulderControlMotorsStopped = false;
      } else if (!Controller1RightShoulderControlMotorsStopped) {
        Catapult.stop();
        // set the toggle so that we don't constantly tell the motor to stop when the buttons are released
        Controller1RightShoulderControlMotorsStopped = true;
      }
      // check the ButtonY/ButtonA status to control DoubleReverse
      if (Controller1.ButtonY.pressing()) {
        DoubleReverse.spin(forward);
        Controller1YAButtonsControlMotorsStopped = false;
      } else if (Controller1.ButtonA.pressing()) {
        DoubleReverse.spin(reverse);
        Controller1YAButtonsControlMotorsStopped = false;
      } else if (!Controller1YAButtonsControlMotorsStopped) {
        DoubleReverse.stop();
        // set the toggle so that we don't constantly tell the motor to stop when the buttons are released
        Controller1YAButtonsControlMotorsStopped = true;
      }
      // Toggles the drivetrain to be either coast or hold
      if (Controller1.ButtonUp.pressing()){
        if (DrivetrainToggle == false){
          LockIt();
          DrivetrainToggle = true;
        }
        else if (DrivetrainToggle == true){
          UnlockIt();
          DrivetrainToggle = false;
        }
      }
      
      // Toggles the bottom button to toggle both solenoid sides.
      if (Controller1.ButtonDown.pressing()){
       if (SolenoidToggle == false){
          DoubleSolenoid(true);
          SolenoidToggle = true;
          solToggleL = true;
          solToggleR = true;
        }
        else if (SolenoidToggle == true){
          DoubleSolenoid(false);
          SolenoidToggle = false;
          solToggleL = false;
          solToggleR = false;
        }
      }

      // Left Solenoid Toggle
      if (Controller1.ButtonLeft.pressing()){
         if (solToggleL == false){
          SolenoidA.set(true);
          solToggleL = true;
          wait(100, msec);
        }
        else if (solToggleL == true){
          SolenoidA.set(false);
          solToggleL = false;
          wait(100, msec);
        }
      }

      if (Controller1.ButtonRight.pressing()){
        if (solToggleR == false){
          SolenoidB.set(true);
          solToggleR = true;
          wait(100, msec);
        }
        else if (solToggleR == true){
          SolenoidB.set(false);
          solToggleR = false;
          wait(100, msec);
        }
      }

      if (Controller1.ButtonX.pressing()){
        if (isJoystickSwapped == false){
          isJoystickSwapped = true;
        }
        else if (isJoystickSwapped == true){
          isJoystickSwapped = false;
        }
      }

      if (Controller1.ButtonA.pressing()){
        Catapult.spinTo(-540, degrees);
      }
          wait(20, msec);
                    
    }
  } 
}

int main() {
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  pre_auton();

  while (true) {
    wait(100, msec);
  }
}
