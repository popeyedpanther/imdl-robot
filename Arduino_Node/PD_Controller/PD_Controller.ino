#include <DualVNH5019MotorDriver.h> // For use with the Dual motor drivers from Pololu
#include <Encoder.h>
#include <PID_v1.h>

//-------Define variables here-------------------------------

// ---PWM Pin Setting---

// PWM pins for the two drive motors
const int PWMDrivePin_Left = 11;    // Timer 1 16-bit
const int PWMDrivePin_Right = 12;   // Timer 1 16-bit

// ---Pololu Motor Pins---
const int Drive_INA1=22;     // Left Drive Motor
const int Drive_INB1=23;     // Left Drive Motor
const int Drive_EN1DIAG1=24; // Left Drive Motor

const int Drive_INA2=26;     // Right Drive Motor
const int Drive_INB2=27;     // Right Drive Motor
const int Drive_EN2DIAG2=25; // Right Drive Motor

// Current Sensors
const int leftCurrentPin = 2;    // A8, CS1
const int rightCurrentPin = 3;   // A9, CS2

// Encoder Input
const int leftEncoderAPin = 21;   // Digital Pin 21, 2 interrupt pin for Mega 2560
const int leftEncoderBPin = 20;   // Digital Pin 20, 3 interrupt pin for Mega 2560

const int rightEncoderAPin = 19;  // Digital Pin 19, 4 interrupt pin for Mega 2560
const int rightEncoderBPin = 18;  // Digital Pin 18, 5 interrupt pin for Mega 2560

// Used for sensor sampling times
unsigned long currentMillis;            // Stores the current time reading in milliseconds

// Encoder Counting Variables
long leftOldPosition;
long rightOldPosition;
long leftNewPosition = 0;
long rightNewPosition = 0;

// Encoder Conversion Constant
double C = (3.54*3.14159)/4741.41;

// PID input variables
double leftSetpoint, leftInput, leftOutput;
double rightSetpoint, rightInput, rightOutput;

boolean leftDone = false, rightDone = false;

// PID Tuning Paramters
double lKp = 2, lKi = 0, lKd = 1;
double rKp = 2, rKi = 0, rKd = 1;

// Define drive motor object
DualVNH5019MotorDriver driveMotors(Drive_INA1,Drive_INB1,PWMDrivePin_Left,\
Drive_EN1DIAG1,leftCurrentPin,Drive_INA2,Drive_INB2,PWMDrivePin_Right,Drive_EN2DIAG2,\
rightCurrentPin, 1);

// Define encoder object
Encoder leftEncoder(leftEncoderAPin, leftEncoderBPin);
Encoder rightEncoder(rightEncoderBPin, rightEncoderBPin);

/* Define PID object
 *  PID will take the velocity as an input. So the derivative will be calculated be calling the PID function
 */
PID leftPID(&leftInput, &leftOutput, &leftSetpoint, lKp, lKi, lKd, DIRECT);
PID rightPID(&rightInput, &rightOutput, &rightSetpoint, rKp, rKi, rKd, DIRECT);

void setup() {
  // put your setup code here, to run once:
  
  // Initialize drive motor object
  driveMotors.init();

  /* Set PID output limits
   *  This limits correspond to the inputs in the VNH5019 Motor Driver setSpeed method.
   */
  leftPID.SetOutputLimits(-100,100);
  rightPID.SetOutputLimits(-100,100);

  leftPID.SetMode(AUTOMATIC);
  rightPID.SetMode(AUTOMATIC);
  
}

void loop() {
  // put your main code here, to run repeatedly:

  // Need to convert to actual position measurements.
  leftInput = leftEncoder.read()*C; 
  leftDone = leftPID.Compute();

  rightInput = rightEncoder.read()*C;
  rightDone = rightPID.Compute();

  // Missing a conversion for speed.

  if(leftDone && rightDone){
  // Set the speeds together
  driveMotors.setSpeeds(leftOutput, rightOutput);
  leftDone = false;
  rightDone = false;
  }
}


