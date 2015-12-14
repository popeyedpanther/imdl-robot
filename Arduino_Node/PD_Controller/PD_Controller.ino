#include <DualVNH5019MotorDriver.h> // For use with the Dual motor drivers from Pololu
#include <Encoder.h>
#include <PID_v1.h>

//-------Define variables here-------------------------------

// ---PWM Pin Setting---

// PWM pins for the two drive motors
const int PWMDrivePin_Left = 11;    // Timer 1 16-
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
double rightOldPosition;
long leftNewPosition = 0;
double rightNewPosition = 0;

// Encoder Conversion Constant
double C = (3.54*3.14159)/4741.41;

// PID input variables
double leftSetpoint, leftInput, leftOutput;
double rightSetpoint, rightInput, rightOutput;

boolean leftDone = false, rightDone = false;

// PID Tuning Paramters
double lKp = 15, lKi = 5, lKd = 0;
double rKp = 15, rKi = 5, rKd = 0;

double K = 1;

// Define drive motor object
DualVNH5019MotorDriver driveMotors(Drive_INA1,Drive_INB1,PWMDrivePin_Left,\
Drive_EN1DIAG1,leftCurrentPin,Drive_INA2,Drive_INB2,PWMDrivePin_Right,Drive_EN2DIAG2,\
rightCurrentPin, 1);

// Define encoder object
Encoder leftEncoder(leftEncoderBPin, leftEncoderAPin);
Encoder rightEncoder(rightEncoderAPin, rightEncoderBPin);

/* Define PID object
 *  PID will take the velocity as an input. So the derivative will be calculated be calling the PID function
 */
PID leftPID(&leftInput, &leftOutput, &leftSetpoint, lKp, lKi, lKd, DIRECT);
PID rightPID(&rightInput, &rightOutput, &rightSetpoint, rKp, rKi, rKd, DIRECT);


// Random test variables
unsigned int changespeedsTimer = 0;
unsigned int changeSpeeds = 10000;
boolean stopped = false;

unsigned int timer = 0;

boolean driveStop = false;

double stopDist = 20;
double leftOffset= 0.75, rightOffset = 0.75;



void setup() {
  // put your setup code here, to run once:
  
  // Initialize drive motor object
  driveMotors.init();

  /* Set PID output limits
   *  This limits correspond to the inputs in the VNH5019 Motor Driver setSpeed method.
   */
  leftPID.SetSampleTime(50);
  rightPID.SetSampleTime(50); 
   
  leftPID.SetOutputLimits(-100,100);
  rightPID.SetOutputLimits(-100,100);

  leftPID.SetMode(AUTOMATIC);
  rightPID.SetMode(AUTOMATIC);
  
  rightPID.SetControllerDirection(REVERSE);
  
  Serial.begin(9600);
  
  leftSetpoint = 5;
  rightSetpoint = 5;
  

  
}

void loop() {
  // put your main code here, to run repeatedly:

  
  unsigned int currentTime = millis();

  /*
  if((currentTime - changespeedsTimer) > changeSpeeds){
    changespeedsTimer = currentTime;
    leftSetpoint = 0;
    rightSetpoint = 0;

    if(stopped){
          leftSetpoint = -5;
          rightSetpoint = 5;
    }
    
    stopped = true;  
  }abs(leftInput)>(stopDist-leftOffset)
  */

  // Need to convert to actual position measurements.
  leftInput = leftEncoder.read()*C; 
  if(!driveStop) {leftDone = leftPID.Compute();} 
  
  rightInput = rightEncoder.read()*C;
  if(!driveStop){rightDone = rightPID.Compute();}
  
  if(abs(leftInput)>(stopDist-leftOffset) && abs(rightInput)>(stopDist-rightOffset) && rightSetpoint != 0){
    Serial.print(leftInput);
    Serial.print(' ');
    Serial.println(rightInput);
    leftSetpoint = 0;
    rightSetpoint = 0;
  }
  
  if((leftDone && rightDone)){
      // Set the speeds together
      //Serial.print(K*leftOutput);
      //Serial.print("  ");
      //Serial.println(K*rightOutput);
      driveMotors.setSpeeds(K*leftOutput, K*rightOutput);
      leftDone = false;
      rightDone = false;
    }
    
  if((currentTime - timer > 200)){
    timer = currentTime;
    Serial.print(leftInput);
    Serial.print(' ');
    Serial.println(rightInput);
    rightOldPosition = rightNewPosition;
    rightNewPosition = rightInput;
    if(abs(rightNewPosition - rightOldPosition)< 0.0001){
      //Serial.end();
    }
    
  }
  

  /*
  // Missing a conversion for speed.
  if (!(abs(leftInput) > (10-0.86)) && !(rightInput>(10-0.7))){
    if((leftDone && rightDone)){
      // Set the speeds together
      driveMotors.setSpeeds(K*leftOutput, K*rightOutput);
      leftDone = false;
      rightDone = false;
    }
  }

  /*
  else{

    leftSetpoint = 0;
    rightSetpoint = 0;
    driveMotors.setSpeeds(0, 0);
    driveStop = true;
    Serial.print(leftInput);
    Serial.print(' ');
    Serial.println(rightInput);
  }
  */
}


