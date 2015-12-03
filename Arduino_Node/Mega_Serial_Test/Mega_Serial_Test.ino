
#include <DualVNH5019MotorDriver.h> // For use with the Dual motor drivers from Pololu
#include <Encoder.h>
#include <PID_v1.h>
#include <Servo.h>
#include <Messenger.h>

//-------Define variables here-------------------------------
// Behavior Variable
int activeBehavior = 0;

// ---PWM Pin Setting---

// PWM pins for the two drive motors
const int PWMDrivePin_Left = 11;    // Timer 1 16-bit
const int PWMDrivePin_Right = 12;   // Timer 1 16-bit

// Additional PWM Pins for Manipulator. May not be used
const int PWMWristPin = 7;
const int PWMGraspPin = 6;

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
int leftDistance, rightDistance; // Intermediate before the setpoints are actually changed


boolean leftDone = false, rightDone = false;

// PID Tuning Paramters
double lKp = 3, lKi = 0, lKd = 1.5;
double rKp = 3, rKi = 0, rKd = 1.5;

double K = 10;

// Serial Communications stuff
int temp;
boolean newWristGraspCmd = false;
boolean newMotorDistance = false;
Messenger piMessage = Messenger(':');
int wristCmd = 180;
int graspCmd = 50;
unsigned int serialReadTimer = 0;
unsigned int serialPeriod = 20;


// Define drive motor object
DualVNH5019MotorDriver driveMotors(Drive_INA1,Drive_INB1,PWMDrivePin_Left,\
Drive_EN1DIAG1,leftCurrentPin,Drive_INA2,Drive_INB2,PWMDrivePin_Right,Drive_EN2DIAG2,\
rightCurrentPin, 1);

// Define encoder object
Encoder leftEncoder(leftEncoderAPin, leftEncoderBPin);
Encoder rightEncoder(rightEncoderBPin, rightEncoderBPin);

// Define servo objects
Servo Wrist;
Servo Grasp;

unsigned int timer = 0;

boolean driveStop = false;

/* Define PID object
 *  PID will take the velocity as an input. So the derivative will be calculated be calling the PID function
 */
PID leftPID(&leftInput, &leftOutput, &leftSetpoint, lKp, lKi, lKd, DIRECT);
PID rightPID(&rightInput, &rightOutput, &rightSetpoint, rKp, rKi, rKd, DIRECT);

void messageParse(){
  // This will set the variables that need to be changed
  // from the message
  if (piMessage.available()){
    temp = piMessage.readInt();
    if (temp != 9){ activeBehavior = temp;}
    
    leftDistance = piMessage.readInt();
    rightDistance = piMessage.readInt();
    if (leftDistance != 99 && rightDistance != 99){ 
      newMotorDistance = true;
    }
    
    wristCmd = piMessage.readInt();
    graspCmd = piMessage.readInt();
    if (wristCmd != 999 || graspCmd != 999){ 
      newWristGraspCmd = true;
    }    
  }  
}

void setup() {
  // put your setup code here, to run once:
  
  // Initialize drive motor object
  driveMotors.init();
  
  Wrist.attach(PWMWristPin);
  Grasp.attach(PWMGraspPin);  
  
  Wrist.write(wristCmd);
  Grasp.write(graspCmd);


  /* Set PID output limits
   *  This limits correspond to the inputs in the VNH5019 Motor Driver setSpeed method.
   */
  leftPID.SetSampleTime(50);
  rightPID.SetSampleTime(50); 
   
  leftPID.SetOutputLimits(-10,10);
  rightPID.SetOutputLimits(-10,10);

  leftPID.SetMode(AUTOMATIC);
  rightPID.SetMode(AUTOMATIC);
  
  rightPID.SetControllerDirection(REVERSE);
  
  Serial.begin(9600);
  piMessage.attach(messageParse);
  
  leftSetpoint = 3;
  rightSetpoint = 3;
  

  
}

void loop() {
  // put your main code here, to run repeatedly:
  unsigned int currentTime = millis();
  
  if (currentTime-serialReadTimer > serialPeriod){
    serialReadTimer = currentTime;
    while( Serial.available() ) piMessage.process(Serial.read());
  }
  
  if (newWristGraspCmd){
    Serial.println("Repeat Back " + String(activeBehavior)+ " " + String(wristCmd) + " " + String(graspCmd));  
    // Make sure to check inout bounds
    if((wristCmd >= 40 && wristCmd <= 180) && wristCmd != 999){
      Wrist.write(wristCmd);
      //Serial.println("Repeat Back " + String(panCmd));  
    }
    if((graspCmd >= 45 && graspCmd <= 135) && graspCmd != 999){
      Grasp.write(graspCmd);
      //Serial.println("Repeat Back " + String(State)+ " " + String(panCmd) +" " String(tiltCmd));  
    } 
    newWristGraspCmd = false;
  }  
  
  if (newMotorDistance){
    Serial.println("Repeat Back " + String(activeBehavior)+ " " + String(leftDistance) + " " + String(rightDistance));  
    newMotorDistance = false;
  }  

  // Need to convert to actual position measurements.
  leftInput = leftEncoder.read()*C; 
  if(!driveStop) {leftDone = leftPID.Compute();} 
  
  rightInput = rightEncoder.read()*C;
  if(!driveStop){rightDone = rightPID.Compute();}

  /*
  if(currentTime - timer > 2000){
    Serial.print(leftInput);
    Serial.print(' ');
    Serial.println(rightInput);
    
  }
  */
  // Missing a conversion for speed.
  if (!(abs(leftInput) > (10-0.86)) && !(rightInput>(10-0.7))){
    if((leftDone && rightDone)){
      // Set the speeds together
      driveMotors.setSpeeds(K*leftOutput, K*rightOutput);
      leftDone = false;
      rightDone = false;
    }
  }
  else{

    leftSetpoint = 0;
    rightSetpoint = 0;
    driveMotors.setSpeeds(0, 0);
    driveStop = true;
    //Serial.print(leftInput);
    //Serial.print(' ');
    //Serial.println(rightInput);
  }
  
}


