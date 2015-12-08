
/* Patrick's Block Header
 *  
 *  
 *  
 *  
 *  
 *  
 *  
 */


//-------Include Libraries-----------------------------------
#include <Servo.h>
#include <DualVNH5019MotorDriver.h> // For use with the Dual motor drivers from Pololu
#include <Encoder.h>
#include <PID_v1.h>
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

// ---Sensor Pins---

// Bump Sensors
const int leftBumpPin = 2;            // Digital Pin 2, 0 interrupt pin for Mega 2560
const int rightBumpPin = 3;           // Digital Pin 3, 1 interrupt pin for MEga 2560

// Encoder Input
const int leftEncoderAPin = 21;   // Digital Pin 21, 2 interrupt pin for Mega 2560
const int leftEncoderBPin = 20;   // Digital Pin 20, 3 interrupt pin for Mega 2560

const int rightEncoderAPin = 19;  // Digital Pin 19, 4 interrupt pin for Mega 2560
const int rightEncoderBPin = 18;  // Digital Pin 18, 5 interrupt pin for Mega 2560

// IR sensors
const int leftIRPin = 0;          // A0
const int rightIRPin = 1;         // A1

// Current Sensors
const int leftCurrentPin = 2;    // A8, CS1
const int rightCurrentPin = 3;   // A9, CS2

// ---Sensor sampling periods---
const unsigned long irPeriod = 100;      // Sampling period for IR Sensors (ms)
const unsigned long currentPeriod = 100; // Sampling period for current sensor (ms)
const unsigned long encoderPeriod = 50;  // Sampling period for encoder sensor (ms)
const unsigned long serialPeriod = 75;  // Sampling period for serial read (ms)
const unsigned long stoppedPeriod = 150; // Sampling period for stopped measurement (ms)


// Used for sensor sampling times
unsigned long currentMillis;            // Stores the current time reading in milliseconds
unsigned long previousMillis_IR=0;      // Stores the previous time the IR sensors were read
unsigned long previousMillis_Current=0; // Stores the previous time the Current sensors were read
unsigned long previousMillis_Encoder=0; // Stores the previous time the encoder were read
unsigned long previousMillis_Serial=0;  // Stores the previous time the serial weas read
unsigned long previousMillis_Stopped=0; // Stores the previous time if the robot was stopped

// ---Constants---
const int eps = 0.75;

// Encoder Conversion Constant
const double C = (3.54*3.14159)/4741.41;

// ---Define non-constant variables---
int inByte = 0;     // Variable that will store incoming byte from serial

int irLeft[4] = {0, 0, 0, 0};     // Stores the past 3 Left IR Readings for use in an average
int irRight[4] = {0, 0, 0, 0};    // Stores the past 3 Right IR Reading for use in an average
int irValue;                   // Temporary value to store mesured IR reading
float irLeftAvg;                 // Used to store left IR average reading
float irRightAvg;                // Used to store right IR average reading

int currentLeft[4] = {0, 0, 0, 0};   // Stores Left Drive Motor Current Reading
int currentRight[4] = {0, 0, 0, 0};  // Stores Right Drive Motor Current Reading
int currentValue;                   // For current measurement (amps)
float currentLeftAvg;               // Used to store the average current sensor reading for the left motor            
float currentRightAvg;              // Used to store the average current sensor reading for the right motor   

// Encoder Counting Variables
long leftOldPosition;
long rightOldPosition;
long leftNewPosition = 0;
long rightNewPosition = 0;

// PID input variables
double leftSetpoint, leftInput, leftOutput;
double rightSetpoint, rightInput, rightOutput;

boolean leftDone = false, rightDone = false;
double leftOffset = 0.50, rightOffset = 1.25; // Distance offsets to account stopping time.

// PID Tuning Paramters
double lKp = 2.75, lKi = 0, lKd = 2.25;
double rKp = 3.75, rKi = 0, rKd = 2.25;

double K = 5;

// Serial Communications stuff
double temp = 0;
boolean newBehavior = false;
boolean newDistance = false;   // Signifies if a new command has been recieved           
boolean newWristGraspCmd = false;   // Signifies if a new command has been recieved
boolean newRequest = false;
boolean newRobotSpeed = false;
boolean OAoff = false;              // To turn obstacle avoidance on or off
boolean gripOff = false;            // To turn gripper motion off


Messenger piMessage = Messenger(':');

int wristCmd = 160, graspCmd = 130;

double leftDistance = 0, rightDistance = 0;
int requestState = 0, requestComplete = 0, oaState = 0;
boolean oaOverride = false;
boolean isStopped = true;
double robotSpeed = 5;

// State Update Variables
float dx = 0, dy = 0, dtheta = 0;
int motionDirection = 0, oldMotionDirection = 0;
double leftStartPoint = 0, rightStartPoint = 0;
double leftStopPoint = 0, rightStopPoint = 0;


// Sensor Flags
boolean irFlag =false;              // Will be set true if an IR condition is met
  int irRecomnd = 0;                // Recommendation will be set depending on specific combintaion of IR readings
boolean currentFlag = false;        // Will be set true if Current sense condition is met
  int currentRecomnd = 0;           // Will be set depending of specific combinations
volatile boolean bumpFlag = false;  // Will be set true if a bump sensor is triggered
  volatile int bumpRecomnd = 0;     // Will indicate whether left or right sensor was triggered



// Arbiter Variables
boolean actionLock = false;
boolean actionOverride = false;
unsigned long timerLockout = 750;
unsigned long actionTimer =0;
boolean Reverse = false;
boolean Turn = false;

//----Define Objects-----

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

// Define servo objects
Servo Wrist;
Servo Grasp;

// Test Variables
const String leftString = "Left IR Reading: ";
const String rightString = "Right IR Reading: ";

boolean readyBypass = false;

//------- Message parsing function -------------------------

void messageParse(){
  // This will set the variables that need to be changed
  // from the message
  if (piMessage.available()){
    temp = piMessage.readInt();
    if (temp != 9 && temp >= 0 && temp <= 4){
      newBehavior = true;
      activeBehavior = temp;  
    }
    
    temp = piMessage.readDouble();
    if( temp != 99){leftDistance = temp;}
    temp = piMessage.readDouble();
    if( temp != 99){rightDistance = temp;}
    if (leftDistance != 99 || rightDistance != 99){ newDistance = true; }

    wristCmd = piMessage.readInt();
    graspCmd = piMessage.readInt();  
    if (wristCmd != 999 || graspCmd != 999){ newWristGraspCmd = true;}
      
    temp = piMessage.readDouble();
    if (temp != 99 && (temp >= 0 && temp < 15)){ robotSpeed = temp; }

    temp = piMessage.readInt();
    if (temp != 9 && temp >= 0 && temp <= 2){ 
      newRequest = true;
      requestState = temp;
    } 

    // Repurpose this for some other information
    oaState  = piMessage.readInt();
    if ( oaState == 0 ) {OAoff = true;}
  }  
}

 
//-------Setup Function-----------------------------------

void setup()  // Needs to stay in setup until all necessary communications can be verified
{
  
  // Attach the servo objects to pins
  Wrist.attach(PWMWristPin);
  Grasp.attach(PWMGraspPin);

  Wrist.write(wristCmd);
  Grasp.write(graspCmd);  

  // Initialize drive motor object
  driveMotors.init();

  //---- Bump Switches----
  // Set interrupt pins to input
  pinMode(leftBumpPin,INPUT);
  pinMode(rightBumpPin,INPUT);

  // Turn on pullup resistors
  digitalWrite(leftBumpPin, HIGH);
  digitalWrite(rightBumpPin, HIGH);

  // Attached Interrupt pins
  attachInterrupt(0, bumpLeft, RISING);         // Digital Pin 2
  attachInterrupt(1, bumpRight, RISING);        // Digital Pin 3

  //---- PID Settings ----
  leftPID.SetSampleTime(50);
  rightPID.SetSampleTime(50); 
   
  leftPID.SetOutputLimits(-100,100);
  rightPID.SetOutputLimits(-100,100);

  leftPID.SetMode(AUTOMATIC);
  rightPID.SetMode(AUTOMATIC);
  
  rightPID.SetControllerDirection(REVERSE);
  
  
  // Initiliaze serial communications
  Serial.begin(9600);           // set up Serial library at 9600 bps  boolean readyBypass = true;
  piMessage.attach(messageParse); 

  while(1){
    // Set readyBypass to true to skip waiting for Odroid confirmation and button switch confimation
    if (readyBypass){break;}
    
    if (Serial.available() > 0){ inByte = Serial.read();}
    
    if (inByte == 115){
        Serial.println('g');
        break;
    }

    Serial.println('r');

    delay(100);
  }
}

//--------Main Loop--------------------------------------

void loop()
{
  
  currentMillis = millis(); // Program run time in milliseconds.
  
  // Read serial and call parser
  if (currentMillis - previousMillis_Serial > serialPeriod){
    previousMillis_Serial = currentMillis;
    while( Serial.available() ) piMessage.process(Serial.read());
  }

  // Act of behavior here like locking certain commands or something
  if( newBehavior){
    if(activeBehavior == 0){
      // Robot should be inactive
      // Obstacle avoidance should be off and the robot should not move.
      // Gripper should not move also
      OAoff = true;
      oaOverride = false;
      gripOff = true;
      }
    else if(activeBehavior == 1){
      // Search/Wander Behavior
      // Obstacle avoidance should be on, should recieve commands from Odroid
      OAoff = false;
      gripOff = true;
    }
    else if(activeBehavior == 2){
      // Align and Pickup behavior
      // Obstacle avoidance should be off, should recieve commands fro Odroid
      OAoff = true;
      oaOverride = false;
      gripOff  = false;
    }
    else if(activeBehavior == 3){
      // Deposit behavior
      // Obstacle avoidance should be on, robot should recieve commands from Odroid
      OAoff = true;
      oaOverride = false;
      gripOff = false;
    }
    else if(activeBehavior == 4){
      // Localize bahvior
      // Robot should stop moving, Obstacle avoidance should be turned off
      // Gripper should not be moving also
      OAoff = true;
      gripOff = true;
    }
  }

 if(!gripOff){
  if (newWristGraspCmd){
    // Debug Print
    Serial.println(String(activeBehavior) + ' ' + String(wristCmd) + ' ' + String(graspCmd));
    // Makes sure desired angles are acceptable
    if((wristCmd >= 40 && wristCmd <= 170) && wristCmd != 999){
      Wrist.write(wristCmd);
    }
    if((graspCmd >= 45 && graspCmd <= 135) && graspCmd != 999){
      Grasp.write(graspCmd);
    } 
    newWristGraspCmd = false;
  }  
 }
  currentMillis = millis(); // Program run time in milliseconds. Used for sensor sampling.

  //---- Distance Measurement IR Smart Sensor ----
  if (currentMillis - previousMillis_IR >= irPeriod){
    previousMillis_IR = currentMillis;
    
    // Read in the left IR voltage and put into a buffer
    irValue = analogRead(leftIRPin);
    irLeft[0] = irLeft[1];
    irLeft[1] = irLeft[2];
    irLeft[2] = irLeft[3];
    irLeft[3] = irValue;
    delay(1);
    
    // Read in the right IR voltage and put into a buffer
    irValue = analogRead(rightIRPin);
    irRight[0] = irRight[1];
    irRight[1] = irRight[2];
    irRight[2] = irRight[3];
    irRight[3] = irValue;
    
    // Calculate the average input
    irLeftAvg = (float(irLeft[0]) + float(irLeft[1]) + float(irLeft[2]) + float(irLeft[3]))/4;
    irRightAvg = (float(irRight[0]) + float(irRight[1]) + float(irRight[2]) + float(irRight[3]))/4;

    // Convert voltage reading to units of inches.
    irLeftAvg = 573.01 * pow(float(irLeftAvg),-0.909);
    irRightAvg = 1768.2* pow(float(irRightAvg), -1.114);

    // Obstacle Avoidance Logic
    if (irLeftAvg < 3.5 && irRightAvg >= 3.5){
      // reverse and turn right
      bumpRecomnd = 3;
      bumpFlag = true;
    }
    else if (irLeftAvg >= 3.5 && irRightAvg < 3.5){
      // reverse and turn left
      bumpRecomnd = 4;
      bumpFlag = true;
    }
    else if (irLeftAvg < 3.5 && irRightAvg < 3.5){
      // reverse and turn left or right
      if(random(0,9)/5 == 1){
        bumpRecomnd = 3;
      }
      else{
        bumpRecomnd = 4;
      }
      bumpFlag = true;
    }
    else if((irRightAvg-irLeftAvg)< eps && (irLeftAvg >= 3.5 && irLeftAvg < 5.5)\
          && (irRightAvg >= 3.5 && irRightAvg < 5.5)) {
      // Turn left or right
      if(random(0,9)/5 == 1){
        irRecomnd = 1;
      }
      else{
        irRecomnd = 2;
      }  
    }
    else if(irLeftAvg >= 3.5 && irLeftAvg < 5.5 && irRightAvg > 5.5){
      irRecomnd = 1;  // Turn right some random amount
    }
    else if(irRightAvg >= 3.5 && irRightAvg < 5.5 && irLeftAvg > 5.5){
      irRecomnd = 2;  // Turn left some random amount
    }
    else{
      irRecomnd = 0;
    }
     
    // If the IR recommends something then set the flag to true
    if(irRecomnd != 0){
      irFlag = true;
    }
    else{
      irFlag = false;
    }

    // Debugging outputs
    //  Serial.print(leftString + String(irLeftAvg) + " " );
    //  Serial.println(rightString + String(irRightAvg));  
    
  }
  //--------------------------------------------------------------------------------------------------------------
  //---- Amperage Measurement Smart Sensor ----
  if (currentMillis - previousMillis_Current >= currentPeriod){
    previousMillis_Current = currentMillis;
    
    currentValue  = driveMotors.getM1CurrentMilliamps();
    currentLeft[0] = currentLeft[1];
    currentLeft[1] = currentLeft[2];
    currentLeft[2] = currentLeft[3];
    currentLeft[3] = currentValue;
    delay(1);
        
    currentValue = driveMotors.getM2CurrentMilliamps();
    currentRight[0] = currentRight[1];
    currentRight[1] = currentRight[2];
    currentRight[2] = currentRight[3];
    currentRight[3] = currentValue;
       
    // Calculate average current reading over three samples to try to not in spikes.
    currentLeftAvg = (float(currentLeft[0]) + float(currentLeft[1]) + float(currentLeft[2]) + float(currentLeft[3]))/4;
    currentRightAvg = (float(currentRight[0]) + float(currentRight[1]) + float(currentRight[2]) + float(currentRight[3]))/4;

    // Convert to real units (amps)
    currentLeftAvg = 0.034 * currentLeftAvg;
    currentRightAvg = 0.034 * currentRightAvg;

    // Tune this value
    if(currentLeftAvg >= 4 || currentRightAvg >= 4) {
      currentRecomnd = 1;  // Arbitraty number for now just to trigger the flag.
    }
    else{
      currentRecomnd = 0;
    }
     
    // If the current sensor recommends something then set the flag to true
    if(currentRecomnd != 0){
      currentFlag = true;
      actionOverride = true;
    }
    else{ 
      currentFlag = false;
    }
    
    // Debug variable declaration
    //currentFlag = false;

    /* Debugging Outputs
      Serial.print("Left Current: " + String(currentLeftAvg) + " " );
      Serial.println("Right Current: " + String(currentRightAvg)); 
    */
  }
//--------------------------------------------------------------------------------------------------------------
  
  if(!OAoff){          
    // Any sensor flag will trigger alternative behavior
    if (currentFlag || bumpFlag || irFlag) {
      oaOverride = true;
      if (currentRecomnd == 1) {
      // Stop motion, robot could be stuck.
      Reverse = false;
      Turn = false;
      motionDirection = 0;
      currentFlag = false;
      }
      else if(bumpRecomnd == 3){
        // Reverse and right turn motion
        Reverse = true;
        Turn = true;
        motionDirection = 4;
        bumpFlag = false;
      } 
      else if(bumpRecomnd == 4){
        // Reverse and left turn motion
        Reverse = true;
        Turn = true;
        motionDirection= 3;
        bumpFlag = false;
        /*
        if(!actionLock){
          actionLock = true;
          driveMotors.setM1Speed(-75);
          driveMotors.setM2Speed(75);
          actionTimer =  currentMillis;
          bumpRecomnd = 0;
        }
        */
      } 
      else if(irRecomnd == 1){
        // Turn Right
        Reverse = false;
        Turn = true;
        motionDirection = 4;                                      
      }
      else if(irRecomnd == 2){
        // Turn left
        Reverse = false;
        Turn = true;
        motionDirection = 3;   
      }
    }
  }

  //---- Obstacle Avoidance Action ----
   
  if(Reverse && isStopped){
    // perform action
    motionDirection = 2;
  }
    
  if(Turn && !Reverse && isStopped){
      // Perform Turn
  
      // Which way to turn?
      if(motionDirection == 3){
        // Turn Left
  
          Turn = false;
          motionDirection = 0;
          oaOverride = false;
      }
      else if(motionDirection == 4){
        // Turn Right
  
        motionDirection = 0;
      }
  }

  //---- Commanded Robot Move ----
  if(newDistance && !oaOverride){   
    if( leftDistance < 0 && rightDistance > 0){
      //Forward motion
      motionDirection = 1;
    }
    else if(leftDistance > 0 && rightDistance < 0){
      // Reverse Motion
      motionDirection = 2;
    }
    else if(leftDistance > 0 && rightDistance > 0){
      // Turning Left
      motionDirection = 3;
    }
    else if(leftDistance < 0 && rightDistance < 0){
      // Turning Right
      motionDirection = 4;
    }
    else{
      //Stop
      motionDirection = 0;
    }
  }



  if(newDistance || oaOverride){

      if(isStopped && motionDirection == 1){
        // Forward Motion
        leftStartPoint = leftEncoder.read()*C;
        rightStartPoint = rightEncoder.read()*C;
        leftSetpoint = robotSpeed;
        rightSetpoint = robotSpeed;
        Serial.println("I made it forward");
        newDistance = false;
        
      }
      else if(isStopped && motionDirection == 2){
        // Reverse Motion
        leftStartPoint = leftEncoder.read()*C;
        rightStartPoint = rightEncoder.read()*C;
        leftSetpoint = -robotSpeed;
        rightSetpoint = -robotSpeed;
        Serial.println("I made it reverse");
        newDistance = false;
        
      }
      else if(isStopped && motionDirection == 3){
        // Left Turn
        leftStartPoint = leftEncoder.read()*C;
        rightStartPoint = rightEncoder.read()*C;
        leftSetpoint = -robotSpeed;
        rightSetpoint = robotSpeed;
        Serial.println("I made it left");
        newDistance = false;
        
      }
      else if(isStopped && motionDirection == 4){
        // Right Turn
        leftStartPoint = leftEncoder.read()*C;
        rightStartPoint = rightEncoder.read()*C;
        leftSetpoint = robotSpeed;
        rightSetpoint = -robotSpeed;
        Serial.println("I made it right");
        newDistance = false;
        
      }
    }   

  if((motionDirection != oldMotionDirection || motionDirection == 0) && !isStopped){
      // Now doing a different motion
      oldMotionDirection = motionDirection;
      if(!isStopped){
        leftSetpoint = 0;
        rightSetpoint = 0;

        if(abs(currentMillis - previousMillis_Stopped) > stoppedPeriod){
          if(abs(leftStopPoint - leftEncoder.read()*C) < 0.00001 && abs(rightStopPoint - rightEncoder.read()*C) < 0.00001){
            isStopped = true;
          }
          leftStopPoint = leftInput;
          rightStoptPoint = rightInput;
        }
      }
    }
  else if((leftInput - leftStartPoint) > (leftDistance-leftOffset) \
                  && (rightInput - rightStartPoint) > (rightDistance-rightOffset)){
                    
                    
  }

  /*
  if((leftInput - leftStartPoint)>(leftDistance-leftOffset) && abs(rightInput)>(rightDistance-rightOffset) && rightSetpoint != 0){
    leftSetpoint = 0;
    rightSetpoint = 0;
  }
  */

// PD controller
// Need to convert to actual position measurements.
  leftInput = leftEncoder.read()*C; 
  leftDone = leftPID.Compute();
  
  rightInput = rightEncoder.read()*C;
  rightDone = rightPID.Compute();

  if(leftDone && rightDone){
    // Set the speeds together
    driveMotors.setSpeeds(K*leftOutput, K*rightOutput);
    leftDone = false;
    rightDone = false;
  }


  
}


  
//--------------------------------------------------------------------------------------------------------------
// ---Extra Error Function---
// This function is used by the Pololu motor drivers to handle errors in operation
void stopIfFault()
{
  if (driveMotors.getM1Fault())
  {
    //Serial.println("M1 fault:");
    while(1);
  }
  if (driveMotors.getM2Fault())
  {
    //Serial.println("M2 fault:");
    while(1);
  }
}

// ---Left Bump Sensor Interrupt Function---
void bumpLeft()
{
  bumpFlag = true;
  actionOverride = true;
  bumpRecomnd = 3;
  
}
// ---Right Bump Sensor Interrupt Function---
void bumpRight()
{
  bumpFlag = true;
  actionOverride = true;
  bumpRecomnd = 4;
}
