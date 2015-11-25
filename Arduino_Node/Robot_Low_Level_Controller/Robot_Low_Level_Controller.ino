
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
// Patrick's Custom Motor Controls ( Mostly for Drive motor control with hall effect encoder)
#include <DualVNH5019MotorDriver.h> // For use with the Dual motor drivers from Pololu
#include <LiquidCrystal.h> // For use a LCD display
#include <Encoder.h>

//-------Define variables here-------------------------------

// ---PWM Pin Setting---

// PWM pins for the two drive motors
const int PWMDrivePin_Left = 11;    // Timer 1 16-bit
const int PWMDrivePin_Right = 12;   // Timer 1 16-bit

// Additional PWM Pins for Manipulator. May not be used
const int PWMLowerPin = 7;
const int PWMGraspPin = 6;

// ---LED Pins---

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

// Push button start
//const int Push_button = A13; // For push to start button

// ---Sensor sampling periods---
const unsigned long irPeriod = 100;      // Sampling period for IR Sensors (ms)
const unsigned long currentPeriod = 100; // Sampling period for current sensor (ms)
const unsigned long encoderPeriod = 50;  // Sampling period for encodert sensor (ms)

// ---Constants---
const int eps = 20;

// ---Define non-constant variables---
int inByte = 0;     // Variable that will store incoming byte from serial

int irLeft[4] = {0, 0, 0, 0};     // Stores the past 3 Left IR Readings for use in an average
int irRight[4] = {0, 0, 0, 0};    // Stores the past 3 Right IR Reading for use in an average
int irValue;                   // Temporary value to store mesured IR reading
float irLeftAvg;                 // Used to store left IR average reading
float irRightAvg;                // Used to store right IR average reading

int currentLeft[4] = {0, 0, 0, 0};   // Stores Left Drive Motor Current Reading
int currentRight[4] = {0, 0, 0, 0};  // Stores Right Drive Motor Current Reading
int currentValue;                 // For current measurement (amps)
float currentLeftAvg;               // Used to store the average current sensor reading for the left motor            
float currentRightAvg;              // Used to store the average current sensor reading for the right motor   

// Sensor Flags
boolean irFlag =false;              // Will be set true if an IR condition is met
  int irRecomnd = 0;                // Recommendation will be set depending on specific combintaion of IR readings
boolean currentFlag = false;        // Will be set true if Current sense condition is met
  int currentRecomnd = 0;           // Will be set depending of specific combinations
volatile boolean bumpFlag = false;  // Will be set true if a bump sensor is triggered
  volatile int bumpRecomnd = 0;     // Will indicate whether left or right sensor was triggered

// Used for sensor sampling times
unsigned long currentMillis;            // Stores the current time reading in milliseconds
unsigned long previousMillis_IR=0;      // Stores the previous time the IR sensors were read
unsigned long previousMillis_Current=0; // Stores the previous time the Current sensors were read
unsigned long previousMillis_Encoder=0; // Stores the previous time the encoder were read

// Arbiter Variables
boolean actionLock = false;
boolean actionOverride = false;
unsigned long timerLockout = 750;
unsigned long actionTimer =0;

// Encoder Counting Variables
long leftOldPosition = -999;
long rightOldPosition = -999;

long leftNewPosition;
long rightNewPosition;

//----Define Objects-----
// Define encoder object
Encoder leftEncoder(leftEncoderAPin, leftEncoderBPin);
Encoder rightEncoder(rightEncoderBPin, rightEncoderBPin);

// Define servo objects
Servo Lower;
Servo Grasp;

// Define drive motor object
DualVNH5019MotorDriver driveMotors(Drive_INA1,Drive_INB1,PWMDrivePin_Left,\
Drive_EN1DIAG1,leftCurrentPin,Drive_INA2,Drive_INB2,PWMDrivePin_Right,Drive_EN2DIAG2,\
rightCurrentPin, 1);

// Test Variables
const String leftString = "Left IR Reading: ";
const String rightString = "Right IR Reading: ";
 
//-------Setup Function-----------------------------------

void setup()  // Needs to stay in setup until all necessary communications can be verified
{
  
  // Attach the servo objects to pins
  Lower.attach(PWMLowerPin);
  Grasp.attach(PWMGraspPin);  

  // Initialize drive motor object
  driveMotors.init();

  // Set interrupt pins to input
  pinMode(leftBumpPin,INPUT);
  pinMode(rightBumpPin,INPUT);

  // Turn on pullup resistors
  digitalWrite(leftBumpPin, HIGH);
  digitalWrite(rightBumpPin, HIGH);

  // Attached Interrupt pins
  attachInterrupt(0, bumpLeft, RISING);         // Digital Pin 2
  attachInterrupt(1, bumpRight, RISING);        // Digital Pin 3
  
  // Initiliaze serial communications
  Serial.begin(9600);           // set up Serial library at 9600 bps  boolean readyBypass = true; 

  while(1){
    // Set readyBypass to true to skip waiting for Raspberry Pi 2 confirmation and button switch confimation
    if (readyBypass){
      break;
    }
    
    incomingbyte = Serial.read()

    // Also need to read in the start button state

    if (inByte == 0){
      // Turn LED on for ready or write to LCD board.
      if (){
        break;
      }
      
    }

  }
    
  }

*/
  /* Should Send confirmation to Raspberry Pi 2 if all services running
   *  and then wait for a message from the Odroid to continue to the Main Loop
   */
  
}

//--------Main Loop--------------------------------------

void loop() // run over and over again
{
  // Contains all actions the robot will perform.
  // Obstacle avoidance behaviors should override and sent commands from the Raspberry Pi 2 until they have completed.
  // Should always check IR and Bump sensors before performing requested tasks.
  // Camera positioning should not be overrid by obstacle avoidance behaviors

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
    delay(1);
    
    // Calculate the average input
    irLeftAvg = (float(irLeft[0]) + float(irLeft[1]) + float(irLeft[2]) + float(irLeft[3]))/4;
    irRightAvg = (float(irRight[0]) + float(irRight[1]) + float(irRight[2]) + float(irRight[3]))/4;

    // Convert voltage reading to units of inches.
    irLeftAvg = 573.01 * pow(float(irLeftAvg),-0.909);
    irRightAvg = 1768.2* pow(float(irRightAvg), -1.114);

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
    delay(1);
       
    // Calculate average current reading over three samples to try to not in spikes.
    currentLeftAvg = (float(currentLeft[0]) + float(currentLeft[1]) + float(currentLeft[2]) + float(currentLeft[3]))/4;
    currentRightAvg = (float(currentRight[0]) + float(currentRight[1]) + float(currentRight[2]) + float(currentRight[3]))/4;

    // Convert to real units (amps)
    currentLeftAvg = 0.034 * currentLeftAvg;
    currentRightAvg = 0.034 * currentRightAvg;
    
    if(currentLeftAvg >= 4 || currentRightAvg >= 4) {
    
      currentRecomnd = 1;  // Arbitraty number for now just to trigger the flag.
    }
    else{
      currentRecomnd = 0;
    }
     
    // If the current sensor recommends something then set the flag to true
    if(currentRecomnd != 0){
      currentFlag = true;
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
  if((currentMillis - actionTimer) >= random(750,1750) || actionOverride){     
     actionLock = false;
     actionOverride = false;
  }
       
  // Driving will be done here
  // Any sensor flag will trigger alternative behavior
  if (currentFlag || bumpFlag || irFlag) {
    if (currentRecomnd == 1) {
    // Stop motion, robot could be stuck.
      
    }
    else if(bumpRecomnd == 3){
      // Reverse motion
      if(!actionLock){
        actionLock = true;
        driveMotors.setM1Speed(-75);
        driveMotors.setM2Speed(75);
        actionTimer =  currentMillis;
        bumpRecomnd = 0;
      } 
    else if(bumpRecomnd == 4){
      // Reverse motion
      if(!actionLock){
        actionLock = true;
        driveMotors.setM1Speed(-75);
        driveMotors.setM2Speed(75);
        actionTimer =  currentMillis;
        bumpRecomnd = 0;
      }      
                                     
    }
  }
  else{ // This branch is for normal operations
    // Forward motion
    if(!actionLock){
    driveMotors.setM1Speed(-75);
    driveMotors.setM2Speed(75);
    }
  }
  
}

//--------------------------------------------------------------------------------------------------------------
// ---Extra Error Function---
// This function is used by the Pololu motor drivers to handle errors in operation
void stopIfFault()
{
  if (driveMotors.getM1Fault())
  {
    //Serial.println("M1 fault");
    while(1);
  }
  if (driveMotors.getM2Fault())
  {
    //Serial.println("M2 fault");
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

// ---Encoder Interrupt Functions go Here---
