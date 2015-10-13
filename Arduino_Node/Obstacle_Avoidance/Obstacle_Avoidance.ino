
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


//-------Define variables here-------------------------------

// ---PWM Pin Setting---

// PWM pins for the two drive motors
const int PWMDrivePin_Left = 11;    // Timer 1 16-bit
const int PWMDrivePin_Right = 12;   // Timer 1 16-bit

// PWM pin for camera pan and tilt
// const int PWMPanPin = 10; // for the gearmotor, Chosen because it is on timer 3 (Dont use Pins 3 and 2 unless you want 20kHz PWM)
// const int PWMTiltPin = 8; // for the servomotor

// Additional PWM Pins for Manipulator. May not be used
// const int PWMLowerPin = 7;
// const int PWMGraspPin = 6;

// ---LED Pins---

// ---Pololu Motor Pins---
const int Drive_INA1=22;     // Left Drive Motor
const int Drive_INB1=23;     // Left Drive Motor
const int Drive_EN1DIAG1=24; // Left Drive Motor

const int Drive_INA2=26;     // Right Drive Motor
const int Drive_INB2=27;     // Right Drive Motor
const int Drive_EN2DIAG2=25; // Right Drive Motor

// const int Pan_INA1;       // Pan Motor
// const int Pan_INB1;       // Pan Motor
// const int Pan_EN1DIAG1;   // Pan Motor

// ---Sensor Pins---

// Bump Sensors
const int Bump_Left = 2;            // Digital Pin 2, interrupt pin for Mega 2560
const int Bump_Right = 3;           // Digital Pin 3, interrupt pin for MEga 2560

// IR sensors
const int IR_Left_Pin = 0;          // A0
const int IR_Right_Pin = 1;         // A1

// Encoder Input
const int Left_Encoder_1_Pin = 2;   // A2
const int Left_Encoder_2_Pin = 3;   // A3

const int Right_Encoder_1_Pin = 4;  // A4
const int Right_Encoder_2_Pin = 5;  // A5

const int Pan_Encoder_1_Pin = 6;    // A6
const int Pan_Encoder_2_Pin = 7;    // A7

// Current Sensors
const int Current_Left_Pin = 8;    // A8, CS1
const int Current_Right_Pin = 9;   // A9, CS2
const int Current_Pan_Pin = 10;     // A10, CS1

// Push button start
const int Push_button = A13; // For push to start button

// ---Sensor sampling periods---
const unsigned long IR_Period = 30;      // Sampling period for IR Sensors (ms)
const unsigned long Current_Period = 100; // Sampling period for current sensor (ms)
const unsigned long Bump_Period = 100;    // Sampling period for Bump Switches (ms)

// ---Constants---
const int eps = 2;

// ---Define non-constant variables---
int inByte = 0;     // Variable that will store incoming byte from serial

int IR_Left[3] = {0, 0, 0};     // Stores the past 3 Left IR Readings for use in an average
int IR_Right[3] = {0, 0, 0};    // Stores the past 3 Right IR Reading for use in an average
int IR_Value;       // Temporary value to store mesured IR reading
float IR_Left_Avg;  // Used to store left IR average reading
float IR_Right_Avg; // Used to store right IR average reading

int Current_Left[3];   // Stores Left Drive Motor Current Reading
int Current_Right[3];  // Stores Right Drive Motor Current Reading
int Current_Pan[3];    // Stores Pan Motor Current Reading
int Current_Value;     // For current measurement (amps)
float Current_Left_Avg;
float Current_Right_Avg;
float Current_Pan_Avg;

// Sensor Flags
boolean irFlag;              // Will be set if an IR condition is met
  int irRecomnd;             // Recommendation will be set depending on specific combintaion of IR readings
boolean currentFlag;         // Will be set if Current sense condition is met
  int currentRecomnd;        // Will be set depending of specific combinations
volatile boolean bumpFlag;   //
  volatile int bumpRecomnd;  //

// Sensor Sampling
unsigned long currentMillis;          // Stores the current time reading in milliseconds
unsigned long previousMillis_IR;      // Stores the previous time the IR sensors were read
unsigned long previousMillis_Current; // Stores the previous time the Current sensors were read
unsigned long previousMillis_Bump;    // Stores the previous time the Bump sensors were read


//// Define servo objects
//Servo Tilt;
//Servo Lower;
//Servo Grasp;

// Define drive motor object
DualVNH5019MotorDriver Drive_Motors(Drive_INA1,Drive_INB1,PWMDrivePin_Left,\
Drive_EN1DIAG1,Current_Left_Pin,Drive_INA2,Drive_INB2,PWMDrivePin_Right,Drive_EN2DIAG2,\
Current_Right_Pin, 1);

//Drive_Motors._INA1 = Drive_INA1;
//Drive_Motors._INB1 = Drive_INB1;
//Drive_Motors._PWM1 = PWMDrivePin_Left;
//Drive_Motors._END1DIAG1 = Drive_END1DIAG1;
//Drive_Motors._CS1 = Current_Left_Pin;
//
//Drive_Motors._INA2 = Drive_INA2;
//Drive_Motors._INB2 = Drive_INB2;
//Drive_Motors._PWM2 = PWMDrivePin_Right;
//Drive_Motors._END2DIAG2 = Drive_END2DIAG2;
//Drive_Motors._CS2 = Current_Right_Pin;
 
//DualVNH5019MotorDriver Pan_Motor;
//Pan_Motor._Timer = 3;
//Pan_Motor._INA1 = Pan_INA1;
//Pan_Motor._INB1 = Pan_INB1;
//Pan_Motor._PWM1 = PWMPanPin;
//Pan_Motor._END1DIAG1 = Pan_END1DIAG1;
//Pan_Motor._CS1 = Current_Pan_Pin;
 
//-------Setup Function-----------------------------------

void setup()  // Needs to stay in setup until all necessary communications can be verified
{
  
//  // Attach the servo objects to pins
//  Tilt.attach(PWMTiltPin);
//  Lower.attach(PWMLowerPin);
//  Grasp.attach(PWMGraspPin);

  // Initialize drive motor object
  Drive_Motors.init();
//  Pan_Motor.init()

  // Attached Interrupt pins
  attachInterrupt(digitalPinToInterrupt(Bump_Left), bumpLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(Bump_Right), bumpRight, RISING);
  

  // Initiliaze serial communications
/*  Serial.begin(9600);           // set up Serial library at 9600 bps  boolean readyBypass = true; 
  
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

  // Read all sensor input before deciding course of action
  // Rate of Sensor Collection 

  // Encoder counter will always run and cannot be blocked.

  
  currentMillis = millis(); // Program run time in milliseconds. Used for sensor sampling.

  if (currentMillis - previousMillis_IR >= IR_Period){
    previousMillis_IR = currentMillis;
    IR_Value = analogRead(IR_Left_Pin);
    
    IR_Left[0] = IR_Left[1];
    IR_Left[1] = IR_Left[2];
    IR_Left[2] = IR_Value;
    
    IR_Value = analogRead(IR_Right_Pin);
    
    IR_Right[0] = IR_Right[1];
    IR_Right[1] = IR_Right[2];
    IR_Right[2] = IR_Value;
    
    
    
    
    irFlag = true;
    }

  if (currentMillis - previousMillis_Current >= Current_Period){
    previousMillis_Current = currentMillis;
    Current_Left  = Drive_Motors.getM1CurrentMilliamps();
    Current_Right = Drive_Motors.getM2CurrentMilliamps();
    // This needs to set a flag for high current draw
    //Current_Pan  = Pan_Motor.getM1CurrentMilliamps();


    }
//  // This if statement might not be needed if using interrupts
//  if (currentMillis - previousMillis_Bump >= Bump_Period){
//    previousMillis_Bump = currentMillis; 
//    // These should be attached as interrupts
//    // I need a function to be called when they are hit.
//    Bump_Left  = analogRead(Bump_Left_Pin); // Need to convert these into true or false
//    Bump_Right = analogRead(Bump_Right_Pin);
//    }
    
  // Driving will be done here
  // Any sensor flag will trigger alternative behavior
  if (currentFlag || bumpFlag || irFlag) { 
    if (currentRecomnd == 1 || bumpRecomnd == 1 || irRecomnd == 1) {
      
    }
    else if(currentRecomnd == 2 || bumpRecomnd == 2 || irRecomnd == 2)
      
    }
    else if(currentRecomnd == 3 || irRecomnd == 3)
    }
  // Make sure to reset flag
  }
  else{ // This branch is for normal operations
    
    
  }
    
  
  // Different rate of sensor collection
  // if (obstacle avoidance check){

  // end
  // Perform requested camera movements
//  Serial.println("Hello world!");  // prints hello with ending line break 

//  delay(1000);
}

// ---Extra Error Function---
// This function is used by the Pololu motor drivers to handle errors in operation
void stopIfFault()
{
  if (Drive_Motors.getM1Fault())
  {
    //Serial.println("M1 fault");
    while(1);
  }
  if (Drive_Motors.getM2Fault())
  {
    //Serial.println("M2 fault");
    while(1);
  }
}

// ---Left Bump Sensor Interrupt Function---
void bumpLeft()
{
  bumpFlag = true
  bumpRecomnd = 1;
  
}
// ---Right Bump Sensor Interrupt Function---
void bumpRight()
{
  bumpFlag = true
  bumpRecomnd = 2;
  
  
}

// ---Encoder Interrupt Functions go Here---
