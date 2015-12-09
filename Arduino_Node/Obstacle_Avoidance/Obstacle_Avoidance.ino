
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

// const int Pan_INA1=28;       // Pan Motor
// const int Pan_INB1=29;       // Pan Motor
// const int Pan_EN1DIAG1=30;   // Pan Motor

// ---Sensor Pins---

// Bump Sensors
const int Bump_Left = 2;            // Digital Pin 2, interrupt pin for Mega 2560
const int Bump_Right = 3;           // Digital Pin 3, interrupt pin for MEga 2560

// IR sensors
const int IR_Left_Pin = 0;          // A0
const int IR_Right_Pin = 1;         // A1

// Encoder Input (!!Will be replaced with interrupt pins!!)
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
//const int Push_button = A13; // For push to start button

// ---Sensor sampling periods---
const unsigned long irPeriod = 100;      // Sampling period for IR Sensors (ms)
const unsigned long currentPeriod = 100; // Sampling period for current sensor (ms)

// ---Constants---
const int eps = 20;

// ---Define non-constant variables---
int inByte = 0;     // Variable that will store incoming byte from serial

int irLeft[3] = {0, 0, 0};     // Stores the past 3 Left IR Readings for use in an average
int irRight[3] = {0, 0, 0};    // Stores the past 3 Right IR Reading for use in an average
int irValue;                   // Temporary value to store mesured IR reading
int irLeftAvg;                 // Used to store left IR average reading
int irRightAvg;                // Used to store right IR average reading

int currentLeft[3] = {0, 0, 0};   // Stores Left Drive Motor Current Reading
int currentRight[3] = {0, 0, 0};  // Stores Right Drive Motor Current Reading
int currentPan[3] = {0, 0, 0};    // Stores Pan Motor Current Reading
int currentValue;                 // For current measurement (amps)
int currentLeftAvg;               // Used to store the average current sensor reading for the left motor            
int currentRightAvg;              // Used to store the average current sensor reading for the right motor   
int currentPanAvg;                // Used to store the average current sensor reading for the pan motor   

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
unsigned long previousMillis_Bump=0;    // Stores the previous time the Bump sensors were read

// Arbiter Variables
boolean actionLock = false;
boolean actionOverride = false;
unsigned long timerLockout = 750;
unsigned long actionTimer =0;

//// Define servo objects
//Servo Tilt;
//Servo Lower;
//Servo Grasp;

// Define drive motor object
DualVNH5019MotorDriver driveMotors(Drive_INA1,Drive_INB1,PWMDrivePin_Left,\
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

// Add new panMotor declaration here //
 
//DualVNH5019MotorDriver panMotor;
//Pan_Motor._Timer = 3;
//Pan_Motor._INA1 = Pan_INA1;
//Pan_Motor._INB1 = Pan_INB1;
//Pan_Motor._PWM1 = PWMPanPin;
//Pan_Motor._END1DIAG1 = Pan_END1DIAG1;
//Pan_Motor._CS1 = Current_Pan_Pin;

// Test Variables
const String leftString = "Left IR Reading: ";
const String rightString = "Right IR Reading: ";
 
//-------Setup Function-----------------------------------

void setup()  // Needs to stay in setup until all necessary communications can be verified
{
  
//  // Attach the servo objects to pins
//  Tilt.attach(PWMTiltPin);
//  Lower.attach(PWMLowerPin);
//  Grasp.attach(PWMGraspPin);  

  // Initialize drive motor object
  driveMotors.init();
//  panMotor.init();

  pinMode(Bump_Left,INPUT);
  pinMode(Bump_Right,INPUT);

  // Attached Interrupt pins
  attachInterrupt(0, bumpLeft, RISING);  // Digital Pin 2
  attachInterrupt(1, bumpRight, RISING); // Digital Pin 3
  // Attach 3 more for gearmotor encoders
  
  // Initiliaze serial communications
//  Serial.begin(9600);           // set up Serial library at 9600 bps  boolean readyBypass = true; 

/*  
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


  // Encoder counter will always run and cannot be blocked.

 /* if(bumpRecomnd == 2){
   Serial.println("bump Left"); 
    bumpRecomnd = 0;  
  }
  else if(bumpRecomnd == 3){
    Serial.println("Bump Right");
    bumpRecomnd = 0;
  }
 */ 
  currentMillis = millis(); // Program run time in milliseconds. Used for sensor sampling.

  // Distance Measurement IR Smart Sensor
  if (currentMillis - previousMillis_IR >= irPeriod){
    previousMillis_IR = currentMillis;
    
    // Read in the left IR voltage and put into a buffer
    irValue = analogRead(IR_Left_Pin);
    irLeft[0] = irLeft[1];
    irLeft[1] = irLeft[2];
    irLeft[2] = irValue;
    
    // Read in the right IR voltage and put into a buffer
    irValue = analogRead(IR_Right_Pin);
    irRight[0] = irRight[1];
    irRight[1] = irRight[2];
    irRight[2] = irValue;
    
    // Calculate the average input
    irLeftAvg = (float(irLeft[0])+float(irLeft[1])+float(irLeft[2]))/3;
    irRightAvg = (float(irRight[0])+float(irRight[1])+float(irRight[2]))/3;
        
    // !! Reivisit this first if statement !!
    if (irLeftAvg > 180 || irRightAvg > 200){
      // reverse and turn left or right
      if(random(0,9)/5 == 1){
        bumpRecomnd = 3;
      }
      else{
        bumpRecomnd = 4;
      }
      bumpFlag = true;
    }
    else if((irRightAvg-irLeftAvg)< eps && (irLeftAvg <= 175 && irLeftAvg > 120)\
          && (irRightAvg <= 190 && irRightAvg > 130)) {
    
      irRecomnd = 1;  // Turn left or right
    }
    else if(irLeftAvg <= 175 && irLeftAvg > 120 && irRightAvg < 130){
      irRecomnd = 2;  // Turn right some random amount
    }
    else if(irRightAvg <= 190 && irRightAvg > 130 && irLeftAvg < 120){
      irRecomnd = 3;  // Turn left some random amount
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
    
    /* Debugging outputs
      Serial.print(leftString + String(irLeftAvg) + " " );
      Serial.println(rightString + String(irRightAvg));  
      Serial.println(irFlag);
      Serial.println(irRecomnd);
    */
    
  }

  // Amperage Measurement Smart Sensor
  if (currentMillis - previousMillis_Current >= currentPeriod){
    previousMillis_Current = currentMillis;
    
    currentValue  = driveMotors.getM1CurrentMilliamps();
    currentLeft[0] = currentLeft[1];
    currentLeft[1] = currentLeft[2];
    currentLeft[2] = currentValue;
        
    currentValue = driveMotors.getM2CurrentMilliamps();
    currentRight[0] = currentRight[1];
    currentRight[1] = currentRight[2];
    currentRight[2] = currentValue;
        
    // Calculate average current reading over three samples to try to not in spikes.
    currentLeftAvg = (float(currentLeft[0]) + float(currentLeft[1]) + float(currentLeft[2]))/3;
    currentRightAvg = (float(currentRight[0]) + float(currentRight[1]) + float(currentRight[2]))/3;
    
    if(currentLeftAvg >= 1500 || currentRightAvg >= 1500) {
    
      currentRecomnd = 6;  // Arbitraty number for now just to trigger the flag.
    }
    else{
      currentRecomnd = 0;
    }
     
    // If the IR recommends something then set the flag to true
    if(currentRecomnd != 0){
      currentFlag = true;
    }
    else{
      currentFlag = false;
    }
    
    currentFlag = false;

    /* Debugging Outputs
      Serial.print("Left Current: " + String(currentLeftAvg) + " " );
      Serial.println("Right Current: " + String(currentRightAvg)); 

    */
  }

  if((currentMillis - actionTimer) >= random(1500,2500) || actionOverride){     
     actionLock = false;

     if(bumpFlag && (bumpRecomnd == 1 || bumpRecomnd == 2)){
      if(bumpRecomnd == 1){
        if(!actionLock){ // Right Turn
          actionLock = true;
          driveMotors.setM1Speed(75);
          driveMotors.setM2Speed(75);
          actionTimer = currentMillis;
          bumpRecomnd = 0;
        }
      }
      else if(bumpRecomnd == 2){
        if(!actionLock){ // Left Turn
          actionLock = true;
          driveMotors.setM1Speed(-75);
          driveMotors.setM2Speed(-75);
          actionTimer = currentMillis;
          bumpRecomnd = 0;
        }
      }
      bumpFlag = false;
     }  
     actionOverride = false;
  }
       
  // Driving will be done here
  // Any sensor flag will trigger alternative behavior
  if (currentFlag || bumpFlag || irFlag) {
    if ((irRecomnd == 1) && !bumpFlag) {
    // Reverse and then turn left or right
      if(random(0,9)/5 == 1){
        if(!actionLock){ // Right Turn
          actionLock = true;
          driveMotors.setM1Speed(75);
          driveMotors.setM2Speed(75);
          actionTimer = currentMillis;
        }
      }
      else{
        if(!actionLock){ // Left Turn
          actionLock = true;
          driveMotors.setM1Speed(-75);
          driveMotors.setM2Speed(-75);
          actionTimer = currentMillis;
        }
      }
      
    }
    else if((irRecomnd == 2) && !bumpFlag ){
      if(!actionLock){ // Right Turn
        actionLock = true;
        driveMotors.setM1Speed(75);
        driveMotors.setM2Speed(75);
        actionTimer = currentMillis;
      }
      
    }
    else if((irRecomnd == 3) && !bumpFlag){
      if(!actionLock){ // Left Turn
        actionLock = true;
        driveMotors.setM1Speed(-75);
        driveMotors.setM2Speed(-75);
        actionTimer = currentMillis;
      }
      
    }
    else if(bumpRecomnd == 3){
      // Reverse motion
      if(!actionLock){
        actionLock = true;
        driveMotors.setM1Speed(75);
        driveMotors.setM2Speed(-75);
        actionTimer =  currentMillis;
        bumpRecomnd = 1;
      }
    }
      else if(bumpRecomnd == 4){
      // Reverse motion
      if(!actionLock){
        actionLock = true;
        driveMotors.setM1Speed(75);
        driveMotors.setM2Speed(-75);
        actionTimer =  currentMillis;
        bumpRecomnd = 2;
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
