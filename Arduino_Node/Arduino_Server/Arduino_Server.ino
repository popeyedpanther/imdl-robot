
/* Patrick's Block Header
 *  
 *  
 *  
 *  
 *  
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
const int PWMDrivePin_Left = 12;    // Timer 1 16-bit
const int PWMDrivePin_Right = 11;   // Timer 1 16-bit

// PWM pin for camera pan and tilt
const int PWMPanPin = 5; // for the gearmotor, Chosen because it is on timer 3 (Dont use Pins 3 and 2 unless you want 20kHz PWM)
const int PWMTiltPin = 10; // for the servomotor, Timer 2 8-bit

// Additional PWM Pins for Manipulator. May not be used
const int PWMLowerPin = 7;
const int PWMGraspPin = 6;

// ---LED Pins---

// ---Pololu Motor Pins---
const int Drive_INA1;     // Left Drive Motor
const int Drive_INB1;     // Left Drive Motor
const int Drive_EN1DIAG1; // Left Drive Motor

const int Drive_INA2;     // Right Drive Motor
const int Drive_INB2;     // Right Drive Motor
const int Drive_EN2DIAG2; // Right Drive Motor

const int Pan_INA1;       // Pan Motor
const int Pan_INB1;       // Pan Motor
const int Pan_EN1DIAG1;   // Pan Motor

// ---Sensor Pins---

// IR sensors
const int IR_Left_Pin = 0;          // A0
const int IR_Right_Pin = 1;         // A1

// Bump Sensors
const int Bump_Left_Pin = 2;        // A2
const int Bump_Right_Pin = 3;       // A3

// Encoder Inputs
const int Left_Encoder_1_Pin = 4;   // A4
const int Left_Encoder_2_Pin = 5;   // A5

const int Right_Encoder_1_Pin = 6;  // A6
const int Right_Encoder_2_Pin = 7;  // A7

const int Pan_Encoder_1_Pin = 8;    // A8
const int Pan_Encoder_2_Pin = 9;    // A9

// Current Sensors
const int Current_Left_Pin = 10;    // A10, CS1
const int Current_Right_Pin = 11;   // A11, CS2
const int Current_Pan_Pin = 12;     // A12, CS1

// Push button start
const int Push_button = A13; // For push to start button

// ---Sensor sampling periods---
const unsigned long IR_Period = 500;      // Sampling period for IR Sensors (ms)
const unsigned long Current_Period = 250; // Sampling period for current sensor (ms)
const unsigned long Bump_Period = 100;    // Sampling period for Bump Switches (ms)

// ---Constants---
const int eps = 2;

// ---Define non-constant variables---
int inByte = 0;     // Variable that will store incoming byte from serial
int IR_Left;        // Stores Left IR Reading
int IR_Right;       // Stores Right IR Reading
int Current_Left;   // Stores Left Drive Motor Current Reading
int Current_Right;  // Stores Right Drive Motor Current Reading
int Current_Pan;    // Stores Pan Motor Current Reading
int Bump_Left;      // May not be needed
int Bump_Right;     // May not be needed

// Sensor Sampling
unsigned long currentMillis;          // Stores the current time reading in milliseconds
unsigned long previousMillis_IR;      // Stores the previous time the IR sensors were read
unsigned long previousMillis_Current; // Stores the previous time the Current sensors were read
unsigned long previousMillis_Bump;    // Stores the previous time the Bump sensors were read


// Define servo objects
Servo Tilt;
Servo Lower;
Servo Grasp;

// Define drive motor object
DualVNH5019MotorDriver Drive_Motors;
Drive_Motors._INA1 = Drive_INA1;
Drive_Motors._INB1 = Drive_INB1;
Drive_Motors._PWM1 = PWMDrivePin_Left;
Drive_Motors._END1DIAG1 = Drive_END1DIAG1;
Drive_Motors._CS1 = Current_Left_Pin;

Drive_Motors._INA2 = Drive_INA2;
Drive_Motors._INB2 = Drive_INB2;
Drive_Motors._PWM2 = PWMDrivePin_Right;
Drive_Motors._END2DIAG2 = Drive_END2DIAG2;
Drive_Motors._CS2 = Current_Right_Pin;
 
DualVNH5019MotorDriver Pan_Motor;
Pan_Motor._Timer = 3;
Pan_Motor._INA1 = Pan_INA1;
Pan_Motor._INB1 = Pan_INB1;
Pan_Motor._PWM1 = PWMPanPin;
Pan_Motor._END1DIAG1 = Pan_END1DIAG1;
Pan_Motor._CS1 = Current_Pan_Pin;
 
//-------Setup Function-----------------------------------

void setup()  // Needs to stay in setup until all necessary communications can be verified
{
  
  // Setupt PWM pins for gearmotors to output
  pinMode(PWMDrivePin_Left,OUTPUT);
  pinMode(PWMDrivePin_Right,OUTPUT);
  pinMode(PWMPanPin,OUTPUT);

  // Attach the servo objects to pins
  Tilt.attach(PWMTiltPin);
  Lower.attach(PWMLowerPin);
  Grasp.attach(PWMGraspPin);

  // Initialize drive motor object
  Drive_Motors.init()
  Pan_Motor.init()

  // Initiliaze serial communications
  Serial.begin(9600);           // set up Serial library at 9600 bps

  boolean readyBypass = true; 
  
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
  /*
   * 
   * 
   * 
   * 
   * 
   */
  
  currentMillis = millis() // Program run time in milliseconds. Used for sensor sampling.

  if (currentMillis - previousMillis_IR >= IR_Period){
    previousMillis_IR = currentMillis;
    IR_Left  = analogRead(IR_Left_Pin);
    IR_Right = analogRead(IR_Right_Pin);


    }

  if (currentMillis - previousMillis_Current >= Current_Period){
    previousMillis_Current = currentMillis;
    Current_Left  = Drive_Motors.getM1CurrentMilliamps();
    Current_Right = Drive_Motors.getM2CurrentMilliamps();
    Current_Pan  = Pan_Motor.getM1CurrentMilliamps();


    }

  if (currentMillis - previousMillis_Bump >= Bump_Period){
    previousMillis_Bump = currentMillis; 
    Bump_Left  = analogRead(Bump_Left_Pin)
    Bump_Right = analogRead(Bump_Right_Pin)


    }
    
  // Different rate of sensor collection
  // if (obstacle avoidance check){

  // end
  // Perform requested camera movements
  Serial.println("Hello world!");  // prints hello with ending line break 

  delay(1000);
}

// ---Extra Error Function---
void stopIfFault()
{
  if (md.getM1Fault())
  {
    Serial.println("M1 fault");
    while(1);
  }
  if (md.getM2Fault())
  {
    Serial.println("M2 fault");
    while(1);
  }
}
