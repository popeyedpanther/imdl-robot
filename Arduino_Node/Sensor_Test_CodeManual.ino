
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
#include <DualVNH5019MotorDriver.h> // For use with the Dual motor drivers from Pololu

//-------Define variables here-------------------------------

// ---PWM Pin Setting---

// PWM pins for the two drive motors
const int PWMDrivePin_Left = 11;    // Timer 1 16-bit
const int PWMDrivePin_Right = 12;   // Timer 1 16-bit

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
const int leftEncoderAPin = 2;   // Digital Pin 21, 2 interrupt pin for Mega 2560
const int leftEncoderBPin = 3;   // Digital Pin 20, 3 interrupt pin for Mega 2560

const int rightEncoderAPin = 4;  // Digital Pin 19, 4 interrupt pin for Mega 2560
const int rightEncoderBPin = 5;  // Digital Pin 18, 5 interrupt pin for Mega 2560

// IR sensors
const int leftIRPin = 0;          // A0
const int rightIRPin = 1;         // A1

// Current Sensors
const int leftCurrentPin = 8;    // A8, CS1
const int rightCurrentPin = 9;   // A9, CS2

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

// Used for sensor sampling times
unsigned long currentMillis;            // Stores the current time reading in milliseconds
unsigned long previousMillis_IR=0;      // Stores the previous time the IR sensors were read
unsigned long previousMillis_Current=0; // Stores the previous time the Current sensors were read
unsigned long previousMillis_Bump=0;    // Stores the previous time the Bump sensors were read

// Encoder Counting Variables
volatile unsigned int encoderLeftPos = 0; // A counter
unsigned int lastReportedLeftPos = 1;     // Change managment
static boolean leftRotating = false;      // Debounce managment
volatile boolean leftA_set = false;
volatile boolean leftB_set = false;

volatile unsigned int encoderRightPos = 0; // A counter
unsigned int lastReportedRightPos = 1;     // Change managment
static boolean rightRotating = false;      // Debounce managment
volatile boolean rightA_set = false;
volatile boolean rightB_set = false;

// Define drive motor object
DualVNH5019MotorDriver driveMotors(Drive_INA1,Drive_INB1,PWMDrivePin_Left,\
Drive_EN1DIAG1,leftCurrentPin,Drive_INA2,Drive_INB2,PWMDrivePin_Right,Drive_EN2DIAG2,\
rightCurrentPin, 1);

// Test Variables
const String leftString = "Left IR Reading: ";
const String rightString = "Right IR Reading: ";
boolean firsttime = true;
 
//-------Setup Function-----------------------------------

void setup()  // Needs to stay in setup until all necessary communications can be verified
{

  // Initialize drive motor object
  driveMotors.init();

  // Set interrupt pins to input
  pinMode(leftBumpPin,INPUT);
  pinMode(rightBumpPin,INPUT);
  pinMode(leftEncoderAPin,INPUT);
  pinMode(leftEncoderBPin,INPUT);
  pinMode(rightEncoderAPin,INPUT);
  pinMode(rightEncoderBPin,INPUT);

  // Turn on pullup resistors
  digitalWrite(leftBumpPin, HIGH);
  digitalWrite(rightBumpPin, HIGH);
  digitalWrite(leftEncoderAPin, HIGH);
  digitalWrite(leftEncoderBPin, HIGH);
  digitalWrite(rightEncoderAPin, HIGH);
  digitalWrite(rightEncoderBPin, HIGH);

  // Attached Interrupt pins
  attachInterrupt(0, bumpLeft, RISING);         // Digital Pin 2
  attachInterrupt(1, bumpRight, RISING);        // Digital Pin 3
  attachInterrupt(2, doLeftEncoderA, CHANGE);   // Digital Pin 21
  attachInterrupt(3, doLeftEncoderB, CHANGE);   // Digital Pin 20
//  attachInterrupt(4, doRightEncoderA, CHANGE);  // Digital Pin 19
//  attachInterrupt(5, doRightEncoderB, CHANGE);  // Digital Pin 18
  
  // Initiliaze serial communications
  Serial.begin(9600);           // set up Serial library at 9600 bps  boolean readyBypass = true; 

}

//--------Main Loop--------------------------------------

void loop() // run over and over again
{
  // Contains all actions the robot will perform.
  // Obstacle avoidance behaviors should override and sent commands from the Raspberry Pi 2 until they have completed.
  // Should always check IR and Bump sensors before performing requested tasks.
  // Camera positioning should not be overrid by obstacle avoidance behaviors


  // Encoder counter will always run and cannot be blocked.

  // Reset the deboucer for each encoder
  leftRotating = true;
  rightRotating = true;
  
  currentMillis = millis(); // Program run time in milliseconds. Used for sensor sampling.

  // Distance Measurement IR Smart Sensor
  if (currentMillis - previousMillis_IR >= irPeriod){
    previousMillis_IR = currentMillis;
    
    // Read in the left IR voltage and put into a buffer
    irValue = analogRead(leftIRPin);
    irLeft[0] = irLeft[1];
    irLeft[1] = irLeft[2];
    irLeft[2] = irValue;
    
    // Read in the right IR voltage and put into a buffer
    irValue = analogRead(rightIRPin);
    irRight[0] = irRight[1];
    irRight[1] = irRight[2];
    irRight[2] = irValue;
    
    // Calculate the average input
    irLeftAvg = (float(irLeft[0])+float(irLeft[1])+float(irLeft[2]))/3;
    irRightAvg = (float(irRight[0])+float(irRight[1])+float(irRight[2]))/3;
          
    // Debugging outputs
      Serial.print(leftString + String(irLeftAvg) + " " );
      Serial.println(rightString + String(irRightAvg));  
  
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
    
    /* Debugging Outputs
      Serial.print("Left Current: " + String(currentLeftAvg) + " " );
      Serial.println("Right Current: " + String(currentRightAvg)); 

    */
  }

  // Supply Motor direction here to test encoders
  if(firsttime){ // This branch is for normal operations
    // Forward motion
    driveMotors.setM1Speed(-75);
    driveMotors.setM2Speed(75);
    firsttime = false;
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
  Serial.println("bump Left"); 
}

// ---Right Bump Sensor Interrupt Function---
void bumpRight()
{
  Serial.println("Bump Right");
}

// ---Encoder Interrupt Functions go Here---
// By rafbuff
// http://playground.arduino.cc/Main/RotaryEncoders#Example14
// Interrupt on A changing state
void doLeftEncoderA(){
  // debounce
  if ( leftRotating ) delay (1);  // wait a little until the bouncing is done

  // Test transition, did things really change? 
  if( digitalRead(leftEncoderAPin) != leftA_set ) {  // debounce once more
    leftA_set = !leftA_set;

    // adjust counter + if A leads B
    if ( leftA_set && !leftB_set ) 
      leftEncoderPos += 1;

    leftRotating = false;  // no more debouncing until loop() hits again
  }
}

// Interrupt on B changing state, same as A above
void doLeftEncoderB(){
  if ( leftRotating ) delay (1);
  if( digitalRead(leftEncoderBPin) != leftB_set ) {
    leftB_set = !leftB_set;
    //  adjust counter - 1 if B leads A
    if( leftB_set && !leftA_set ) 
      leftEncoderPos -= 1;

    leftRotating = false;
  }
}
