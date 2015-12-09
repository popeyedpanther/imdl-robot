

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
#include <Servo.h>                    // Used for controllig hobby servo motors
#include <DualVNH5019MotorDriver.h>   // For use with the Dual motor drivers from Pololu
#include <Encoder.h>                  // Used for dead reckoning and feedback for PD control
#include <PID_v1.h>                   // Used to perform PD control on motors
#include <Messenger.h>                // Used to parse serial communications
//#include <Utility.h>                  // Just for the sign function


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

// Sensor Flags
boolean irFlag =false;              // Will be set true if an IR condition is met
  int irRecomnd = 0;                // Recommendation will be set depending on specific combintaion of IR readings
boolean currentFlag = false;        // Will be set true if Current sense condition is met
  int currentRecomnd = 0;           // Will be set depending of specific combinations
volatile boolean bumpFlag = false;  // Will be set true if a bump sensor is triggered
  volatile int bumpRecomnd = 0;     // Will indicate whether left or right sensor was triggered

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
int requestState = 0, requestComplete = 0, OAState = 0;

double robotSpeed = 5;

// Arbiter Variables
boolean OAOverride = false, OADone = true;            // Did a recommended OA motion finish?
int turnDirection = 0, motionDirection = 0;
unsigned long OATimer = 0;
boolean Reverse = false;
boolean Turn = false;

// State Update Variables/ Dead Reckoning
float dx = 0, dy = 0, dtheta = 0;
float leftWheelChange = 0, rightWheelChange = 0;
float b = 10.375; // inches (distance between wheel centers)
double leftStartPoint = 0, rightStartPoint = 0;
double leftStopPoint = 0, rightStopPoint = 0;
int oldMotionDirection = 0;
boolean isStopped = true;


// PID input variables
double leftSetpoint, leftInput, leftOutput;
double rightSetpoint, rightInput, rightOutput;

boolean leftDone = false, rightDone = false;
double leftOffset = 0.50, rightOffset = 1.25; // Distance offsets to account stopping time.

// PID Tuning Paramters
double lKp = 2.75, lKi = 0, lKd = 2.25;
double rKp = 3.75, rKi = 0, rKd = 2.25;

double K = 5;

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
    OAState  = piMessage.readInt();
    if (OAState == 0 ) {OAoff = true;}
  }  
}

 
//--------------------------------------------------------Setup---------------------------------------------------------------//
//----------------------------------------------------------------------------------------------------------------------------//

void setup()  // Needs to stay in setup until all necessary communications can be verified
{
  
  // Attach the servo objects to pins
  Wrist.attach(PWMWristPin);
  Grasp.attach(PWMGraspPin);

  // Update gripper to default position before starrting
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

//--------------------------------------------------------Main Loop-----------------------------------------------------------//
//----------------------------------------------------------------------------------------------------------------------------//

void loop()
{
  
  currentMillis = millis(); // Program run time in milliseconds.
  
  // Read serial and call parser
  if (currentMillis - previousMillis_Serial > serialPeriod){
    previousMillis_Serial = currentMillis;
    while( Serial.available() ) piMessage.process(Serial.read());
  }

  // Act of behavior here like locking certain commands or something
  updateBehavior();   // Behavior TAB

  // Update the gripper with new position commands
  updateGripper();  // Actuators TAB

  currentMillis = millis(); // Program run time in milliseconds. Used for sensor sampling.

//-------------------------------------------------Sensors------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------

  //---- Distance Measurement IR Smart Sensor ----
  if (currentMillis - previousMillis_IR >= irPeriod){
    previousMillis_IR = currentMillis;
    updateIR();   // Sensor TAB
  }

  //---- Amperage Measurement Smart Sensor ----
  if (currentMillis - previousMillis_Current >= currentPeriod){
    previousMillis_Current = currentMillis;
    updateCurrent();    // Sensor TAB
  }
  

//----------------------------------------------Smart Arbiter---------------------------------------------------
//--------------------------------------------------------------------------------------------------------------

  // Based on the obstacle avoidance recommendation and commanded inputs smartArbiter will decide what 
  // motions happen
  smartArbiter();

//---------------------------------------------Dead Reckoning---------------------------------------------------
//--------------------------------------------------------------------------------------------------------------
  deadReckoning();

//---------------------------------------------Motor Controller-------------------------------------------------
//--------------------------------------------------------------------------------------------------------------
      
  // Perform PD controller update
  motorController();   // Actuator TAB


//-------------------------------------------Serial Send Update-------------------------------------------------
//--------------------------------------------------------------------------------------------------------------

}
//----------------------------------------------------------------------------------------------------------------------------//
//----------------------------------------------------------------------------------------------------------------------------//
