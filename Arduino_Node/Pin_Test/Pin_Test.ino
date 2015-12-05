#include <DualVNH5019MotorDriver.h>

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

// Define drive motor object
DualVNH5019MotorDriver driveMotors(Drive_INA1,Drive_INB1,PWMDrivePin_Left,\
Drive_EN1DIAG1,leftCurrentPin,Drive_INA2,Drive_INB2,PWMDrivePin_Right,Drive_EN2DIAG2,\
rightCurrentPin, 1);

void setup(){
  
  // Initialize drive motor object
  driveMotors.init();

  
  // Set interrupt pins to input
  pinMode(leftEncoderAPin,INPUT);
  pinMode(leftEncoderBPin,INPUT);

  // Turn on pullup resistors
  digitalWrite(leftEncoderAPin, HIGH);
  digitalWrite(leftEncoderBPin, HIGH);

  // Attached Interrupt pins
  attachInterrupt(2, leftEncA, CHANGE);         // Digital Pin 2
  attachInterrupt(3, leftEncB, CHANGE);        // Digital Pin 3

  
  Serial.begin(9600);
  
}

void loop(){
    driveMotors.setSpeeds(-75, 75);
  
  
}

void leftEncA(){
  Serial.println("Boom");
}

void leftEncB(){
  Serial.println("Pow");
}

