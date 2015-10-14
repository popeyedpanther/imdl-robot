#include "DualVNH5019MotorDriver.h"

// This file has been modified to allow for custom PWM pins for use on a 
// Arduino Mega 2560
// Patrick Neal
// Unveristy of Florida

// Constructors ////////////////////////////////////////////////////////////////

DualVNH5019MotorDriver::DualVNH5019MotorDriver()
{
  //Pin map
  _INA1 = 2;
  _INB1 = 4;
  _PWM1 = 9;
  _EN1DIAG1 = 6;
  _CS1 = A0; 
  _INA2 = 7;
  _INB2 = 8;
  _PWM2 = 10;
  _EN2DIAG2 = 12;
  _CS2 = A1;
  _Timer = 1;
}

DualVNH5019MotorDriver::DualVNH5019MotorDriver(unsigned char INA1, unsigned char INB1, unsigned char PWM1, unsigned char EN1DIAG1, unsigned char CS1, 
                                               unsigned char INA2, unsigned char INB2, unsigned char PWM2, unsigned char EN2DIAG2, unsigned char CS2,
											   int Timer)
{
  //Pin map
  //PWM1 and PWM2 can be remapped but becareful what time the pins are mapped to.
  _INA1 = INA1;
  _INB1 = INB1;
  _PWM1 = PWM1;
  _EN1DIAG1 = EN1DIAG1;
  _CS1 = CS1;
  _INA2 = INA2;
  _INB2 = INB2;
  _PWM2 = PWM2;
  _EN2DIAG2 = EN2DIAG2;
  _CS2 = CS2;
  _Timer = Timer;
}

// Public Methods //////////////////////////////////////////////////////////////
void DualVNH5019MotorDriver::init()
{
// Define pinMode for the pins and set the frequency for timer1.

  pinMode(_INA1,OUTPUT);
  pinMode(_INB1,OUTPUT);
  pinMode(_PWM1,OUTPUT);
  pinMode(_EN1DIAG1,INPUT);
  pinMode(_CS1,INPUT);
  pinMode(_INA2,OUTPUT);
  pinMode(_INB2,OUTPUT);
  pinMode(_PWM2,OUTPUT);
  pinMode(_EN2DIAG2,INPUT);
  pinMode(_CS2,INPUT);
  #if defined(__AVR_ATmega168__)|| defined(__AVR_ATmega328P__) || defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega2560__)
  // Timer 1 configuration
  // prescaler: clockI/O / 1
  // outputs enabled
  // phase-correct PWM
  // top of 400
  //
  // PWM frequency calculation
  // 16MHz / 1 (prescaler) / 2 (phase-correct) / 400 (top) = 20kHz
  if (_Timer == 1)
  {
	TCCR1A = 0b10100000;
	TCCR1B = 0b00010001;
	ICR1 = 400;
  }
  else if (_Timer == 3)
  {
	TCCR3A = 0b10100000;
	TCCR3B = 0b00010001;
	ICR3 = 400;
  }
  else if (_Timer == 4)
  {
	TCCR4A = 0b10100000;
	TCCR4B = 0b00010001;
	ICR4 = 400;
  }
  else if (_Timer == 5)
  {
	TCCR5A = 0b10100000;
	TCCR5B = 0b00010001;
	ICR5 = 400;
  }
  #endif
}
// Set speed for motor 1, speed is a number betwenn -400 and 400
void DualVNH5019MotorDriver::setM1Speed(int speed)
{
  unsigned char reverse = 0;
  
  if (speed < 0)
  {
    speed = -speed;  // Make speed a positive quantity
    reverse = 1;  // Preserve the direction
  }
  if (speed > 400)  // Max PWM dutycycle
    speed = 400;
  #if defined(__AVR_ATmega168__)|| defined(__AVR_ATmega328P__) || defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega2560__)
  if (_Timer == 1)
  {
	OCR1A = speed;
  }
  else if (_Timer == 3)
  {
	OCR3A = speed;
  }
  else if (_Timer == 4)
  {
	OCR4A = speed;
  }
  else if (_Timer == 5)
  {
	OCR5A = speed;
  }
  #else
  analogWrite(_PWM1,speed * 51 / 80); // default to using analogWrite, mapping 400 to 255
  #endif
  if (speed == 0)
  {
    digitalWrite(_INA1,LOW);   // Make the motor coast no
    digitalWrite(_INB1,LOW);   // matter which direction it is spinning.
  }
  else if (reverse)
  {
    digitalWrite(_INA1,LOW);
    digitalWrite(_INB1,HIGH);
  }
  else
  {
    digitalWrite(_INA1,HIGH);
    digitalWrite(_INB1,LOW);
  }
}

// Set speed for motor 2, speed is a number betwenn -400 and 400
void DualVNH5019MotorDriver::setM2Speed(int speed)
{
  unsigned char reverse = 0;
  
  if (speed < 0)
  {
    speed = -speed;  // make speed a positive quantity
    reverse = 1;  // preserve the direction
  }
  if (speed > 400)  // Max 
    speed = 400;
  #if defined(__AVR_ATmega168__)|| defined(__AVR_ATmega328P__) || defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega2560__)
    if (_Timer == 1)
  {
	OCR1B = speed;
  }
  else if (_Timer == 3)
  {
	OCR3B = speed;
  }
  else if (_Timer == 4)
  {
	OCR4B = speed;
  }
  else if (_Timer == 5)
  {
	OCR5B = speed;
  }
  #else
  analogWrite(_PWM2,speed * 51 / 80); // default to using analogWrite, mapping 400 to 255
  #endif 
  if (speed == 0)
  {
    digitalWrite(_INA2,LOW);   // Make the motor coast no
    digitalWrite(_INB2,LOW);   // matter which direction it is spinning.
  }
  else if (reverse)
  {
    digitalWrite(_INA2,LOW);
    digitalWrite(_INB2,HIGH);
  }
  else
  {
    digitalWrite(_INA2,HIGH);
    digitalWrite(_INB2,LOW);
  }
}

// Set speed for motor 1 and 2
void DualVNH5019MotorDriver::setSpeeds(int m1Speed, int m2Speed)
{
  setM1Speed(m1Speed);
  setM2Speed(m2Speed);
}

// Brake motor 1, brake is a number between 0 and 400
void DualVNH5019MotorDriver::setM1Brake(int brake)
{
  // normalize brake
  if (brake < 0)
  {
    brake = -brake;
  }
  if (brake > 400)  // Max brake
    brake = 400;
  digitalWrite(_INA1, LOW);
  digitalWrite(_INB1, LOW);
  #if defined(__AVR_ATmega168__)|| defined(__AVR_ATmega328P__) || defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega2560__)
  if (_Timer == 1)
  {
	OCR1A = brake;
  }
  else if (_Timer == 3)
  {
	OCR3A = brake;
  }
  else if (_Timer == 4)
  {
	OCR4A = brake;
  }
  else if (_Timer == 5)
  {
	OCR5A = brake;
  }
  #else
  analogWrite(_PWM1,brake * 51 / 80); // default to using analogWrite, mapping 400 to 255
  #endif
}

// Brake motor 2, brake is a number between 0 and 400
void DualVNH5019MotorDriver::setM2Brake(int brake)
{
  // normalize brake
  if (brake < 0)
  {
    brake = -brake;
  }
  if (brake > 400)  // Max brake
    brake = 400;
  digitalWrite(_INA2, LOW);
  digitalWrite(_INB2, LOW);
  #if defined(__AVR_ATmega168__)|| defined(__AVR_ATmega328P__) || defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega2560__)
    if (_Timer == 1)
  {
	OCR1B = brake;
  }
  else if (_Timer == 3)
  {
	OCR3B = brake;
  }
  else if (_Timer == 4)
  {
	OCR4B = brake;
  }
  else if (_Timer == 5)
  {
	OCR5B = brake;
  }
  #else
  analogWrite(_PWM2,brake * 51 / 80); // default to using analogWrite, mapping 400 to 255
  #endif
}

// Brake motor 1 and 2, brake is a number between 0 and 400
void DualVNH5019MotorDriver::setBrakes(int m1Brake, int m2Brake)
{
  setM1Brake(m1Brake);
  setM2Brake(m2Brake);
}

// Return motor 1 current value in milliamps.
unsigned int DualVNH5019MotorDriver::getM1CurrentMilliamps()
{
  // 5V / 1024 ADC counts / 144 mV per A = 34 mA per count
  return analogRead(_CS1) * 34;
}

// Return motor 2 current value in milliamps.
unsigned int DualVNH5019MotorDriver::getM2CurrentMilliamps()
{
  // 5V / 1024 ADC counts / 144 mV per A = 34 mA per count
  return analogRead(_CS2) * 34;
}

// Return error status for motor 1 
unsigned char DualVNH5019MotorDriver::getM1Fault()
{
  return !digitalRead(_EN1DIAG1);
}

// Return error status for motor 2 
unsigned char DualVNH5019MotorDriver::getM2Fault()
{
  return !digitalRead(_EN2DIAG2);
}
