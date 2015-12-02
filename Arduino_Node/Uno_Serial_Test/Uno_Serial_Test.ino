


/* Patrick Header Here
 *
 */
// Tilt servo should cyle through 60-140 with 140 facing down and 60 facing up
// Pan servo should be cycled through 0-180 with 0 facing right

#include <Servo.h>
//#include <Pixy.h>
#include <SPI.h>
#include <Messenger.h>
//#include <PixySPI_SS.h>
// Possibly include LCD library

//----- Variable Declarations -----
// State Variable
int State = 0;

// 
const int panPWM = 6;
const int tiltPWM = 5;

// Binary true/false array to store if the object has been recovered yet.
int blocksFound[2] = {
  0, 0}; // Zero is false

//
int temp;

// Serial Communications
const int buffersize = 4;
int panCmd=90;
int tiltCmd=90;
boolean newPanTiltCmd = false;

// Create a message object
Messenger piMessage = Messenger(':');

// Define pan tilt servo objects
Servo Pan;
Servo Tilt;

// Define pixy object
//Pixy ffPixy; // Forward Facing Pixy

void messageParse(){
  // This will set the variables that need to be changed
  // from the message
  if (piMessage.available()){
    temp = piMessage.readInt();
    if (temp != 9){ 
      State = temp;
    }
    temp = piMessage.readInt();
    if (temp != 999){ 
      panCmd = temp;
      newPanTiltCmd = true;
    }
    temp = piMessage.readInt();
    if (temp != 999){ 
      tiltCmd = temp;
      newPanTiltCmd = true;
    }    
  }  
}

void setup() {

  // Attach servo to specific pins
  Pan.attach(panPWM);
  Tilt.attach(tiltPWM);
  // Align servo to default position
  Pan.write(panCmd);
  Tilt.write(tiltCmd);

  // Initialize Pixy object?
  // ffPixy.init();

  // Start serial and wait for the "Go" command
  Serial.begin(9600);
  Serial.flush();
  piMessage.attach(messageParse);

  // Stay in a loop until read to move on
  /*while (1){
   
   
   */
}

void loop() {

  while( Serial.available() ) piMessage.process(Serial.read());

  // put your main code here, to run repeatedly:
  /* This code should be looking for colored blocks that meet certain color codes.
   *  It should be able to store whether or not the colored code was moved already
   *  Once it finds a new color code it should send a command to the Raspberry Pi where it will begin the align and pickup behavior.
   *  During this behavior the pan and tilt functions will be disabled. Until the localize behavior is started. This will only take in desired servo angles
   *  and apply them. SHould include some deadband to stop jittering.
   *  This will maybe include the code to display the LCD.
   */

  if (newPanTiltCmd){
    Serial.println("Repeat Back " + String(State)+ " " + String(panCmd) + " " + String(tiltCmd));  
    // Make sure to check inout bounds
    if((panCmd >= 0 && panCmd <= 180) && panCmd != 999){
      Pan.write(panCmd);
      //Serial.println("Repeat Back " + String(panCmd));  
    }
    if((tiltCmd >= 80 && tiltCmd <= 130) && tiltCmd != 999){
      Tilt.write(tiltCmd);
      //Serial.println("Repeat Back " + String(State)+ " " + String(panCmd) +" " String(tiltCmd));  
    } 
    newPanTiltCmd = false;
  }  
}

