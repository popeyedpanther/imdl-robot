
/* Patrick Header Here
 *
 */
// Tilt servo should cyle through 60-140 with 140 facing down and 60 facing up
// Pan servo should be cycled through 0-180 with 0 facing right

#include <Servo.h>
#include <Pixy.h>
#include <SPI.h>
#include <Messenger.h>
//#include <LiquidCrystal.h>

//----- Variable Declarations -----

// Servomotor Pins
const int panPWM = 6;
const int tiltPWM = 5;

// Button Pin
const int startPin = 7;
int buttonState = HIGH;
int button;
int previousButton = LOW;

//---- Timing Variables ----
unsigned long currentMillis;

unsigned long debouncePeriod = 200;


// Binary true/false array to store if the object has been recovered yet.
int blocksFound[2] = {0, 0}; // Zero is false

// Serial Communications stuff
int inByte;
int activeBehavior = 0;
boolean newBehavior = false;
int panCmd = 90, tiltCmd = 90;        
boolean newPanTiltCmd = false;  // Signifies if a new command has been recieved
boolean newRequest = false;
boolean startButton = true;    // Start button needs to be pressed in order for Bob to start moving
boolean readyBypass = false;    //Used to bypass the serial ready check

//----Define Objects----
// Create a message object
Messenger piMessage = Messenger(':');

// Define pan tilt servo objects
Servo Pan;
Servo Tilt;

// Define pixy object
Pixy ffPixy; // Forward Facing Pixy

//----Serial Communications Parser----
void messageParse(){
  // This will set the variables that need to be changed
  // from the message
  if (piMessage.available()){
    activeBehavior = piMessage.readInt();
    if (activeBehavior != 9){ 
      newBehavior = true;
    }
    panCmd = piMessage.readInt();
    tiltCmd = piMessage.readInt();
    if (panCmd != 999 || tiltCmd != 999){ 
      newPanTiltCmd = true;
    }
  }  
}

void setup() {
  
  // Attach servo to specific pins
  Pan.attach(panPWM);
  Tilt.attach(tiltPWM);
  // Align servos to default locatios
  Pan.write(panCmd);
  Tilt.write(tiltCmd);
  
  // Initialize Pixy object
  ffPixy.init();
  
  // Start serial and wait for the "Go" command
  Serial.begin(9600);
  piMessage.attach(messageParse);
  delay(50);

  while( !Serial.available()){
    Serial.write('r');
    delay(300);
  }

  Serial.read();

  /*
  // Stay in a loop until read to move on
  while(1){
    // Set readyBypass to true to skip waiting for Odroid confirmation and button switch confimation
    if (readyBypass){
      break;
    }
    
    if (Serial.available() > 0){
      inByte = Serial.read();
    }
    
    
    if (inByte == 115 && startButton){
        Serial.println('g');
        break;
    }

    Serial.println('r');

    delay(100);
  }
  */
}
  
void loop() {
  // put your main code here, to run repeatedly:
  /* This code should be looking for colored blocks that meet certain color codes.
   *  It should be able to store whether or not the colored code was moved already
   *  Once it finds a new color code it should send a command to the Raspberry Pi where it will begin the align and pickup behavior.
   *  During this behavior the pan and tilt functions will be disabled. Until the localize behavior is started. This will only take in desired servo angles
   *  and apply them. SHould include some deadband to stop jittering.
   *  This will maybe include the code to display the LCD.
   */

  while( Serial.available() ) piMessage.process(Serial.read());

  // Update Pan servo is new command is recieved.
  updatePanTilt(); 
  
  /*
  // Pixy Read
  static int i = 0;
  int j;
  uint16_t blocks;
  char buf[32]; 
  
  // grab blocks!
  blocks = ffPixy.getBlocks();
  
  // If there are detect blocks, print them!
  if (blocks)
  {
    i++;
    
    // do this (print) every 50 frames because printing every
    // frame would bog down the Arduino
    if (i%50==0)
    {
      sprintf(buf, "Detected %d:\n", blocks);
      Serial.print(buf);
      for (j=0; j<blocks; j++)
      {
        sprintf(buf, "  block %d: ", j);
        Serial.print(buf); 
        ffPixy.blocks[j].print();
      }
    }
  }  
  */

}
