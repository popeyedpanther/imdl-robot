
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

// Serial Communications stuff
int temp;
int activeBehavior = 0;
boolean newBehavior = false;
int panCmd = 90, tiltCmd = 90;        
boolean newPanTiltCmd = false;  // Signifies if a new command has been recieved
boolean newRequest = false;
boolean startButton = true;    // Start button needs to be pressed in order for Bob to start moving
boolean readyBypass = false;    //Used to bypass the serial ready check

// Pixy tracking variables
boolean pixyDebug = false;
const int cCodesSize = 1;
int activeSignature = 0, foundObject = 0, objectPlaced = 0;
int cCodes[cCodesSize] = {83}, cCodesDone[cCodesSize] = {0};     // Dont forget to update cCodesSize variable if more than one CCode is used.
int xPosition = 0, yPosition = 0;


//----Define Objects----
// Create a message object, individual messages will be seperated by :
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
    // Current behavior of Bob
    temp = piMessage.readInt();
    if (temp != 9){ 
      newBehavior = true;
      activeBehavior = temp;
    }
    
    // Request an update
    temp = piMessage.readInt();
    if (temp == 1){ 
      newRequest = true;
    }
    
    // Object has been placed
    temp = piMessage.readInt();
    if (temp == 1){ 
      objectPlaced = temp;
    }
    
    panCmd = piMessage.readInt();
    tiltCmd = piMessage.readInt();
    if (panCmd != 999 || tiltCmd != 999){ 
      newPanTiltCmd = true;
    }
  }  
}

//--------------------------------------------------------Setup---------------------------------------------------------------//
//----------------------------------------------------------------------------------------------------------------------------//

void setup() {
  
  // Attach servo to specific pins
  Pan.attach(panPWM);
  Tilt.attach(tiltPWM);
  // Align servos to default locatios
  Pan.write(panCmd);
  Tilt.write(tiltCmd);
  
  // Initialize Pixy object
  ffPixy.init();
  
  // Start serial connection
  Serial.begin(9600);
  // Attach a call function to the messenger object to call when serial information is recieved
  piMessage.attach(messageParse);
  delay(50);

  // Serial Handshake with the Odroid
  while( !Serial.available()){
    Serial.write('r');
    delay(300);
  }

  Serial.read();
}

//--------------------------------------------------------Main Loop-----------------------------------------------------------//
//----------------------------------------------------------------------------------------------------------------------------//
  
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

//---------------------------------------------Pan/Tilt Update--------------------------------------------------
//--------------------------------------------------------------------------------------------------------------

  // Update Pan servo is new command is recieved.
  updatePanTilt(); 
  
//---------------------------------------------Pixy Tracking----------------------------------------------------
//--------------------------------------------------------------------------------------------------------------

  pixyTracking();
  
//---------------------------------------------Serial Response--------------------------------------------------
//--------------------------------------------------------------------------------------------------------------

  serialResponse();

}
//----------------------------------------------------------------------------------------------------------------------------//
//----------------------------------------------------------------------------------------------------------------------------//
