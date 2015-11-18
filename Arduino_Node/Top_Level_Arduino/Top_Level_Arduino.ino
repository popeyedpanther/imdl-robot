/* Patrick Header Here
 *
 */
// Tilt servo should cyle through 60-140 with 140 facing down and 60 facing up
// Pan servo should be cycled through 0-180 with 0 facing right

#include <Servo.h>
#include <Pixy.h>
#include <SPI.h>
//#include <PixySPI_SS.h>
// Possibly include LCD library

//----- Variable Declarations -----
// Servomotor Pins
const int panPWM = 9;
const int tiltPWM = 10;

// Binary true/false array to store if the object has been recovered yet.
int blocksFound[2] = {0, 0}; // Zero is false

// Define pan tilt servo objects
Servo Pan;
Servo Tilt;

// Define pixy object
Pixy ffPixy; // Forward Facing Pixy

void setup() {
  
  // Attach servo to specific pins
  Pan.attach(panPWM);
  Tilt.attach(tiltPWM);
  // Initialize Pixy object?
  ffPixy.init();
  // Start serial and wait for the "Go" command
  Serial.begin(9600);
  // Stay in a loop until read to move on
  /*while (1){
  

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

}
