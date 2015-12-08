void updateGripper(){
 if(!gripOff){
  if (newWristGraspCmd){
    // Debug Print
    Serial.println(String(activeBehavior) + ' ' + String(wristCmd) + ' ' + String(graspCmd));
    // Makes sure desired angles are acceptable
    if((wristCmd >= 40 && wristCmd <= 170) && wristCmd != 999){
      Wrist.write(wristCmd);
    }
    if((graspCmd >= 45 && graspCmd <= 135) && graspCmd != 999){
      Grasp.write(graspCmd);
    } 
    newWristGraspCmd = false;
  }  
 }
}

//----------------------------------------------------------------------------------------------------------------------------//
//----------------------------------------------------------------------------------------------------------------------------//

void PDController(){
// PD controller
// Need to convert to actual position measurements.
  leftInput = leftEncoder.read()*C; 
  leftDone = leftPID.Compute();
  
  rightInput = rightEncoder.read()*C;
  rightDone = rightPID.Compute();

  if(leftDone && rightDone){
    // Set the speeds together
    driveMotors.setSpeeds(K*leftOutput, K*rightOutput);
    leftDone = false;
    rightDone = false;
  }  
}

//----------------------------------------------------------------------------------------------------------------------------//
//----------------------------------------------------------------------------------------------------------------------------//

// This function is used by the Pololu motor drivers to handle errors in operation
void stopIfFault()
{
  if (driveMotors.getM1Fault())
  {
    //Serial.println("M1 fault:");
    while(1);
  }
  if (driveMotors.getM2Fault())
  {
    //Serial.println("M2 fault:");
    while(1);
  }
}

