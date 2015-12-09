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

void motorController(){

  if(newDistance || OAOverride){

    if(isStopped && motionDirection == 1){
      // Forward Motion
      leftStartPoint = leftEncoder.read()*C;
      rightStartPoint = rightEncoder.read()*C;
      leftSetpoint = robotSpeed;
      rightSetpoint = robotSpeed;
      isStopped = false;
      Serial.println("I made it forward");
      newDistance = false;
      
    }
    else if(isStopped && motionDirection == 2){
      // Reverse Motion
      leftStartPoint = leftEncoder.read()*C;
      rightStartPoint = rightEncoder.read()*C;
      leftSetpoint = -robotSpeed;
      rightSetpoint = -robotSpeed;
      isStopped = false;
      Serial.println("I made it reverse");
      newDistance = false;
      
    }
    else if(isStopped && motionDirection == 3){
      // Left Turn
      leftStartPoint = leftEncoder.read()*C;
      rightStartPoint = rightEncoder.read()*C;
      leftSetpoint = -robotSpeed;
      rightSetpoint = robotSpeed;
      isStopped = false;
      Serial.println("I made it left");
      newDistance = false;
      
    }
    else if(isStopped && motionDirection == 4){
      // Right Turn
      leftStartPoint = leftEncoder.read()*C;
      rightStartPoint = rightEncoder.read()*C;
      leftSetpoint = robotSpeed;
      rightSetpoint = -robotSpeed;
      isStopped = false;
      Serial.println("I made it right");
      newDistance = false;
      
    }
  }   
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

