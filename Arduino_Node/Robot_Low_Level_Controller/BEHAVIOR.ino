void updateBehavior(){
  if( newBehavior){
    if(activeBehavior == 0){
      // Robot should be inactive
      // Obstacle avoidance should be off and the robot should not move.
      // Gripper should not move also
      OAoff = true;
      OAOverride = false;
      gripOff = true;
      }
    else if(activeBehavior == 1){
      // Search/Wander Behavior
      // Obstacle avoidance should be on, should recieve commands from Odroid
      OAoff = false;
      gripOff = true;
      // Reset all sensor flags
      bumpFlag = false;
      irFlag = false;
      currentFlag = false;
    }
    else if(activeBehavior == 2){
      // Align and Pickup behavior
      // Obstacle avoidance should be off, should recieve commands fro Odroid
      OAoff = true;
      OAOverride = false;
      gripOff  = false;
    }
    else if(activeBehavior == 3){
      // Deposit behavior
      // Obstacle avoidance should be off, robot should recieve commands from Odroid
      // Gripper positioning will interfere with OA.
      OAoff = true;
      OAOverride = false;
      gripOff = false;
    }
    else if(activeBehavior == 4){
      // Localize bahvior
      // Robot should stop moving, Obstacle avoidance should be turned off
      // Gripper should not be moving also
      OAoff = true;
      OAOverride  = false;
      gripOff = true;
    }
    newBehavior = false;
  }
}

//----------------------------------------------------------------------------------------------------------------------------//
//----------------------------------------------------------------------------------------------------------------------------//

void obstacleAvoidance(){
  if(!OAoff){          
    // Any sensor flag will trigger alternative behavior
    if (currentFlag || bumpFlag || irFlag) {
      OAOverride = true;
      if (currentRecomnd == 1) {
        // Stop motion, robot could be stuck.
        Reverse = false;
        Turn = false;
        currentFlag = false;
        OADone = false;
        //Serial.println("Current");
      }
      else if(bumpRecomnd == 3){
        // Reverse and right turn motion
        Reverse = true;
        Turn = true;
        turnDirection = 3;
        bumpRecomnd = 0;
        bumpFlag = false;
        OADone = false;
        //Serial.println("bump Left");
      } 
      else if(bumpRecomnd == 4){
        // Reverse and left turn motion
        Reverse = true;
        Turn = true;
        turnDirection = 4;
        bumpRecomnd = 0;
        bumpFlag = false;
        OADone = false;
        Serial.println("bump right");
      } 
      else if(irRecomnd == 1 && OADone){
        // Turn Right
        Reverse = false;
        Turn = true;
        turnDirection = 3;  
        //Serial.println("IR Left");        
      }
      else if(irRecomnd == 2 && OADone){
        // Turn left
        Reverse = false;
        Turn = true;
        turnDirection = 4;
        Serial.println("IR Right");
      }
    }
  }
}

