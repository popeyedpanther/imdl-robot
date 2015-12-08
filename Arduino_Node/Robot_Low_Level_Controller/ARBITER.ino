void smartArbiter(){
  if(OAOverride){
    if(Reverse){
      if(motionDirection != 2){
        motionDirection = 2;
        OATimer = millis();
        Reverse = false;
      }
    }
    else if(Turn){
      if((millis()-OATimer) > random(1750,2500)){
        if(turnDirection == 3){
          motionDirection = 4;
        }
        else if(turnDirection == 4){
          motionDirection = 3;
        }
        OATimer = millis();
        Turn = false;
      }
    }
    else{
      if((millis()-OATimer) > random(1750,2500)){
        motionDirection = 0;
        OAOverride = false;
        OADone = true;
      }
    }
  }
  else{
    //---- Commanded Robot Move ----
    if(newDistance && !OAOverride){   
      if( leftDistance < 0 && rightDistance > 0){
        //Forward motion
        motionDirection = 1;
      }
      else if(leftDistance > 0 && rightDistance < 0){
        // Reverse Motion
        motionDirection = 2;
      }
      else if(leftDistance > 0 && rightDistance > 0){
        // Turning Left
        motionDirection = 3;
      }
      else if(leftDistance < 0 && rightDistance < 0){
        // Turning Right
        motionDirection = 4;
      }
      else{
        //Stop
        motionDirection = 0;
      }
    }
  } 
}

