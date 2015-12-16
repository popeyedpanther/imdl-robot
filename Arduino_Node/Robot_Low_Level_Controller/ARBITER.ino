void smartArbiter()
{
  if(OAOverride)
  {
    if(Reverse)
    {
        motionDirection = 2;
        D = random(10, 20);
        controllerLeftDistance = -D;
        controllerRightDistance = -D;
        OATimer = millis();
        Reverse = false; 
    }
    else if(Turn)
    {
      if((millis()-OATimer) > random(1750,2500))
      {
        if(turnDirection == 3)
        {
          motionDirection = 4;
          D = random(10, 20);
          controllerLeftDistance = -D;
          controllerRightDistance = D;
        }
        else if(turnDirection == 4)
        {
          motionDirection = 3;
          D = random(10, 20);
          controllerLeftDistance = D;
          controllerRightDistance = -D;
        }
        OATimer = millis();
        Turn = false;
        //Serial.println("Turn Happened");
      }
    }
    else
    {
      if((millis()-OATimer) > random(1750,2500))
      {
        motionDirection = 0;
        OAOverride = false;
        OADone = true;
      }
    }
  }
  else
  {
    //---- Commanded Robot Move ----
    if(newDistance && !OAOverride){   
      if( leftDistance > 0 && rightDistance > 0)
      {
        //Forward motion
        motionDirection = 1;
        controllerLeftDistance = leftDistance;
        controllerRightDistance = rightDistance;
      }
      else if(leftDistance < 0 && rightDistance < 0)
      {
        // Reverse Motion
        motionDirection = 2;
        controllerLeftDistance = leftDistance;
        controllerRightDistance = rightDistance;
      }
      else if(leftDistance < 0 && rightDistance > 0)
      {
        // Turning Left
        motionDirection = 3;
        controllerLeftDistance = leftDistance;
        controllerRightDistance = rightDistance;
      }
      else if(leftDistance > 0 && rightDistance < 0)
      {
        // Turning Right
        motionDirection = 4;
        controllerLeftDistance = leftDistance;
        controllerRightDistance = rightDistance;
      }
      else
      {
        //Stop
        motionDirection = 0;
      }
    }
  } 
}

