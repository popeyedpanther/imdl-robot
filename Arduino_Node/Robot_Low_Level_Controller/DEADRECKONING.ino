void deadReckoning(){

  if(abs(leftInput - leftStartPoint) > (abs(controllerLeftDistance)-leftOffset) \
                  && abs(rightInput - rightStartPoint) > (abs(controllerRightDistance)-rightOffset) && !isStopped){
    
    motionDirection = 0;

  }
  
  if((motionDirection != oldMotionDirection || motionDirection == 0) && !isStopped){

      if(!isStopped){
        leftSetpoint = 0;
        rightSetpoint = 0;

        if(abs(currentMillis - previousMillis_Stopped) > stoppedPeriod){
          previousMillis_Stopped = currentMillis;
          // Have the wheels stopped moving?
          if(abs(leftStopPoint - leftEncoder.read()*C) < 0.00001 && abs(rightStopPoint - rightEncoder.read()*C) < 0.00001){
            isStopped = true;
            // Now doing a different motion
            oldMotionDirection = motionDirection;
          }
          leftStopPoint = leftInput;
          rightStopPoint = rightInput;
        }
        if(isStopped){
          updateState();
        }
     }
   }
   else if(isStopped){
     oldMotionDirection = motionDirection;
   }
}

