void updateState(){

      leftWheelChange = (float)leftEncoder.read()*C - leftStartPoint;
      rightWheelChange = (float)rightEncoder.read()*C - rightStartPoint;

      if( controllerLeftDistance < 0 && controllerRightDistance > 0){
        //Forward motion
        dx = dx + (abs(leftWheelChange) + abs(rightWheelChange))/2 * cos(dtheta);
        dy = dy + (abs(leftWheelChange) + abs(rightWheelChange))/2 * sin(dtheta);
      }
      else if(controllerLeftDistance > 0 && controllerRightDistance < 0){
        // Reverse Motion
        motionDirection = 2;
        dx = dx - (abs(leftWheelChange) + abs(rightWheelChange))/2 * cos(dtheta);
        dy = dy - (abs(leftWheelChange) + abs(rightWheelChange))/2 * sin(dtheta);
      }
      else if(controllerLeftDistance > 0 && controllerRightDistance > 0){
        // Turning Left
        dtheta = dtheta + (abs(rightWheelChange) + abs(leftWheelChange))/b;
      }
      else if(controllerLeftDistance < 0 && controllerRightDistance < 0){
        // Turning Right
        dtheta = dtheta - (abs(leftWheelChange) + abs(rightWheelChange))/b;
      }

  
}

