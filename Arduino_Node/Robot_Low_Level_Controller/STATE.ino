void updateState(){

      leftWheelChange = (float)leftEncoder.read()*C - leftStartPoint;
      rightWheelChange = (float)rightEncoder.read()*C - rightStartPoint;

      if( leftDistance < 0 && rightDistance > 0){
        //Forward motion
        dx = dx + (abs(leftWheelChange) + abs(rightWheelChange))/2 * cos(dtheta);
        dy = dy + (abs(leftWheelChange) + abs(rightWheelChange))/2 * sin(dtheta);
      }
      else if(leftDistance > 0 && rightDistance < 0){
        // Reverse Motion
        motionDirection = 2;
        dx = dx - (abs(leftWheelChange) + abs(rightWheelChange))/2 * cos(dtheta);
        dy = dy - (abs(leftWheelChange) + abs(rightWheelChange))/2 * sin(dtheta);
      }
      else if(leftDistance > 0 && rightDistance > 0){
        // Turning Left
        dtheta = dtheta + (rightWheelChange + abs(leftWheelChange))/b;
      }
      else if(leftDistance < 0 && rightDistance < 0){
        // Turning Right
        dtheta = dtheta - (abs(leftWheelChange) + abs(rightWheelChange))/b;
      }

  
}

