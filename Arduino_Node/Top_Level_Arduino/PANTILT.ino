void updatePanTilt(){
  if (newPanTiltCmd){
    // Debug Print
    // Serial.println(String(activeBehavior) + ' ' + String(wristCmd) + ' ' + String(graspCmd));
    // Makes sure desired angles are acceptable
    if((panCmd >= 0 && panCmd <= 170) && panCmd != 999){
      Pan.write(panCmd);
    }
    if((tiltCmd >= 80 && tiltCmd <= 100) && tiltCmd != 999){
      Tilt.write(tiltCmd);
    } 
    newPanTiltCmd = false;
  }  
}
