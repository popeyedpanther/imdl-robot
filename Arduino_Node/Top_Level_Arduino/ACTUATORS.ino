void updatePanTilt(){
  if (newPanTiltCmd){
    Serial.println("Repeat Back " + String(activeBehavior)+ " " + String(panCmd) + " " + String(tiltCmd));  
    // Make sure to check inout bounds
    if((panCmd >= 5 && panCmd <= 172) && panCmd != 999){
      Pan.write(panCmd);
      //Serial.println("Repeat Back " + String(panCmd));  
    }
    if((tiltCmd >= 80 && tiltCmd <= 130) && tiltCmd != 999){
      Tilt.write(tiltCmd);
      //Serial.println("Repeat Back " + String(State)+ " " + String(panCmd) +" " String(tiltCmd));  
    } 
    newPanTiltCmd = false;
  }     
}
