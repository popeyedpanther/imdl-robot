void serialResponse(){
  // Needs to send a response to the Odroid request
  if(newRequest){
    dtostrf(dx, 7, 2, buffer);      // Change in X position
    Serial.print(buffer);
    Serial.print(":");
    dtostrf(dy, 7, 2, buffer);      // Change in Y position
    Serial.print(buffer);
    Serial.print(":");
    dtostrf(dtheta, 7, 2, buffer);  // Change in orientation
    Serial.print(buffer);
    Serial.print(":");
    Serial.print(motionComplete);   // Was commaned motion complete?
    Serial.print(":");
    if (OAOverride){
      OAOverrideSerial = 1;
    }
    else if(!OAOverride){
      OAOverrideSerial = 0;
    }
    Serial.print(OAOverrideSerial);       // Is obstacle avoidance controlling?
    Serial.println(":");
    newRequest = false;
  }
}

/*
if(newRequest && requestState == 1){
  dtostrf(dx, 7, 2, buffer);
  Serial.print(buffer);
  Serial.print(":");
  dtostrf(dy, 7, 2, buffer);
  Serial.print(buffer);
  Serial.print(":");
  dtostrf(dtheta, 7, 2, buffer);
  Serial.print(buffer);
  Serial.println(":");
  newRequest = false;
  
}
*/
