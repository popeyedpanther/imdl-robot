void serialResponse(){
  // Needs to send a response to the Odroid request
  // Should include current pan tilt angles, if an object has been found, pixel x, pixel y of object
  if(newRequest){
    dtostrf(dx, 7, 2, buffer);
    Serial.print(buffer);
    Serial.print(":");
    dtostrf(dy, 7, 2, buffer);
    Serial.print(buffer);
    Serial.print(":");
    dtostrf(dtheta, 7, 2, buffer);
    Serial.print(buffer);
    Serial.print(":");
    Serial.print(motionComplete);
    Serial.print(":");
    Serial.print(OAOverride);
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
