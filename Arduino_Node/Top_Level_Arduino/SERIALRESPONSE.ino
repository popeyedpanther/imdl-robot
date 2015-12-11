void serialResponse(){
  // Needs to send a response to the Odroid request
  // Should include current pan tilt angles, if an object has been found, pixel x, pixel y of object
  if(newRequest){
    Serial.print(Pan.read());
    Serial.print(':');
    Serial.print(Tilt.read());
    Serial.print(':');
    Serial.print(foundObject);
    Serial.print(':');
    Serial.print(xPosition);
    Serial.print(':');
    Serial.println(yPosition);
    newRequest = false;
  }
}
