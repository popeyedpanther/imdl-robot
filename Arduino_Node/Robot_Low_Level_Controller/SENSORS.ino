
void updateIR(){
// Read in the left IR voltage and put into a buffer
  irValue = analogRead(leftIRPin);
  irLeft[0] = irLeft[1];
  irLeft[1] = irLeft[2];
  irLeft[2] = irLeft[3];
  irLeft[3] = irValue;
  delay(1);
  
  // Read in the right IR voltage and put into a buffer
  irValue = analogRead(rightIRPin);
  irRight[0] = irRight[1];
  irRight[1] = irRight[2];
  irRight[2] = irRight[3];
  irRight[3] = irValue;
  
  // Calculate the average input
  irLeftAvg = (float(irLeft[0]) + float(irLeft[1]) + float(irLeft[2]) + float(irLeft[3]))/4;
  irRightAvg = (float(irRight[0]) + float(irRight[1]) + float(irRight[2]) + float(irRight[3]))/4;

  // Convert voltage reading to units of inches.
  irLeftAvg = 573.01 * pow(float(irLeftAvg),-0.909);
  irRightAvg = 1768.2* pow(float(irRightAvg), -1.114);

  // Obstacle Avoidance Logic
  if (irLeftAvg < 3.5 && irRightAvg >= 3.5){
    // reverse and turn right
    bumpRecomnd = 3;
    bumpFlag = true;
  }
  else if (irLeftAvg >= 3.5 && irRightAvg < 3.5){
    // reverse and turn left
    bumpRecomnd = 4;
    bumpFlag = true;
  }
  else if (irLeftAvg < 3.5 && irRightAvg < 3.5){
    // reverse and turn left or right
    if(random(0,9)/5 == 1){
      bumpRecomnd = 3;
    }
    else{
      bumpRecomnd = 4;
    }
    bumpFlag = true;
  }
  else if((irRightAvg-irLeftAvg)< eps && (irLeftAvg >= 3.5 && irLeftAvg < 5.5)\
        && (irRightAvg >= 3.5 && irRightAvg < 5.5)) {
    // Turn left or right
    if(random(0,9)/5 == 1){
      irRecomnd = 1;
    }
    else{
      irRecomnd = 2;
    }  
  }
  else if(irLeftAvg >= 3.5 && irLeftAvg < 5.5 && irRightAvg > 5.5){
    irRecomnd = 1;  // Turn right some random amount
  }
  else if(irRightAvg >= 3.5 && irRightAvg < 5.5 && irLeftAvg > 5.5){
    irRecomnd = 2;  // Turn left some random amount
  }
  else{
    irRecomnd = 0;
  }
   
  // If the IR recommends something then set the flag to true
  if(irRecomnd != 0){
    irFlag = true;
  }
  else{
    irFlag = false;
  }

  // Debugging outputs
  //  Serial.print(leftString + String(irLeftAvg) + " " );
  //  Serial.println(rightString + String(irRightAvg));  
}

//----------------------------------------------------------------------------------------------------------------------------//
//----------------------------------------------------------------------------------------------------------------------------//

void updateCurrent(){
  currentValue  = driveMotors.getM1CurrentMilliamps();
  currentLeft[0] = currentLeft[1];
  currentLeft[1] = currentLeft[2];
  currentLeft[2] = currentLeft[3];
  currentLeft[3] = currentValue;
  delay(1);
      
  currentValue = driveMotors.getM2CurrentMilliamps();
  currentRight[0] = currentRight[1];
  currentRight[1] = currentRight[2];
  currentRight[2] = currentRight[3];
  currentRight[3] = currentValue;
     
  // Calculate average current reading over three samples to try to not in spikes.
  currentLeftAvg = (float(currentLeft[0]) + float(currentLeft[1]) + float(currentLeft[2]) + float(currentLeft[3]))/4;
  currentRightAvg = (float(currentRight[0]) + float(currentRight[1]) + float(currentRight[2]) + float(currentRight[3]))/4;

  // Convert to real units (amps)
  currentLeftAvg = 0.034 * currentLeftAvg;
  currentRightAvg = 0.034 * currentRightAvg;

  // Tune this value
  if(currentLeftAvg >= 4 || currentRightAvg >= 4) {
    currentRecomnd = 1;  // Arbitraty number for now just to trigger the flag.
  }
  else{
    currentRecomnd = 0;
  }
   
  // If the current sensor recommends something then set the flag to true
  if(currentRecomnd != 0){
    currentFlag = true;
  }
  else{ 
    currentFlag = false;
  }
  
  // Debug variable declaration
  currentFlag = false;
  currentRecomnd = 0;

  /* Debugging Outputs
    Serial.print("Left Current: " + String(currentLeftAvg) + " " );
    Serial.println("Right Current: " + String(currentRightAvg)); 
  */
}


//----------------------------------------------------------------------------------------------------------------------------//
//----------------------------------------------------------------------------------------------------------------------------//

// ---Left Bump Sensor Interrupt Function---
void bumpLeft()
{
  bumpFlag = true;
  bumpRecomnd = 3;
  
}
// ---Right Bump Sensor Interrupt Function---
void bumpRight()
{
  bumpFlag = true;
  bumpRecomnd = 4;
}
