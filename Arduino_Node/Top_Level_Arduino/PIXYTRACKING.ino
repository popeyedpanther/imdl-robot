void pixyTracking(){
  // Local variables
  static int p = 0, index = 0, count = 0;
  int i, j, k;
  uint16_t blocks;
  char buf[32]; 
  
  // grab blocks!
  blocks = ffPixy.getBlocks();
  
  // Reset object and place in cCodesDone array
  if(objectPlaced == 1){
    foundObject = 0;
    cCodesDone[count] = activeSignature;
    count++;
    objectPlaced = 0;
  }  
  
  // Search for color code objects
  for(i=0; i<cCodesSize; i++){
   for(j=0; j<blocks; j++){
     // If the object matches a color code signature
     if(ffPixy.blocks[j].signature == cCodes[i]){
       for(k=0; k<cCodesSize; k++){
         // Only say an object was found if it also has not already been found
         if(ffPixy.blocks[j].signature != cCodesDone[k]){
           foundObject = 1;
           activeSignature = ffPixy.blocks[j].signature;
           index = j;
           break;
         }
       }
     }
     if(foundObject == 1){break;}
   }
   if(foundObject == 1){break;} 
  }
  
  if(foundObject == 1 && activeBehavior == 2){
    // Find the signature and update x and y position
    for(j=0; j<blocks; j++){
      // If the object matches a color code signature
      if(ffPixy.blocks[j].signature == activeSignature){
        xPosition = ffPixy.blocks[j].x;
        yPosition = ffPixy.blocks[j].y;
        break;  
      }
    }
  }
  
  if(pixyDebug){
    // If there are detect blocks, print them!
    if (blocks)
    {
      p++;
      
      // do this (print) every 50 frames because printing every
      // frame would bog down the Arduino
      if (p%50==0)
      {
        
        Serial.println(ffPixy.blocks[0].signature);
        
        /*
        sprintf(buf, "Detected %d:\n", blocks);
        Serial.print(buf);
        for (j=0; j<blocks; j++)
        {
          sprintf(buf, "  block %d: ", j);
          Serial.print(buf); 
          ffPixy.blocks[j].print();
        }
        */
      }
    }     
  } 
  
  
}
