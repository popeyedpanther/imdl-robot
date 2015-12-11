void pixyTracking(){
  // Local variables
  static int p = 0, index = 0, count = 0;
  int i, j, k;
  uint16_t blocks;
  char buf[32]; 
  
  // grab blocks!
  blocks = ffPixy.getBlocks();
  
  // Reset object and place in cCodesDone array
  if(objectPlaced){
    foundObject = false;
    cCodesDone[count] = activeSignature;
    count++;
  }  
  
  // Search for color code objects
  for(i=0; i<cCodesSize; i++){
   for(j=0; j<blocks; j++){
     // If the object matches a color code signature
     if(ffPixy.blocks[j].signature == cCodes[i]){
       for(k=0; k<cCodesSize; k++){
         // Only say an object was found if it also has not already been found
         if(ffPixy.blocks[j].signature != cCodesDone[k]){
           foundObject = true;
           activeSignature = ffPixy.blocks[j].signature;
           index = j;
           break;
         }
       }
     }
     if(foundObject){break;}
   }
   if(foundObject){break;} 
  }
  
  if(foundObject && activeBehavior == 2){
    // Need to update the Odroid to change behaviors.
    // Need to get object to the center of screen.
    Serial.println(ffPixy.blocks[index].signature);
    
    
    
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
