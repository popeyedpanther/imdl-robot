void pixyTracking(){
  static int i = 0;
  int j;
  uint16_t blocks;
  char buf[32]; 
  
  // grab blocks!
  blocks = pixy.getBlocks();
  
  
  if(pixyDebug){
    // If there are detect blocks, print them!
    if (blocks)
    {
      i++;
      
      // do this (print) every 50 frames because printing every
      // frame would bog down the Arduino
      if (i%50==0)
      {
        
        Serial.println(pixy.blocks[0].signature);
        
        /*
        sprintf(buf, "Detected %d:\n", blocks);
        Serial.print(buf);
        for (j=0; j<blocks; j++)
        {
          sprintf(buf, "  block %d: ", j);
          Serial.print(buf); 
          pixy.blocks[j].print();
        }
        */
      }
    }     
  }
}

