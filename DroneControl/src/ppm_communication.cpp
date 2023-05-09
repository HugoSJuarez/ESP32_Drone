#include "ppm_communication.h"

void ppmSetup(byte pin, int timeout){
  ppmTimeout=timeout;
  attachInterrupt(pin, readRCRx, RISING);
}

//Function to readRCRx using ESP32 RAM instead of flash memory for. faster response  
void IRAM_ATTR readRCRx(void){
  timeInterval[pulse] = micros();
  
  if(pulse==0){
    pulse++;
  }
  else{
      if(pulse==9){
        ch[0] = timeInterval[pulse]-timeInterval[pulse-1];
        timeInterval[0]=timeInterval[pulse];
        pulse=1;
        oneRead=1;
      }
      else{
        ch[pulse] = timeInterval[pulse]-timeInterval[pulse-1];
        if(ch[pulse]>ppmTimeout){
          timeInterval[0]=timeInterval[pulse];
          pulse = 1;
        }
        else{
          pulse++;
        }
    }
  }
}