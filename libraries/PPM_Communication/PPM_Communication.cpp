#include "ppm_communication.h"

//Setup for PPM Reading
void PPM::setup(byte pin, int timeout){
  ppmTimeout=timeout;
  attachInterrupt(pin, readRCRxAux, RISING);
}

//Auxiliar function to call for readRCRx since it is static
void PPM::readRCRxAux(){
  readRCRx(timeInterval, pulse, ppmTimeout, ch, oneRead, chName);
}
//Function to readRCRx using ESP32 RAM instead of flash memory for. faster response  
void IRAM_ATTR readRCRx(volatile unsigned long* timeInterval, 
                        volatile char &pulse,
                        int &ppmTimeout,
                        volatile unsigned long* ch,
                        byte &oneRead,
                        String* chName){
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


//Reference for classes variables
String* PPM::chName = new String [9]{String("Ch0"), String("Throttle"), String("Aileron"), String("Elevator"), String("Rudder"), String("Arm"), String("Not Used"), String("Not Used"), String("Not Used")};
volatile unsigned long* PPM::ch= new volatile unsigned long [9];
volatile unsigned long* PPM::timeInterval= new volatile unsigned long [10];
int PPM::ppmTimeout=3000;
volatile char PPM::pulse=0;
byte PPM::oneRead=0;