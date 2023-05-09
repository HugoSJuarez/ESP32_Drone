#ifndef PPM_COMMUNICATION_H
#define PPM_COMMUNICATION_H

#include <Arduino.h>

class PPM{
    private:
        static volatile unsigned long* timeInterval;
        static volatile char pulse;
        static int ppmTimeout; 
    
    public:
        static void setup(byte pin, int timeout);
        static volatile unsigned long* ch;
        static byte oneRead;
        static String* chName;
        static void readRCRxAux(void);
        
};

void IRAM_ATTR readRCRx(volatile unsigned long* timeInterval, 
                        volatile char &pulse,
                        int &ppmTimeout,
                        volatile unsigned long* ch,
                        byte &oneRead,
                        String* chName);

#endif