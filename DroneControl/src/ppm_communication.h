#ifndef PPM_COMMUNICATION_H
#define PPM_COMMUNICATION_H

#include <Arduino.h>

static volatile unsigned long timeInterval[10];
static volatile char pulse = 0;
static int ppmTimeout;

static volatile unsigned long ch[9];
static byte oneRead=0;
static String chNames[9]={String("Ch0"), String("Throttle"), String("Aileron"), String("Elevator"), String("Rudder"), String("Arm"), String("Not Used"), String("Not Used"), String("Not Used")};


void readRCRx(void);
void ppmSetup(byte pin, int timeout);
int addTwoInts(int a, int b);

#endif