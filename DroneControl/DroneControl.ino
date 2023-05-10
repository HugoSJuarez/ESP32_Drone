#include <PPM_Communication.h>

float inputThrottle;
byte motorPin1=25;
byte motorPin2=26;
byte blueLED = 19;
byte redLED = 23;

byte rcConnected = 0;
byte onceUp = 0;
byte onceDown = 0;

PPM myPPM = PPM();

void setup() {
  delay(1000);
  //Setting LEDS RGB
  pinMode(blueLED,OUTPUT);
  pinMode(redLED,OUTPUT);
  

  //Configuration motors and PPM
  Serial.begin(115200);
  myPPM.setup(9,3000);
  ledcSetup(0, 250, 12);
  ledcAttachPin(motorPin1, 0);
  ledcAttachPin(motorPin2, 0);

}

void loop() {

    while(!myPPM.oneRead){
      Serial.println("Waiting for RC ...");
      digitalWrite(redLED, LOW);
      digitalWrite(blueLED, LOW);
      delay(500);
    }

    rcConnected = 1;
    //myPPM.ch[5]<1100 || myPPM.ch[1]>1100
    while(rcConnected){
      if(!onceUp && !onceDown){
        if(myPPM.ch[5]<1100){
          if(myPPM.ch[1]>1900){
            onceUp=1;
          }
          if(onceUp && myPPM.ch[1]<1100){
            onceDown=1;
          }
        }
        else{
          onceUp = 0;
          onceDown = 0;
        }
      }
      else if(myPPM.ch[5]<1100 && myPPM.ch[1]<1100  ){
        Serial.println("Connected to RC !!!");
        digitalWrite(redLED, LOW);
        digitalWrite(blueLED, HIGH);
        break;
      }
    }
    
    

    while(myPPM.ch[5]>1700){
      digitalWrite(redLED, HIGH);
      digitalWrite(blueLED, LOW);
      Serial.print("LastTime: ");
      Serial.print(myPPM.lastTime);
      for(byte i=0; i<9; i++){
      Serial.print(myPPM.chName[i]);
      Serial.print(": ");
      Serial.print(myPPM.ch[i]);
      Serial.print(" us, ");
      }
      inputThrottle=1.024*myPPM.ch[1];
      Serial.print("Input: ");
      Serial.println(inputThrottle);
      ledcWrite(0,inputThrottle);
      delay(50);
      if(micros()-myPPM.lastTime>50000){
        myPPM.oneRead=0;
        break;
      }

    }
    ledcWrite(0,1000*1.024);
  
}
