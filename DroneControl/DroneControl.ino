#include <PPM_Communication.h>

float inputThrottle;
char motorPin1=25;
char motorPin2=26;

PPM myPPM = PPM();

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  myPPM.setup(9,3000);
  ledcSetup(0, 250, 12);
  ledcAttachPin(motorPin1, 0);
  ledcAttachPin(motorPin2, 0);
  Serial.println("Waiting for RC ...");
  while(!myPPM.oneRead){
    delay(500);
  }
  Serial.println("Connected to RC !!!");
}

void loop() {
  Serial.print(myPPM.chName[1]);
  Serial.print(": ");
  Serial.print(myPPM.ch[1]);
  Serial.print(" us, ");
  inputThrottle=1.024*myPPM.ch[1];
  Serial.print("Input: ");
  Serial.println(inputThrottle);
  ledcWrite(0,inputThrottle);
  delay(50);

}
