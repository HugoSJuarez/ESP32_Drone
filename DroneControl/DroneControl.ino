#include "src/ppm_communication.h"

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  int result=addTwoInts(10,2);
  Serial.println(result);
}

void loop() {
  // put your main code here, to run repeatedly:

}
