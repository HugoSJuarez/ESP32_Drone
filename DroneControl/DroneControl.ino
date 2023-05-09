#include "src/ppm_communication.h"

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  ppmSetup(9, 3000);
}

void loop() {
  // put your main code here, to run repeatedly:

}
