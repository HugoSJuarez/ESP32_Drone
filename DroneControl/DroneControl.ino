#include <PPM_Communication.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

//Branch to try the PID controlls

//Number of rotors 4 or 6
byte rotorNumber=6;

//LEDs Pin
byte blueLED = 19;
byte redLED = 23;

//LED Prototype functions
void noColor(void);
void redColor(void);
void blueColor(void);
void magentaColor(void);

//Drone initialization variables
byte droneInit = 0;
byte onceUp = 0;
byte onceDown = 0;
byte armed = 0;
byte connectionLost = 0;

//                Rate Controller
//PID outputs
float rollRate;
float pitchRate;
float yawRate;

//PID for Roll
// float kpRollRate = 0.6;
// float kiRollRate = 0;
// float kdRollRate = 0;

// float kpRollRate = 3;
// float kiRollRate = 0.048;
// float kdRollRate = 2;

float kpRollRate = 1.35*0.6;
float kiRollRate = 0.0325/0.004;
float kdRollRate = 0.004/9;

// float kpRollRate = 1.3*0.6; ESTE
// float kiRollRate = 0.0275/0.004; ESTE
// float kdRollRate = 0.004/9; ESTE
//PID for Pitch
// float kpPitchRate = 0.6;
// float kiPitchRate = 3.5;
// float kdPitchRate = 0.03;
float kpPitchRate = 1.35*0.6;
float kiPitchRate = 0.0325/0.004;
float kdPitchRate = 0.004/9;

//PID for Yaw
float kpYawRate = 2;
float kiYawRate = 12;
float kdYawRate = 0;
// float kpYawRate = 0.6;
// float kiYawRate = 0.02;
// float kdYawRate = 0.2;

//Errors
float errRollRate;
float errPitchRate;
float errYawRate;
float prevErrRollRate = 0;
float prevErrPitchRate = 0;
float prevErrYawRate = 0;

//Previous I values
float prevIRollRate = 0;
float prevIPitchRate = 0;
float prevIYawRate = 0;

//            Angle Controller
//PID outputs
float rollAngle;
float pitchAngle;
//Yaw wont be taken in consideration cause once yaw is put, i dont want the drone to return to 0

//PID for Roll
float kpRollAngle = 2;
float kiRollAngle = 0;
float kdRollAngle = 0;
// float kpRollAngle = 0.8;
// float kiRollAngle = 0.01;
// float kdRollAngle = 0.2;


//PID for Pitch
float kpPitchAngle = 2;
float kiPitchAngle = 0;
float kdPitchAngle = 0;
// float kpPitchAngle = 0.8;
// float kiPitchAngle = 0.01;
// float kdPitchAngle = 0.2;

//Errors
float errRollAngle;
float errPitchAngle;
float prevErrRollAngle = 0;
float prevErrPitchAngle = 0;

//Previous I values
float prevIRollAngle = 0;
float prevIPitchAngle = 0;


//------------------------
//Time between new PID
float t = 0.004;

//Input of RC
float desireRollRate;
float desirePitchRate;
float desireRollAngle;
float desirePitchAngle;
float desireYawRate;
float desireThrottle;

float pastRollAngle;

//Input from BNO055 sensor
//Angle
float actualRollAngle;
float actualPitchAngle;

//Rate
float actualRollRate;
float actualPitchRate;
float actualYawRate;

// float desireThrottle; Need to be implemented with barometer

//Motors input
float inputMotor[6];

//Motor pins and channel
byte motorPin[6] = { 25, 26, 27, 18, 5, 10};
byte motorCh[6] = { 0, 1, 2, 3, 4, 5 };

//Initiliazation PPM reader
PPM myPPM = PPM();

//Time in Micro for loop to restart
float loopTime;

//Prototypes functions PID
void PIDControll(float &orientation, float &err, float &prevErr, float &prevI, float &Kp, float &Ki, float &Kd, float t);
void resetPID(void);

//State functions prototypes
void waitRcConnect(void);
void droneInitProtocol(void);
void droneFlight(void);
void droneMotorsOff(void);
void droneMotorsIdle(void);
void calibrateDrone(void);
byte isDroneArmed(void);

//BNO055 object
//Default SCL Pin in ESP32 GPIO 22
//Default SDA Pin in ESP32 GPIO 21
Adafruit_BNO055 myIMU = Adafruit_BNO055();

void setup() {

  //Setting RGB Leds
  pinMode(blueLED, OUTPUT);
  pinMode(redLED, OUTPUT);

  //BNO055 initalization
  Serial.begin(115200);
  if( !myIMU.begin() ){
    Serial.print("BNO055 not Found, check connection!");
    while(1);
  }
  delay(1000);
  int8_t temp = myIMU.getTemp();
  init_Calib_Values();
  myIMU.setExtCrystalUse(true);

  //PPM setup
  myPPM.setup(9,3000);
  
  //Setting up motors
  for(byte i=0; i<rotorNumber; i++){
    // Setting with a frquency of 250Hz and a resolution of 12bytes;
    ledcSetup(motorCh[i], 250, 12);
    ledcAttachPin(motorPin[i], motorCh[i]);
  }

  for(byte i=0; i<rotorNumber; i++) ledcWrite(motorCh[i], 990);

  // calibrateDrone();
  
  waitRcConnect();

  droneInitProtocol();

  

  //Start loop timer
  loopTime = micros();
}

void loop() {
  
  if ( isDroneArmed() && !isConnectionLost()){
    blueColor();
    droneFlight();
  }
  else if( !isDroneArmed() ){
    redColor();
    droneMotorsOff();
  }
  else if(isConnectionLost()){
    droneMotorsIdle();
    redColor();
    delay(1000);
    noColor();
    connectionLost = 0;
    waitRcConnect();
  }
  //Wait timer happens
  while(micros()-loopTime < t*1000000);
  loopTime=micros();
}

void noColor(void){
  digitalWrite(redLED, HIGH);
  digitalWrite(blueLED, HIGH);
}

void redColor(void){
  digitalWrite(redLED, LOW);
  digitalWrite(blueLED, HIGH);
}
void blueColor(void){
  digitalWrite(redLED, HIGH);
  digitalWrite(blueLED, LOW);
}
void magentaColor(void){
  digitalWrite(redLED, LOW);
  digitalWrite(blueLED, LOW);
}

//PID Controll:
//Input = Kp * err + prevI + Ki*(err + prevErr)*t/2 + Kd * (err-prevErr)/t 
void PIDControll(float &orientation, float &err, float &prevErr, float &prevI, float &Kp, float &Ki, float &Kd, float t){
  float P = Kp*err;
  
  float D = Kd*(err-prevErr)/t;
  //To avoid integral windup
  float I;
  if(err<10 && err >-10){
    I = prevI + Ki*(err+prevErr)*t/2;
    if(I>400) I=400;
    else if(I<-400) I=-400;
    prevI = I;
  }
  else{
    I=0;
  }
  float PID = P+I+D;
  if (PID>400) PID = 400;
  else if (PID<-400) PID = -400;
  orientation = PID;
  prevErr = err;
}

void resetPID(void){
  prevErrRollRate=0;
  prevErrPitchRate=0;
  prevErrYawRate=0;
  prevIRollRate=0;
  prevIPitchRate=0;
  prevIYawRate=0;

  prevErrRollAngle=0;
  prevErrPitchAngle=0;
  prevIRollAngle=0;
  prevIPitchAngle=0;
}

void droneFlight(void){

  //Get gyro info
  imu::Vector<3> gyr = myIMU.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  actualRollRate = gyr.x();
  actualPitchRate = gyr.y();
  actualYawRate = -gyr.z();

  //Get angle info
  sensors_event_t event;
  myIMU.getEvent(&event);
  actualRollAngle=-event.orientation.z;
  actualPitchAngle=-event.orientation.y;

  Serial.print("Actual Roll: Deg: ");
  Serial.print(actualRollAngle);
  Serial.print("errRoll: Deg: ");
  Serial.print(errRollAngle);
  Serial.print("prevErrRollAngle: ");
  Serial.print(prevErrRollAngle);
  Serial.print("prevIRollAngle: ");
  Serial.print(prevIRollAngle);
  
  
  // Serial.print(", Pitch: Deg: ");
  // Serial.print(actualPitchAngle);
  // Serial.print("°/s: ");
  // Serial.print(actualPitchRate);
  // Serial.print("Yaw: °/s: ");
  // Serial.print(actualYawRate);

  desireThrottle = (float)myPPM.ch[1];
  desireYawRate = 0.15*((float)myPPM.ch[4]-1500);

  //              Getting desire angles
  desireRollAngle = 0.1*((float)myPPM.ch[2]-1500);
  desirePitchAngle = 0.1*((float)myPPM.ch[3]-1500);

  Serial.print("Desire Roll Channel: ");
  Serial.print(myPPM.ch[2]);
  Serial.print("us ,Desire Roll: ");
  Serial.print(desireRollAngle);
  // errRollAngle = desireRollAngle - actualRollAngle;
  errRollAngle = desireRollAngle-actualRollAngle;
  errPitchAngle = desirePitchAngle - actualPitchAngle;

  PIDControll( rollAngle, errRollAngle, prevErrRollAngle, prevIRollAngle, kpRollAngle, kiRollAngle, kdRollAngle, t);
  PIDControll( pitchAngle, errPitchAngle, prevErrPitchAngle, prevIPitchAngle, kpPitchAngle, kiPitchAngle, kdPitchAngle, t);

  desireRollRate = rollAngle;
  desirePitchRate = pitchAngle;

  // desireRollRate = 0.1*myPPM.ch[2]-150;
  // desirePitchRate= 0.1*myPPM.ch[3]-150;



  //Check inputs
  // Serial.print("Throttle: ");
  // Serial.print(desireThrottle);
  // Serial.print(" , Roll: ");
  // Serial.print(desireRollRate);
  // Serial.print(" , Pitch: ");
  // Serial.print(desirePitchRate);
  // Serial.print(" , Yaw: ");
  // Serial.println(desireYawRate);

  //Get error
  errRollRate = desireRollRate - actualRollRate;
  errPitchRate = desirePitchRate - actualPitchRate;
  errYawRate = desireYawRate - actualYawRate;
  Serial.print("Desire Roll Rate: ");
  Serial.print(desireRollRate);
  Serial.print("Actual Roll °/s: ");
  Serial.print(actualRollRate);
  Serial.print("errRoll: °/s");
  Serial.print(errRollRate);
  Serial.print("prevErrRollRate: ");
  Serial.print(prevErrRollRate);
  Serial.print("prevIRollRate: ");
  Serial.print(prevIRollRate);
  
  //Calculate PID
  PIDControll( rollRate, errRollRate, prevErrRollRate, prevIRollRate, kpRollRate, kiRollRate, kdRollRate, t);
  PIDControll( pitchRate, errPitchRate, prevErrPitchRate, prevIPitchRate, kpPitchRate, kiPitchRate, kdPitchRate, t);
  PIDControll( yawRate, errYawRate, prevErrYawRate, prevIYawRate, kpYawRate, kiYawRate, kdYawRate, t);
  
  
  // Serial.print(", errPitch: Deg: ");
  // Serial.print(errPitchAngle);
  // Serial.print("°/s: ");
  // Serial.print(errPitchRate);
  // Serial.print("errYaw: °/s: ");
  // Serial.print(errYawRate);

  Serial.print(", rollRate: ");
  Serial.print(rollRate);
  // Serial.print(", pitchRate: ");
  // Serial.print(pitchRate);
  // Serial.print(", yawRate: ");
  // Serial.print(yawRate);

  //Check input throttle and apply a limitation
  if ( desireThrottle > 1500 ) desireThrottle = 1500;
  
  // //Hexacopter X
  inputMotor[0] = 1.024*(desireThrottle - rollRate - pitchRate - yawRate);
  inputMotor[1] = 1.024*(desireThrottle - rollRate + yawRate);
  
  inputMotor[2] = 1.024*(desireThrottle - rollRate + pitchRate - yawRate + 150);
  
  inputMotor[3] = 1.024*(desireThrottle + rollRate + pitchRate + yawRate);
  inputMotor[4] = 1.024*(desireThrottle + rollRate - yawRate);
  inputMotor[5] = 1.024*(desireThrottle + rollRate - pitchRate + yawRate);

  //Quadcopter  
  // inputMotor[0] = 1.024*(desireThrottle - rollRate - pitchRate + yawRate);
  // inputMotor[1] = 1.024*(desireThrottle - rollRate + pitchRate - yawRate);
  // inputMotor[2] = 1.024*(desireThrottle + rollRate + pitchRate + yawRate);
  // inputMotor[3] = 1.024*(desireThrottle + rollRate - pitchRate - yawRate);

  


  
//Change to number of rotors
  for (byte i=0; i<rotorNumber; i++){
    if(inputMotor[i]>2000) inputMotor[i]=2000;
    //Adding idle
    if(inputMotor[i]<1050) inputMotor[i]=1050;
  }

  //Adding noThrottle
  if(desireThrottle<1050){
    for (byte i=0; i<6; i++) inputMotor[i]=1000;
    resetPID();
  }
  
  Serial.print("Motores: ");
  Serial.print(", Motor 2: ");
  Serial.print(inputMotor[1]);
  Serial.print(", Motor 5: ");
  Serial.println(inputMotor[4]);
  // Serial.print(", Motor 3: ");
  // Serial.print(inputMotor[2]);
  // Serial.print(", Motor 4: ");
  // Serial.println(inputMotor[3]);

  // inputMotor[0] = 1000;
  // inputMotor[2] = 1000;
  // inputMotor[3] = 1000;
  // inputMotor[5] = 1000;
  
  //Send PWM signal to motors ESC
  for(byte i=0; i<rotorNumber; i++) ledcWrite(motorCh[i], inputMotor[i]);
  // for(byte i=0; i<rotorNumber; i++) ledcWrite(motorCh[i], desireThrottle);
};

void waitRcConnect(void) {
  while(!myPPM.oneRead){
    Serial.println("Waiting for RC ...");
    digitalWrite(redLED, LOW);
    digitalWrite(blueLED, LOW);
    delay(500);
  }
}

void droneInitProtocol(void) {
  while(!droneInit){
    if(myPPM.ch[5]<1100){
      if(!onceUp && myPPM.ch[1]>1900) onceUp = 1;
      else if (onceUp && myPPM.ch[1]<1100) onceDown = 1;
    }
    else {
      onceUp = 0;
      onceDown = 0;
    }
    if (onceUp && onceDown){ 
      droneInit = 1;
      redColor();
    }
  }
}

void droneMotorsOff(void){
  //Send PWM signal to motors ESC
  for(byte i=0; i<rotorNumber; i++) ledcWrite(motorCh[i], 1000);
}

void droneMotorsIdle(void){
  //Send PWM signal to motors ESC
  for(byte i=0; i<rotorNumber; i++) ledcWrite(motorCh[i], 1050);
}

void calibrateDrone(void){
  //Calibration variables
  uint8_t system, gyro, accel, mg = 0;
  float lastSecond = millis();
  //for(byte i=0; i<rotorNumber; i++) ledcWrite(motorCh[i], 1.024*980);
  myIMU.getCalibration(&system, &gyro, &accel, &mg);
  magentaColor();
  byte isMagenta = 1;
  while (system!=3 || accel!=3){
    //Send command to calibrate
    myIMU.getCalibration(&system, &gyro, &accel, &mg);
    imu::Vector<3> accele = myIMU.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    Serial.print(accele.x());
    Serial.print(accele.y());
    Serial.print(accele.z());
    Serial.print(", Calibration: ");
    Serial.print(accel);
    Serial.print(", ");
    Serial.print(gyro);
    Serial.print(", ");
    Serial.print(mg);
    Serial.print(", ");
    Serial.println(system);
    if(millis()-lastSecond>=500){
      if(isMagenta==1){
        noColor();
        isMagenta=0;
        }
      else{
        magentaColor();
        isMagenta=1;
      }
      lastSecond = millis();
    }
  }
  magentaColor();

}

void init_Calib_Values(void){
  adafruit_bno055_offsets_t calibData;
  calibData.accel_offset_x = -15729;
  calibData.accel_offset_y = 10485; 
  calibData.accel_offset_z = -28836; 

  calibData.gyro_offset_x = 26214; 
  calibData.gyro_offset_y = -16357; 
  calibData.gyro_offset_z = -28836; 

  calibData.mag_offset_x = 16370; 
  calibData.mag_offset_y = 26214; 
  calibData.mag_offset_z = 26214; 

  calibData.accel_radius = -2622;

  calibData.mag_radius = 23592;

  myIMU.setSensorOffsets(calibData);
}


void waitPitchCorrect(void){
  float lastTimePitch = millis();
  byte magenta = 1;
  int timesCorrect = 0;
  while(timesCorrect<50){
    imu::Vector<3> gyr = myIMU.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    Serial.print(gyr.y());
    Serial.print(", ");
    Serial.println(timesCorrect);
    if(gyr.y()<1 && gyr.y()>-1)timesCorrect++;
    if(millis()-lastTimePitch>=500){
      if(magenta){
        noColor();
        magenta=0;
      }
      else{
        magentaColor();
        magenta = 1;
      }
      lastTimePitch=millis();
    }
    delay(100);
  };
}

byte isDroneArmed(void){
  if (!armed && myPPM.ch[5]>1500 && myPPM.ch[1]<1100) armed = 1;
  else if(armed && myPPM.ch[5]<1500) armed = 0;
  return armed; 
}

byte isConnectionLost(void){
  float time_Value = millis()-myPPM.lastTime;
  if(time_Value>500){
    myPPM.oneRead=0;
    Serial.println(time_Value);
    connectionLost=1;
  }
  return connectionLost;
}
