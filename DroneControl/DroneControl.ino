#include <PPM_Communication.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

//Branch to try the PID controlls

//PID outputs
float rollRate;
float pitchRate;
float yawRate;

//PID for Roll
float kpRollRate = 0.6;
float kiRollRate = 3.5;
float kdRollRate = 0.03;

//PID for Pitch
float kpPitchRate = 0.6;
float kiPitchRate = 3.5;
float kdPitchRate = 0.03;


//PID for Yaw
float kpYawRate = 2;
float kiYawRate = 12;
float kdYawRate = 0;

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

//Time between new PID
float t = 0.004;

//Input of RC
float desireRollRate;
float desirePitchRate;
float desireYawRate;
float desireThrottle;

//Input from BNO055 sensor
float actualRollRate;
float actualPitchRate;
float actualYawRate;
// float desireThrottle; Need to be implemented with barometer

//Motors input
float inputMotor[6];

//Motor pins and channel
byte motorPin[6] = { 25, 26, 0, 0, 0, 0};
byte motorCh[6] = { 0, 1, 2, 3, 4, 5 };

//Initiliazation PPM reader
PPM myPPM = PPM();

//Time in Micro for loop to restart
float loopTime;

//Prototypes functions PID
void PIDControll(float &orientation, float &err, float &prevErr, float &prevI, float &Kp, float &Ki, float &Kd, float t);
void resetPID(void);

//BNO055 object
Adafruit_BNO055 myIMU = Adafruit_BNO055();

//Default SCL Pin in ESP32 GPIO 22
//Default SDA Pin in ESP32 GPIO 21


void setup() {

  //Setting up PPM
  Serial.begin(115200);
  if( !myIMU.begin() ){
    Serial.print("BNO055 not Found, check connection!");
    while(1);
  }
  delay(1000);
  int8_t temp = myIMU.getTemp();
  //Dont use crystal of sensor use the one on the board
  myIMU.setExtCrystalUse(true);

  //PPM setup
  myPPM.setup(9,3000);
  
  //Setting up motors frecuency and resolution for 6 motors as well as write channel
  for(byte i=0; i<6; i++){
    ledcSetup(motorCh[i], 250, 12);
    ledcAttachPin(motorPin[i], motorCh[i]);
  }
  
  //Check RC Connections
  Serial.println("Waiting for RC ...");
  while(!myPPM.oneRead){
    delay(500);
  }
  Serial.println("Connected to RC !!!");

  //Start loop timer
  loopTime = micros();
}

void loop() {

  //Get gyro info
  imu::Vector<3> gyr = myIMU.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  actualRollRate = -gyr.x();
  actualPitchRate = gyr.y();
  actualYawRate = -gyr.z();
  
  desireThrottle = 1.024*myPPM.ch[1];
  desireRollRate = 1.024*(0.15*myPPM.ch[2]-225);
  desirePitchRate = 1.024*(0.15*myPPM.ch[3]-225);
  desireYawRate = 1.024*(0.15*myPPM.ch[4]-225);
  
  //Check inputs
  Serial.print("Throttle: ");
  Serial.print(desireThrottle);
  Serial.print(" , Roll: ");
  Serial.print(desireRollRate);
  Serial.print(" , Pitch: ");
  Serial.print(desirePitchRate);
  Serial.print(" , Yaw: ");
  Serial.println(desireYawRate);

  //Get error
  errRollRate = desireRollRate - actualRollRate;
  errPitchRate = desirePitchRate - actualPitchRate;
  errYawRate = desireYawRate - actualYawRate;
  
  //Calculate PID
  PIDControll( rollRate, errRollRate, prevErrRollRate, prevIRollRate, kpRollRate, kiRollRate, kdRollRate, t);
  PIDControll( pitchRate, errPitchRate, prevErrPitchRate, prevIPitchRate, kpPitchRate, kiPitchRate, kdPitchRate, t);
  PIDControll( yawRate, errYawRate, prevErrYawRate, prevIYawRate, kpYawRate, kiYawRate, kdYawRate, t);
  
  //Check input throttle and apply a limitation
  if ( desireThrottle > 1800 ) desireThrottle = 1800;
  
  inputMotor[0] = desireThrottle + rollRate - pitchRate - yawRate;
  inputMotor[1] = desireThrottle + rollRate + yawRate;
  inputMotor[2] = desireThrottle + rollRate + pitchRate - yawRate;
  inputMotor[3] = desireThrottle - rollRate + pitchRate + yawRate;
  inputMotor[4] = desireThrottle - rollRate - yawRate;
  inputMotor[5] = desireThrottle - rollRate - pitchRate + yawRate;
  

  for (byte i=0; i<6; i++){
    if(inputMotor[i]>2000) inputMotor[i]=2000;
    //Adding idle
    if(inputMotor[i]<1180) inputMotor[i]=1180;
  }

  //Adding noThrottle
  if(desireThrottle<1050){
    for (byte i=0; i<6; i++) inputMotor[i]=1000;
    resetPID;
  }

  //Send PWM signal to motors ESC
  for(byte i=0; i<6; i++) ledcWrite(motorCh[i], inputMotor[i]);

  //Wait timer happens
  while(micros()-loopTime < t*1000000);
  loopTime=micros();
}

//PID Controll:
//Input = Kp * err + prevI + Ki*(err + prevErr)*t/2 + Kd * (err-prevErr)/t 
void PIDControll(float &orientation, float &err, float &prevErr, float &prevI, float &Kp, float &Ki, float &Kd, float t){
  float P = Kp*err;
  float I = prevI + Ki*(err+prevErr)*t/2;
  float D = Kd*(err-prevErr)/t;
  //To avoid integral windup
  if(I>400) I=400;
  else if(I<-400) I=-400;
  float PID = P+I+D;
  if (PID>400) PID = 400;
  else if (PID<-400) PID = -400;

}

void resetPID(void){
  prevErrRollRate=0;
  prevErrPitchRate=0;
  prevErrYawRate=0;
  prevIRollRate=0;
  prevIPitchRate=0;
  prevIYawRate=0;
}