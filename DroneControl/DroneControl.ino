#include <PPM_Communication.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <MS5611.h>

//Kalman Filter for Altitude
#include <BasicLinearAlgebra.h>
using namespace BLA;
float altitudeK;
float velocityK;
BLA::Matrix<2,2> F;
BLA::Matrix<2,1> G;
BLA::Matrix<2,2> P;
BLA::Matrix<2,2> Q;
BLA::Matrix<2,1> S;
BLA::Matrix<1,2> H;
BLA::Matrix<2,2> I;
BLA::Matrix<1,1> accK;
BLA::Matrix<2,1> K;
BLA::Matrix<1,1> R;
BLA::Matrix<1,1> L;
BLA::Matrix<1,1> M;

void kelmanInitValues(void);
void kalmanAltitude(void);

//                Barometer sensor
MS5611 MS5611(0x77);

float altitudeAtFloor=0;
float altitude;
float accZVert;

float kpVelAltitude = 3.5;
float kiVelAltitude = 0.0015;
float kdVelAltitude = 0.01;

float errVelAltitude;
float prevErrVelAltitude=0;
float prevIVelAltitude=0;

void getAltitudeReferenceLevel(void);
float getAltitude(void);
void getAccZVert(void);


//                Flight Mode

byte rotorNumber = 6;

//Number of motors inputs
float inputMotor[6];

//Motor pins and channel
byte motorPin[6] = { 25, 26, 27, 18, 5, 10};
byte motorCh[6] = { 0, 1, 2, 3, 4, 5 };

//                LEDs Pin
byte blueLED = 19;
byte redLED = 23;

//                LED Prototype functions
void noColor(void);
void redColor(void);
void blueColor(void);
void magentaColor(void);

//                Drone initialization variables
byte droneInit = 0;
byte onceUp = 0;
byte onceDown = 0;
byte armed = 0;
byte connectionLost = 0;

//                Rate Controller
//PID outputs Rate °/s
float rollRate;
float pitchRate;
float yawRate;

//PID for Roll Rate °/s
float kpRollRate = 1*0.6;
float kiRollRate = 0.0325/0.004;
float kdRollRate = 0.004/9;
// float kpRollRate = 2.5*0.6;
// float kiRollRate = 0.0325/0.004;
// float kdRollRate = 0.004/9;

//PID for Pitch Rate °/s
float kpPitchRate = 1*0.6;
float kiPitchRate = 0.0325/0.004;
float kdPitchRate = 0.004/9;
// float kpPitchRate = 2.5*0.6;
// float kiPitchRate = 0.0325/0.004;
// float kdPitchRate = 0.004/9;


//PID for Yaw Rate °/s
float kpYawRate = 2;
float kiYawRate = 12;
float kdYawRate = 0;

//Errors Rate °/s
float errRollRate;
float errPitchRate;
float errYawRate;
float prevErrRollRate = 0;
float prevErrPitchRate = 0;
float prevErrYawRate = 0;

//Previous I values Rate °/s
float prevIRollRate = 0;
float prevIPitchRate = 0;
float prevIYawRate = 0;

//            Angle Controller
//PID outputs Angle °
float rollAngle;
float pitchAngle;
//Yaw wont be taken in consideration cause once yaw is put, i dont want the drone to return to 0

//PID for Roll Angle °
float kpRollAngle = 2;
float kiRollAngle = 0;
float kdRollAngle = 0;

//PID for Pitch Angle °
float kpPitchAngle = 2;
float kiPitchAngle = 0;
float kdPitchAngle = 0;

//Errors Angle °
float errRollAngle;
float errPitchAngle;
float prevErrRollAngle = 0;
float prevErrPitchAngle = 0;

//Previous I values Angle °
float prevIRollAngle = 0;
float prevIPitchAngle = 0;

//Time between new PID
float t = 0.004;

//            Input of RC
float desireRollRate;
float desirePitchRate;
float desireRollAngle;
float desirePitchAngle;
float desireYawRate;

float velAltitude;
float desireVelAltitude;

float pastRollAngle;

//             Input from BNO055 sensor

// Class for BNO055 usage
Adafruit_BNO055 myIMU = Adafruit_BNO055();

// Angle °
float actualRollAngle;
float actualPitchAngle;

// Rate  °/s
float actualRollRate;
float actualPitchRate;
float actualYawRate;

//             Initiliazation PPM reader
PPM myPPM = PPM();

//             Time in Micro for loop to restart
float loopTime;

//             Prototypes functions PID
void PIDControll(float &orientation, float &err, float &prevErr, float &prevI, float &Kp, float &Ki, float &Kd, float t);
void resetPID(void);

//             Drone State functions prototypes
void waitRcConnect(void);
void droneInitProtocol(void);
void droneFlight(void);
void droneMotorsOff(void);
void droneMotorsIdle(void);
void calibrateDrone(void);
byte isDroneArmed(void);


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
  if( !MS5611.begin()){
    Serial.print("MS5611 not Found, check connection!");
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

  //Barometer setup
  getAltitudeReferenceLevel();

  //Kelman Setup
  kelmanInitValues();

  //First time drone is powered it will set all motors off
  droneMotorsOff();

  //Intialization Protocol for safety
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

  prevErrVelAltitude=0;
  prevIVelAltitude=0;
}

void droneFlight(void){

  //Get gyro info from BNO055
  imu::Vector<3> gyr = myIMU.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  actualRollRate = gyr.x();
  actualPitchRate = gyr.y();
  actualYawRate = -gyr.z();

  //Get angle info from BNO055
  sensors_event_t event;
  myIMU.getEvent(&event);
  actualRollAngle=-event.orientation.z;
  actualPitchAngle=-event.orientation.y;

  //Get acc in vertical Z
  getAccZVert();

  //Getting altitude
  altitude = getAltitude()-altitudeAtFloor;
  Serial.print("Altitude [cm]: ");
  Serial.print(altitude);

  //Running Kelman Filter
  kalmanAltitude();

  Serial.print("Altitude [cm]: ");
  Serial.print(altitudeK);
  Serial.print("Vertical velocity [cm/s]: ");
  Serial.print(velocityK);


  //Getting desire velAltitude
  desireVelAltitude=0.3*(myPPM.ch[1]-1500);
  errVelAltitude=desireVelAltitude-velocityK;
  PIDControll(velAltitude, errVelAltitude, prevErrVelAltitude, prevIVelAltitude, kpVelAltitude, kiVelAltitude, kdVelAltitude, t);
  velAltitude += 1500;
  
  //Getting desire rates °/s base on inputs
  desireYawRate = 0.15*((float)myPPM.ch[4]-1500);

  //Getting desire angles ° base on controller inputs
  desireRollAngle = 0.1*((float)myPPM.ch[2]-1500);
  desirePitchAngle = 0.1*((float)myPPM.ch[3]-1500);
  
  //Get error for Angles °
  errRollAngle = desireRollAngle-actualRollAngle;
  errPitchAngle = desirePitchAngle - actualPitchAngle;

  //Calculate PID for Angles °
  PIDControll( rollAngle, errRollAngle, prevErrRollAngle, prevIRollAngle, kpRollAngle, kiRollAngle, kdRollAngle, t);
  PIDControll( pitchAngle, errPitchAngle, prevErrPitchAngle, prevIPitchAngle, kpPitchAngle, kiPitchAngle, kdPitchAngle, t);

  //Save the output of the PID for Angles ° as the input for the PID of Rates °/s
  desireRollRate = rollAngle;
  desirePitchRate = pitchAngle;

  //Get error for Rate °/s
  errRollRate = desireRollRate - actualRollRate;
  errPitchRate = desirePitchRate - actualPitchRate;
  errYawRate = desireYawRate - actualYawRate;
  
  //Calculate PID for Rate °/s
  PIDControll( rollRate, errRollRate, prevErrRollRate, prevIRollRate, kpRollRate, kiRollRate, kdRollRate, t);
  PIDControll( pitchRate, errPitchRate, prevErrPitchRate, prevIPitchRate, kpPitchRate, kiPitchRate, kdPitchRate, t);
  PIDControll( yawRate, errYawRate, prevErrYawRate, prevIYawRate, kpYawRate, kiYawRate, kdYawRate, t);

  //Check input throttle and apply a limitation 
  if ( velAltitude > 1500 ) velAltitude = 1500;
  
  //Hexacopter X
  if(rotorNumber == 6){
  
    inputMotor[0] = 1.024*(velAltitude - rollRate - pitchRate - yawRate);
    inputMotor[1] = 1.024*(velAltitude - rollRate + yawRate);
    inputMotor[2] = 1.024*(velAltitude - rollRate + pitchRate - yawRate);
    inputMotor[3] = 1.024*(velAltitude + rollRate + pitchRate + yawRate);
    inputMotor[4] = 1.024*(velAltitude + rollRate - yawRate) + 132;
    inputMotor[5] = 1.024*(velAltitude + rollRate - pitchRate + yawRate);
  }

  //Quadcopter X
  else if (rotorNumber == 4) {
    inputMotor[0] = 1.024*(velAltitude - rollRate - pitchRate + yawRate);
    inputMotor[1] = 1.024*(velAltitude - rollRate + pitchRate - yawRate);
    inputMotor[2] = 1.024*(velAltitude + rollRate + pitchRate + yawRate);
    inputMotor[3] = 1.024*(velAltitude + rollRate - pitchRate - yawRate);
  }

  //Check if the output is the min or max and limitate it
  for (byte i=0; i<rotorNumber; i++){
    if(inputMotor[i]>2000) inputMotor[i]=2000;
    //Adding idle
    if(inputMotor[i]<1050) inputMotor[i]=1050;
  }

  //Adding noThrottle
  if(velAltitude<1050){
    for (byte i=0; i<6; i++) inputMotor[i]=1000;
    resetPID();
  }
  
  // inputMotor[0] = 990;
  // inputMotor[2] = 990;
  // inputMotor[3] = 990;
  // inputMotor[5] = 990;
  //Send PWM signal to motors ESC
  for(byte i=0; i<rotorNumber; i++) ledcWrite(motorCh[i], inputMotor[i]);
  // for(byte i=0; i<rotorNumber; i++) ledcWrite(motorCh[i], velAltitude);
};

void waitRcConnect(void) {
  while(!myPPM.oneRead){
    digitalWrite(redLED, LOW);
    digitalWrite(blueLED, LOW);
    delay(500);
  }
}

void droneInitProtocol(void) {
  //Protocol indicate first time drone powers up, you have to disarm it and then take the throttle up and down for it to allow activation
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
  //Send the min for motors to not move
  for(byte i=0; i<rotorNumber; i++) ledcWrite(motorCh[i], 990);
}

void droneMotorsIdle(void){
  
  //Get gyro info from BNO055
  imu::Vector<3> gyr = myIMU.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  actualRollRate = gyr.x();
  actualPitchRate = gyr.y();
  actualYawRate = -gyr.z();

  //Get angle info from BNO055
  sensors_event_t event;
  myIMU.getEvent(&event);
  actualRollAngle=-event.orientation.z;
  actualPitchAngle=-event.orientation.y;

  //Getting desire rates °/s base on inputs
  velAltitude = (float)1300;
  desireYawRate = (float)0;

  //Getting desire angles ° base on controller inputs
  desireRollAngle = (float)0;
  desirePitchAngle = (float)0;
  
  //Get error for Angles °
  errRollAngle = desireRollAngle-actualRollAngle;
  errPitchAngle = desirePitchAngle - actualPitchAngle;

  //Calculate PID for Angles °
  PIDControll( rollAngle, errRollAngle, prevErrRollAngle, prevIRollAngle, kpRollAngle, kiRollAngle, kdRollAngle, t);
  PIDControll( pitchAngle, errPitchAngle, prevErrPitchAngle, prevIPitchAngle, kpPitchAngle, kiPitchAngle, kdPitchAngle, t);

  //Save the output of the PID for Angles ° as the input for the PID of Rates °/s
  desireRollRate = rollAngle;
  desirePitchRate = pitchAngle;

  //Get error for Rate °/s
  errRollRate = desireRollRate - actualRollRate;
  errPitchRate = desirePitchRate - actualPitchRate;
  errYawRate = desireYawRate - actualYawRate;
  
  //Calculate PID for Rate °/s
  PIDControll( rollRate, errRollRate, prevErrRollRate, prevIRollRate, kpRollRate, kiRollRate, kdRollRate, t);
  PIDControll( pitchRate, errPitchRate, prevErrPitchRate, prevIPitchRate, kpPitchRate, kiPitchRate, kdPitchRate, t);
  PIDControll( yawRate, errYawRate, prevErrYawRate, prevIYawRate, kpYawRate, kiYawRate, kdYawRate, t);

  //Check input throttle and apply a limitation 
  if ( velAltitude > 1500 ) velAltitude = 1500;
  
  //Hexacopter X
  if(rotorNumber == 6){
  
    inputMotor[0] = 1.024*(velAltitude - rollRate - pitchRate - yawRate);
    inputMotor[1] = 1.024*(velAltitude - rollRate + yawRate);
    inputMotor[2] = 1.024*(velAltitude - rollRate + pitchRate - yawRate + 150);
    inputMotor[3] = 1.024*(velAltitude + rollRate + pitchRate + yawRate);
    inputMotor[4] = 1.024*(velAltitude + rollRate - yawRate);
    inputMotor[5] = 1.024*(velAltitude + rollRate - pitchRate + yawRate);
  }

  //Quadcopter X
  else if (rotorNumber == 4) {
    inputMotor[0] = 1.024*(velAltitude - rollRate - pitchRate + yawRate);
    inputMotor[1] = 1.024*(velAltitude - rollRate + pitchRate - yawRate);
    inputMotor[2] = 1.024*(velAltitude + rollRate + pitchRate + yawRate);
    inputMotor[3] = 1.024*(velAltitude + rollRate - pitchRate - yawRate);
  }

  //Check if the output is the min or max and limitate it
  for (byte i=0; i<rotorNumber; i++){
    if(inputMotor[i]>2000) inputMotor[i]=2000;
    //Adding idle
    if(inputMotor[i]<1050) inputMotor[i]=1050;
  }

  //Adding noThrottle
  if(velAltitude<1050){
    for (byte i=0; i<6; i++) inputMotor[i]=1000;
    resetPID();
  }  
  //Send PWM signal to motors ESC
  for(byte i=0; i<rotorNumber; i++) ledcWrite(motorCh[i], inputMotor[i]);
}

void calibrateDrone(void){
  //Calibration variables
  uint8_t system, gyro, accel, mg = 0;
  float lastSecond = millis();
  myIMU.getCalibration(&system, &gyro, &accel, &mg);
  magentaColor();
  byte isMagenta = 1;
  while (system!=3 || accel!=3){
    //Send command to calibrate
    myIMU.getCalibration(&system, &gyro, &accel, &mg);
    imu::Vector<3> accele = myIMU.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
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

void kelmanInitValues(void){
  //Initial Values
  F={1, 0.004, 0, 1};
  G={0.5*0.004*0.004, 0.004};
  H={1,0};
  I={1, 0, 0, 1};
  Q=G*~G*10*10;
  R={30*30};
  P={0, 0, 0, 0};
  S={0, 0};
}

void kalmanAltitude(void){
  accK={accZVert};
  S=F*S+G*accK;
  P=F*P*~F+Q;
  L=H*P*~H+R;
  K=P*~H*Invert(L);
  M={altitude};
  S=S+K*(M-H*S);
  altitudeK=S(0,0);
  velocityK=S(1,0);
  P=(I-K*H)*P;
}

void getAccZVert(void){
  imu::Vector<3> acc = myIMU.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  //Checar que esten del lado correcto
  float accX=acc.x();
  float accY=acc.y();
  float accZ=acc.z();
  accZVert=-accX*sin(actualPitchAngle)+accY*cos(actualPitchAngle)*sin(actualRollAngle)+accZ*cos(actualPitchAngle)*cos(actualRollAngle);
  //Convert to cm/s^2
  accZVert=(accZVert-1)*9.81*100;
}

void getAltitudeReferenceLevel(void){
  for(int i=0; i<2000; i++){
    altitudeAtFloor+=getAltitude();
    delay(1);
  }
  altitudeAtFloor /=2000;
}

float getAltitude(void) {
  return 44330*(1-pow(MS5611.getPressure()/1013.25,1/5.255))*100;
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


