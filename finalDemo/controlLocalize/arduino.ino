#include <Encoder.h>

#include <DualMC33926MotorShield.h>

#include <Wire.h>



DualMC33926MotorShield md;


#define ARD_ADDR 8 //Computer vision

unsigned long desired_Ts_ms = 25;  //time between display
unsigned long last_time_ms;
unsigned long start_time_ms;
double current_time;
double timeState1;
int state2trig;

volatile uint8_t offset = 0;            //varis to interface with rasberry pi
volatile uint8_t instruction[32] = {0};
volatile uint8_t msgLength = 0;

double batteryVoltage = 7.8;

double linPosError = 0;    //control functions
double linVelError = 0;
double linPosIntegral = 0;
double linVelIntegral = 0;
double rotPosError = 0;
double rotVelError = 0;
double rotPosIntegral = 0;
double rotVelIntegral = 0;



double LM1meters = 0;
double RM2meters = 0;
double oldLMmeters = 0;
double oldRMmeters = 0;

double avgMeters = 0;
double avgVelocity = 0;
double oldAvgMeters = 0;
double avgRotPos = 0;
double avgRotVel = 0;
double oldAvgRotPos = 0;
double LMvelocity = 0;
double RMvelocity = 0;


double desiredVel = 0;
double desiredPos = 0;
double desiredRotPos = 0;  // subtract .04458599 * drt to get accurate
double desiredRotVel = 0;

double voltageLin = 0;
double voltageRot = 0;

int markerDetect = 0;
double markerAngle = 0;
double markerDist = 0;
double amountRotated = 0;
double currPos = 0;

double numberArray[4];
int dataIn = 0;
int idx = 0;


Encoder motor1Enc(3,6);
Encoder motor2Enc(2,5);

int state = 0;
int enterState5 = 0;

int rotationDone = 0;

void setup() {
  Serial.begin(115200);
  md.init();

  Wire.begin(ARD_ADDR);   //intialize interface
  Wire.onReceive(receive);


  pinMode(4, OUTPUT);  //enb
  digitalWrite(4, HIGH);

  rotPosIntegral = 0;
  rotVelIntegral = 0;
  linPosIntegral = 0;
  linVelIntegral = 0;

  last_time_ms = millis(); // set up sample time variable
  start_time_ms = last_time_ms;

  state = 0;
  enterState5 = 0;

}

double PIDlinPos (double actual, double desired) {  //use for all control
 
  linPosError = desired - actual;  //error
 
  linPosIntegral += (linPosError * (float)desired_Ts_ms /1000);
 
  return(.3 * linPosError + 0.001 * linPosIntegral);

}

double PIDlinVel (double actual, double desired) {  //use for all control
 
  linVelError = desired - actual;  //error
 
  linVelIntegral += (linVelError * (float)desired_Ts_ms /1000);
 
  return(34.6 * linVelError + 105.6 * linVelIntegral);

}

double PIDrotVel (double actual, double desired) {  //use for all control
 
  rotVelError = desired - actual;  //error
 
  rotVelIntegral += (rotVelError * (float)desired_Ts_ms /1000);

  if(rotVelIntegral >= 1){
    rotPosIntegral = 0;
  }
 
  return(3.5 * rotVelError + 13.7 * rotVelIntegral);

}
double PIDrotPos (double actual, double desired) {  //use for all control
 
  rotPosError = desired - actual;  //error

  if(rotPosIntegral >= 1){
    rotPosIntegral = 0;
  }
 
  rotPosIntegral += (rotPosError * (float)desired_Ts_ms /1000);

 
  return(.9 * rotPosError + .003 * rotPosIntegral);

}

void clearIntegrators(){
  rotPosIntegral = 0;
  rotVelIntegral = 0;
  linVelIntegral = 0;
  linPosIntegral = 0;
}

void CallFullControl(){
 
  desiredVel = PIDlinPos(avgMeters, desiredPos);

  voltageLin = PIDlinVel(avgVelocity, desiredVel);

  desiredRotVel = PIDrotPos(avgRotPos, desiredRotPos);
 
  voltageRot = PIDrotVel(avgRotVel, desiredRotVel);

}

void MotorControl (double voltageLinear, double voltageRotational){   // apply voltage to motor
 
  double voltage[2] = {((voltageLinear + voltageRotational)/2), ((voltageLinear - voltageRotational)/2)};

  int pwm[2] = {(400*(voltage[0])/batteryVoltage), (400*(voltage[1])/batteryVoltage)};

  if(pwm[0] <= 400 and pwm[0] >= -400){
    pwm[0] = 400*(voltage[0])/batteryVoltage;
  }else if(pwm[0] > 400){
    pwm[0] = 400;
  }else{
    pwm[0] = -400;
  }

  if(pwm[1] <= 400 and pwm[1] >= -400){
    pwm[1] = 400*(voltage[1])/batteryVoltage;
  }else if(pwm[1] > 400){
    pwm[1] = 400;
  }else{
    pwm[1] = -400;
  }


  md.setM1Speed(pwm[0]);
  md.setM2Speed(pwm[1]);

  // Serial.print("PWM: R,L");
  // Serial.print(pwm[1]);
  // Serial.print("\t");
  // Serial.println(pwm[0]);

}

void MotorStop (){   // apply voltage to motor
 
  md.setM1Speed(0);
  md.setM2Speed(0);

}


void Rotate(){
  desiredRotVel = .4;
  voltageRot = PIDrotVel(avgRotVel, desiredRotVel);
}

void Circle(){
 
    desiredRotVel = 1.2;
    desiredVel = .5;
    voltageRot = PIDrotVel(avgRotVel, desiredRotVel);
    voltageLin = PIDlinVel(avgVelocity, desiredVel);  
 
  }



void loop() {

  current_time = (double)(last_time_ms - start_time_ms) / 1000;

  printReceived();


    // If there is data on the buffer, read it
  // If there is data on the buffer, read it

  markerDetect = numberArray[0];
 


  LM1meters = -(double)motor1Enc.read()/4 * 0.0079 * .075;
  RM2meters = (double)motor2Enc.read()/4 * 0.0079 * .075;

  LMvelocity = (LM1meters - oldLMmeters) / ((double)desired_Ts_ms/1000);
  RMvelocity = (RM2meters - oldRMmeters) / ((double)desired_Ts_ms/1000);

  avgMeters = (RM2meters + LM1meters)/2;

  avgVelocity = (RMvelocity + LMvelocity)/2;

  avgRotPos = (LM1meters - RM2meters)/.386;  

  avgRotVel = (LMvelocity - RMvelocity)/.386;

  oldAvgMeters = avgMeters;

  oldAvgRotPos = avgRotPos;

  oldLMmeters = LM1meters;

  oldRMmeters = RM2meters;
 
 
  switch(state){  //changes motor position depending on input

    case 0:

    Rotate();

    state = 1;

    case 1:

    Serial.println("case1");

    if(markerDetect == 1){
      MotorStop();
      amountRotated = avgRotPos;
      //markerAngle = (20 * (PI/180))*(1-.04458599);
      timeState1 = current_time;
      state = 6;
    }else{
      Rotate();
    }
   

    break;
    case 2:

    Serial.println("case2");

    desiredRotPos = markerAngle + amountRotated;
   
    desiredRotVel = PIDrotPos(avgRotPos, desiredRotPos);
 
    voltageRot = PIDrotVel(avgRotVel, desiredRotVel);

    if(rotPosError <= 0.01){
     
        //markerDist = (numberArray[2]/100.0) - .33;
        // Serial.println("dist recieved:");
        // Serial.println(markerDist);
       
        state = 3;
       
    }
   

    break;

    // case 8:

    // desiredPos = 1.9;

    // //desiredPos = 1;

    // CallFullControl();

    // // if(linPosError <= 0.01){
    // //   state = 3;
    // //   currPos = avgMeters;
    // //   markerDist = (numberArray[2]/100.0) - .4;
    // // }

    // if(linPosError <= 0.2){
    //   state = 6;
    // }


    // break;

    case 3:
   
    Serial.println("case3");

    if(markerDetect == 1){
      markerDist = (numberArray[2]/100.0) - .33;
    }
   
    desiredPos = markerDist + avgMeters;
     
    //desiredPos = 3;

    CallFullControl();


    if(abs(linPosError) <= 0.2){
      amountRotated = avgRotPos;
      state = 4;
    }
   


    break;

    case 4:

    Serial.println("case4");
   
    desiredRotPos = -1.45 + amountRotated;
   
    CallFullControl();

    if(abs(rotPosError) <= .01){
      state = 5;
      amountRotated = avgRotPos;
    }

   

    break;
    case 5:

    if(avgRotPos < (5.75 + amountRotated)){
      Circle();
    }else{
      state = 6;
    }

    Serial.println("case5");

    break;

    case 6:

    desiredVel = 0;
    desiredRotVel = 0;
    MotorStop();

    if((current_time >= timeState1 + 1) and (state2trig == 0) and (markerDetect == 1)){
      state = 2;
      state2trig = 1;
      markerAngle = ((((double)numberArray[1]/100.0))*(1-.04458599)) + .05 ;
      Serial.print("THIS IS ANGLE: ");
      Serial.println(markerAngle);
    }

    Serial.println("case6");

    break;
   
  }

  if(state == 6){
    MotorStop();
  }else{
    MotorControl(voltageLin, voltageRot);
  }

 // CallFullControl();

  // if(current_time >= 3){
  //   markerDetect = 1;
  // }


  // if((abs(rotPosError) <= 0.001) and (rotationDone == 0)){
  //   md.setM1Speed(0);
  //   md.setM2Speed(0);
  //   desiredPos = 0.3048;
  //   avgRotPos = desiredRotPos;
  //   rotationDone = 1;
  //   rotPosIntegral = 0;
  //   rotVelIntegral = 0;
  //   linPosIntegral = 0;
  //   linVelIntegral = 0;
  //   Serial.println("Rotation Done");
  // }
 

  if (abs(rotPosError) >= 0.01) {
    Serial.print("Rotational:");
    Serial.print("\t");
    Serial.print(desiredRotPos);
    Serial.print("\t");
    Serial.print(avgRotVel,  3);
    Serial.print("\t");
    Serial.print(avgRotPos,  3);
    Serial.print("\t");
    Serial.println(rotPosError,  3);
  }

  if (abs(linPosError) >= 0.01){
    Serial.print("Linear:");
    Serial.print("\t");    
    Serial.print(desiredPos,  3);
    Serial.print("\t");
    Serial.println(linPosError,  3);
  }
 

    while (millis()<last_time_ms + desired_Ts_ms) {
    //wait until desired time passes to go top of the loop
  }

  last_time_ms = millis();

}

// printReceived helps us see what data we are getting from the leader
void printReceived() {
// Print on serial console
Serial.println("INFO received: ");
Serial.print("Marker detected: ");
Serial.println(markerDetect);
Serial.print("Angle: ");
Serial.println(markerAngle);
Serial.print("Distance:");
Serial.println(markerDist);
}
// function called when an I2C interrupt event happens
void receive() {
  // Set the offset, this will always be the first byte.
  offset = Wire.read();
  // If there is information after the offset, it is telling us more aboutthe command.
  numberArray[0] = 0;
  numberArray[1] = 0;
  numberArray[2] = 0;
  numberArray[3] = 0;
  while (Wire.available()) {
    numberArray[idx] = Wire.read();
    idx += 1;
  }
  if (numberArray[3] == 1){
    numberArray[1] = -1 * numberArray[1];
  }
  Serial.println("ISR:");
  Serial.println(numberArray[1]);
  idx = 0;
  dataIn = 1;
}
