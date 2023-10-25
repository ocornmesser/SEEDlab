//Include Libararies 
#include <Encoder.h>
#include <DualMC33926MotorShield.h>
#include <Wire.h>

//Constants and Shield Init.
#define ARD_ADDR 8 //Computer vision
DualMC33926MotorShield md;
Encoder motor1Enc(3,6);
Encoder motor2Enc(2,5);

//Time Related Variables
unsigned long desired_Ts_ms = 25;  //time between display
unsigned long last_time_ms;
unsigned long start_time_ms;
double current_time;

//Communication Variables
volatile uint8_t offset = 0;            //variables to interface with rasberry pi
volatile uint8_t instruction[32] = {0};
volatile uint8_t msgLength = 0;

//Control and State Variables
double batteryVoltage = 7.8;
double linPosError = 0;    
double linVelError = 0;
double linPosIntegral = 0;
double linVelIntegral = 0;
double rotPosError = 0;
double rotVelError = 0;
double rotPosIntegral = 0;
double rotVelIntegral = 0;

uint8_t input = 0;     //rasberry pi inputs
uint8_t lastInput = 0;
uint8_t activeChange = 0;

double LM1meters = 0;  // encoder positions
double RM2meters = 0;
double oldLMmeters = 0;
double oldRMmeters = 0;

double avgMeters = 0;   //these variables are inputs/outputs for the controllers
double avgVelocity = 0;
double oldAvgMeters = 0;
double avgRotPos = 0;
double avgRotVel = 0;  
double oldAvgRotPos = 0;
double LMvelocity = 0;
double RMvelocity = 0;


double desiredVel = 0;
double desiredPos = 2;   // set distance robot should travel
double desiredRotPos = 0;  // set rotation angle in rads
double desiredRotVel = 0;

double voltageLin = 0;
double voltageRot = 0;

//End of Variables

//Functions for PID Motor Control
double PIDlinPos (double actual, double desired) {  //use for all control
 
  linPosError = desired - actual;  //error
 
  linPosIntegral += (linPosError * (float)desired_Ts_ms /1000);
 
  return(1.036 * linPosError + 0.009467 * linPosIntegral);

}

double PIDlinVel (double actual, double desired) {  //use for all control
 
  linVelError = desired - actual;  //error
 
  linVelIntegral += (linVelError * (float)desired_Ts_ms /1000);
 
  return(16.93 * linVelError + 0 * linVelIntegral);

}

double PIDrotVel (double actual, double desired) {  //use for all control
 
  rotVelError = desired - actual;  //error
 
  rotVelIntegral += (rotVelError * (float)desired_Ts_ms /1000);
 
  return(3.5 * rotVelError + 13.7 * rotVelIntegral);

}
double PIDrotPos (double actual, double desired) {  //use for all control
 
  rotPosError = desired - actual;  //error
 
  rotPosIntegral += (rotPosError * (float)desired_Ts_ms /1000);
 
  return(.3 * rotPosError + .001 * rotPosIntegral);

}

//End of Motor Control Functions

//Function to control Motor
void MotorControl (double voltagePos, double voltageRot){   // apply voltage to motor
 
  double voltage[2] = {((voltagePos + voltageRot)/2), ((voltagePos - voltageRot)/2)};

  int pwm[2] = {(400*(voltage[0])/batteryVoltage), (400*(voltage[1])/batteryVoltage)}; // calculate the pwm for each motor 

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

  // Serial.print(pwm[1]);
  // Serial.print("\t");
  // Serial.println(pwm[0]);

}

void setup() {
  Serial.begin(115200);
  md.init();

  Wire.begin(ARD_ADDR);   //intialize interface
  Wire.onReceive(receive);


  pinMode(4, OUTPUT);  //enb
  digitalWrite(4, HIGH);



  last_time_ms = millis(); // set up sample time variable
  start_time_ms = last_time_ms;


}

void loop() {

  current_time = (double)(last_time_ms - start_time_ms) / 1000;
  LM1meters = -(double)motor1Enc.read()/4 * 0.0079 * .075;
  RM2meters = (double)motor2Enc.read()/4 * 0.0079 * .075;

  // Serial.print(LM1meters);
  // Serial.print("\t");
  // Serial.println(RM2meters);

  // These equations help implement velocity and position through each motor. 

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

  desiredVel = PIDlinPos(avgMeters, desiredPos); // implement the PID controllers
  voltageLin = PIDlinVel(avgVelocity, desiredVel);
  desiredRotVel = PIDrotPos(avgRotPos, desiredRotPos); //set desired rotational position to make the robot turn 360 degrees fully
  voltageRot = PIDrotVel(avgRotVel, desiredRotVel);

  MotorControl(voltageLin, voltageRot);
 
  Serial.print("Rotational:"); // print important values for rotation
  Serial.print("\t");
  Serial.print(current_time);
  Serial.print("\t");
  Serial.print(voltageRot, 3);   //h
  Serial.print("\t");
  Serial.print(avgRotVel,  3);
  Serial.print("\t");
  Serial.println(rotPosError,  3);

  //Serial.print("Linear:");  // print important values for linear
  //Serial.print("\t");
  //Serial.print(current_time);
  //Serial.print("\t");
  //Serial.print(voltageLin, 3);   //h
  //Serial.print("\t");
  //Serial.print(avgRotPos,  3);
  //Serial.print("\t");
  //Serial.println(linPosError,  3);

  while (millis()<last_time_ms + desired_Ts_ms) {
    //wait until desired time passes to go top of the loop
  }

  last_time_ms = millis();

}

// printReceived helps us see what data we are getting from the leader
void printReceived() {
// Print on serial console
Serial.print("Offset received: ");
Serial.println(offset);
Serial.print("Message Length: ");    
Serial.println(msgLength);
Serial.print("Instruction received: ");
for (int i=0;i<msgLength;i++) {
Serial.print(String(instruction[i])+"\t");
}
Serial.println("");
}
// function called when an I2C interrupt event happens
void receive() {
// Set the offset, this will always be the first byte.
offset = Wire.read();
// If there is information after the offset, it is telling us more about the command.
while (Wire.available()) {
instruction[msgLength] = Wire.read();
msgLength++;
}
}
