// Include libraries 
#include <Wire.h>
#include "PCA9685.h"
#include <SoftwareSerial.h>

PCA9685 driver;

PCA9685 pwmController(Wire);

// Linearly interpolates between standard 2.5%/12.5% phase length (102/512) for -90/+90 degrees
PCA9685_ServoEval pwmServo;

// Joints labeled proximally (top) to distally (bottom)
#define R1 3 // Right joint 1 PWM pin
#define R2 2 // Right joint 2 PWM pin
#define R3 1 // Right joint 3 PWM pin
#define L1 4 // Left joint 1 PWM pin
#define L2 5 // Left joint 2 PWM pin
#define L3 7 // Left joint 3 PWM pin

// Wheels labeled with battery packs in the front 
#define rxPinR 13 // not used
#define txPinR 2 // digital pin 2 connects to smcSerial RX on the right wheel 
SoftwareSerial smcSerialR = SoftwareSerial(rxPinR, txPinR);
#define rxPinL 14 // not used
#define txPinL 0 // digital pin 0 connects to smcSerial RX on the right wheel 
SoftwareSerial smcSerialL = SoftwareSerial(rxPinL, txPinL);

// Initialize matlab-arduino communication variable 
byte action = 0;

void setup() {
  Serial.begin(9600);
  Serial.println("Ready for Matlab-Arduino communication");

  pwmController.resetDevices(); // Resets all PCA9685 devices on i2c line
  pwmController.init();
  pwmController.setPWMFreqServo(); // 50Hz provides standard 20ms servo phase length

  // Initialize software serial objects with baud rate of 19.2 kbps
  smcSerialR.begin(19200);
  smcSerialL.begin(19200);
//  Serial.println("software serial connected");
  // The Simple Motor Controller must be running for at least 1 ms before we try to send serial
  // data, so we delay for 5ms here. 
  delay(5);

  // If the Simple Motor Controller has automatic baud detection 
  // enabled, we first need to send it the byte 0xAA (170 in decimal) 
  // so that it can learn the baud rate. 
  smcSerialR.write(0xAA);
  smcSerialL.write(0xAA);
//  Serial.println("automatic baud detection");

  // Next we need to send the Exit Safe Start command, which 
  // clears the safe-start violation and lets the motor run. 
  exitSafeStart();
//  Serial.println("exit Safe Start");
  
}

void loop() {
  // Check to see if there is data in the serial port
  if (Serial.available() > 0){
    // Read the action command sent from matlab via the serial port
    action = Serial.read();
    // Perform corresponding action
    if (action == 0){
      neutral_pose();
    } else if (action == 1){
      neutral_pose();
      delay(1500);
      wave_right();
    } else if (action == 2){
      neutral_pose();
      delay(1500);
      wave_left();
    } else if (action == 3){
      neutral_pose();
      delay(1500);
      high_five();
    } else if (action == 4){
      neutral_pose();
      delay(1500);
      high_five_right();
    } else if (action == 5){
      neutral_pose();
      delay(1500);
      high_five_left();
    } else if (action == 6){
      neutral_pose();
      delay(1500);
      mid_five();
    } else if (action == 7){
      neutral_pose();
      delay(1500);
      mid_five_right();
    } else if (action == 8){
      neutral_pose();
      delay(1500);
      mid_five_left();
    } else if (action == 9){
      neutral_pose();
      delay(1500);
      low_five();
    } else if (action == 10){
      neutral_pose();
      delay(1500);
      low_five_right();
    } else if (action == 11){
      neutral_pose();
      delay(1500);
      low_five_left();
    } else if (action == 12){
      neutral_pose();
      delay(1500);
      highright_lowleft();
    } else if (action == 13){
      neutral_pose();
      delay(1500);
      highleft_lowright();
    } else if (action == 14){
      neutral_pose();
      delay(1500);
      Forward();
      delay(1000);
      Stop();
    } else if (action == 15){
      neutral_pose();
      delay(1500);
      Reverse();
      delay(1000);
      Stop();
    } else if (action == 16){
      neutral_pose();
      delay(1500);
      Turn_Right();
      delay(2000);
      Stop();
    } else if (action == 17){
      neutral_pose();
      delay(1500);
      Turn_Left();
      delay(2000);
      Stop();
    }
  }
}


// Individual Action Functions

void neutral_right(){
  pwmController.setChannelPWM(R1,pwmServo.pwmForAngle(0));
  pwmController.setChannelPWM(R2,pwmServo.pwmForAngle(-90));
  pwmController.setChannelPWM(R3,pwmServo.pwmForAngle(0)); 
}

void neutral_left(){ 
  pwmController.setChannelPWM(L1,pwmServo.pwmForAngle(0));
  pwmController.setChannelPWM(L2,pwmServo.pwmForAngle(90));
  pwmController.setChannelPWM(L3,pwmServo.pwmForAngle(0)); 
}

void neutral_pose(){
  // send the SAR into a human neutral pose
  neutral_right();
  neutral_left();
  Stop();
}


void wave_left(){
// wave with the left arm, right neutral
// set other joints
pwmController.setChannelPWM(L1,pwmServo.pwmForAngle(-90));
pwmController.setChannelPWM(L2,pwmServo.pwmForAngle(0));
delay(500);
// waving motion 
pwmController.setChannelPWM(L3,pwmServo.pwmForAngle(90));
delay(500);
pwmController.setChannelPWM(L3,pwmServo.pwmForAngle(0));
delay(500);
pwmController.setChannelPWM(L3,pwmServo.pwmForAngle(90));
delay(500);
pwmController.setChannelPWM(L3,pwmServo.pwmForAngle(0));
delay(500);
pwmController.setChannelPWM(L3,pwmServo.pwmForAngle(90));
delay(500);
pwmController.setChannelPWM(L3,pwmServo.pwmForAngle(0));
}

void wave_right(){
// wave with the right arm, left neutral
// set other joints
pwmController.setChannelPWM(R1,pwmServo.pwmForAngle(90));
pwmController.setChannelPWM(R2,pwmServo.pwmForAngle(0));
delay(500);
// waving motion 
pwmController.setChannelPWM(R3,pwmServo.pwmForAngle(-90));
delay(500);
pwmController.setChannelPWM(R3,pwmServo.pwmForAngle(0));
delay(500);
pwmController.setChannelPWM(R3,pwmServo.pwmForAngle(-90));
delay(500);
pwmController.setChannelPWM(R3,pwmServo.pwmForAngle(0));
delay(500);
pwmController.setChannelPWM(R3,pwmServo.pwmForAngle(-90));
delay(500);
pwmController.setChannelPWM(R3,pwmServo.pwmForAngle(0));
}

void high_five_right(){ 
  pwmController.setChannelPWM(R1,pwmServo.pwmForAngle(0));
  pwmController.setChannelPWM(R2,pwmServo.pwmForAngle(90));
  pwmController.setChannelPWM(R3,pwmServo.pwmForAngle(0));
}

void high_five_left(){ 
  pwmController.setChannelPWM(L1,pwmServo.pwmForAngle(0));
  pwmController.setChannelPWM(L2,pwmServo.pwmForAngle(-90));
  pwmController.setChannelPWM(L3,pwmServo.pwmForAngle(0));
}

void high_five(){
  // double high five!
  high_five_right(); 
  high_five_left(); 
}

void low_five_right(){ 
  pwmController.setChannelPWM(R3,pwmServo.pwmForAngle(-90));
}
void low_five_left(){ 
  pwmController.setChannelPWM(L3,pwmServo.pwmForAngle(90));
}

void low_five(){
  // double low five!
  low_five_right();
  low_five_left(); 
}

void highleft_lowright(){ 
  // 
  neutral_pose(); 
  pwmController.setChannelPWM(R3,pwmServo.pwmForAngle(-90));
  pwmController.setChannelPWM(L2,pwmServo.pwmForAngle(-90));
}

void highright_lowleft(){ 
  // 
  neutral_pose(); 
  pwmController.setChannelPWM(R2,pwmServo.pwmForAngle(90));
  pwmController.setChannelPWM(L3,pwmServo.pwmForAngle(90));
}

void mid_five_right(){ 
   pwmController.setChannelPWM(R1,pwmServo.pwmForAngle(90));
}

void mid_five_left(){ 
   pwmController.setChannelPWM(L1,pwmServo.pwmForAngle(-90));

}

void mid_five(){ 
  mid_five_right(); 
  mid_five_left(); 
}

// Wheeled mobility functions

// required to allow motors to move 
// must be called when controller restarts and after any error 
void exitSafeStart()
{
  smcSerialR.write(0x83);
  smcSerialL.write(0x83);
}

// Left wheel speed controll (forward & backward)
// Speed should be a number from -3200 to 3200 
void setMotorSpeed_Left(int speed)
{
  if (speed <0)
  { 
    smcSerialL.write(0x86); // motor reverse command 
  }
  else
  {
    smcSerialL.write(0x85); // motor forward command 
  }
  smcSerialL.write(speed & 0x1F);
  smcSerialL.write(speed >> 5 & 0x7F);
}

// Right wheel speed controll (forward & backward)
// Speed should be a number from -3200 to 3200 
void setMotorSpeed_Right(int speed)
{
  if (speed <0)
  { 
    smcSerialR.write(0x86); // motor reverse command 
  }
  else
  {
    smcSerialR.write(0x85); // motor forward command 
  }
  smcSerialR.write(speed & 0x1F);
  smcSerialR.write(speed >> 5 & 0x7F);
}

void Forward(){
  setMotorSpeed_Right(2200);
  setMotorSpeed_Left(-2200);
}

void Reverse(){
  setMotorSpeed_Right(-2200);
  setMotorSpeed_Left(2200);
}

void Turn_Right(){
  setMotorSpeed_Right(-2200);
  setMotorSpeed_Left(-2200);
}

void Turn_Left(){
  setMotorSpeed_Right(2200);
  setMotorSpeed_Left(2200);
}

void Stop(){
  setMotorSpeed_Right(0);
  setMotorSpeed_Left(0);
}
