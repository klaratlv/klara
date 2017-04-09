
#ifndef MOTOR_H
#define MOTOR_H

/********************************************
   Includes
 *******************************************/
#include <PID_v1.h>
#include <TimerThree.h>
#include "LIFO.h"

/********************************************
   Defines
 *******************************************/
//#define MOTOR_DEBUG
#define MIN_SPEED 0
#define MAX_SPEED 255
#define INITIAL_SPEED 150

//Motor 1
#define motorEnPin1 6
#define motorPin1   5
#define motorPin2   4

//Motor 2
#define motorEnPin2 44
#define motorPin3   43
#define motorPin4   42

//Motor 3
#define motorEnPin3 46
#define motorPin5   47
#define motorPin6   48

//Encoders
#define encoder1Pin 3
#define encoder2Pin 19
#define encoder3Pin 18
#define ENCODER_CPR (20.0 * 64.0) //cycles per revolution, takes gear ratio into acount
#define ENCODER_TIMEOUT 20

//PID
#define PID_Kp 8.14
#define PID_Ki 1.36
#define PID_Kd 0.0

/********************************************
   Variables
 *******************************************/
int motorSpeed[4] = {INITIAL_SPEED, 0, 0, 0}; //[0] - default speed for motors 2 and 3, [1]-[3] - motors 1,2,3 speeds

float pidSpeed[4] = {0,};
volatile unsigned long  encoderLastTick1 = 0;
volatile unsigned long  encoderLastTick2 = 0;
volatile unsigned long  encoderLastTick3 = 0;
volatile unsigned long encoderCnt1 = 0;
volatile unsigned long encoderCnt2 = 0;
volatile unsigned long encoderCnt3 = 0;
volatile int actTps1 = 0; //encoder1 actual ticks per sec
volatile int actTps2 = 0; //encoder2 actual ticks per sec
volatile int actTps3 = 0; //encoder3 actual ticks per sec
LIFO lifo1;
LIFO lifo2;
LIFO lifo3;
int wantedTps1; //encoder1 wanted ticks per sec
int wantedTps2; //encoder2 wanted ticks per sec
int wantedTps3; //encoder3 wanted ticks per sec
float pidError1;
float pidError2;
float pidError3;
PID motor1PID(&pidError1, &pidSpeed[1], 0L, PID_Kp, PID_Ki, PID_Kd, REVERSE);
PID motor2PID(&pidError2, &pidSpeed[2], 0L, PID_Kp, PID_Ki, PID_Kd, REVERSE);
PID motor3PID(&pidError3, &pidSpeed[3], 0L, PID_Kp, PID_Ki, PID_Kd, REVERSE);

/********************************************
   Local Functions
 *******************************************/
/*
   Return a valid speed value that's in the range: [MAX_SPEED, MIN_SPEED]
*/
int checkSpdLimit(int spd) {
  return constrain(spd, MIN_SPEED, MAX_SPEED);
}

/* 
  Set motors speed
*/
void setSpeed(int spd1, int spd2, int spd3) {
  motorSpeed[1] = checkSpdLimit(spd1);
  motorSpeed[2] = checkSpdLimit(spd2);
  motorSpeed[3] = checkSpdLimit(spd3);
  analogWrite(motorEnPin1, motorSpeed[1]);
  analogWrite(motorEnPin2, motorSpeed[2]);
  analogWrite(motorEnPin3, motorSpeed[3]); 
}

/*
  Update actual TPS to 0 in case motors stall
*/
void encoderStallCheck() {
  unsigned long curMillis = millis();
  if (curMillis - encoderLastTick1 > ENCODER_TIMEOUT) {
    //actTps1 = 0;
  }  
  if (curMillis - encoderLastTick2 > ENCODER_TIMEOUT) {
    //actTps2 = 0;
  }  
  if (curMillis - encoderLastTick3 > ENCODER_TIMEOUT) {
    //actTps3 = 0;
  }
}

/*
  ISR for encoders
*/
void isrEncoder1() {
    encoderCnt1++;
    encoderLastTick1 = millis();
    lifo1.push(encoderLastTick1);
    actTps1 = 50 * LIFO_SIZE / lifo1.getDiff();
}
void isrEncoder2() {
  encoderCnt2++;
  encoderLastTick2 = millis();
  lifo2.push(encoderLastTick2);
  actTps2 = 50 * LIFO_SIZE / lifo2.getDiff();
}
void isrEncoder3() {
  encoderCnt3++;
  encoderLastTick3 = millis();
  lifo3.push(encoderLastTick3);
  actTps3 = 50 * LIFO_SIZE / lifo3.getDiff();
}

/********************************************
   API Functions
 *******************************************/
void motorControl_init() {
  attachInterrupt(1, isrEncoder1, FALLING); //interrupt 1 connected to pin 3
  attachInterrupt(4, isrEncoder2, FALLING); //interrupt 4 connected to pin 19
  attachInterrupt(5, isrEncoder3, FALLING); //interrupt 5 connected to pin 18
  motor1PID.SetMode(AUTOMATIC);
  motor2PID.SetMode(AUTOMATIC);
  motor3PID.SetMode(AUTOMATIC);
/*
  motor1PID.SetSampleTime(1); 
  motor2PID.SetSampleTime(1);
  motor3PID.SetSampleTime(1);
*/
  pinMode(encoder1Pin, INPUT);
  pinMode(encoder2Pin, INPUT);
  pinMode(encoder3Pin, INPUT);
  pinMode(motorEnPin1, OUTPUT);
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(motorEnPin2, OUTPUT);
  pinMode(motorPin3, OUTPUT);
  pinMode(motorPin4, OUTPUT);
  pinMode(motorEnPin3, OUTPUT);
  pinMode(motorPin5, OUTPUT);
  pinMode(motorPin6, OUTPUT);
  //Change PWM frequency to avoid awful noise from the motors
  //TCCR0B = TCCR0B & B11111000 | B00000001;  // set timer 0 divisor to 1 for PWM frequency of 62500.00 Hz, D4 & D13
  //TCCR1B = TCCR1B & B11111000 | B00000001;  // set timer 1 divisor to 1 for PWM frequency of 31372.55 Hz, D11 & D12
  //TCCR2B = TCCR2B & B11111000 | B00000001;  // set timer 2 divisor to 1 for PWM frequency of 31372.55 Hz, D9 & D10
  //TCCR3B = TCCR3B & B11111000 | B00000001;  // set timer 3 divisor to 1 for PWM frequency of 31372.55 Hz, D2, D3 & D5
  //TCCR4B = TCCR4B & B11111000 | B00000001;    // set timer 4 divisor to 1 for PWM frequency of 31372.55 Hz, D6, D7 & D8
  //TCCR5B = TCCR5B & B11111000 | B00000001;    // set timer 5 divisor to 1 for PWM frequency of 31372.55 Hz, D44, D45 & D46
}

/*
   Pause motors
*/
void motorControl_pause() {
  analogWrite(motorEnPin1, 0);
  analogWrite(motorEnPin2, 0);
  analogWrite(motorEnPin3, 0);
}

/*
   Resume the last drive state
*/
void motorControl_resume() {
  analogWrite(motorEnPin1, motorSpeed[1]);
  analogWrite(motorEnPin2, motorSpeed[2]);
  analogWrite(motorEnPin3, motorSpeed[3]);
}

/*
  Generic driving function
  inputs:
  spd1, spd2, spd3 - motors speed in the range [-255,255]
  Positive values for CW rotation, negative values for CCW rotation
*/
void motorControl_drive(int spd1, int spd2, int spd3) {
  //motor1 direction
  digitalWrite(motorPin1, (spd1 > 0));
  digitalWrite(motorPin2, (spd1 <= 0));
  //motor2 direction
  digitalWrite(motorPin3, (spd2 > 0));
  digitalWrite(motorPin4, (spd2 <= 0));
  //motor3 direction
  digitalWrite(motorPin5, (spd3 > 0));
  digitalWrite(motorPin6, (spd3 <= 0));
  //set motor speed
  setSpeed(abs(spd1), abs(spd2), abs(spd3));
}

/*
   Sets motors' speeds to make the robot drive forward
   speed1 = 0
   speed2 = speed3
*/
void motorControl_driveForward() {
#ifdef MOTOR_DEBUG
  Serial.println(F("MOTOR_DEBUG: driveForward()"));
  Serial.println("MOTOR_DEBUG: speed = " + String(spd));
#endif
  motorControl_drive(0, -motorSpeed[0], motorSpeed[0]);
}

/*
   Sets motors' speeds to make the robot drive backward
   speed1 = 0
   speed2 = speed3
*/
void motorControl_driveBackward() {
#ifdef MOTOR_DEBUG
  Serial.println(F("MOTOR_DEBUG: driveForward()"));
  Serial.println("MOTOR_DEBUG: speed = " + String(spd));
#endif
  motorControl_drive(0, motorSpeed[0], -motorSpeed[0]);
}

/*
   Sets motors' speeds to make the robot turn right
   input params:
   spdOffset: motor2 speed decrease, in the range MIN_SPEED-MAX_SPEED
   motor1spd: speed for motor1, in the range MIN_SPEED-MAX_SPEED
*/
void motorControl_driveRight(int spdOffset, int motor1spd) {
  motorControl_drive(motor1spd, -motorSpeed[0] + spdOffset, motorSpeed[0]);
}

/*
   Sets motors' speeds to make the robot turn left
   input params:
   spdOffset: motor3 speed decrease, in the range MIN_SPEED-MAX_SPEED
    motor1spd: speed for motor1, in the range MIN_SPEED-MAX_SPEED
*/
void motorControl_driveLeft(int spdOffset, int motor1spd) {
  motorControl_drive(-motor1spd, -motorSpeed[0], motorSpeed[0] + spdOffset);
}

/*
   Sets motors' speeds to make the robot rotate clockwise
   input params:
   turnSpeed: motors' speed during rotation
*/
void motorControl_rotateCW(int turnSpeed) {
  motorControl_drive(turnSpeed, turnSpeed, turnSpeed);
}

/*
   Sets motors' speeds to make the robot rotate counter-clockwise
   input params:
   turnSpeed: motors' speed during rotation
*/
void motorControl_rotateCCW(int turnSpeed) {
  motorControl_drive(-turnSpeed, -turnSpeed, -turnSpeed);
}

/*
  PID control loop for motor speeds
*/
void motorControl_pidComputeAndWrite(int motor1dir, int motor2dir, int motor3dir, int correction = 0, float motor1factor = 1) {
  pidError1 = wantedTps1 - actTps1;
  pidError2 = wantedTps2 - actTps2;
  pidError3 = wantedTps3 - actTps3;
  motor1PID.Compute();
  motor2PID.Compute();
  motor3PID.Compute();
  motorControl_drive(motor1dir * pidSpeed[1] + motor1factor * correction, motor2dir * pidSpeed[2] + correction, motor3dir * pidSpeed[3] + correction);
}

/*
  Rotate CW/CCW to a specific angle at a specific turning speed
*/
void motorControl_rotateToAngle(bool isCW, int angle, int wantedTps) {
  if (angle <= 0 or angle > 1080) return;
  unsigned long encoderEndCnt = float(encoderCnt2) + 3.0 * ENCODER_CPR * angle / 360;
    wantedTps1 = wantedTps;
    wantedTps2 = wantedTps;
    wantedTps3 = wantedTps;
  while (encoderCnt2 < encoderEndCnt) {
      motorControl_pidComputeAndWrite(isCW ? 1 : -1, isCW ? 1 : -1, isCW ? 1 : -1);
  }
  motorControl_pause();
}

int speed2;
int speed3;
void motorControl_goStraight(int tps = 40) {
  motorControl_drive(0, speed2, speed3);

  if(actTps2 > tps) {
    speed2 += 3;
  } else if (actTps2 < tps) {
    speed2 -= 3;
  }

  if(actTps3 > tps) {
    speed3 -= 3;
  } else if (actTps3 < tps) {
    speed3 += 3;
  }
}

void motorControl_resetSpeed(int speed2, int speed3) {
  speed2 = speed2;
  speed3 = speed3;
}

void motorControl_goBackwards(int tps=40) {
  motorControl_drive(0, speed2, speed3);

  if(actTps2 > tps) {
    speed2 -= 3;
  } else if (actTps2 < tps) {
    speed2 += 3;
  }

  if(actTps3 > tps) {
    speed3 += 3;
  } else if (actTps3 < tps) {
    speed3 -= 3;
  }
}


#endif
