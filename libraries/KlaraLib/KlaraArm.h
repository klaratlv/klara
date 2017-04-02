/*
   This header file adds support for uArm metal robotic arm
*/

#ifndef ARM_H
#define ARM_H

/********************************************
   Includes
 *******************************************/

/********************************************
   Defines
 *******************************************/
//#define ARM_DEBUG //un-comment this line to print arm's position to Serial
#define armSerial Serial3
// uArm servo ranges, as defined in https://media.readthedocs.org/pdf/uarmdocs/latest/uarmdocs.pdf, section 2.3.2
#define ARM_SERVO_0_MIN_ANGLE 0
#define ARM_SERVO_0_MAX_ANGLE 180
#define ARM_SERVO_1_MIN_ANGLE 20
#define ARM_SERVO_1_MAX_ANGLE 150
#define ARM_SERVO_2_MIN_ANGLE 0
#define ARM_SERVO_2_MAX_ANGLE 150
#define ARM_SERVO_3_MIN_ANGLE 0
#define ARM_SERVO_3_MAX_ANGLE 180
  
/********************************************
   Variables
 *******************************************/
boolean armPumpOnFlag = false;
byte armServoAngles[4] = {0,};

/********************************************
   Local Functions
 *******************************************/

/********************************************
   API Functions
 *******************************************/
void arm_setServoAngle(byte servoId, byte servoAngle) {
  //byte armCommand[9] = {0xF0, 0xAA, 0x11, servoId, servoAngle, 0x00, 0x00, (servoId == 0x3 ? 0x0 : 0x1), 0xF7};
  byte armCommand[9] = {0xF0, 0xAA, 0x11, servoId, servoAngle, 0x00, 0x00, 0x1, 0xF7};
  for (int i = 0; i < 9; i++) {
    armSerial.write(armCommand[i]);
  }
#ifdef ARM_DEBUG
  Serial.println("Servo #" + String(servoId) + ", angle = " + String(servoAngle));
#endif
}

void arm_pumpActivate(bool pumpActivate) {
  byte armCommand[5] = {0xF0, 0xAA, 0x1D, (pumpActivate ? 0x1 : 0x0), 0xF7};
  for (int i = 0; i < 5; i++) {
    armSerial.write(armCommand[i]);
  }
}

void arm_resetPosition() {
  arm_pumpActivate(false);
  arm_setServoAngle(0, 80);
  arm_setServoAngle(1, 80);
  arm_setServoAngle(2, 80);
}

void arm_retract() {
  arm_setServoAngle(0, 70);
  arm_setServoAngle(1, 99);
  arm_setServoAngle(2, 100);
}

void arm_init() {
  armSerial.begin(57600);
  arm_resetPosition();
}

void arm_extend() {
  arm_setServoAngle(1, 30);
  arm_setServoAngle(2, 30);
}

#endif
