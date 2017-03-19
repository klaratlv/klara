/*
   This header file adds support for internet connectivity

  to activate, create this PUT request (using IFTTT or POSTMAN, for example)
  http://blynk-cloud.com/70bfe532b9a84f89a889e0c7dfad386f/pin/V0
  (if this url does not work from IFTTT, replace hostname with 188.166.177.186)
  with payload:
  [
  "100"
  ]

  or this GET request, for example:
  http://blynk-cloud.com/70bfe532b9a84f89a889e0c7dfad386f/update/V0?value=100
  
  if this url does not work from IFTTT, replace hostname with 188.166.177.186 (DNS issue)

  for multiple values, use pin V31, for example:
  http://blynk-cloud.com/70bfe532b9a84f89a889e0c7dfad386f/update/V31?value=101,22,33,44,55,66,77
  
  Any of these calls will create an invocation in on_command() function

  Note:
    supported pins: V0-V31

  requisites:
  1. setup wire library as demonstrated
  2. constantly poll internet via internet_poll() function
*/

#ifndef INTERNET_H
#define INTERNET_H

/********************************************
   Includes
 *******************************************/
#include <Wire.h>

/********************************************
   Defines
 *******************************************/
#define I2C_ADDRESS 8
#define I2C_PACKET_SIZE 7

//Virtual Pins
#define ARM_SERVO_BASE 0
#define ARM_SERVO_SHOULDER 1
#define ARM_SERVO_ELBOW 2
#define ARM_SERVO_WRIST 3
#define ARM_PUMP 4
#define MISSION 31

/********************************************
   Prototypes
 *******************************************/
void accept_mission(int station, int orientation, int action, int param1, int param2, int param3);

/********************************************
   Variables
 *******************************************/
volatile int last_remote_command = -1;
volatile int last_remote_values[I2C_PACKET_SIZE-1] = {-1};

/********************************************
   Local Functions
 *******************************************/
void receiveEvent(int howMany) {
  last_remote_command = Wire.read();
  for(int i = 0; i < (I2C_PACKET_SIZE-1); ++i) {
    last_remote_values[i] = Wire.read();
  }
}

void on_command(volatile int cmd, volatile int vals[]) {
  Serial.println(String("Internet: got command: ") + cmd + ", values: " + vals[0] + "," + vals[1] + "," + vals[2] + "," + vals[3] + "," + vals[4] + "," + vals[5]);
  BT.println(String("Internet: got command: ") + cmd + ", values: " + vals[0] + "," + vals[1] + "," + vals[2] + "," + vals[3] + "," + vals[4] + "," + vals[5]);

  switch (cmd) {
    case ARM_SERVO_BASE:
      arm_setServoAngle(0, map(vals[0], 0, 255, ARM_SERVO_0_MIN_ANGLE, ARM_SERVO_0_MAX_ANGLE));
      break;
    case ARM_SERVO_SHOULDER:
      arm_setServoAngle(1, map(vals[0], 0, 255, ARM_SERVO_1_MIN_ANGLE, ARM_SERVO_1_MAX_ANGLE));
      break;
    case ARM_SERVO_ELBOW:
      arm_setServoAngle(2, map(vals[0], 0, 255, ARM_SERVO_2_MIN_ANGLE, ARM_SERVO_2_MAX_ANGLE));
      break;
    case ARM_SERVO_WRIST:
      arm_setServoAngle(3, map(vals[0], 0, 255, ARM_SERVO_3_MIN_ANGLE, ARM_SERVO_3_MAX_ANGLE));
      break;
	  case ARM_PUMP:
      arm_pumpActivate(vals[0]);
      break;
  case MISSION: 
      accept_mission(vals[0], vals[1], vals[2], vals[3], vals[4], vals[5]);
      break;
  }
}

/********************************************
   API Functions
 *******************************************/
void internet_init() {
  Wire.begin(I2C_ADDRESS);
  Wire.onReceive(receiveEvent);
}

void internet_poll() {
  if (last_remote_command >= 0) {
    on_command(last_remote_command, last_remote_values);
    last_remote_command = -1;
  }
}


void internet_reset() {
  last_remote_command = -1;
  for(int i = 0; i < (I2C_PACKET_SIZE-1); ++i) {
    last_remote_values[i] = -1;
  }  
}

#endif
