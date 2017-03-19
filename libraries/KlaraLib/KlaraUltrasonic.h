/*
   This header file adds suppport for "Maxbotics Rangefinder" ultrasonic sensors
   Any number of "Maxbotics Rangefinder" ultrasonic sensors between 0-3 can be connected.
   NUM_OF_US_SENSORS should be updated according to the number of connected sensors.
   usSensorPins[] should be updated with the digital pins the sensors are connected to.
*/

#ifndef ULTRASONIC_H
#define ULTRASONIC_H

/********************************************
   Includes
 *******************************************/
#include "KlaraMotorControl.h"

/********************************************
   Defines
 *******************************************/
#define NUM_OF_US_SENSORS 3
#define DIST_DIV_CONST 2 //2 LSB per cm
#define STOP_THR 20 //Klara will stop if there's any object in front of her closer than this distance
#define ULTRASONIC_NEW_VAL_WEIGHT 0.1 //weight for new sensor reading in the range [0.0 - 1.0], higher values mean less memory of old readings

/********************************************
   Variables
 *******************************************/
static const int usSensorPins[3] = {A11, A12, A13};
int distArr[NUM_OF_US_SENSORS] = {0}; //holds last distance read from each sensor

/********************************************
   Local Functions
 *******************************************/

/********************************************
   API Functions
 *******************************************/
/*
   Upodate distance read from all ultrasonic sensors
   Write data to distArr[]
*/
void ultrasonic_updateDist() {
  for (int i = 0; i < NUM_OF_US_SENSORS; i++) {
    distArr[i] = analogRead(usSensorPins[i]) * ULTRASONIC_NEW_VAL_WEIGHT + distArr[i] * (1 - ULTRASONIC_NEW_VAL_WEIGHT);
  }
}

void ultrasonic_init() {
  //Sensor priming
  for (int i = 0; i < 50; i++) {
    ultrasonic_updateDist();
  }
}


#endif
