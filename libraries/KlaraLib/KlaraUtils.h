#ifndef UTILS_H
#define UTILS_H

/********************************************
  Includes
*******************************************/
#include <math.h>

/********************************************
  Defines
*******************************************/
enum action_t {
  NONE,
  BASKETBALL,
  BRICKS,
  AUDIO,
  DANCE,
  LED_PANEL,
  PARTY, //AUDIO + DANCE + LED_PANEL
  DELIVERY,
  ANNOUNCE
};

/********************************************
  Functions
*******************************************/
int mapConstrain(int val, int srcMin, int srcMax, int destMin, int destMax) {
  return(constrain(map(val, srcMin, srcMax+1, destMin, destMax+1), destMin, destMax));
}

int mySign(int x) {
  if (x > 0) return 1;
  if (x < 0) return -1;
  return 0;
}

float fscale( float originalMin, float originalMax, float newBegin, float newEnd, float inputValue, float curve) {
  float OriginalRange = 0;
  float NewRange = 0;
  float zeroRefCurVal = 0;
  float normalizedCurVal = 0;
  float rangedValue = 0;
  bool invFlag = 0;

  // condition curve parameter
  // limit range

  if (curve > 10) curve = 10;
  if (curve < -10) curve = -10;

  curve = (curve * -.1) ; // - invert and scale - this seems more intuitive - postive numbers give more weight to high end on output 
  curve = pow(10, curve); // convert linear scale into lograthimic exponent for other pow function

  /*
  Serial.println(curve * 100, DEC);   // multply by 100 to preserve resolution  
  Serial.println(); 
  */

  // Check for out of range inputValues
  if (inputValue < originalMin) {
    inputValue = originalMin;
  }
  if (inputValue > originalMax) {
    inputValue = originalMax;
  }

  // Zero Refference the values
  OriginalRange = originalMax - originalMin;

  if (newEnd > newBegin){ 
    NewRange = newEnd - newBegin;
  }
  else
  {
    NewRange = newBegin - newEnd; 
    invFlag = 1;
  }

  zeroRefCurVal = inputValue - originalMin;
  normalizedCurVal  =  zeroRefCurVal / OriginalRange;   // normalize to 0 - 1 float

  // Check for originalMin > originalMax  - the math for all other cases i.e. negative numbers seems to work out fine 
  if (originalMin > originalMax ) {
    return 0;
  }

  if (invFlag == 0){
    rangedValue =  (pow(normalizedCurVal, curve) * NewRange) + newBegin;

  }
  else     // invert the ranges
  {   
    rangedValue =  newBegin - (pow(normalizedCurVal, curve) * NewRange); 
  }
  return rangedValue;
}

#endif