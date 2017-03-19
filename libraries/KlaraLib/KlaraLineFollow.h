
#ifndef LINE_FOLLOW_H
#define LINE_FOLLOW_H

/********************************************
  Includes
*******************************************/
#include "KlaraMotorControl.h"

/********************************************
  Defines
*******************************************/
//#define LINE_FOLLOW_DEBUG //enable debug printouts
#define DEBUG_SERIAL Serial //select the communication method for sending debug data: 'Serial' or 'BT'
#define DEBUG_INTERVAL_MS 1 //delay between every two debug printouts
#define NUM_OF_NAV_LOCATIONS 2 //number of locations for navigation
#define NUM_OF_LINE_SENSORS 5 //number of line following sensors
#define LINE_FOLLOW_NEW_VAL_WEIGHT 3 //weight for new sensor reading in the range [1 - 10], higher values mean less memory of old readings
#define BRIGHT_DARK_THR 350 //thrshold to distinguish between dark/bright surface
#define LINE_NOT_DETECTED_TIMEOUT_MS 3000 //max time allowed to drive while the line is not detected
#define ROTATE_AND_FIND_LINE_TIMEOUT_MS 5000 //max time allowed to complete the 180deg rotation and find the line
#define TICK_TIMEOUT 100 //timeout to determine whether the encoder is spinning or not
#define CALIBRATION_SPEED 60
#define MS_AFTER_CARPET_FLAG_CHANGE 1000 //time window after carpetFlag change in which we ignore junction detection
#define IGNORE_NAV_AFTER_TURN 2000 //time window after turning right/left in which we ignore navigation to avoid re-detecting a junction right away
#define PIVOT_FACTOR_90_DEG 0.5 //factor that decreases the speed of motor 2 or 3 when rotating
#define PIVOT_FACTOR_180_DEG 0.0 //factor that decreases the speed of motor 2 or 3 when rotating
#define VISIT_ALL_STATIONS 99
#define NUM_OF_STATIONS 10
#define JUNCTION_IGNORE_PERIOD 1000 //time window to ignore a junction after a junction detection (to avoid re-detecting the same junction)
#define ORIENTATION_FORWARD 0
#define ORIENTATION_BACKWARD 1

enum lineSensorState_t {
  ON_LINE,
  LINE_NOT_DETECTED,
  RIGHT_OF_LINE,
  LEFT_OF_LINE,
  JUNCTION_RIGHT,
  JUNCTION_LEFT,
  ON_STOP_MARK,
  PERPENDICULAR,
};
String lineSensorStateStrings[] = {"ON_LINE", "LINE_NOT_DETECTED", "RIGHT_OF_LINE", "LEFT_OF_LINE", "JUNCTION_RIGHT", "JUNCTION_LEFT", "ON_STOP_MARK", "PERPENDICULAR"};

enum navigationState_t {
  HOME,
  EOL,
  ON_MAIN_LINE,
  TOWARDS_STOP_MARK,
  TOWARDS_MAIN_LINE,
  JOIN_MAIN_LINE,
  LINE_LOST,
};
String navigationStateStrings[] = {"HOME", "EOL", "ON_MAIN_LINE", "TOWARDS_STOP_MARK", "TOWARDS_MAIN_LINE", "JOIN_MAIN_LINE", "LINE_LOST"};

enum direction_t {
  RIGHT,
  LEFT,
};
String junctionOptionStrings[] = {"RIGHT", "LEFT"};

/********************************************
  Variables
*******************************************/
static const int lineFollowPins[NUM_OF_LINE_SENSORS] = {A4, A5, A6, A7, A14}; // Line-follow sensors pinout: #1 - right-most, #5 - left-most
int lineSensorVals[NUM_OF_LINE_SENSORS]; //Line-follow sensors data

// The following arrays hold parameters for navigation on concrete (arr[0]) and on carper (arr[1]):
int diffThr[] = {120, 30}; //threshold that determines whether or not a correction is needed
int stopMarkThr[] = {2530, 3000}; //stop mark detection threshold: a stop mark is detected if the sum of the 3 mid sensors > thr
int maxSensorDiff[] = {750, 150}; //Max allowed diff for correction calculation
int lineNotDetectedThr[] = {300, 50}; //Line not detected if maxDiff < thr
int forwardSpeed[] = {35, 35}; //Nominal forward speed during navigation (motors 2 & 3)
float turnSpeed[] = {30, 30}; //Nominal rotation speed for navigation
int lineSensorOffsets[2][NUM_OF_LINE_SENSORS] = { //sensorVal = analogRead - sensorOffset
  {0, -6, 68, 122, 0},
  {0, 5, 25, 48, 0},
};
int lineThr[2][NUM_OF_LINE_SENSORS] = { //Line detection threshold for each sensor: the sensor is above the line if sensorVal > thr
  {830, 910, 830, 810, 950},
  {890, 920, 900, 845, 920},
};
int minCorr[] = {5, 20}; //min TPS correction for line following 
int maxCorr[] = {200, 250}; //max TPS correction for line following
int junctionStickyThr[] = {25, 50}; //Sticky threshold in ms for junction detection
int stopMarkStickyThr[] = {10, 50}; //Sticky threshold in ms for stop mark
int noLineStickyThr[] = {150, 150}; //Sticky threshold in ms for 'line not detected'

int lineFollowDiff = 0; //difference between sensor #3 and sensor #1
unsigned long noLinePrevMillis = 0; //the first moment that the line wasn't detected
unsigned long carpetFlagChangeMillis = 0; //time stamp for the last surface change
navigationState_t navigationState = HOME;
navigationState_t prevNavigationState;
lineSensorState_t lineSensorState;
direction_t returnDir;
bool navTowardsHomeFlag = false; //indicates if navigation is towards home base
bool carpetFlag = false; //indicates navigation on carpet/concrete
bool stopMarkFlag = false; //stop mark was detected during last sensor read
bool junctionRightFlag = false; //junction right was detected during last sensor read
bool junctionLeftFlag = false; //junction left was detected during last sensor read
bool noLineFlag = false; //no line was detected during last sensor read
unsigned long junctionRightStickyMillis; //time stamp for sensor state sticky mechanism
unsigned long junctionLeftStickyMillis; //time stamp sensor state sticky mechanism
unsigned long stopMarkStickyMillis; //time stamp sensor state sticky mechanism
unsigned long noLineStickyMillis; //time stamp sensor state sticky mechanism
int maxDiff; //max difference between the 3 center sensors
long lastCalibratedTstamp = -1;
int motor1dir;
int motor2dir;
int motor3dir;
int junctionCnt = 0;
unsigned long lastJunctionTstamp = 0;

#ifdef LINE_FOLLOW_DEBUG
char msgbuff[20];
bool linePrintFlag = false;
unsigned long debugPrevMillis = 0;
bool debugPrintFlag;
#endif


/********************************************
  Prototypes
*******************************************/
void lineFollow_calc();
void lineFollow_handler();
void stationActivity(); 
void onHomeArrival();
void onLineLost();
extern bool onMissionFlag;
extern int missionStation;
extern int missionOrientation;

/********************************************
  Local Functions
*******************************************/
/*
  Returns the max absolute difference between the 3 mid sensors
*/
int calcMaxDiff() {
  int maxVal = max(lineSensorVals[1], max(lineSensorVals[2], lineSensorVals[3]));
  int minVal = min(lineSensorVals[1], min(lineSensorVals[2], lineSensorVals[3]));
  return (maxVal - minVal);
}

/*
  Returns 'true' if the sensor is above the line
*/  
bool isSensorOnLine(int sensorId) {
  return (lineSensorVals[sensorId] > lineThr[carpetFlag][sensorId]);
}

/*
  Rotate CW/CCW until the line is above one of the 3 mid sensors
*/
void lineFollow_rotateAndFindLine(bool rotateCW, float pivotFactor = 1.0) {
  unsigned long rotatePrevMillis;
  wantedTps1 = turnSpeed[carpetFlag];
  wantedTps2 = turnSpeed[carpetFlag] * (rotateCW ? pivotFactor : 1.0);
  wantedTps3 = turnSpeed[carpetFlag] * (rotateCW ? 1.0 : pivotFactor);
  int dir = rotateCW ? 1 : -1;
  //Rotate untill none of the 3 center sensors are above the line
  do {
    lineFollow_calc();
    motorControl_pidComputeAndWrite(dir, dir, dir);
  } while (maxDiff > lineNotDetectedThr[carpetFlag]);
  //Rotate untill one of the center sensors is above the line
  rotatePrevMillis = millis();
  do {
    lineFollow_calc();
    motorControl_pidComputeAndWrite(dir, dir, dir);
  } while ( !isSensorOnLine(1) and !isSensorOnLine(2) and !isSensorOnLine(3) and (millis() < rotatePrevMillis + ROTATE_AND_FIND_LINE_TIMEOUT_MS) );
  motorControl_pause();
  if (millis() > rotatePrevMillis + ROTATE_AND_FIND_LINE_TIMEOUT_MS) {
    prevNavigationState = navigationState;
    navigationState = LINE_LOST;
    BT.println(F("LINE_FOLLOW: Line lost during rotation, stopping"));
  }
}

/*
  Rotate CW/CCW to a specific angle and then continue line following
*/
void turnAndGo(bool isRight, float pivotFactor, int blindDeg, int ignoreNavMs = IGNORE_NAV_AFTER_TURN) {
  motorControl_rotateToAngle(isRight, blindDeg, turnSpeed[carpetFlag]);
  lineFollow_rotateAndFindLine(isRight, pivotFactor);
  //Avoid finding a junction again right after turning
  unsigned long ms = millis();
  while (millis() < ms + ignoreNavMs) {
    lineFollow_calc();
    lineFollow_handler();
  }
}

void updateSensorStateFlags(lineSensorState_t state) {
  stopMarkFlag = (state == ON_STOP_MARK);
  junctionRightFlag = (state == JUNCTION_RIGHT);
  junctionLeftFlag = (state == JUNCTION_LEFT);
  noLineFlag = (state == LINE_NOT_DETECTED);
}

/********************************************
  API Functions
*******************************************/
void lineFollow_init() {
  for (int i = 0; i < NUM_OF_LINE_SENSORS; i++)
  pinMode(lineFollowPins[i], INPUT);
}

/*
  Reset variables when reaching home base
*/  
void lineFollow_homeReset() {
  navTowardsHomeFlag = false;
  onMissionFlag = false;
  navigationState = HOME;
  junctionCnt = 0;
}

/*
  Read all line-following sensors
*/
void lineFollow_readSensors() {
  for (int i = 0; i < NUM_OF_LINE_SENSORS; i++) {
    int val = analogRead(lineFollowPins[i]) - lineSensorOffsets[carpetFlag][i];
    lineSensorVals[i] = ( val * LINE_FOLLOW_NEW_VAL_WEIGHT + lineSensorVals[i] * (10 - LINE_FOLLOW_NEW_VAL_WEIGHT) ) / 10;
  }
}

/*
  Calculate line follow state according to sensors' vals
*/
void lineFollow_calc() {
  lineFollowDiff = lineSensorVals[3] - lineSensorVals[1];
  // set line sensor state
  maxDiff = calcMaxDiff();
  bool tmp = (lineSensorVals[0] > BRIGHT_DARK_THR and lineSensorVals[1] > BRIGHT_DARK_THR and lineSensorVals[2] > BRIGHT_DARK_THR and lineSensorVals[3] > BRIGHT_DARK_THR and lineSensorVals[4] > BRIGHT_DARK_THR) ? true : false;
  if (tmp != carpetFlag) {
    carpetFlag = tmp;
    carpetFlagChangeMillis = millis();
  }
  
  /********************************************
    Reached a stop mark
  *******************************************/
  if ( (isSensorOnLine(1) and isSensorOnLine(2) and isSensorOnLine(3)) or 
      (lineSensorVals[1] + lineSensorVals[2] + lineSensorVals[3] > stopMarkThr[carpetFlag]) ) {
      //BT.println("stop mark");
      if (!stopMarkFlag){
      stopMarkStickyMillis = millis();
      updateSensorStateFlags(ON_STOP_MARK);
    }
    else if (millis() > stopMarkStickyMillis + stopMarkStickyThr[carpetFlag]) {
      lineSensorState = ON_STOP_MARK;
    }
  }
  /********************************************
    Perpendicular to line
  *******************************************/
  else if (isSensorOnLine(0) and isSensorOnLine(4)) {
    //BT.println("perpendicular");
    lineSensorState = PERPENDICULAR;
    updateSensorStateFlags(PERPENDICULAR);
  }
  /********************************************
    There's a junction to the right
  *******************************************/
  else if ( isSensorOnLine(4) and (!isSensorOnLine(0)) ) {
    //BT.println("junction right");
    if (millis() > carpetFlagChangeMillis + MS_AFTER_CARPET_FLAG_CHANGE){
      if (!junctionRightFlag){
        junctionRightStickyMillis = millis();
        updateSensorStateFlags(JUNCTION_RIGHT);
      }
      else if (millis() > junctionRightStickyMillis + junctionStickyThr[carpetFlag]) {
        lineSensorState = JUNCTION_RIGHT;
      }
    }
  }
  /********************************************
    There's a junction to the left
  *******************************************/
  else if ( isSensorOnLine(0) and (!isSensorOnLine(4)) ) {
    //BT.println("junction left");
    if (millis() > carpetFlagChangeMillis + MS_AFTER_CARPET_FLAG_CHANGE){
      if (!junctionLeftFlag){
        junctionLeftStickyMillis = millis();
        updateSensorStateFlags(JUNCTION_LEFT);
      }
      else if (millis() > junctionLeftStickyMillis + junctionStickyThr[carpetFlag]) {
        lineSensorState = JUNCTION_LEFT;
      }
    }
  }
  /********************************************
    No line detected
  *******************************************/
  else if (maxDiff < lineNotDetectedThr[carpetFlag]) {
    if (!noLineFlag){
      noLineStickyMillis = millis();
      updateSensorStateFlags(LINE_NOT_DETECTED);
    }
    else if (millis() > noLineStickyMillis + noLineStickyThr[carpetFlag]) {
      lineSensorState = LINE_NOT_DETECTED;
      #ifdef LINE_FOLLOW_DEBUG
        ledPanel_drawText(37, 22, RED, 1, "line");// for debug
        linePrintFlag = true;
      #endif
    }
  }
  /********************************************
    Robot is right of the line
  *******************************************/
  else if ( lineFollowDiff < -diffThr[carpetFlag] ) {
    lineSensorState = RIGHT_OF_LINE;
    updateSensorStateFlags(RIGHT_OF_LINE);
  }
  /********************************************
    Robot is left of the line
  *******************************************/
  else if ( lineFollowDiff > diffThr[carpetFlag] ) {
    lineSensorState = LEFT_OF_LINE;
    updateSensorStateFlags(LEFT_OF_LINE);
  }
  /********************************************
    Robot is on the line
  *******************************************/
  else {
    lineSensorState = ON_LINE;
    updateSensorStateFlags(ON_LINE);
  }
  
  if (!noLineFlag) {
    noLinePrevMillis = millis(); //reset watchdog
    #ifdef LINE_FOLLOW_DEBUG
      if (linePrintFlag) {
        linePrintFlag = false;
        ledPanel_drawText(37, 22, BLACK, 1, "line");// for debug
      }
    #endif
  }
  /********************************************
    Debug printouts
  *******************************************/
#ifdef LINE_FOLLOW_DEBUG
  if (millis() > debugPrevMillis + DEBUG_INTERVAL_MS) {
    debugPrevMillis = millis();
    debugPrintFlag = true;
  }
  else
  debugPrintFlag = false;
  if (debugPrintFlag) {
    //DEBUG_SERIAL.println(F("=== LINE_FOLLOW_DEBUG ===\t"));
    DEBUG_SERIAL.println(String(lineSensorVals[0]) + "\t" + String(lineSensorVals[1]) + "\t" + String(lineSensorVals[2]) + "\t" + String(lineSensorVals[3]) + "\t" + String(lineSensorVals[4]) + "\t");
    //DEBUG_SERIAL.println(String(lineSensorVals[1]) + "\t" + String(lineSensorVals[2]) + "\t" + String(lineSensorVals[3]));
    //DEBUG_SERIAL.println(lineSensorVals[1] + lineSensorVals[2] + lineSensorVals[3]);
    /*
      DEBUG_SERIAL.print(String(lineSensorVals[1]) + "\t");
      DEBUG_SERIAL.print(String(lineSensorVals[3]) + "\t");
      DEBUG_SERIAL.println(String(lineFollowDiff) + "([1] [3] [diff])");
      */
    //DEBUG_SERIAL.println("diff = " + String(lineFollowDiff));
    //DEBUG_SERIAL.println(lineFollowDiff);
    //DEBUG_SERIAL.println("maxDiff = " + String(maxDiff));
    //DEBUG_SERIAL.println("carpetFlag = " + carpetFlag);
    //DEBUG_SERIAL.println("lineSensorState = " + lineSensorStateStrings[lineSensorState]);
    //DEBUG_SERIAL.println("navigationState = " + navigationStateStrings[navigationState]);
    //DEBUG_SERIAL.println(F("=== LINE_FOLLOW_DEBUG END ===\t"));
  }
#endif
}

/*
  Handle line following state - Activate motors to keep on track
*/
void lineFollow_handler() {
  int correction;
  /********************************************
    Right of line
  *******************************************/
  if (RIGHT_OF_LINE == lineSensorState) {
    motor1dir = -1;
    motor2dir = -1;
    motor3dir = 1;
  }
  /********************************************
    Left of line
  *******************************************/
  else if (LEFT_OF_LINE == lineSensorState) {
    motor1dir = 1;
    motor2dir = -1;
    motor3dir = 1;
  }
  /********************************************
    On line
  *******************************************/
  else {
    motor1dir = 1;
    motor2dir = -1;
    motor3dir = 1;
  }
  wantedTps1 = 0;
  wantedTps2 = forwardSpeed[carpetFlag];
  wantedTps3 = forwardSpeed[carpetFlag];
  if(abs(lineFollowDiff) < diffThr[carpetFlag]) {
    correction = 0;
  }
  else {
    int amount = mapConstrain(abs(lineFollowDiff), diffThr[carpetFlag], maxSensorDiff[carpetFlag], minCorr[carpetFlag], maxCorr[carpetFlag]);
    correction = (lineFollowDiff > 0) ? amount : -amount;
  }
  float motor1factor = fscale(diffThr[carpetFlag], maxSensorDiff[carpetFlag], 1.0, 4.0, abs(lineFollowDiff), -8);
  motorControl_pidComputeAndWrite(motor1dir, motor2dir, motor3dir, correction, motor1factor);
}

/*
  Handle navigation state according to sensor state
*/
void navigation_handler() {
  lineFollow_calc();
  /********************************************
    No line detected
  *******************************************/
  if (LINE_NOT_DETECTED == lineSensorState and LINE_LOST != navigationState) {
    if ((millis() - noLinePrevMillis) > LINE_NOT_DETECTED_TIMEOUT_MS) {
      motorControl_pause();
      prevNavigationState = navigationState;
      navigationState = LINE_LOST;
      BT.println(F("LINE_FOLLOW_DEBUG: Line is lost, stopping"));
    }
  }
  
  switch (navigationState) {
  case ON_MAIN_LINE:
    lineFollow_handler();
    /********************************************
        There's a junction to the right/left
      *******************************************/
    if (JUNCTION_RIGHT == lineSensorState or JUNCTION_LEFT == lineSensorState) {
      BT.println(F("LINE_FOLLOW: junction to the right/left"));
      //Avoid counting the same junction more that once
      if (millis() > lastJunctionTstamp + JUNCTION_IGNORE_PERIOD) {
        junctionCnt++;
        lastJunctionTstamp = millis();
      }
      BT.println( "LINE_FOLLOW: junctionCnt = " + String(junctionCnt) + ", missionStation = " + String(missionStation) );
      if (junctionCnt == missionStation) {
        returnDir = (JUNCTION_RIGHT == lineSensorState) ? LEFT : RIGHT;
        turnAndGo(JUNCTION_RIGHT == lineSensorState, PIVOT_FACTOR_90_DEG, 60);
        //Update navigation state
        prevNavigationState = navigationState;
        navigationState = TOWARDS_STOP_MARK;
      }
      else if (VISIT_ALL_STATIONS == missionStation) {
        returnDir = (JUNCTION_RIGHT == lineSensorState) ? RIGHT : LEFT;
        turnAndGo(JUNCTION_RIGHT == lineSensorState, PIVOT_FACTOR_90_DEG, 60);
        //Update navigation state
        prevNavigationState = navigationState;
        navigationState = TOWARDS_STOP_MARK; 
      }
    }
    
    /********************************************
        Reached home/EOL
      *******************************************/
    else if (ON_STOP_MARK == lineSensorState) {
      if (navTowardsHomeFlag) {
        BT.println(F("LINE_FOLLOW: reached home"));
        //Rotate back and stop
        turnAndGo(true, PIVOT_FACTOR_180_DEG, 135, 1250);
        motorControl_pause();
        onHomeArrival();
      }
      else if (NUM_OF_STATIONS == junctionCnt) {
        if (ORIENTATION_BACKWARD == missionOrientation) {
          //Rotate back
          turnAndGo(true, PIVOT_FACTOR_180_DEG, 135, 1250);
        }
        //Stop
        motorControl_pause();
        //Interact with employee at station
        stationActivity(); // EOL is also a station
        ledPanel_clear();
        ledPanel_flashingTextStep("Home\nsweet\nhome...", 120);
        BT.println(F("LINE_FOLLOW_DEBUG: reached EOL"));
        navTowardsHomeFlag = true;
        if (ORIENTATION_FORWARD == missionOrientation) {
          //Rotate back towards home
          turnAndGo(true, PIVOT_FACTOR_180_DEG, 135);
        }
      }
    }
    break;
    
  case TOWARDS_STOP_MARK:
    lineFollow_handler();
    /********************************************
        Reached stop mark
      *******************************************/
    if (ON_STOP_MARK == lineSensorState) {
      BT.println(F("LINE_FOLLOW: reached a stop mark"));
      if (ORIENTATION_BACKWARD == missionOrientation) {
        //Rotate back
        turnAndGo(true, PIVOT_FACTOR_180_DEG, 100, 1250);
      }
      //Stop
      motorControl_pause();
      //Interact with employee at station
      stationActivity();
      ledPanel_clear();
      if (missionStation != VISIT_ALL_STATIONS) {
        navTowardsHomeFlag = true;
        ledPanel_flashingTextStep("Home\nsweet\nhome...", 120);
      }
      else {
        ledPanel_flashingTextStep("On my\nway...", 120);
      }
      //Turn back towards main line
      BT.println(F("LINE_FOLLOW: moving on from stop mark"));
      if (ORIENTATION_FORWARD == missionOrientation)
        turnAndGo(true, PIVOT_FACTOR_180_DEG, 100);
      //Update navigation state
      prevNavigationState = navigationState;
      navigationState = TOWARDS_MAIN_LINE;
    }
    break;
    
  case TOWARDS_MAIN_LINE:
    lineFollow_handler();
    if (LINE_NOT_DETECTED  == lineSensorState) {
      //Update navigation state
      prevNavigationState = navigationState;
      navigationState = JOIN_MAIN_LINE;
      BT.println(F("LINE_FOLLOW: no line detected, driving forward to join main line"));
    }
    break;
    
  case JOIN_MAIN_LINE:
    if (lineSensorState == PERPENDICULAR or lineSensorState == JUNCTION_LEFT or lineSensorState == JUNCTION_RIGHT) {
      BT.println(F("LINE_FOLLOW: joining main line"));
      //Turn right or left on to main line
      turnAndGo(RIGHT == returnDir, PIVOT_FACTOR_90_DEG, 45);
      //Update navigation state
      prevNavigationState = navigationState;
      navigationState = ON_MAIN_LINE;
    }
    break;
    
  case LINE_LOST:
    onLineLost();
  } //switch(navigationState)
}

void lineFollow_calibrate() {
  unsigned long lineSensorSums[NUM_OF_LINE_SENSORS] = {0,};
  unsigned int sampleCnt = 0;
  unsigned long start = encoderCnt1;
  wantedTps1 = 100;
  wantedTps2 = 100;
  wantedTps3 = 100;
  while((encoderCnt1 - start) < (3.43 * ENCODER_CPR) ) {
    motorControl_pidComputeAndWrite(1, 1, 1);
    for(int j = 0; j < NUM_OF_LINE_SENSORS; j++) {
      lineSensorSums[j] += analogRead(lineFollowPins[j]);
    }
    sampleCnt++;
    delay(1);
  }
  motorControl_pause();
  //Calculate sensor offsets relative to mid sensor (sensor2)
  DEBUG_SERIAL.println("sampleCnt = " + String(sampleCnt));
  for(int j = 0; j < NUM_OF_LINE_SENSORS; j++) {
    DEBUG_SERIAL.println("lineSensorSums[" + String(j) + "] = " + String(lineSensorSums[j]));
    lineSensorOffsets[carpetFlag][j] = (lineSensorSums[j] / sampleCnt) - (lineSensorSums[2] / sampleCnt);
    #ifdef LINE_FOLLOW_DEBUG
    DEBUG_SERIAL.println("lineSensorOffsets[carpetFlag][" + String(j) + "] = " + String(lineSensorOffsets[carpetFlag][j]));
    #endif
  }
  lastCalibratedTstamp = millis();
}

long lineFollow_lastCalibrated() {
  return lastCalibratedTstamp;
}

#endif
