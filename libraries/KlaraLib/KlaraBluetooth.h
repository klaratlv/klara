/*
   This header file adds support for bluetooth connectivity
*/
#ifndef BLUETOOTH_H
#define BLUETOOTH_H

/********************************************
   Defines
 *******************************************/
//#define BT_DEBUG
#define BT Serial2
#define BT_DEVICE "HC-06" //choose BT device: "HC-05" or "HC-06"
#define BT_BAUD_RATE 9600
#define BT_DATA_TIMEOUT 40 //timeout for waiting for BT data, must be less than 50ms which is the rate 'BT RC Con troller' app sends data
#define AtCommandDelay 2000
#define keyPin 34
#define statePin 35
#define MANUAL_MODE_TIMEOUT 1000 //exit manual mode if no input received for this long

/********************************************
   Variables
 *******************************************/
String btCmd;
char btChar;
byte charHandlerMode = 0; //0 - drive mode , 1 - arm mode
bool manualModeFlag = false;

/********************************************
   Externals
 *******************************************/
void stationActivity();
extern action_t missionAction;
extern int missionParam1;

/********************************************
   Local Functions
 *******************************************/

/********************************************
   API Functions
 *******************************************/
void bluetooth_init() {
  BT.begin(BT_BAUD_RATE);
  BT.setTimeout(BT_DATA_TIMEOUT);
  
  /*  Un-comment the following 2 lines once in case the BT module is replaced to change the BT device name
  Afterwards re-comment thses lines */
  //  delay(AtCommandDelay);
  //  BT.println(F("AT+NAMEKlara"));

  /* Un-comment to change the bluetooth baud rate */
    //delay(AtCommandDelay);
    //BT.println(F("AT+BAUD7")); //115200
    //delay(AtCommandDelay);

  if (BT_DEVICE == "HC-05") {
    pinMode(keyPin, OUTPUT);
    pinMode(statePin, INPUT);
    digitalWrite(keyPin, HIGH);
  }
}

/*
   Read a '\n' terminated string command via bluetooth
   Stops reading after BT_DATA_TIMEOUT ms in case '\n' was not received
   returns 0 if command was read successfuly
   return 1 if there's nothing to read
   return 2 if the command is empty (command length = 0)
*/
int bluetooth_getCmd() {
  if (BT.available()) {
    btCmd = BT.readStringUntil('\n');
    btCmd.trim();
    if (btCmd.length() > 0) {
#ifdef BT_DEBUG
      BT.print(F("BT_DEBUG: received "));
      BT.println(btCmd);
#endif
      return 0;
    }
    else return 2; //data length = 0
  }
  else return 1; //no BT data
}

/*
   wait for a bluetooth command
   returns 0 if the command was received successfuly
   returns 1 if timeout occured before the command was received
   input:
   cmd - command to wait for
   timeout - time to wait in ms, 0 for indefinite wait
*/
int bluetooth_waitForCmd(String cmd, int timeout) {
  unsigned long startMillis = millis();
  while (millis() < startMillis + timeout or timeout == 0) {
    if (0 == bluetooth_getCmd())
      if (btCmd == cmd)
        return 0;
  }
  return 1;
}

/*
   Read a char via bluetooth
   returns a char that was read from the BT buffer if buffer is not empty
   return 0 if there's no char to read
*/
char bluetooth_getChar() {
  if (BT.available()) {
    char c = BT.read();
    #ifdef BT_DEBUG
      if (c != 'S') {
        BT.print(F("BT_DEBUG: received "));
        BT.println(c);
        Serial.print(F("BT_DEBUG: received "));
        Serial.println(c);
      }
    #endif
    return c;
  }
  else return 0; //no BT data
}

/*
   wait for a bluetooth char
   returns 0 if char received successfuly
   returns 1 if timeout occured before the char was received
   input:
   c - char to wait for
   timeout - time to wait in ms, 0 for indefinite wait
*/
int bluetooth_waitForCmd(char c, int timeout) {
  unsigned long startMillis = millis();
  while (millis() < startMillis + timeout or timeout == 0) {
    if (bluetooth_getChar() == c)
      return 0;
  }
  return 1;
}

/*
   Handler for a single char BT command
   
   Bluetooth RC Car app - Supported commands:
   S - stop
   F - forward
   B - backward
   L - left
   R - right
   G - forward left
   I - forward right
   H - backward left
   J - backward right
   W - front lights on
   w - front lights off
   U - back lights on
   u - back lights off
   V - horn on
   v - horn off
   X - alert on
   x - alert off
   0-9 - speed select
   q - max speed
   
    btChar | charHandlerMode | Action
    -------------------------------------------
       X   |      x          | charHandlerMode = 1
       x   |      x          | charHandlerMode = 0
       
       S   |      x          | Pause motors
       F   |      0          | Drive Forward
       B   |      0          | Drive Backward       
       L   |      0          | Drive Left       
       R   |      0          | Drive Right
       G   |      0          | Drive CW       
       I   |      0          | Drive CCW
       W   |      0          | Play basketball
       V   |      0          | Dance
       v   |      0          | Dance
       
       W   |      1          | Pump ON
       w   |      1          | Pump OFF
       F   |      1          | Move servo2 backward
       B   |      1          | Move servo2 forward
       G   |      1          | Move servo1 backward
       I   |      1          | Move servo1 forward
       L   |      1          | Move servo0 forward
       R   |      1          | Move servo0 backward 
*/
void bluetooth_charHandler(char btChar) {
  switch (btChar) {
    case 'X':
      charHandlerMode = 1;
      break;
    case 'x':
      charHandlerMode = 0;
      break;
    case 'S':
      motorControl_pause();
      break;
      case 'V':
        if (0 == charHandlerMode) {
          missionAction = DANCE;
          //missionParam1 = 10;
          stationActivity();
        }
        break;
    case 'W':
      if (0 == charHandlerMode) {
        Serial.println("Basketball !"); //for debug
        basketball_play(0);
      }
      else {
        Serial.println("Pump ON"); //for debug
        arm_pumpActivate(true);
      }
      break;
    case 'w':
      if (1 == charHandlerMode) {
        Serial.println("Pump OFF"); //for debug
        arm_pumpActivate(false);
      }
    case 'F':
      if (0 == charHandlerMode) {
        Serial.println("Forward"); //for debug
        motorControl_driveForward();
      }
      else {
        if (armServoAngles[2] > ARM_SERVO_2_MIN_ANGLE)
          armServoAngles[2]--;
        Serial.println("Servo 2 angle = " + String(armServoAngles[2])); //for debug
        arm_setServoAngle(2, armServoAngles[2]);
      }
      break;
    case 'B':
      if (0 == charHandlerMode) {
        Serial.println("Backward"); //for debug
        motorControl_driveBackward();
      }
      else {
        if (armServoAngles[2] < ARM_SERVO_2_MAX_ANGLE)
          armServoAngles[2]++;
        Serial.println("Servo 2 angle = " + String(armServoAngles[2])); //for debug
        arm_setServoAngle(2, armServoAngles[2]);
      }
      break;
    case 'I':
      if (0 == charHandlerMode) {
        Serial.println("Right"); //for debug
        motorControl_driveRight(motorSpeed[0]/4, 0);
      }
      else {
        if (armServoAngles[1] < ARM_SERVO_1_MAX_ANGLE)
          armServoAngles[1]++;
        Serial.println("Servo 1 angle = " + String(armServoAngles[1])); //for debug
        arm_setServoAngle(1, armServoAngles[1]);
      }
      break;
    case 'G':
      if (0 == charHandlerMode) {
        Serial.println("Left"); //for debug
        motorControl_driveLeft(motorSpeed[0]/4, 0);
      }
      else {
        if (armServoAngles[1] > ARM_SERVO_1_MIN_ANGLE)
          armServoAngles[1]--;
        Serial.println("Servo 1 angle = " + String(armServoAngles[1])); //for debug
        arm_setServoAngle(1, armServoAngles[1]);
      }
      break;
    case 'R':
      if (0 == charHandlerMode) {
        Serial.println("CW"); //for debug
        motorControl_rotateCW(70); 
      }
      else {
        if (armServoAngles[0] > ARM_SERVO_0_MIN_ANGLE)
          armServoAngles[0]--;
        Serial.println("Servo 0 angle = " + String(armServoAngles[0])); //for debug
        arm_setServoAngle(0, armServoAngles[0]);
      }
      break;
    case 'L':
      if (0 == charHandlerMode) {
        Serial.println("CCW"); //for debug
        motorControl_rotateCCW(70);
      }
      else {
        if (armServoAngles[0] < ARM_SERVO_0_MAX_ANGLE)
          armServoAngles[0]++;
        Serial.println("Servo 0 angle = " + String(armServoAngles[0])); //for debug
        arm_setServoAngle(0, armServoAngles[0]);
      }
      break;
  }
}

void bluetooth_manualMode() {
    unsigned long tStamp = millis();
    char c;
    ledPanel_clear();
    ledPanel_drawText(0, 1, LIGHT_BLUE, 1, "Manual\nMode\nbaby !");
    while (millis() < tStamp + MANUAL_MODE_TIMEOUT) {
      c = bluetooth_getChar();
      if (c != 0) {
        tStamp = millis();
        bluetooth_charHandler(c);
      }
    }
    ledPanel_clear();
    //internet_reset(); //reset internet requests that were received while in manual mode
}

void bluetooth_pleaseDisconnect() {
    unsigned long tStamp = millis();
    char c;
    ledPanel_drawText(0, 1, LIGHT_BLUE, 1, "Please\ndisconnect\nbluetooth");
    while (millis() < tStamp + 100) {
       c = bluetooth_getChar();
       if (c != 0)
        tStamp = millis();
    }
    ledPanel_clear();
    while (BT.available()) {
        BT.read();
    }
}
  
  
#endif
