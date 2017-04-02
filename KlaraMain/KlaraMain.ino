
/********************************************
   Includes
 *******************************************/
#include <KlaraUtils.h>
#include <KlaraAudio.h>
#include <KlaraArm.h>
#include <KlaraRFID.h>
#include <KlaraMotorControl.h>
#include <KlaraLedPanel.h>
#include <KlaraBasketball.h>
#include <KlaraBluetooth.h>
#include <KlaraBricks.h>
#include <KlaraLineFollow.h>
#include <KlaraUltrasonic.h>
#include <KlaraLedStrip.h>
#include <KlaraRTC.h>
#include <KlaraBattery.h>
#include <KlaraBricks.h>
#include <KlaraInternet.h>
#include <KlaraDelivery.h>

/********************************************
   Defines
 *******************************************/
#define MIN_DANCE_RHYTHM 75

/********************************************
   Variables
 *******************************************/
int returnVal = 0;
DateTime rightNow;
bool obstacleFlag = false;
bool onMissionFlag = false;
bool manualControlFlag = false;
int missionStation;
int missionOrientation;
action_t missionAction;
int missionParam1;
int missionParam2;
int missionParam3;
char charArr[10];
int led13State = 0;

/********************************************
  Main
*******************************************/
void setup() {
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
  internet_init();
  rtc_init();
  motorControl_init();
  lineFollow_init();
  audio_init();
  ledStrip_init();
  bluetooth_init();
  ledPanel_init();
  rfid_init();
  ultrasonic_init();
  arm_init();
  timer3_init();
}

void loop() {
  led13State = !led13State;
  digitalWrite(LED_BUILTIN, led13State);
  /********************************
    Line follow & navigation
  *******************************/
  if (onMissionFlag and !obstacleFlag)
    navigation_handler();
  /********************************
    Ultrasonic
  *******************************/
  //if (onMissionFlag)
  //ultrasonic_handler(); //reading is not stable, causes robot to stop for no reason
  /********************************
    LEDs (panel + strip)
  *******************************/
  ledStrip_rainbowStep(50);
  if (HOME == navigationState)
    ledPanel_hiKlaraStep();
  else
    //ledPanel_circlesUpdate(); //causes problems with navigation
    ;
  /******************************
    Bluetooth Manual Control
  *******************************/
  if (HOME == navigationState or LINE_LOST == navigationState) {
    if ('S' == bluetooth_getChar())
      bluetooth_manualMode();
  }
  /********************************
    Real Time Clock
  *******************************/
  if (HOME == navigationState) {
    rightNow = rtc_getTime();
  }
  /********************************
    Battery Monitor
  *******************************/
  // do not comment this code! otherwise battery may... explode
  /*
  if (HOME == navigationState) {
    batteryLevel = battery_getVoltage();
    battery_handler();
  }
  */
  
  /********************************
    Internet
  *******************************/
  if (HOME == navigationState)
    internet_poll();
}

/********************************************
   Functions
*******************************************/
void onRfid(unsigned long rfidUid) {}

void timer3_init() {
  Timer3.initialize(2500); //microseconds
  Timer3.attachInterrupt(timer3_isr);
}

void timer3_isr() {
  encoderStallCheck();
  lineFollow_readSensors();
}

void ultrasonic_handler() {
  ultrasonic_updateDist();
  if (distArr[0] < STOP_THR) {
    //obstacle in front
    if (!obstacleFlag) {
      motorControl_pause();
      obstacleFlag = true;
      BT.println("Obstacle ahead, stopping");
    }
  }
  else if (obstacleFlag) {
    obstacleFlag = false;
    motorControl_resume();
  }
}

void accept_mission(int station, int orientation, int action, int param1, int param2, int param3) {
  //check condition to accept mission (no mission is running, not in manual mode, battery not low)
  if (!onMissionFlag and !manualControlFlag and battery_getVoltage() > MIN_VOLTAGE_FOR_MISSION) {
    if ( (VISIT_ALL_STATIONS == station) or (VISIT_USER_STATIONS == station) or (station <= NUM_OF_STATIONS and station >= 0) )  {
      missionAction = (action_t)action;
      missionOrientation = constrain(orientation, 0, 1);
      missionParam1 = param1;
      missionParam2 = param2;
      missionParam3 = param3;
      if (0 == station) {
        stationActivity();
      }
      else {
        if(action == ANNOUNCE) {
          char* panelStr = getMsg(param1);
          ledPanel_clear();
          ledPanel_flashingTextStep(panelStr, 120);
          audio_play(1, param2);
        }

        onMissionFlag = true;
        navigationState = ON_MAIN_LINE;
        missionStation = station;
        missionOrientation = orientation;

        if(action != ANNOUNCE) {
          ledPanel_clear();
          ledPanel_flashingTextStep("On my\nway...", 120);
        }

        if (missionAction == DELIVERY) {
          setupDelivery();
        }

        ledPanel_clear();
        ledPanel_flashingTextStep("On my\nway...", 120);
      }
    }
    else {
      //mission invalid
      BT.println("Invalid mission, aborting");
      Serial.println("Invalid mission, aborting");
    }
  }
}

void stationActivity() {
  switch (missionAction) {
    case BASKETBALL: //1
      basketball_play(missionParam1);
      break;
    case BRICKS: //2
      bricks_play();
      ledPanel_clear();
      bluetooth_pleaseDisconnect();
      break;
    case AUDIO: //3
      {
        audio_play(1, missionParam1);
        missionParam2 = constrain(missionParam2, 5, 255);
        unsigned long start = millis();
        while (millis() < start + 1000 * missionParam2)
          ledPanel_circlesUpdate();
        ledPanel_clear();
        break;
      }
    case DANCE: //4
      dance(missionParam1, missionParam2);
      break;
    case LED_PANEL: //5
      ledPanel_clear();
      displayMsg(missionParam1, missionParam2, missionParam3);
      ledPanel_clear();
      break;
    case PARTY: //6
      ledPanel_clear();
      audio_play(1, missionParam1);
      danceWithMsg(missionParam2, 100, missionParam3, 150);
      audio_fadeout(3000);
      ledPanel_clear();
      break;
    case DELIVERY: //7
      handle_delivery();
    case ANNOUNCE: //8
      delay(2000);
      break;
    case INTERVIEW: //9
      welcomeInterviewee();
      break;
  }
}

void battery_handler() {
  while (batteryLevel < MIN_VOLTAGE_FOR_MISSION) {
    if (millis() % 3000 < 2200)
      ledPanel_clear();
    else {
      sprintf(charArr, "%d.%dV", int(batteryLevel), (int(batteryLevel * 10)) % 10);
      ledPanel_drawText(0, 1, LIGHT_BLUE, 1, charArr);
      ledPanel_drawText(0, 9, LIGHT_BLUE, 1, "Charge me");
    }
    batteryLevel = battery_getVoltage();
  }
}

void onLineLost() {
  motorControl_pause();
  ledPanel_clear();
  ledPanel_drawText(0, 4, GRAY, 1, "Take me\nhome");
  //Wait for BT connection
  bluetooth_waitForCmd('S', 0);
  bluetooth_manualMode();
  //When BT is disconnected we asume we're home
  onHomeArrival();
}

void onHomeArrival() {
  lineFollow_homeReset();
  audio_stop();
  ledPanel_clear();
}

void dance(int duration, int rhythm) {
  unsigned long start = millis();
  rhythm = max(rhythm << 2, MIN_DANCE_RHYTHM);
  while (millis() < start + duration * 1000) {
    motorControl_rotateCW(120);
    arm_setServoAngle(0, 110);
    arm_setServoAngle(1, 110);
    arm_setServoAngle(2, 110);
    delay(rhythm);
    motorControl_rotateCCW(120);
    arm_setServoAngle(0, 30);
    arm_setServoAngle(1, 40);
    arm_setServoAngle(2, 70);
    delay(rhythm);
  }
  motorControl_pause();
  arm_resetPosition();
}

void displayMsg(int msgId, int duration, int colorchangeInterval) {
  char* panelStr = getMsg(msgId);
  ledPanel_clear();
  unsigned long delayTime = constrain(duration, 5, 60) * 1000;
  unsigned long start = millis();
  while (millis() < start + delayTime)
    ledPanel_flashingTextStep(panelStr, constrain(colorchangeInterval, 50, 255));
  ledPanel_clear();
}

void danceWithMsg(int duration, int rhythm, int msgId, int colorchangeInterval) {
  char* panelStr = getMsg(msgId);
  unsigned long start = millis();
  unsigned long start2;
  rhythm = max(rhythm << 2, MIN_DANCE_RHYTHM);
  while (millis() < start + duration * 1000) {
    motorControl_rotateCW(120);
    arm_setServoAngle(0, 110);
    arm_setServoAngle(1, 110);
    arm_setServoAngle(2, 110);
    start2 = millis();
    while (millis() < start2 + rhythm) {
      ledPanel_flashingTextStep(panelStr, constrain(colorchangeInterval, 50, 255));
    }
    motorControl_rotateCCW(120);
    arm_setServoAngle(0, 30);
    arm_setServoAngle(1, 40);
    arm_setServoAngle(2, 70);
    start2 = millis();
    while (millis() < start2 + rhythm) {
      ledPanel_flashingTextStep(panelStr, constrain(colorchangeInterval, 50, 255));
    }
  }
  motorControl_pause();
  arm_resetPosition();
}

char* getMsg(int msgId) {
  switch (msgId) {
    case 1:
      return "Welcome\nto\nKlarna :)";
    case 2:
      return "Happy\nB-day !";
    case 3:
      return "Whasssup ?";
      break;
    case 4:
      return "Everybody\nwants 2B\n  A PANDA";
      break;
    case 5:
    {
      int r = random(4);
      switch(r) {
        case 0:
          return "The Answer\n  Is 42";
        case 1:
          return "I was,\n  Once,\n Like You";
        case 2:
          return "Trust me,\n I'm A\n  Robot";
        case 3:
          return "42 Is\n   The\n  Answer";
      }
    }
      return "";
      break;
    case 6:
      return "Waiting \nfor\npackage";
      break;
    case 7:
      return "I'm about\nto drop\n it!!";
      break;
    case 8:
      return "What about \nmy tip?!?!";
      break;
    case 9:
      return "FIKA TIME!";
      break;
    case 10:
      return "";
      break;
    case 11:
      return "";
      break;
    case 12:
      return "";
      break;
    case 99:
      return "You are\nfired !!!";
    default:
      return "wrong\nmsg id";
  }
}

void welcomeInterviewee()
{
  unsigned long start = millis();
  while(millis() < start + 2000)
  {
    lineFollow_handler();
  }

  //  motorControl_rotateToAngle(true, 180, turnSpeed[false]);
  lineFollow_rotateAndFindLine(true);

  ultrasonic_updateDist();
  int start_distance = distArr[0];

   motorControl_driveForward();
   do {
     ultrasonic_updateDist();
   } while (distArr[0] > 35); //TODO
   motorControl_pause();

  arm_resetPosition();
  arm_setServoAngle(2, 20);
  for (size_t i = 0; i < 3; i++) {
    arm_setServoAngle(1, 45);
    delay(500);
    arm_setServoAngle(1, 90);
    delay(500);
  }
  arm_resetPosition();

  // danceWithMsg(10, 100, 5, 150);

  motorControl_driveBackward();
  do {
    ultrasonic_updateDist();
  } while (distArr[0] < start_distance);
  motorControl_pause();

  // Go back to home
  lineFollow_rotateAndFindLine(true);
}
