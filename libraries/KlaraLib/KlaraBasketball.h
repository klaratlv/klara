/*
   This header file adds support for Klara's basketball game
*/

#ifndef BASKETBALL_H
#define BASKETBALL_H

/********************************************
   Includes
 *******************************************/

/********************************************
   Defines
 *******************************************/
//#define BASKET_DEBUG
#define BASKET_PIN A10
#define BASKET_THR_MARGIN 45
#define BASKET_TIMEOUT 200
#define BASKET_MIN_GAME_DURATION 20
#define BASKET_MAX_GAME_DURATION 120
#define BASKETBALL_ADAPTIVE_THR_SAMPLES 30

/********************************************
   Variables
 *******************************************/
int score = 0;
int playFlag;
long timeLeftMs; //should not be unsigned due to game end condition
int basketballThr;
unsigned long timeLeftDisplayPrevMillis = 0;
extern char charArr[];

/********************************************
   Local Functions
 *******************************************/
/*
   Display the time left to play on LED panel
*/
void timeLeftDisplay() {
  if (millis() > timeLeftDisplayPrevMillis + 1000) {
    timeLeftDisplayPrevMillis = millis();
    int x0 = 7;
    int y0 = 20;
    matrix.fillRect(x0, y0, 64 - x0, 7, BLACK); // Clear screen
    matrix.setTextColor(LIGHT_BLUE);
    matrix.setCursor(x0, y0);
    matrix.println("Time: " + String(timeLeftMs / 1000));
  }
}

/*
   Display the current score on LED panel
*/
void scoreDisplay() {
  int x0 = 7;
  int y0 = 6;
  matrix.fillRect(x0 - 2, y0, 64, 9, BLACK); // Clear screen
  ledPanel_setColor(random(10));
  matrix.setCursor(x0, y0);
  matrix.println("Score: " + String(score));
}

void finalScoreDisplay() {
  audio_play(2, 3); //air horn
  delay(1500);
  audio_play(2, 5); //crowd cheering
  int tmp = random(10);
  int colors[3] = {tmp, tmp + 3, tmp + 6};
  for (int i = 0; i < 8; i++) {
    matrix.fillScreen(BLACK); // Clear screen
    ledPanel_setColor(colors[0]);
    matrix.setCursor(18, 3);
    matrix.println(F("Final"));
    ledPanel_setColor(colors[1]);
    matrix.setCursor(18, 13);
    matrix.println(F("score"));
    ledPanel_setColor(colors[2]);
    if (score < 10)
      matrix.setCursor(29, 23);
    else
      matrix.setCursor(26, 23);
    matrix.println(score);
    tmp = colors[2];
    colors[2] = colors[1];
    colors[1] = colors[0];
    colors[0] = tmp;
    delay(800);
  }
}

void shoot321() {
  for (int i = 3; i > 0; i--) {
    matrix.fillScreen(BLACK); // Clear screen
    ledPanel_setColor(i * 3);
    matrix.setCursor(30, 11);
    matrix.println(i);
    delay(1000);
  }
  matrix.fillScreen(BLACK); // Clear screen
  ledPanel_setColor(4);
  matrix.setCursor(6, 7);
  matrix.println(F("SHOOT !!!"));
  audio_play(2, 3); //air horn
}

bool pleasePleaseConnect() {
  ledPanel_clear();
  bool startFlag = false;
  int timeToWait = 15900;
  unsigned long start = millis();
  while (millis() < start + timeToWait and !startFlag) {
    ledPanel_drawText(0, 1, LIGHT_BLUE, 1, "Place rfid\ntag to\nstart");
    ledPanel_drawText(46, 18, BLACK, 1, charArr);
    sprintf(charArr, "%2d", (start + timeToWait - millis()) / 1000);
    ledPanel_drawText(46, 18, LIGHT_BLUE, 1, charArr);
    if (rfid_isCardPresent())
      startFlag = true;
  }
  ledPanel_clear();
  return startFlag;
}

/********************************************
   API Functions
 *******************************************/
/*
   Set adaptive threshold
*/
void basketball_init() {
  unsigned long sum = 0;
  for (int i = 0; i < BASKETBALL_ADAPTIVE_THR_SAMPLES; i++) {
    sum += analogRead(BASKET_PIN);
    delay(1); //for ADC
  }
  basketballThr = (sum / BASKETBALL_ADAPTIVE_THR_SAMPLES) - BASKET_THR_MARGIN;
  Serial.println("BASKET_DEBUG: Basketball sensor average val = " + String(sum / BASKETBALL_ADAPTIVE_THR_SAMPLES)); //for debug
  Serial.println("BASKET_DEBUG: Basketball threshold margin = " + String(BASKET_THR_MARGIN)); //for debug
  Serial.println("BASKET_DEBUG: Setting basketball sensor threshold = " + String(basketballThr)); //for debug
}

/*
   Check condition for starting a basketball game
   We are looking for someone to wave their hand against the sensor,
   so we expect the sensor reading to significantly cross the THR
*/
bool basketball_checkStartCondition() {
  return (analogRead(BASKET_PIN) < basketballThr - 10);
}

/*
   Play basketball !
*/
void basketball_play(int gameTime = BASKET_MIN_GAME_DURATION) {
  basketball_init();
  if (false == pleasePleaseConnect()) return;
  gameTime = constrain(gameTime, BASKET_MIN_GAME_DURATION, BASKET_MAX_GAME_DURATION);
  playFlag = HIGH;
  unsigned long prevShotMillis = 0;
  unsigned long startMillis;
  shoot321();
  timeLeftMs = gameTime * 1000;
  startMillis = millis();
  score = 0;
  do {
    if (millis() > prevShotMillis + BASKET_TIMEOUT) {
      int sensorVal = analogRead(BASKET_PIN);
      delay(1); //for ADC
      #ifdef BASKET_DEBUG
          Serial.println("BASKET_DEBUG: timeLeftMs = " + String(timeLeftMs)); //for debug
        //Serial.println("BASKET_DEBUG: sensorVal = " + String(sensorVal)); //for debug
      #endif
      timeLeftMs = (gameTime * 1000) - (millis() - startMillis); //update time left to play
      timeLeftDisplay();
      if (sensorVal < basketballThr) {
        #ifdef BASKET_DEBUG
          Serial.println("BASKET_DEBUG: sensorVal = " + String(sensorVal)); //for debug
        #endif
        prevShotMillis = millis();
        score++;
        scoreDisplay();
        audio_play(2, 6); //short crowd cheering
      }
    }
    // exit conditions
    if (timeLeftMs <= 0)
      playFlag = LOW;
  } while (playFlag);
  finalScoreDisplay();
  matrix.fillScreen(BLACK); // Clear screen
}


#endif
