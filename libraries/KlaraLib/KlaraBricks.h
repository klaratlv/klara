
#ifndef BRICKS_H
#define BRICKS_H

/********************************************
   Includes
 *******************************************/

/********************************************
   Defines
 *******************************************/
#define NUM_OF_BRICKS 7
#define MAX_SPEED 10
#define BRICKS_START_TIMEOUT_MS 20000 //time to wait for player to initiate game
#define BRICKS_IDLE_TIMEOUT 10000

/********************************************
   Variables
 *******************************************/
int paddleX;
int paddleY;
int oldPaddleX;
int oldPaddleY;
int paddleWidth;
int paddleHeight;
int ballX;
int ballY;
int oldBallX;
int oldBallY;
int ballDirectionX;
int ballDirectionY ;
int ballSpeed;
int updateBallMs;
int bricksX[NUM_OF_BRICKS] = {3, 12, 21, 30, 39, 48, 57};
int bricksY[NUM_OF_BRICKS] = {1, 3, 1, 3, 1, 3, 1};
int brickWidth;
int brickHeight;
int activeBricks;
boolean ballInboundsFlag ;
unsigned long prevBallMillis;
unsigned long prevInputMillis;
extern char charArr[];

/********************************************
   Local Functions
 *******************************************/
void resetVars() {
  paddleX = matrix.width() / 2 - 2;
  paddleY = matrix.height() - 1;
  oldPaddleX = 0;
  oldPaddleY = 0;
  paddleWidth = 5;
  paddleHeight = 1;
  ballX = matrix.width() / 2;
  ballY = matrix.height() - 3;
  oldBallX = ballX;
  oldBallY = ballY;
  ballDirectionX = 1;
  ballDirectionY = -1;
  ballSpeed = 3; //in the range [1,MAX_SPEED]
  updateBallMs = 75 - 5 * ballSpeed;
  brickWidth = 5;
  brickHeight = 2;
  activeBricks = NUM_OF_BRICKS;
  ballInboundsFlag = true;
  prevBallMillis = 0;
  prevInputMillis = millis();
  int x0 = 3;
  int y0 = 1;
  for (int i = 0; i < NUM_OF_BRICKS; i++) {
    bricksX[i] = x0;
    bricksY[i] = y0;
    x0 += 18;
    if (x0 >= matrix.width() - brickWidth) {
      if (y0 % 2)
        x0 = 12;
      else
        x0 = 3;
      y0 += 3;
    }
  }
}

boolean inRect(int x, int y, int rectX, int rectY, int rectWidth, int rectHeight) {
  boolean result = false;
  if (x >= rectX and x <= (rectX + rectWidth) and y >= rectY and y <= (rectY + rectHeight))
    return true;
  else
    return result;
}

void playFaster() {
  if (ballSpeed < MAX_SPEED)
    ballSpeed++;
}

void updateBall() {
  if (millis() > prevBallMillis + updateBallMs) {
    prevBallMillis = millis();
    // if the ball goes offscreen, reverse the direction:
    if (ballX > matrix.width() - 1 || ballX < 0)
      ballDirectionX = -ballDirectionX;
    if (ballY < 0)
      ballDirectionY = -ballDirectionY;
    if (ballY > matrix.height() - 1) {
      ballInboundsFlag = false;
      return;
    }
    // check if the ball and the paddle occupy the same space on screen
    if (inRect(ballX, ballY, paddleX, paddleY, paddleWidth, paddleHeight)) {
      ballDirectionY = -ballDirectionY;
      int paddleMidX = paddleX + paddleWidth / 2;
      //if ball hits middle of paddle, it goes straight up
      if (inRect(ballX, ballY, paddleMidX, paddleY, 0, paddleHeight))
        ballDirectionX = 0;
      else if (ballX > paddleMidX)
        ballDirectionX = 1;
      else
        ballDirectionX = -1;
    }
    // update the ball's position
    ballX += ballDirectionX;
    ballY += ballDirectionY;
    // erase the ball's previous position
    if (oldBallX != ballX || oldBallY != ballY) {
      matrix.drawPixel(oldBallX, oldBallY, BLACK);
    }
    // draw the ball's current position
    matrix.drawPixel(ballX, ballY, GRAY);

    oldBallX = ballX;
    oldBallY = ballY;
  }
}

void drawBricks() {
  for (int i = 0; i < NUM_OF_BRICKS; i++)
    matrix.fillRect(bricksX[i], bricksY[i], brickWidth, brickHeight, LIGHT_GREEN);
}

void updateBricks() {
  for (int i = 0; i < NUM_OF_BRICKS; i++) {
    if (inRect(ballX, ballY, bricksX[i], bricksY[i], brickWidth, brickHeight)) {
      audio_play(2, 8); //explosion
      matrix.fillRect(bricksX[i], bricksY[i], brickWidth, brickHeight, BLACK);
      bricksX[i] = -99;
      bricksY[i] = -99;
      activeBricks--;
      //playFaster();
    }
  }
}

void getInput() {
  char cmd = bluetooth_getChar();
  if (cmd != 0 and cmd != 'S') {
    prevInputMillis = millis();
    switch (cmd) {
      case 'L':
        if (paddleX > 0)
          paddleX--;
        break;
      case 'R':
        if (paddleX < matrix.width() - 1)
          paddleX++;
        break;
      case 'F':
        if (paddleY > matrix.height() / 4)
          paddleY--;
        break;
      case 'B':
        if (paddleY < matrix.height() - 1)
          paddleY++;
        break;
    }
  }
}

void updatePaddle() {
  // erase the paddle's previous position
  if (oldPaddleX != paddleX || oldPaddleY != paddleY) {
    matrix.fillRect(oldPaddleX, oldPaddleY, paddleWidth, paddleHeight, BLACK);
  }
  // draw the paddle's current position
  matrix.fillRect(paddleX, paddleY, paddleWidth, paddleHeight, LIGHT_BLUE);

  oldPaddleX = paddleX;
  oldPaddleY = paddleY;
}

void gameOver() {
  ledPanel_clear();
  ledPanel_drawText(4, 10, GRAY, 1, "GAME OVER");
  audio_play(2, 7); //wa wa wa
  delay(2000);
  ledPanel_clear();
}

void victory() {
  ledPanel_clear();
  ledPanel_drawText(4, 10, LIGHT_GREEN, 1, "VICTORY !");
  audio_play(2, 6); //crowd cheer
  delay(2000);
  ledPanel_clear();
}

bool pleaseConnect() {
  ledPanel_clear();
  bool startFlag = false;
  unsigned long timeToConnect = 39900;
  unsigned long start = millis();
  while (millis() < start + timeToConnect) {
    ledPanel_drawText(0, 1, LIGHT_BLUE, 1, "Connect\nvia BT\nto play");
    ledPanel_drawText(48, 18, BLACK, 1, charArr);
    sprintf(charArr, "%2d", (start + timeToConnect - millis()) / 1000);
    ledPanel_drawText(48, 18, LIGHT_BLUE, 1, charArr);
    if (0 == bluetooth_waitForCmd('S', 1)) {
      startFlag = true;
      break;
    }
  }
  ledPanel_clear();
  return startFlag;
}

/********************************************
   API Functions
 *******************************************/
void bricks_play() {
  bool startFlag = false;
  if (false == pleaseConnect()) return;
  ledPanel_clear();
  resetVars();
  drawBricks();
  updatePaddle();
  matrix.drawPixel(ballX, ballY, GRAY); //draw ball
  unsigned long timerStartMillis = millis();
  while (!startFlag) {
    if (0 == bluetooth_waitForCmd('F', 1) or 0 == bluetooth_waitForCmd('B', 1) or 0 == bluetooth_waitForCmd('L', 1) or 0 == bluetooth_waitForCmd('R', 1))
      startFlag = true;
    else if (millis() > timerStartMillis + BRICKS_START_TIMEOUT_MS)
      return;
  }
  while (true) {
    updateBall();
    if (!ballInboundsFlag) {
      gameOver();
      return;
    }
    updateBricks();
    if (0 == activeBricks) {
      victory();
      return;
    }
    getInput();
    updatePaddle();
    if (millis() > prevInputMillis + BRICKS_IDLE_TIMEOUT) {
      return;
    }
  }
}

#endif
