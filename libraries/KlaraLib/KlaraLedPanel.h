/*
   This header file adds support for 64x32 RGB LED panel
*/

#ifndef LED_PANEL_H
#define LED_PANEL_H

/********************************************
   Includes
 *******************************************/
#include <Adafruit_GFX.h>
#include <RGBmatrixPanel.h>
//#include <Fonts/FreeSerif9pt7b.h>

/********************************************
   Defines
 *******************************************/
#define CIRCLES_UPDATE_INTERVAL 150
#define CIRCLES_MIN_RADIUS 1
#define CIRCLES_MAX_RADIUS 30
#define HI_KLARA_UPDATE_INTERVAL 150
#define X_SPACING 6
#define Y_SPACING 9

// Pin definitions
#define CLK 11
#define OE  9
#define LAT 10
#define A   A0
#define B   A1
#define C   A2
#define D   A3

// Color definitions
#define NUM_OF_COLORS 12
#define BLACK 0x0000
#define BLUE 0x001F
#define LIGHT_BLUE 0x000F
#define RED 0xF800
#define LIGHT_RED 0x7800
#define GREEN 0x07E0
#define LIGHT_GREEN 0x03E0
#define CYAN 0x07FF
#define MAGENTA 0xF81F
#define YELLOW 0xFFE0
#define WHITE 0xFFFF
#define GRAY 0x7FFF

/******************************************** 
   Variables
 *******************************************/
RGBmatrixPanel matrix(A, B, C, D, CLK, LAT, OE, false, 64);
int xPos = 3;
int yPos = 0;
int isHiKlara = HIGH;
unsigned long circlesPrevMillis = 0;
unsigned long hiKlaraPrevMillis = 0;
unsigned long flashingTextPrevMillis = 0;
int hiKlaraColor = 0;
int flashingTextColor = 0;
int circleRadius = 1;
int colors[NUM_OF_COLORS] = {BLACK, WHITE, GRAY, BLUE, LIGHT_BLUE, RED, LIGHT_RED, GREEN, LIGHT_GREEN, CYAN, MAGENTA, YELLOW};

/********************************************
   External Functions
 *******************************************/
extern bool basketball_checkStartCondition();

/********************************************
   API Functions
 *******************************************/
void ledPanel_init() {
  matrix.begin();
}

/*
  Clear the display
*/
void ledPanel_clear() {
  matrix.fillScreen(BLACK);
}

/*
  Change text color
*/
void ledPanel_setColor(int i)
{
  switch (i % 9)
  {
    case 0:
      matrix.setTextColor(matrix.Color333(5, 0, 0));
      break;
    case 1:
      matrix.setTextColor(matrix.Color333(5, 2, 0));
      break;
    case 2:
      matrix.setTextColor(matrix.Color333(2, 5, 0));
      break;
    case 3:
      matrix.setTextColor(matrix.Color333(0, 5, 0));
      break;
    case 4:
      matrix.setTextColor(matrix.Color333(0, 5, 2));
      break;
    case 5:
      matrix.setTextColor(matrix.Color333(0, 2, 5));
      break;
    case 6:
      matrix.setTextColor(matrix.Color333(0, 0, 5));
      break;
    case 7:
      matrix.setTextColor(matrix.Color333(2, 0, 5));
      break;
    case 8:
      matrix.setTextColor(matrix.Color333(5, 0, 2));
      break;
  }
}

/*
  Draw text on led panel
  inputs:
  x, y - start of text coordinates
  textColor - best to use one of the defined colors above:  BLUE, MAGENTA, etc.
  textSize - well, it's the text size, dah...
*/
void ledPanel_drawText(int x, int y, unsigned int textColor, int textSize, char* text) {
  matrix.setTextColor(textColor);
  matrix.setCursor(x, y);
  matrix.setTextSize(textSize);
  matrix.print(text);
}

void ledPanel_circlesUpdate() {
  if (millis() > circlesPrevMillis + CIRCLES_UPDATE_INTERVAL) {
    circlesPrevMillis = millis();
    cli(); //disable interrupts when refreshing the LED panel
    matrix.fillScreen(BLACK); // Clear screen
    matrix.drawCircle(31, 15, circleRadius - 1, matrix.Color333(3, 0, 0));
    matrix.drawCircle(31, 15, circleRadius, matrix.Color333(0, 0, 7));
    sei(); //enable interrupts
    circleRadius++;
    if (circleRadius > CIRCLES_MAX_RADIUS)
      circleRadius = CIRCLES_MIN_RADIUS;
  }
}

void ledPanel_fillArrow(int x0, int y0, int width, uint16_t color) {
  //round up to the nearest odd number
  if (width % 2 == 0)
    width += 1;
  int a = (width - 1) / 2;
  int b = width * 0.25  - 0.75;
  matrix.fillTriangle(x0, y0, (x0 - a), (y0 + a), (x0 + a), (y0 + a), color);
  matrix.fillRect((x0 - b), (y0 + a), a, a + 1, color);
}

void ledPanel_drawArrow(int x0, int y0, int width, uint16_t color) {
  //round up to the nearest odd number
  if (width % 2 == 0)
    width += 1;
  int a = (width - 1) / 2;
  int b = width * 0.25  - 0.75;
  matrix.drawTriangle(x0, y0, (x0 - a), (y0 + a), (x0 + a), (y0 + a), color);
  matrix.drawRect((x0 - b), (y0 + a), a, a + 1, color);
}

/*
  Print "Hi, I'm Klara" on panel
*/
void ledPanel_hiKlaraStep() {
  if (millis() > hiKlaraPrevMillis + HI_KLARA_UPDATE_INTERVAL) {
    hiKlaraPrevMillis = millis();
    ledPanel_setColor(hiKlaraColor++);
    matrix.setCursor(7, 5);
    matrix.println(F("H"));
    ledPanel_setColor(hiKlaraColor++);
    matrix.setCursor(13, 5);
    matrix.println(F("i"));
    ledPanel_setColor(hiKlaraColor++);
    matrix.setCursor(17, 5);
    matrix.println(F(","));
  
    ledPanel_setColor(hiKlaraColor++);
    matrix.setCursor(6, 15);
    matrix.println(F("I"));
    ledPanel_setColor(hiKlaraColor++);
    matrix.setCursor(10, 15);
    matrix.println(F("'"));
    ledPanel_setColor(hiKlaraColor++);
    matrix.setCursor(15, 15);
    matrix.println(F("m"));
  
    ledPanel_setColor(hiKlaraColor++);
    matrix.setCursor(24, 15);
    matrix.println(F("K"));
    ledPanel_setColor(hiKlaraColor++);
    matrix.setCursor(30, 15);
    matrix.println(F("l"));
    ledPanel_setColor(hiKlaraColor++);
    matrix.setCursor(36, 15);
    matrix.println(F("a"));
    ledPanel_setColor(hiKlaraColor++);
    matrix.setCursor(42, 15);
    matrix.println(F("r"));
    ledPanel_setColor(hiKlaraColor++);
    matrix.setCursor(48, 15);
    matrix.println(F("a"));
  
    if (hiKlaraColor > 81)
      hiKlaraColor = 0;
  }
}

void ledPanel_flashingTextStep(char text[], int stepDelay, int xIndent = 0, int yIndent = 0) {
  int len = strlen(text);
  int xPos = xIndent;
  int yPos = yIndent;
  if (millis() > flashingTextPrevMillis + stepDelay) {
    flashingTextPrevMillis = millis();
    for (int charCnt = 0; charCnt < len; charCnt++) {
      ledPanel_setColor(flashingTextColor++);
      matrix.setCursor(xPos, yPos);
      if (text[charCnt] == '\n') {
        yPos += Y_SPACING;
        xPos = xIndent;
      }
      else {
        cli(); //disable interrupts when refreshing the LED panel
        matrix.print(text[charCnt]);
        sei(); //enable interrupts
        xPos += X_SPACING;
        if (xPos > 58) {
          xPos = xIndent;
          yPos += Y_SPACING;
        }
      }
      if (flashingTextColor > 81)
        flashingTextColor = 0;
    }
  }
}

#endif
