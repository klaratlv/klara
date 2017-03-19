
#ifndef RTC_H
#define RTC_H

/********************************************
   Includes
 *******************************************/
#include <RTClib.h>

/********************************************
   Defines
 *******************************************/

/********************************************
   Variables
 *******************************************/
RTC_DS1307 rtc;
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
bool isPrint = true;
unsigned long int rtcPrevMillis = 0;

/********************************************
   Local Functions
 *******************************************/

/********************************************
   API Functions
 *******************************************/
/*
   Get time & date from RTC
*/
DateTime rtc_getTime() {
  if (! rtc.begin()) {
    Serial.println(F("Couldn't find RTC"));
  }
  if (! rtc.isrunning()) {
    Serial.println(F("RTC is NOT running!"));
  }
  return rtc.now();
}

void rtc_init() {
  rtc.begin();
  delay(10);
  rtc_getTime(); //clean RTC garbage
}

/*
   Reset the rtc's time & date according to the connected PC's time & date
*/
void rtc_reset(int yr = 0, int mon = 0, int dy = 0, int hr = 0, int mnt = 0, int sec = 0) {
  if (0 == yr) {
		Serial.println("RTC: Adjusting time");
		rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); //set the RTC to the date & time this sketch was compiled
	}
	else
    rtc.adjust(DateTime(yr, mon, dy, hr, mnt, sec));
}

#endif
