/*
   This header file adds support for serial audio shield
*/

#ifndef AUDIO_H
#define AUDIO_H

/********************************************
   Includes
 *******************************************/
#include <SoftwareSerial.h>
#include <Arduino.h>

/********************************************
   Defines
 *******************************************/
#define INIT_VOLUME 30

//should connect to TX of the Serial MP3 Player module
#define ARDUINO_RX 14
//connect to RX of the module
#define ARDUINO_TX 12

#define CMD_SEL_DEV 0X09
#define DEV_TF 0X02

#define CMD_SET_DAC 0X1A
#define DAC_ON  0X00
#define DAC_OFF 0X01

#define CMD_VOLUME_UP 0X04
#define CMD_VOLUME_DOWN 0X05
#define CMD_SET_VOLUME 0X06

#define CMD_PLAY 0X0D
#define CMD_PLAY_W_INDEX 0X08
#define CMD_PLAY_W_VOL 0X22
#define CMD_PLAY_FOLDER_FILE 0X0F
#define CMD_STOP_PLAY 0X16
#define CMD_PAUSE 0X0E
#define CMD_SINGLE_CYCLE_PLAY 0X08
#define CMD_SINGLE_CYCLE 0X19
#define SINGLE_CYCLE_ON 0X00
#define SINGLE_CYCLE_OFF 0X01

#define CMD_NEXT_SONG 0X01
#define CMD_PREV_SONG 0X02

#define CMD_SLEEP_MODE 0X0A
#define CMD_WAKE_UP 0X0B
#define CMD_RESET 0X0C

#define CMD_FOLDER_CYCLE 0X17
#define CMD_SHUFFLE_PLAY 0X18

#define CMD_GROUP_DISPLAY 0X21
/********************************************
   Variables
 *******************************************/
SoftwareSerial mp3_serial(ARDUINO_RX, ARDUINO_TX);
static int8_t Send_buf[8] = {0};
int audioVol = INIT_VOLUME;

/********************************************
   Local Functions
 *******************************************/
void sendCommand(int8_t command, int16_t dat) {
  delay(20);
  Send_buf[0] = 0x7e; //starting byte
  Send_buf[1] = 0xff; //version
  Send_buf[2] = 0x06; //the number of bytes of the command without starting byte and ending byte
  Send_buf[3] = command; //
  Send_buf[4] = 0x01; //0x00 = no feedback, 0x01 = feedback
  Send_buf[5] = (int8_t)(dat >> 8);//datah
  Send_buf[6] = (int8_t)(dat); //datal
  Send_buf[7] = 0xef; //ending byte
  for (uint8_t i = 0; i < 8; i++) {
    mp3_serial.write(Send_buf[i]) ;
  }
}

/********************************************
   API Functions
 *******************************************/
void audio_set_vol(byte vol) {
  audioVol = vol;
  sendCommand(CMD_SET_VOLUME, vol);
}

void audio_play(byte folder, byte track) {
  //play the first song with volume 15 class: 0X0F01
  // sendCommand(CMD_PLAY_W_VOL, int(volume<< 8) | track);
  sendCommand(CMD_PLAY_FOLDER_FILE, folder << 8 | track); //>>>
}

void audio_play(byte track) {
  audio_play(1, track);
}

void audio_stop() {
  sendCommand(CMD_STOP_PLAY, 0);
}

void audio_fadeout(int fadeTimeMs) {
  int delayMs = fadeTimeMs / audioVol;
  for (int i = audioVol; i >= 0; i--) {
      sendCommand(CMD_SET_VOLUME, i);
      delay(delayMs);
  }
  audio_stop();
  delay(100);
  audio_set_vol(audioVol);
}

void audio_pause() {
  sendCommand(CMD_PAUSE, 0);
}

void audio_resume() {
  sendCommand(CMD_PLAY, 0);
}

void audio_next() {
  sendCommand(CMD_NEXT_SONG, 0);
}

void audio_prev() {
  sendCommand(CMD_PREV_SONG, 0);
}

void audio_init() {
  mp3_serial.begin(9600);
  //Wait chip initialization is complete
  delay(500);
  //wait for 200ms
  sendCommand(CMD_SEL_DEV, DEV_TF);//select the TF card
  delay(200);
  audio_stop();
  audio_set_vol(INIT_VOLUME);
}

#endif
