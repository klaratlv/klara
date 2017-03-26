#define SONG_ID_FOR_SETUP_AND_DRIVING 4
#define SONG_ID_FOR_ARRIVAL 4

void displayMsg(int msgId, int duration, int colorchangeInterval);

void setupDelivery() {
  arm_extend();
  arm_pumpActivate(true);
  audio_play(1, SONG_ID_FOR_SETUP_AND_DRIVING);
  displayMsg(6, 10, 250);
}

void handle_delivery() {
  ledPanel_clear();
  audio_play(1, SONG_ID_FOR_ARRIVAL);
  displayMsg(7, 10, 250);
  audio_fadeout(10000);
  arm_pumpActivate(false);
  delay(3000);
  arm_resetPosition();
  ledPanel_clear();
  displayMsg(8, 7, 250);
  ledPanel_clear();
}
