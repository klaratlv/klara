/*
  Support for MFRC522 RFID reader via SPI interface

*/

#ifndef RFID_H
#define RFID_H

/********************************************
  Includes
*******************************************/
#include <MFRC522.h>

/********************************************
  Defines
*******************************************/
#define RST_PIN 8
#define SS_PIN 53

/********************************************
  Variables
*******************************************/
MFRC522 mfrc522(SS_PIN, RST_PIN);

/********************************************
  External Functions
*******************************************/
void onRfid(unsigned long rfidUid);

/********************************************
  Local Functions
*******************************************/

/********************************************
  API Functions
*******************************************/
void rfid_init() {
  SPI.begin();
  mfrc522.PCD_Init();//(SS_PIN, RST_PIN);
  Serial.print("RFID ");
  mfrc522.PCD_DumpVersionToSerial();
}

/*
  Check if RFID card is present
*/
bool rfid_isCardPresent() {
  return mfrc522.PICC_IsNewCardPresent();
}
  
/*
  When a card is present -returns the LSByte of the card's UID
  When a card isn't present - returns 0
*/
void rfid_handler() {
  unsigned long rfidUid;
  if (rfid_isCardPresent) {
    if (mfrc522.PICC_ReadCardSerial()) {
      rfidUid = mfrc522.uid.uidByte[0] | (mfrc522.uid.uidByte[1] << 8) | (mfrc522.uid.uidByte[2] << 16)  | (mfrc522.uid.uidByte[3] << 24);
      //Serial.print(mfrc522.uid.uidByte[0]);
      //Serial.print("\t");
      //Serial.print(mfrc522.uid.uidByte[1]);
      //Serial.print("\t");
      //Serial.print(mfrc522.uid.uidByte[2]);
      //Serial.print("\t");
      //Serial.println(mfrc522.uid.uidByte[3]);
      onRfid(rfidUid);
    }
  }
  return 0; //no card is present or error reading card
}


#endif
