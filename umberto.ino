#include <SPI.h>
#include <MFRC522.h>
#include "SoftwareSerial.h"

#define TOUCH_SENSOR_PIN 2
#define SS_PIN 10
#define RST_PIN 5

#define Start_Byte 0x7E
#define Version_Byte 0xFF
#define Command_Length 0x06
#define End_Byte 0xEF
// Returns info with command 0x41 [0x01: info, 0x00: no info]
#define Acknowledge 0x01

MFRC522 mfrc522(SS_PIN, RST_PIN);
MFRC522::MIFARE_Key key;

SoftwareSerial mySerial(6, 7);  // RX, TX

byte receive_buffer[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
byte nfc_status[3] = { 0, 0, 0 };
bool is_playing = false;

/*
 * Print helper
 */
void dump_byte_array(byte* buffer, byte bufferSize) {
  for (byte i = 0; i < bufferSize; i++) {
    Serial.print(buffer[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}

/* ========================================================================= */
/*                                MP3 MODULE                                 */

/**
   command for the mp3 module
*/
void execute_CMD(byte Command, byte Data1, byte Data2) {
  // Calculate the checksum (2 bytes)
  word Checksum = -(Version_Byte + Command_Length + Command + Acknowledge + Data1 + Data2);
  // Build the command
  byte command_line[10] = { Start_Byte, Version_Byte,
                            Command_Length, Command, Acknowledge,
                            Data1, Data2, highByte(Checksum),
                            lowByte(Checksum), End_Byte };


  // Send the command line to the module
  for (byte k = 0; k < 10; k++) {
    mySerial.write(command_line[k]);
  }
}

void reset_rec_buf() {
  for (uint8_t i = 0; i < 10; i++) {
    receive_buffer[i] = 0;
  }
}

bool receive() {
  reset_rec_buf();
  if (mySerial.available() < 10) {
    Serial.println("no data available");
    return false;
  }
  for (uint8_t i = 0; i < 10; i++) {
    short b = mySerial.read();
    if (b == -1) {
      Serial.println("b is -1");
      return false;
    }
    receive_buffer[i] = b;
  }
  // When you reset the module in software,
  // received buffer elements are shifted.
  // To correct that we do the following:
  short b = receive_buffer[0];
  for (uint8_t i = 0; i < 10; i++) {
    if (i == 9) {
      receive_buffer[i] = b;
    } else {
      receive_buffer[i] = receive_buffer[i + 1];
    }
  }  // End correcting receive_buffer
  return true;
}

void query_playback_status() {
  execute_CMD(0x42, 0, 0);
  delay(100);
}

void stop_playback() {
  execute_CMD(0x16, 0, 0);
  delay(100);
}

/* ========================================================================= */
/*                                NFC MODULE                                 */

/**
   convert record from rfid to number for mp3 module cmd
*/
byte convert(char c1, char c2) {
  String str;
  str += c1;
  str += c2;
  int num = str.toInt();
  return byte(num);
}

/**
   first byte indicates the status
   1 : OK
   2 : NO TAG AVAILABLE
   3 : Auth failed
   4 : Read failed
*/
void read_nfc(byte* nfc_status) {
  nfc_status[0] = 0;  // reset status

  MFRC522::StatusCode status;
  byte buffer[18];
  byte size = sizeof(buffer);
  byte sector = 1;
  byte blockAddr = sector * 4;
  byte trailerBlock = blockAddr + 3;

  status = (MFRC522::StatusCode)mfrc522.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_B, trailerBlock, &key, &(mfrc522.uid));
  if (status != MFRC522::STATUS_OK) {
    nfc_status[0] = 3;
    return nfc_status;
  }

  status = (MFRC522::StatusCode)mfrc522.MIFARE_Read(blockAddr, buffer, &size);
  if (status != MFRC522::STATUS_OK) {
    nfc_status[0] = 4;
    return nfc_status;
  }

  // success read record
  byte one = convert(buffer[9], buffer[10]);
  byte two = convert(buffer[11], buffer[12]);
  nfc_status[0] = 1;
  nfc_status[1] = one;
  nfc_status[2] = two;

  return nfc_status;
}


void setup() {
  Serial.begin(9600);

  pinMode(TOUCH_SENSOR_PIN, INPUT);

  // init rfid
  mySerial.begin(9600);
  SPI.begin();         // init SPI bus
  mfrc522.PCD_Init();  // init MFRC522
  for (byte i = 0; i < 6; i++) {
    key.keyByte[i] = 0xFF;
  }

  // init mp3 module
  execute_CMD(0x0C, 0, 0);  // reset
  delay(100);
}

void loop() {
  // Look for new cards
  if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
    read_nfc(nfc_status);
    if (nfc_status[0] == 1) {
      Serial.println("read successfull");
      dump_byte_array(nfc_status, 3);
      execute_CMD(0x12, nfc_status[1], nfc_status[2]);
      is_playing = true;
    }
    mfrc522.PICC_HaltA();       // Halt PICC
    mfrc522.PCD_StopCrypto1();  // Stop encryption on PCD
  } else if (is_playing) {
    // Declare buffer for the STATUS_CODE returned in the ATQA buffer
    byte bufferATQA[2];
    byte bufferSize = sizeof(bufferATQA);
    // Reset baud rates
    mfrc522.PCD_WriteRegister(mfrc522.TxModeReg, 0x00);
    mfrc522.PCD_WriteRegister(mfrc522.RxModeReg, 0x00);
    // Reset ModWidthReg
    mfrc522.PCD_WriteRegister(mfrc522.ModWidthReg, 0x26);

    if (mfrc522.PICC_WakeupA(bufferATQA, &bufferSize)) {
      Serial.println("stop");
      stop_playback();
      mfrc522.PICC_HaltA();  // We must re-halt it now that we've woken it up
      is_playing = false;
    }
  }
}
