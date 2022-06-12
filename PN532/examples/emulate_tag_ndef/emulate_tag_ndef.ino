#include <emulatetag.h>
#include <NdefMessage.h>

#if 1
#include <SPI.h>
#include <PN532_SPI.h>
#include <PN532.h>

PN532_SPI pn532spi(SPI, 10);
EmulateTag nfc(pn532spi);
#elif 0
#include <PN532_HSU.h>
#include <PN532.h>

PN532_HSU pn532hsu(Serial1);
EmulateTag nfc(pn532hsu);
#endif

uint8_t uid[3] = {0x12, 0x34, 0x56};
NdefMessage message;
int messageSize;

uint8_t memory_area[160];
/*= {
  0x00, 0x00, 0x00, 0x00,  // Block 0
  0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00,  // Block 2 (Static lock bytes: CC area and data area are writeable)
  0xE1, 0x10, 0x06, 0x0F,  // Block 3 (CC - NFC-Forum Tag Type 2 version 1.0, Data area (from block 4 to the end) is 48 bytes, Read-only mode)

  0x03, 33,   0xd1, 0x02,  // Block 4 (NDEF)
  0x1c, 0x53, 0x70, 0x91,
  0x01, 0x09, 0x54, 0x02,
  0x65, 0x6e, 0x4c, 0x69,

  0x62, 0x6e, 0x66, 0x63,
  0x51, 0x01, 0x0b, 0x55,
  0x03, 0x6c, 0x69, 0x62,
  0x6e, 0x66, 0x63, 0x2e,

  0x6f, 0x72, 0x67, 0x00,
  0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00,
};*/

#define PAGESIZE 4
#define SIZE_USER_DATA sizeof(memory_area) - (4*PAGESIZE) // 4 page for header block 0 - 3

#define READ            0x30
#define WRITE           0xA2
#define SECTOR_SELECT   0xC2
#define HALT            0x50

void setup()
{
  memset(memory_area, 0, sizeof(memory_area));

  // init CC to 144 bytes
  memory_area[0xC] = 0xE1;
  memory_area[0xD] = 0x10;
  memory_area[0xE] = 0x12;

  Serial.begin(115200);
  Serial.println("------- Emulate Tag --------");

  message = NdefMessage();
  message.addUriRecord("https://github.com/mywalkb");
  messageSize = message.getEncodedSize();
  if (messageSize > SIZE_USER_DATA)
  {
    Serial.println("userdata is too small");
    while (1);
  }

  Serial.print("Ndef encoded message size: ");
  Serial.println(messageSize);

  memory_area[0x10] = 3;
  memory_area[0x11] = messageSize;
  message.encode(&memory_area[0x12]);

  // uid must be 3 bytes!
  nfc.setUid(uid);

  nfc.init();
}

void loop()
{
  // uncomment for overriding ndef in case a write to this tag occured
  //nfc.setNdefFile(ndefBuf, messageSize);

  // start emulation (blocks)
  nfc.emulate(&processData, 5000);

  // or start emulation with timeout
  /*if(!nfc.emulate(1000)){ // timeout 1 second
      Serial.println("timed out");
    }*/

  // deny writing to the tag
  // nfc.setTagWriteable(false);

  delay(1000);
}

uint8_t processData(const uint8_t *data_in, const size_t data_in_len, uint8_t *data_out, const size_t data_out_len)
{
    int8_t res = 0;

    switch (data_in[0]) {
      case READ:
        if (data_out_len >= 16) {
          if (data_in[1] >= (sizeof(memory_area)/4)) {
            memset(data_out, 0, 16);
          } else {
            memcpy(data_out, memory_area + (data_in[1] * 4), 16);
          }
          res = 16;
        } else {
          res = -1;
        }
        break;
      case WRITE:
        Serial.print("Write: ");
        Serial.println(data_in_len);
        break;
      case HALT:
        res = -2;
        break;
      default:
        res = -3;
    }
    return res;
}
