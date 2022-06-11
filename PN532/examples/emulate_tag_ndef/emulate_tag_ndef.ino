#include <emulatetag.h>

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

void setup()
{
  Serial.begin(115200);
  Serial.println("------- Emulate Tag --------");

  // uid must be 3 bytes!
  nfc.setUid(uid);

  nfc.init();
}

void loop()
{
  // uncomment for overriding ndef in case a write to this tag occured
  //nfc.setNdefFile(ndefBuf, messageSize);

  // start emulation (blocks)
  nfc.emulate(5000);

  // or start emulation with timeout
  /*if(!nfc.emulate(1000)){ // timeout 1 second
      Serial.println("timed out");
    }*/

  // deny writing to the tag
  // nfc.setTagWriteable(false);

  delay(1000);
}
