/**************************************************************************/
/*!
    @file     emulatetag.cpp
    @author   Armin Wieser
    @license  BSD
*/
/**************************************************************************/

#include "emulatetag.h"
#include "PN532_debug.h"

#include <string.h>

#define MAX_TGREAD

// Command APDU
#define C_APDU_CLA 0
#define C_APDU_INS 1  // instruction
#define C_APDU_P1 2   // parameter 1
#define C_APDU_P2 3   // parameter 2
#define C_APDU_LC 4   // length command
#define C_APDU_DATA 5 // data

#define C_APDU_P1_SELECT_BY_ID 0x00
#define C_APDU_P1_SELECT_BY_NAME 0x04

// Response APDU
#define R_APDU_SW1_COMMAND_COMPLETE 0x90
#define R_APDU_SW2_COMMAND_COMPLETE 0x00

#define R_APDU_SW1_NDEF_TAG_NOT_FOUND 0x6a
#define R_APDU_SW2_NDEF_TAG_NOT_FOUND 0x82

#define R_APDU_SW1_FUNCTION_NOT_SUPPORTED 0x6A
#define R_APDU_SW2_FUNCTION_NOT_SUPPORTED 0x81

#define R_APDU_SW1_MEMORY_FAILURE 0x65
#define R_APDU_SW2_MEMORY_FAILURE 0x81

#define R_APDU_SW1_END_OF_FILE_BEFORE_REACHED_LE_BYTES 0x62
#define R_APDU_SW2_END_OF_FILE_BEFORE_REACHED_LE_BYTES 0x82

// ISO7816-4 commands
#define ISO7816_SELECT_FILE 0xA4
#define ISO7816_READ_BINARY 0xB0
#define ISO7816_UPDATE_BINARY 0xD6

typedef enum
{
  NONE,
  CC,
  NDEF
} tag_file; // CC ... Compatibility Container

bool EmulateTag::init()
{
  pn532.begin();
  return pn532.SAMConfig();
}

void EmulateTag::setNdefFile(const uint8_t *ndef, const int16_t ndefLength)
{
  if (ndefLength > (NDEF_MAX_LENGTH - 2))
  {
    DMSG("ndef file too large (> NDEF_MAX_LENGHT -2) - aborting");
    return;
  }

  ndef_file[0] = ndefLength >> 8;
  ndef_file[1] = ndefLength & 0xFF;
  memcpy(ndef_file + 2, ndef, ndefLength);
}

void EmulateTag::setUid(uint8_t *uid)
{
  uidPtr = uid;
}

int8_t EmulateTag::emulate(const uint16_t tgInitAsTargetTimeout)
{
     // TODO replace NULL with default function
     return emulate(NULL, tgInitAsTargetTimeout);
}

int8_t EmulateTag::emulate(ProcessEmulEvent callback, const uint16_t tgInitAsTargetTimeout)
{
    pn532.writeRegister(REG_CIU_RxMode, 0x80); // no accept invalid frame, no accept multiple frames, CRC enabled, rx speed 106 kbits
    pn532.writeRegister(REG_CIU_TxMode, 0x80); // CRC enabled, tx speed 106 kbits

    pn532.setRFField(0, 0);

    uint8_t prev = pn532.readRegister(REG_CIU_Status2);
    prev = bitClear(prev, 3); // disable CRYPTO1
    pn532.writeRegister(REG_CIU_Status2, prev);

    prev = pn532.readRegister(REG_CIU_ManualRCV);
    prev = bitClear(prev, 4); // disable ParityDisable
    pn532.writeRegister(REG_CIU_ManualRCV, prev);

    prev = pn532.readRegister(REG_CIU_TxAuto);
    prev = bitSet(prev, 2); // enable InitialRFOn
    pn532.writeRegister(REG_CIU_TxAuto, prev);

    pn532.setParameters(0x10); // enable fAutomaticRATS

    uint8_t command[] = {
      PN532_COMMAND_TGINITASTARGET,
      1, // MODE: Passive only

      0x04, 0x00,       // SENS_RES
      0x00, 0x00, 0x00, // NFCID1
      0x00,             // SEL_RES

      0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, // FeliCaParams
      0, 0,

      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // NFCID3t

      0, // length of general bytes
      0  // length of historical bytes
    };

  if (uidPtr != 0)
  { // if uid is set copy 3 bytes to nfcid1
    memcpy(command + 4, uidPtr, 3);
  }

  uint8_t mode;

  uint8_t abtRx[256];
  uint8_t abtTx[256];

  int8_t res = pn532.tgInitAsTarget(command, sizeof(command), &mode, abtRx, tgInitAsTargetTimeout);
  if (res <= 0)
  {
    DMSG("tgInitAsTarget failed or timed out!");
    return -1;
  }

  // TODO check mode

  size_t szRx = res;
  int8_t io_res = res;
  while (io_res >= 0) {
    // TODO manage null callback
    io_res = callback(abtRx, szRx, abtTx, sizeof(abtTx));
    if (io_res > 0) {
      if ((res = pn532.tgResponseToInitiator(abtTx, io_res, 1000)) < 0) {
        return res;
      }
    }
    if (io_res >= 0) {
      if ((res = pn532.tgGetInitiatorCommand(abtRx, sizeof(abtRx), 1000)) < 0) {
        return res;
      }
      szRx = res;
    }
  }
  pn532.inRelease();
  return io_res;
}

void EmulateTag::setResponse(responseCommand cmd, uint8_t *buf, uint8_t *sendlen, uint8_t sendlenOffset)
{
  switch (cmd)
  {
  case COMMAND_COMPLETE:
    buf[0] = R_APDU_SW1_COMMAND_COMPLETE;
    buf[1] = R_APDU_SW2_COMMAND_COMPLETE;
    *sendlen = 2 + sendlenOffset;
    break;
  case TAG_NOT_FOUND:
    buf[0] = R_APDU_SW1_NDEF_TAG_NOT_FOUND;
    buf[1] = R_APDU_SW2_NDEF_TAG_NOT_FOUND;
    *sendlen = 2;
    break;
  case FUNCTION_NOT_SUPPORTED:
    buf[0] = R_APDU_SW1_FUNCTION_NOT_SUPPORTED;
    buf[1] = R_APDU_SW2_FUNCTION_NOT_SUPPORTED;
    *sendlen = 2;
    break;
  case MEMORY_FAILURE:
    buf[0] = R_APDU_SW1_MEMORY_FAILURE;
    buf[1] = R_APDU_SW2_MEMORY_FAILURE;
    *sendlen = 2;
    break;
  case END_OF_FILE_BEFORE_REACHED_LE_BYTES:
    buf[0] = R_APDU_SW1_END_OF_FILE_BEFORE_REACHED_LE_BYTES;
    buf[1] = R_APDU_SW2_END_OF_FILE_BEFORE_REACHED_LE_BYTES;
    *sendlen = 2;
    break;
  }
}
