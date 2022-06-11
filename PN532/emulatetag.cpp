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

bool EmulateTag::emulate(const uint16_t tgInitAsTargetTimeout)
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

  uint8_t memory_area[] = {
  0x00, 0x00, 0x00, 0x00,  // Block 0
  0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0xFF, 0xFF,  // Block 2 (Static lock bytes: CC area and data area are read-only locked)
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
};

  if (uidPtr != 0)
  { // if uid is set copy 3 bytes to nfcid1
    memcpy(command + 4, uidPtr, 3);
  }

  uint8_t mode;
  uint8_t initiatorcommand[16];

  if (1 != pn532.tgInitAsTarget(command, sizeof(command), &mode, initiatorcommand, tgInitAsTargetTimeout))
  {
    DMSG("tgInitAsTarget failed or timed out!");
    return false;
  }

  uint8_t cmd = initiatorcommand[0];
  uint8_t block = initiatorcommand[1];

  uint8_t data_out[16];

  while (true) {
      if (cmd == 0x30) { // READ
          memcpy(data_out, memory_area + (block * 4), 16);
          pn532.tgResponseToInitiator(data_out, sizeof(data_out));
      } else if (cmd == 0x50) { // HALT
          break;
      } else {
          break;
      }
      uint8_t len;
      if (pn532.tgGetInitiatorCommand(initiatorcommand, &len) && len>=2) {
          cmd = initiatorcommand[0];
          block = initiatorcommand[1];
      } else {
          break;
      }
  }

/*
  tagWrittenByInitiator = false;

  uint8_t rwbuf[128];
  uint8_t sendlen;
  int16_t status;
  tag_file currentFile = NONE;
  uint16_t cc_size = sizeof(compatibility_container);
  bool runLoop = true;
*/

/*
  while (runLoop)
  {
    status = pn532.tgGetData(rwbuf, sizeof(rwbuf));
    if (status < 0)
    {
      DMSG("tgGetData failed!\n");
      pn532.inRelease();
      return true;
    }

    uint8_t p1 = rwbuf[C_APDU_P1];
    uint8_t p2 = rwbuf[C_APDU_P2];
    uint8_t lc = rwbuf[C_APDU_LC];
    uint16_t p1p2_length = ((int16_t)p1 << 8) + p2;

    switch (rwbuf[C_APDU_INS])
    {
    case ISO7816_SELECT_FILE:
      switch (p1)
      {
      case C_APDU_P1_SELECT_BY_ID:
        if (p2 != 0x0c)
        {
          DMSG("C_APDU_P2 != 0x0c\n");
          setResponse(COMMAND_COMPLETE, rwbuf, &sendlen);
        }
        else if (lc == 2 && rwbuf[C_APDU_DATA] == 0xE1 && (rwbuf[C_APDU_DATA + 1] == 0x03 || rwbuf[C_APDU_DATA + 1] == 0x04))
        {
          setResponse(COMMAND_COMPLETE, rwbuf, &sendlen);
          if (rwbuf[C_APDU_DATA + 1] == 0x03)
          {
            currentFile = CC;
          }
          else if (rwbuf[C_APDU_DATA + 1] == 0x04)
          {
            currentFile = NDEF;
          }
        }
        else
        {
          setResponse(TAG_NOT_FOUND, rwbuf, &sendlen);
        }
        break;
      case C_APDU_P1_SELECT_BY_NAME:
        const uint8_t ndef_tag_application_name_v2[] = {0, 0x7, 0xD2, 0x76, 0x00, 0x00, 0x85, 0x01, 0x01};
        if (0 == memcmp(ndef_tag_application_name_v2, rwbuf + C_APDU_P2, sizeof(ndef_tag_application_name_v2)))
        {
          setResponse(COMMAND_COMPLETE, rwbuf, &sendlen);
        }
        else
        {
          DMSG("function not supported\n");
          setResponse(FUNCTION_NOT_SUPPORTED, rwbuf, &sendlen);
        }
        break;
      }
      break;
    case ISO7816_READ_BINARY:
      switch (currentFile)
      {
      case NONE:
        setResponse(TAG_NOT_FOUND, rwbuf, &sendlen);
        break;
      case CC:
        if (p1p2_length > NDEF_MAX_LENGTH)
        {
          setResponse(END_OF_FILE_BEFORE_REACHED_LE_BYTES, rwbuf, &sendlen);
        }
        else
        {
          memcpy(rwbuf, compatibility_container + p1p2_length, lc);
          setResponse(COMMAND_COMPLETE, rwbuf + lc, &sendlen, lc);
        }
        break;
      case NDEF:
        if (p1p2_length > NDEF_MAX_LENGTH)
        {
          setResponse(END_OF_FILE_BEFORE_REACHED_LE_BYTES, rwbuf, &sendlen);
        }
        else
        {
          memcpy(rwbuf, ndef_file + p1p2_length, lc);
          setResponse(COMMAND_COMPLETE, rwbuf + lc, &sendlen, lc);
        }
        break;
      }
      break;
    case ISO7816_UPDATE_BINARY:
      if (!tagWriteable)
      {
        setResponse(FUNCTION_NOT_SUPPORTED, rwbuf, &sendlen);
      }
      else
      {
        if (p1p2_length > NDEF_MAX_LENGTH)
        {
          setResponse(MEMORY_FAILURE, rwbuf, &sendlen);
        }
        else
        {
          memcpy(ndef_file + p1p2_length, rwbuf + C_APDU_DATA, lc);
          setResponse(COMMAND_COMPLETE, rwbuf, &sendlen);
          tagWrittenByInitiator = true;

          uint16_t ndef_length = (ndef_file[0] << 8) + ndef_file[1];
          if ((ndef_length > 0) && (updateNdefCallback != 0))
          {
            updateNdefCallback(ndef_file + 2, ndef_length);
          }
        }
      }
      break;
    default:
      DMSG("Command not supported!");
      DMSG_HEX(rwbuf[C_APDU_INS]);
      DMSG("\n");
      setResponse(FUNCTION_NOT_SUPPORTED, rwbuf, &sendlen);
    }
    status = pn532.tgSetData(rwbuf, sendlen);
    if (status < 0)
    {
      DMSG("tgSetData failed\n!");
      pn532.inRelease();
      return true;
    }
  }
*/
  pn532.inRelease();
  return true;
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
