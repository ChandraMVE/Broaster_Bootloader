/*
 * firmwareupgrade.h
 *
 *  Created on: 15-Dec-2023
 *      Author: Dell
 */

#ifndef FIRMWAREUPGRADE_H_
#define FIRMWAREUPGRADE_H_

#include "bsp_api.h"
#include "hal_data.h"
// It will initial firmware upgrade and erase code image when receives this page number
#define FIRMWARE_UPGRADE_START_PAGE         0xfffe

#define FIRMWARE_UPGRADE_HEART_BEAT         0xfffd
// It will verify the total page received and CRC check after receive this page number.
// If everything is ok, it will change the boot state to copy image and reboot the MCU.
#define FIRMWARE_UPGRADE_END_PAGE           0xffff
#define FIRMWARE_UPGRADE_CODE_START_PAGE    (512)           // First page after bootloader
#define FIRMWARE_UPGRADE_CODE_END_PAGE      (4303)          // Page right before code image

#define FIRMWARE_UPGRADE_RES_SUCCESS                    0
#define FIRMWARE_UPGRADE_RES_WRONG_PAGE_NUMBER          1
#define FIRMWARE_UPGRADE_RES_WRONG_CRC_OR_TOTAL_PAGE    2
#define FIRMWARE_UPGRADE_RES_FLASH_ERASE_FAILED         3
#define FIRMWARE_UPGRADE_RES_FLASH_WRITE_FAILED         4

#define STX 0x02
#define ETX 0x03
#define CR  0x0d
#define LF  0x0a
#define START_HEADING   0x01
#define BUF_LENGTH  1024
#define BUF_PACKET_RESPONSE 200

// Returns the low byte of 16-bit value
#define LOW_BYTE(n)     (uint8_t)((n) & 0xff)

// Returns the high byte of 16-bit value
#define HIGH_BYTE(n)    (uint8_t)(LOW_BYTE((n) >> 8))

typedef struct
{
    uint8_t rxState;
    uint8_t Type;               // 'D' for command and 'R' for response
    uint16_t Version;
    uint16_t Length;
    uint16_t transId;
    uint16_t rxCrc;
    uint16_t localRxCrc;
    uint16_t localTxCrc;
    uint16_t dataIn;
    uint16_t txIn;
    uint8_t dataBuf[BUF_LENGTH];
    uint8_t packetResponseBuf[BUF_PACKET_RESPONSE];
}SPacket_t;

typedef struct
{
    uint32_t State;
    uint16_t totalPage;
    uint16_t Crc;
}SBootInfor_t;

static SPacket_t packet = {0};

void CopySckToUpgradePayloadPkt(uint8_t *dataBufferPtr);
void FirmwareUpgrade(void);
#endif /* FIRMWAREUPGRADE_H_ */
