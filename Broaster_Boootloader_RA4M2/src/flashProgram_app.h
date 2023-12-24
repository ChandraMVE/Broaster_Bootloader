/*
 * flashTest.h
 *
 *  Created on: Nov 25, 2022
 *      Author: xbie
 */

#ifndef FLASHPROGRAM_APP_H_
#define FLASHPROGRAM_APP_H_

#include "bsp_api.h"
#include "hal_data.h"
//-------------------------------------------------------------------------------------------
/* Code Flash */
#define FLASH_HP_CF_BLOCK_SIZE_32KB       (32*1024)    /* Block Size 32 KB */
#define FLASH_HP_CF_BLOCK_SIZE_8KB        (8*1024)     /* Block Size 8KB */
//Boot loader   64k
#define FLASH_HP_CF_BLCOK_0               0x00000000U  /*    8 KB:  0x00000000 - 0x00001FFF */
#define FLASH_HP_CF_BLOCK_1               0x00002000U  /*    8 KB:  0x00002000 - 0x00003FFF */
#define FLASH_HP_CF_BLOCK_2               0x00004000U  /*    8 KB:  0x00004000 - 0x00005FFF */
#define FLASH_HP_CF_BLOCK_3               0x00006000U  /*    8 KB:  0x00006000 - 0x00007FFF */
#define FLASH_HP_CF_BLOCK_4               0x00008000U  /*    8 KB:  0x00008000 - 0x00009FFF */
#define FLASH_HP_CF_BLOCK_5               0x0000A000U  /*    8 KB:  0x0000A000 - 0x0000BFFF */
#define FLASH_HP_CF_BLOCK_6               0x0000C000U  /*    8 KB:  0x0000C000 - 0x0000DFFF */
#define FLASH_HP_CF_BLOCK_7               0x0000E000U  /*    8 KB:  0x0000E000 - 0x0000FFFF */
// Application - 224k - 7 block and each block has 32k
#define FLASH_HP_CF_BLOCK_8               0x00010000U  /*   32 KB: 0x00010000 - 0x00017FFF */
#define FLASH_HP_CF_BLOCK_9               0x00018000U  /*   32 KB: 0x00018000 - 0x0001FFFF */
#define FLASH_HP_CF_BLCOK_10              0x00020000U  /*   32 KB: 0x00020000 - 0x00027FFF */
#define FLASH_HP_CF_BLOCK_11              0x00028000U  /*   32 KB: 0x00028000 - 0x0002FFFF */
#define FLASH_HP_CF_BLOCK_12              0x00030000U  /*   32 KB: 0x00030000 - 0x00037FFF */
#define FLASH_HP_CF_BLCOK_13              0x00038000U  /*   32 KB: 0x00038000 - 0x0003FFFF */
#define FLASH_HP_CF_BLOCK_14              0x00040000U  /*   32 KB: 0x00040000 - 0x00047FFF */
// Code image - 224k - 7 block and each block has 32k
#define FLASH_HP_CF_BLOCK_15              0x00048000U  /*   32 KB: 0x00048000 - 0x0004FFFF */
#define FLASH_HP_CF_BLCOK_16              0x00050000U  /*   32 KB: 0x00050000 - 0x00057FFF */
#define FLASH_HP_CF_BLOCK_17              0x00058000U  /*   32 KB: 0x00058000 - 0x0005FFFF */
#define FLASH_HP_CF_BLOCK_18              0x00060000U  /*   32 KB: 0x00060000 - 0x00067FFF */
#define FLASH_HP_CF_BLCOK_19              0x00068000U  /*   32 KB: 0x00068000 - 0x0006FFFF */
#define FLASH_HP_CF_BLOCK_20              0x00708000U  /*   32 KB: 0x00708000 - 0x00077FFF */
#define FLASH_HP_CF_BLCOK_21              0x00078000U  /*   32 KB: 0x00078000 - 0x0007FFFF */

#define CODE_PAGE_SIZE                      (128)
#define MAX_CODE_PAGES                      (1792)          // 224k / 128 = 1792
#define APPLICATION_START_ADDRESS           FLASH_HP_CF_BLOCK_8
#define TOTAL_CODE_FLASH_BLOCK              (7)             // Application and code image use the same amount flash blocks
#define TOTAL_CODE_FLASH_BYTES              (32*1024*7)
#define CODE_IMAGE_START_ADDRESS            FLASH_HP_CF_BLOCK_15

//-------------------------------------------------------------------------------------------

#define FLASH_HP_DF_BLOCK_SIZE            (64)
/* Data Flash 8k total. So it has up to 8 * 1024 / 64 = 128 blocks.*/
#define FLASH_HP_DF_BLOCK_0               0x08000000U /*   64 B */
#define FLASH_HP_DF_BLOCK_1               0x08000040U /*   64 B */
#define FLASH_HP_DF_BLOCK_2               0x08000080U /*   64 B */
#define FLASH_HP_DF_BLOCK_3               0x080000C0U /*   64 B */
#define FLASH_HP_DF_BLOCK_4               0x08000100U /*   64 B */
#define FLASH_HP_DF_BLOCK_5               0x08000140U /*   64 B */
#define FLASH_HP_DF_BLOCK_6               0x08000180U /*   64 B */
#define FLASH_HP_DF_BLOCK_7               0x080001C0U /*   64 B */
#define FLASH_HP_DF_BLOCK_8               0x08000200U /*   64 B */
#define FLASH_HP_DF_BLOCK_9               0x08000240U /*   64 B */
#define FLASH_HP_DF_BLOCK_10              0x08000280U /*   64 B */
#define FLASH_HP_DF_BLOCK_11              0x080002C0U /*   64 B */
#define FLASH_HP_DF_BLOCK_12              0x08000300U /*   64 B */
#define FLASH_HP_DF_BLOCK_13              0x08000340U /*   64 B */
#define FLASH_HP_DF_BLOCK_14              0x08000380U /*   64 B */
#define FLASH_HP_DF_BLOCK_15              0x080003C0U /*   64 B */
#define FLASH_HP_DF_BLOCK_16              0x08000400U /*   64 B */
#define FLASH_HP_DF_BLOCK_17              0x08000440U /*   64 B */
#define FLASH_HP_DF_BLOCK_18              0x08000480U /*   64 B */

#define BOOTLOADER_INFOR_START_ADDRESS          FLASH_HP_DF_BLOCK_0
#define TOTAL_BOOTLOADER_BLOCK                  (1)                     // We give 64 to bootloader information. It has total 1 block and each block has 64 bytes

#define EEP_FEATURE_ADDRESS                     FLASH_HP_DF_BLOCK_1     // 64 bytes for eepFeature
#define EEP_PROBE_ADDRESS                       FLASH_HP_DF_BLOCK_2     // 64 bytes for eepProbe_t
#define EEP_RELAY_ADDRESS                       FLASH_HP_DF_BLOCK_3     // 64 bytes for eepIo_t
#define EEP_PROFILE_NAME_ADDRESS                FLASH_HP_DF_BLOCK_4     // 64 bytes for eepProfileName_t
#define EEP_DIGITAL_INPUT_ADDRESS               FLASH_HP_DF_BLOCK_5     // 64 bytes for SEepZone_t for zone 1
#define EEP_PROBE_CALIBRATION_ADDRESS           FLASH_HP_DF_BLOCK_6     // 64 bytes for SEepZone_t for zone 2

#define EEP_ZONE_1_START_ADDRESS                FLASH_HP_DF_BLOCK_7     // 64 bytes for SEepZone_t for zone 3
#define EEP_ZONE_2_START_ADDRESS                FLASH_HP_DF_BLOCK_8     // 64 bytes for SEepZone_t for zone 4
#define EEP_ZONE_3_START_ADDRESS                FLASH_HP_DF_BLOCK_9     // 64 bytes for SEepZone_t for zone 3
#define EEP_ZONE_4_START_ADDRESS                FLASH_HP_DF_BLOCK_10    // 64 bytes for SEepZone_t for zone 4

#define EEP_PROBE_ERROR_1                       FLASH_HP_DF_BLOCK_11     // 64 bytes for SEepProbeError_t probe error 1
#define EEP_PROBE_ERROR_2                       FLASH_HP_DF_BLOCK_12     // 64 bytes for SEepProbeError_t probe error 2
#define EEP_PROBE_ERROR_3                       FLASH_HP_DF_BLOCK_13     // 64 bytes for SEepProbeError_t probe error 3
#define EEP_PROBE_ERROR_4                       FLASH_HP_DF_BLOCK_14     // 64 bytes for SEepProbeError_t probe error 4
#define EEP_PROBE_ERROR_5                       FLASH_HP_DF_BLOCK_15     // 64 bytes for SEepProbeError_t probe error 5
#define EEP_PROBE_ERROR_6                       FLASH_HP_DF_BLOCK_16     // 64 bytes for SEepProbeError_t probe error 6
#define EEP_PROBE_ERROR_7                       FLASH_HP_DF_BLOCK_17     // 64 bytes for SEepProbeError_t probe error 7
#define EEP_PROBE_ERROR_8                       FLASH_HP_DF_BLOCK_18     // 64 bytes for SEepProbeError_t probe error 8

#define TOTAL_EEPFEATURE_BLOCK                  (1)                    // We give 1 block to eepFeature.

//-------------------------------------------------------------------------------------------
#define BOOT_LOADER_STATE_COPY_CODE       (12111999)            // Copy code from code image to application area if the state is this value.
#define BLOCK_SIZE                        (128)

extern fsp_err_t fpSaveDataFlash(uint32_t dataAddress,uint32_t dataSize, uint32_t flashAddress,uint16_t numFlashBlock);
extern void fpReadDataFlash(uint32_t dataAddress,uint32_t dataSize,uint32_t flashAddress);
extern fsp_err_t fpEraseCodeFlash(uint32_t flashAddress,uint32_t numFlashBlock);
extern fsp_err_t fpWriteCodePage(uint32_t desAddress,uint8_t * pCode);

#endif /* FLASHPROGRAM_APP_H_ */
