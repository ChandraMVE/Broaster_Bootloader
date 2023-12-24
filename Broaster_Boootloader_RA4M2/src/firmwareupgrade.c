/*
 * firmwareupgrade.c
 *
 *  Created on: 15-Dec-2023
 *      Author: Dell
 */
#include "firmwareupgrade.h"
#include "flashProgram_app.h"

extern bool jumpToApp;
/********************************************************************
* Function Name: _getUint16FromHighLow
* Description  : Return uint16_t value from high and low bytes
* Arguments    : bytes HIGH and LOW
* Return Value : uint16_t result
*********************************************************************/
static uint16_t _getUint16FromHighLow(uint8_t High,uint8_t Low)
{
    uint16_t Data = (uint16_t)High;

    Data <<= 8;
    Data |= (uint16_t)Low;
    return Data;
}

/********************************************************************
* Function Name: _crc_16bit
* Description  : Calculate CRC
* Arguments    : CRCData - Existing CRC value
*                octet -  new data
* Return Value : new CRC value
*********************************************************************/
static uint16_t crc_16bit( uint16_t CRCData, uint16_t octet )
{
    uint16_t    crcLow = (uint16_t)((CRCData & 0xff) ^ (octet & 0xff));

    CRCData = (uint16_t)((CRCData >> 8) ^
                (crcLow << 8) ^
                (crcLow << 3) ^
                (crcLow << 12) ^
                (crcLow >> 4) ^
                (crcLow & 0x0f) ^
                ((crcLow & 0x0f) << 7 ));

    return CRCData;
}

//*************************************************************************************
//  JSON routines
//-------------------------------------------------------------------------------------
/********************************************************************
* Function Name: _addByte2Buf
* Description  : Add byte to buffer.
* Arguments    : pS - pointer to buf
*                Data - Data byte needs to be added to buf.
* Return Value : None
*********************************************************************/
static void _addByte2Buf(uint8_t * pS,uint8_t Data)
{
    *(pS + packet.txIn) = Data;
    packet.txIn ++;
}

/********************************************************************
* Function Name: _addCrcByte2TxBuf
* Description  : Add byte to buf and calculate the CRC at same time
* Arguments    : pS - pointer to buf
*                Data - Data byte needs to be added to buf.
* Return Value : None
*********************************************************************/
static void _addCrcByte2Buf(uint8_t * pS,uint8_t Data)
{
    packet.localTxCrc = crc_16bit(packet.localTxCrc,Data);
    *(pS + packet.txIn) = Data;
    packet.txIn ++;
}

//*************************************************************************************
//  COMMAND DEFINITIONS END
//-------------------------------------------------------------------------------------
static void _sendPacketResponse2Uart(uint8_t * pS)
{
#if 0
    if (xSemaphoreTake(xUartTxSemaphore, 5000 / portTICK_PERIOD_MS) == pdPASS)
    {
        uart0SendPacket(pS, packet.txIn);
    }
    else
    {
        // Uart is not released after 5 seconds. Something is not right. We should reset MCU here?
        // reset mcu
        NVIC_SystemReset();
    }
#endif
}

/********************************************************************
* Function Name: _setFirmwareUpgradeResponse
* Description  : Send firmware upgrade response
* Arguments    : errorCode
* Return Value : None
*********************************************************************/
static void _setFirmwareUpgradeResponse(uint8_t resCode)
{
    // Wait 25ms for uart to complete the packet response.
    R_BSP_SoftwareDelay(25U, BSP_DELAY_UNITS_MILLISECONDS);

    packet.localTxCrc = 0;
    packet.txIn = 0;
    _addByte2Buf(packet.packetResponseBuf,STX);

    _addCrcByte2Buf(packet.packetResponseBuf,LOW_BYTE(packet.Version));
    _addCrcByte2Buf(packet.packetResponseBuf,HIGH_BYTE(packet.Version));
    _addCrcByte2Buf(packet.packetResponseBuf,0x83);     // XB on 4/11/2023. Use fixed length 131 for this response.
    _addCrcByte2Buf(packet.packetResponseBuf,0);
    _addCrcByte2Buf(packet.packetResponseBuf,LOW_BYTE(packet.transId));
    _addCrcByte2Buf(packet.packetResponseBuf,HIGH_BYTE(packet.transId));
    _addCrcByte2Buf(packet.packetResponseBuf,'D');
    _addCrcByte2Buf(packet.packetResponseBuf,0x01);
    _addCrcByte2Buf(packet.packetResponseBuf,0xfe);
    _addCrcByte2Buf(packet.packetResponseBuf,0xff);
    _addCrcByte2Buf(packet.packetResponseBuf,resCode);
    for(uint8_t i = 0; i < 127; i ++)
    {   // Send 127 don't care bytes. send 0x55 to prevent com to lose synchronization.
        _addCrcByte2Buf(packet.packetResponseBuf,0x55);
    }
    _addByte2Buf(packet.packetResponseBuf,LOW_BYTE(packet.localTxCrc));
    _addByte2Buf(packet.packetResponseBuf,HIGH_BYTE(packet.localTxCrc));
    _addByte2Buf(packet.packetResponseBuf,ETX);

    // Stuff following ETXs to prevent missing ETX caused by following command response.
    _addByte2Buf(packet.packetResponseBuf,ETX);
    _addByte2Buf(packet.packetResponseBuf,ETX);

    _sendPacketResponse2Uart(packet.packetResponseBuf);
}

void CopySckToUpgradePayloadPkt(uint8_t *dataBufferPtr)
{
    memcpy(packet.dataBuf, dataBufferPtr, 130);
}

void FirmwareUpgrade(void)
{
    fsp_err_t err;
    uint8_t resCode = FIRMWARE_UPGRADE_RES_SUCCESS;
    //uint32_t page = _getUint16FromHighLow(packet.dataBuf[2],packet.dataBuf[1]);
    uint32_t page = _getUint16FromHighLow(packet.dataBuf[1],packet.dataBuf[0]);
    uint32_t desAddress;
    static SBootInfor_t bootInfor = {0};

    // Feed the dog
    R_WDT_Refresh(&g_wdt0_ctrl);

    if((page >= FIRMWARE_UPGRADE_CODE_START_PAGE) &&
                (page <= FIRMWARE_UPGRADE_CODE_END_PAGE))
    {   // Regular code page needs to be programmed to code image area
        desAddress = (page - FIRMWARE_UPGRADE_CODE_START_PAGE) * CODE_PAGE_SIZE + APPLICATION_START_ADDRESS;

        // Feed the dog
        R_WDT_Refresh(&g_wdt0_ctrl);

        //err = fpWriteCodePage(desAddress,&packet.dataBuf[3]);
        err = fpWriteCodePage(desAddress,&packet.dataBuf[2]);
        if(err == FSP_SUCCESS)
        {
#if 0
            // Verify the writing
            memcpy(packet.dataBuf, (uint8_t *) desAddress, CODE_PAGE_SIZE);
            for(page = 0; page < CODE_PAGE_SIZE; page ++)
            {
                bootInfor.Crc = crc_16bit(bootInfor.Crc,packet.dataBuf[page]);
            }
            bootInfor.totalPage ++;
#endif
        }
        else
        {   // Send "error. Need reboot MCU???
            resCode = FIRMWARE_UPGRADE_RES_FLASH_WRITE_FAILED;
        }

    }
    else if(page == FIRMWARE_UPGRADE_START_PAGE)
    {   // First page received. We need start SIB firmware upgrade
        // Initial boot infor
        bootInfor.Crc = 0;
        bootInfor.totalPage = 0;

        // Feed the dog
        R_WDT_Refresh(&g_wdt0_ctrl);

        // Erase code image
        err = fpEraseCodeFlash(APPLICATION_START_ADDRESS,TOTAL_CODE_FLASH_BLOCK);
        if(err == FSP_SUCCESS)
        {   // Send "Good"

        }
        else
        {   // Send "Error". Need reboot MCU???
            resCode = FIRMWARE_UPGRADE_RES_FLASH_ERASE_FAILED;
        }
    }
    else if(page == FIRMWARE_UPGRADE_END_PAGE)
    {
#if 0
        // Last page recived. SIB firmware upgrade completed. We need check total page and CRC
        uint16_t crc = _getUint16FromHighLow(packet.dataBuf[4],packet.dataBuf[3]);
        uint16_t totalPage = _getUint16FromHighLow(packet.dataBuf[6],packet.dataBuf[5]);
        if((crc == bootInfor.Crc) && (totalPage == bootInfor.totalPage))
        {
            bootInfor.State = BOOT_LOADER_STATE_COPY_CODE;

            _setFirmwareUpgradeResponse(FIRMWARE_UPGRADE_RES_SUCCESS);

            // Wait 20ms for uart to complete the packet response.
            R_BSP_SoftwareDelay(20U, BSP_DELAY_UNITS_MILLISECONDS);

            // reset mcu
            NVIC_SystemReset();
        }
        else
        {   // CRC and total page number are not matched. So firmware upgrade failed
            resCode = FIRMWARE_UPGRADE_RES_WRONG_CRC_OR_TOTAL_PAGE;
        }
#endif
        jumpToApp = true;

    }
    else
    {   // Something is wrong! We need send error state
        resCode = FIRMWARE_UPGRADE_RES_WRONG_PAGE_NUMBER;
    }

    //_setFirmwareUpgradeResponse(resCode);
}
