/***********************************************************************************************************************
 * File Name    : uart.c
 * Description  : Contains UART functions definition.
 **********************************************************************************************************************/
/***********************************************************************************************************************
 * DISCLAIMER
 * This software is supplied by Renesas Electronics Corporation and is only intended for use with Renesas products. No
 * other uses are authorized. This software is owned by Renesas Electronics Corporation and is protected under all
 * applicable laws, including copyright laws.
 * THIS SOFTWARE IS PROVIDED "AS IS" AND RENESAS MAKES NO WARRANTIES REGARDING
 * THIS SOFTWARE, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. ALL SUCH WARRANTIES ARE EXPRESSLY DISCLAIMED. TO THE MAXIMUM
 * EXTENT PERMITTED NOT PROHIBITED BY LAW, NEITHER RENESAS ELECTRONICS CORPORATION NOR ANY OF ITS AFFILIATED COMPANIES
 * SHALL BE LIABLE FOR ANY DIRECT, INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR ANY REASON RELATED TO THIS
 * SOFTWARE, EVEN IF RENESAS OR ITS AFFILIATES HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
 * Renesas reserves the right, without notice, to make changes to this software and to discontinue the availability of
 * this software. By using this software, you agree to the additional terms and conditions found by accessing the
 * following link:
 * http://www.renesas.com/disclaimer
 *
 * Copyright (C) 2020 Renesas Electronics Corporation. All rights reserved.
 ***********************************************************************************************************************/

#include "common_utils.h"
#include "uart_ep.h"
#include "firmwareupgrade.h"
#include "SEGGER_RTT/SEGGER_RTT.H"

extern bool jumpToApp;
bool uart0_TX_ENABLE = 0;

void uart0TxInterrupt(void);
void uart0TxUint8(uint8_t data);

/*******************************************************************************************************************//**
 * @addtogroup r_sci_uart_ep
 * @{
 **********************************************************************************************************************/

/* Counter to update g_temp_buffer index */
static volatile uint8_t g_counter_var = RESET_VALUE;

/* Flag to check whether data is received or not */
static volatile uint8_t g_data_received_flag = 0;

/* Flag for user callback */
static volatile uint8_t g_uart_event = RESET_VALUE;

uint8_t g_temp_buffer[DATA_LENGTH] = {RESET_VALUE};

/*****************************************************************************************************************
 *  @brief       UART Example project to demonstrate the functionality
 *  @param[in]   None
 *  @retval      FSP_SUCCESS     Upon success
 *  @retval      Any Other Error code apart from FSP_SUCCESS
 ****************************************************************************************************************/
fsp_err_t uart_ep(void)
{
    fsp_err_t err = FSP_SUCCESS;

    while (1)
    {
        sckFastlanV5CheckRx();
        R_WDT_Refresh(&g_wdt0_ctrl);

        if(uart0_TX_ENABLE == true)
        {
              uart0TxInterrupt();
              R_BSP_SoftwareDelay(1U, BSP_DELAY_UNITS_MILLISECONDS);
        }

        if(jumpToApp == true)
        {
            break;
        }
    }
    return err;
}

/*******************************************************************************************************************//**
 * @brief       Initialize  UART.
 * @param[in]   None
 * @retval      FSP_SUCCESS         Upon successful open and start of timer
 * @retval      Any Other Error code apart from FSP_SUCCESS  Unsuccessful open
 ***********************************************************************************************************************/
fsp_err_t uart_initialize(void)
{
    fsp_err_t err = FSP_SUCCESS;

    /* Initialize UART channel with baud rate 115200 */
    err = R_SCI_UART_Open (&g_uart0_ctrl, &g_uart0_cfg);
    if (FSP_SUCCESS != err)
    {
        APP_ERR_PRINT ("\r\n**  R_SCI_UART_Open API failed  **\r\n");
    }
    return err;
}


/*******************************************************************************************************************//**
 *  @brief       Deinitialize SCI UART module
 *  @param[in]   None
 *  @retval      None
 **********************************************************************************************************************/
void deinit_uart(void)
{
    fsp_err_t err = FSP_SUCCESS;

    /* Close module */
    err =  R_SCI_UART_Close (&g_uart0_ctrl);
    if (FSP_SUCCESS != err)
    {
        APP_ERR_PRINT ("\r\n**  R_SCI_UART_Close API failed  ** \r\n");
    }
}


#define uart0_RX_BUF_SIZE 512
#define uart0_TX_BUF_SIZE 512
#define STX 0x02
#define ETX 0x03
#define MAX_PAYLOAD 512
#define BYTE16_LOW(n)     (unsigned char)((n) & 0xff)
#define BYTE16_HIGH(n)    (unsigned char)(BYTE16_LOW((n) >> 8))

typedef enum
{
    SCK_PACK_COMMAND_FASTLAN_OX_FF55 = 0xff55,
    SCK_PACK_COMMAND_TYPE_C = 'C',
    SCK_PACK_COMMAND_TYPE_S = 'S',
}SSckPackCommand;

typedef struct
{
    uint16_t rxIn;                      // Index for RX input
    uint16_t rxOut;                     // Index for RX output
    //uint16_t rxCmd;                       // Index for start of the command
    uint8_t rxBuf[uart0_RX_BUF_SIZE];   // RX data buffer

    uint16_t txIn;                      // Index for TX input
    uint16_t txOut;                     // Index for TX output
    uint8_t txBuf[uart0_TX_BUF_SIZE];   // TX data buffer

    uint8_t resetCpu  : 1;              // Reset MCU if it is 1. Add this to support firmware upgrade. XB on 10/10/2019
    uint8_t reflashing: 1;              // Code is running from boot loader if it is 1. Add this to support firmware upgrade over RS232. XB on 10/10/2019
}SUart;

typedef struct
{
    uint8_t rxState;                    // Rx state for incoming package
    uint8_t Status;                     // Status for incoming package detection
    uint8_t TID;
    uint16_t Length;                    // Length of command and payload
    uint16_t Command;                   // Command ID
    uint8_t command_C_S;                // 'C' or 'S'
    uint16_t rxCounter;                 // rx data index of data buffer
    uint16_t Version;                   // Firmware version
    uint16_t localCRC;                  // Local calculated CRC
    uint16_t rxCRC;                     // CRC in incoming package
    uint8_t Payload[MAX_PAYLOAD];       // Data buffer for payload
}SSckFastlan;

typedef enum
{
    RX_STX = 0,
    RX_VERSION_LOW,         // 2 Byte representing version number (3-bits Major,5-bits Minor, 8-bits Maintenance)
    RX_VERSION_HIGH,
    RX_LENGTH_LOW,          // 2 bytes specifying the length of the command and payload portion of the packet. The length field is sent most Least byte (LSB) first followed by the MSB
    RX_LENGTH_HIGH,         // significant byte (MSB). Valid length field values are 1 to 1023.
    RX_TID,                 // 1 byte Transaction ID
    RX_C_OR_S,              // ‘C’ for command Frame and S for Status Frame.
    RX_COMMAND_LOW,         // 2 byte field used to specify (in a C message) the command or (in a S message) the status.
    RX_COMMAND_HIGH,
    RX_DATA_PAYLOAD,        // The payload field size is specified in the LENGTH-3. The payload format is specific to the command or status command.
    RX_CRC_LOW,             // 2 Bytes containing the result of applying the CRC-16 algorithm upon the LENGTH, C or S, COMMAND and PAYLOAD fields of the packet. The CRC-16 algorithm is
    RX_CRC_HIGH,            // defined in the Appendices of this document.
    RX_ETX,                 // 1 Byte consisting of 0x03 (ETX)
}ESckFastlanRxState;

typedef enum
{
    STATUS_NONE = 0x0000,
    STATUS_AWAKE = 0x0001,
    STATUS_UNKNOW_CMD = 0x0002,
    STATUS_REFLASH_SUCCESS = 0x0003,
    STATUS_REFLASH_FAIL = 0x0004,
    STATUS_ERASE_FAIL = 0x0005,
    STATUS_ACK = 0x0006,
    STATUS_FLASH_DATA_READ = 0x0007,
    STATUS_FLASH_WRITE_FAIL = 0x0008,
    STATUS_FLASH_WRITE_SUCCESS = 0x0009,
    STATUS_BAD_CHECKSUM = 0x000A,
    STATUS_BAD_ETX = 0x000B,
    STATUS_BAD_LENGTH = 0x000C,
    STATUS_BAD_FLASH_ADDRESS = 0x000D,
    STATUS_BAD_MESSAGE_TYPE = 0x000E,
    STATUS_SUCCESS = 0x0011,
    STATUS_FAILURE = 0x0012,
    STATUS_NACK = 0x0015,
    STATUS_MEMORY_DATA = 0x0024,
}ESckFastlanStatus;

static uint16_t _crc16bit(uint16_t localCRC,uint8_t octet);
void _processSckFastlanCommands(void);

static volatile SUart    uart0;
SSckFastlan sck;

/******************************************************************
Description: This interrupt routine sends a byte through
                     the SCI 2 TX register.
Inputs:      None
Outputs:     None
******************************************************************/
void uart0TxInterrupt(void)
{
    if (uart0.txIn != uart0.txOut)
    {
       uint8_t tx_buffer = (uint8_t)(uart0.txBuf[uart0.txOut] & 0xFF);

       R_SCI_UART_Write (&g_uart0_ctrl, &tx_buffer, 1);

       APP_PRINT("0x%02x ", tx_buffer);

       if (++uart0.txOut >= uart0_TX_BUF_SIZE)
       {
          uart0.txOut = 0;
       }
    }
    else
    {
        APP_PRINT("\r\n");
       uart0_TX_ENABLE = 0;
    }
}

//void uart0RxInterrupt(uart_callback_args_t *p_args)
void user_uart_callback(uart_callback_args_t *p_args)
{
   /* Logged the event in global variable */
   g_uart_event = (uint8_t)p_args->event;

   if(UART_EVENT_RX_CHAR == p_args->event)
   {
       int data = (uint8_t ) p_args->data;
       uart0.rxBuf[uart0.rxIn++] = (uint8_t)(data & 0xFF);
       if (uart0.rxIn >= uart0_RX_BUF_SIZE)
       {
          uart0.rxIn = 0;
       }
   }
}

void uart0SckTxUint8(uint8_t Data,SSckFastlan * P)
{
    uart0TxUint8(Data);
    P->localCRC = _crc16bit(P->localCRC,Data);
}

void uart0SckTxUint16(uint16_t Data,SSckFastlan * P)
{
    uart0SckTxUint8(BYTE16_LOW(Data),P);
    uart0SckTxUint8(BYTE16_HIGH(Data),P);
}

void uart0TxUint8(uint8_t data)
{
   uart0.txBuf[uart0.txIn++] = data;
   if (uart0.txIn >= uart0_TX_BUF_SIZE)
   {
      uart0.txIn = 0;
   }
}

void uart0TxUint16(uint16_t Data)
{
    uart0TxUint8(BYTE16_LOW(Data));
    uart0TxUint8(BYTE16_HIGH(Data));
}

void uart0TxStart(void)
{
   uart0_TX_ENABLE = 1;
}

uint8_t uart0RxDataReady(void)
{
    if(uart0.rxIn != uart0.rxOut)
    {
        return (1);
    }
    else
    {
        return(0);
    }
}

uint8_t uart0GetRxUint8(void)
{
    uint8_t rxData;

    rxData = uart0.rxBuf[uart0.rxOut ++];
    if (uart0.rxOut >= uart0_RX_BUF_SIZE)
    {
        uart0.rxOut = 0;
    }
    return (rxData);
}

uint8_t uart0TxIsInProgress(void)
{
    return uart0_TX_ENABLE;
}

void uart0ResponseStatus(uint16_t statusCode)
{
    sck.localCRC = 0;
    uart0TxUint8(STX);
    uart0TxUint16(sck.Version);
    uart0SckTxUint16(2,&sck);           // The length is just 2
    uart0TxUint8(sck.TID);
    uart0SckTxUint8(SCK_PACK_COMMAND_TYPE_S,&sck);
    uart0SckTxUint16(statusCode,&sck);
    uart0TxUint16(sck.localCRC);
    uart0TxUint8(ETX);
    uart0TxStart();                     // Start UART1 Tx interrupt and transmit all bytes out
}

static void _uart0ResponseStatusWithData(uint16_t statusCode,uint16_t dataLength, uint8_t *pData)
{
    uint16_t idx;

    sck.localCRC = 0;
    uart0TxUint8(STX);
    uart0TxUint16(sck.Version);
    uart0SckTxUint16(2 + dataLength,&sck);          // The length is 2 bytes command(statusCode) plus data length
    uart0TxUint8(sck.TID);
    uart0SckTxUint8(SCK_PACK_COMMAND_TYPE_S,&sck);
    uart0SckTxUint16(statusCode,&sck);
    for(idx = 0; idx < dataLength; idx ++)
    {
        uart0SckTxUint8(*(pData+ idx),&sck);
    }
    uart0TxUint16(sck.localCRC);
    uart0TxUint8(ETX);
    uart0TxStart();                     // Start UART1 Tx interrupt and transmit all bytes out
}

static void _uart0ResponseStatusWithUint8Data(uint16_t statusCode,uint8_t Data)
{
    uint8_t dataBuf[2];

    dataBuf[0] = Data;
    _uart0ResponseStatusWithData(statusCode,1,dataBuf);
}

static void _uart0ResponseStatusWithUint16Data(uint16_t statusCode,uint16_t Data)
{
    uint8_t dataBuf[2];

    dataBuf[0] = BYTE16_LOW(Data);
    dataBuf[1] = BYTE16_HIGH(Data);
    _uart0ResponseStatusWithData(statusCode,2,dataBuf);
}

void sckFastlanV5Init(void)
{
    sck.rxState = RX_STX;
}

static uint16_t _crc16bit(uint16_t localCRC,uint8_t octet)
{
    unsigned int crcLow;
    crcLow = (localCRC & 0xff) ^ (octet & 0xff);
    localCRC = (localCRC >> 8) ^ (crcLow << 8) ^ (crcLow << 3)
    ^ (crcLow << 12) ^ (crcLow >> 4) ^ (crcLow & 0x0f) ^ ((crcLow & 0x0f) << 7);
    return(localCRC);
}

void sckFastlanV5CheckRx(void)
{
    uint8_t rxData;

    if(0 == uart0RxDataReady())
    {
        return;
    }
    else
    {
        rxData = uart0GetRxUint8();
        APP_PRINT("0x%02x ", rxData);
    }

   switch(sck.rxState)
   {
      case RX_STX:
         if(STX == rxData)
         {
             sck.rxState  = RX_VERSION_LOW;
         }
         break;

      case RX_VERSION_LOW:
          sck.Version = rxData;
          sck.rxState  = RX_VERSION_HIGH;
         break;

      case RX_VERSION_HIGH:
          sck.Version |= ((uint16_t)rxData << 8);
          sck.localCRC = 0;
          sck.rxState  = RX_LENGTH_LOW;
         break;

      case RX_LENGTH_LOW:
          sck.Length = rxData;
          sck.localCRC = _crc16bit(sck.localCRC,rxData);
          sck.rxState = RX_LENGTH_HIGH;
         break;

      case RX_LENGTH_HIGH:
          sck.Length |= ((uint16_t)rxData << 8);
          if((sck.Length >= 2) &&
                  (sck.Length <= 1026))     // Length should be from 1 to 1023
          {
              sck.localCRC = _crc16bit(sck.localCRC,rxData);
              sck.Length -= 2;                  // The length should be the length of payload. We need minus 2 for command bytes
              sck.rxState  = RX_TID;
          }
          else
          { // Wrong length is received
              sck.Status = STATUS_BAD_LENGTH;
              sck.rxState = RX_STX;
          }
         break;

      case RX_TID:
          sck.TID = rxData;
          sck.rxState  = RX_C_OR_S;
         break;

      case RX_C_OR_S:
          sck.command_C_S = rxData;
          sck.localCRC = _crc16bit(sck.localCRC,rxData);
          sck.rxState  = RX_COMMAND_LOW;
         break;

      case RX_COMMAND_LOW:
          sck.Command = rxData;
          sck.localCRC = _crc16bit(sck.localCRC,rxData);
          sck.rxState = RX_COMMAND_HIGH;
          break;

      case RX_COMMAND_HIGH:
          sck.Command |= ((uint16_t)rxData << 8);
          sck.localCRC = _crc16bit(sck.localCRC,rxData);
          sck.rxCounter = 0;
          if(sck.Length != 0)
          {
            sck.rxState  = RX_DATA_PAYLOAD;
          }
          else
          {
              sck.rxState  = RX_CRC_LOW;
          }
          break;

      case RX_DATA_PAYLOAD:
          sck.Payload[sck.rxCounter ++] = rxData;
          sck.localCRC = _crc16bit(sck.localCRC,rxData);
          if(sck.rxCounter >= sck.Length )      //Check if we have accumulated all of the data.
          {
              sck.rxState  = RX_CRC_LOW;                //We have all of the data. Now go get the checksum.
          }
          break;

      case RX_CRC_LOW:
          sck.rxCRC = rxData;
          sck.rxState  = RX_CRC_HIGH;
          break;

      case RX_CRC_HIGH:
          sck.rxCRC |= ((uint16_t)rxData << 8);
          //if(sck.rxCRC == sck.localCRC)
          {
              sck.rxState  = RX_ETX;
          }
          //else
          {
              //sck.Status = STATUS_BAD_CHECKSUM;
              //sck.rxState = RX_STX;
          }
          break;

      case RX_ETX:
          APP_PRINT("\r\n");
          if(ETX == rxData)
          {
              _processSckFastlanCommands();
          }
          else
          {
              sck.Status = STATUS_BAD_ETX;
          }
          sck.rxState = RX_STX;
          break;

      default:
         sck.rxState = RX_STX;
         break;
   }
}

static void _setTxPackage2Port(uint16_t Command,SSckFastlan * P)
{
    uint16_t idx;

    P->localCRC = 0;
    uart0TxUint8(STX);
    uart0TxUint16(P->Version);
    uart0SckTxUint16(P->Length + 2,P);      // Need to add 2 bytes for command
    uart0TxUint8(P->TID);
    uart0SckTxUint8(P->command_C_S,P);
    uart0SckTxUint16(Command,P);
    for(idx = 0; idx < P->Length; idx ++)
    {
        uart0SckTxUint8(*(P->Payload + idx),P);
    }
    uart0TxUint16(P->localCRC);
    uart0TxUint8(ETX);
    uart0TxStart();                     // Start UART1 Tx interrupt and transmit all bytes out
}

void _processSckFastlanCommands(void)
{
    while(1 == uart0TxIsInProgress())
    {
        R_BSP_SoftwareDelay(2U, BSP_DELAY_UNITS_SECONDS);
        break;
    }
    switch(sck.Command)
    {
        case SCK_PACK_COMMAND_FASTLAN_OX_FF55:
        {
            uint8_t *dataBufferPtr;
            dataBufferPtr = &(sck.Payload[0]);
            CopySckToUpgradePayloadPkt(dataBufferPtr);
            FirmwareUpgrade();

            sck.command_C_S = SCK_PACK_COMMAND_TYPE_S;
            _setTxPackage2Port(SCK_PACK_COMMAND_FASTLAN_OX_FF55,&sck);
        }
        break;

        default:
        {

        }
        break;
    }
}

/*******************************************************************************************************************//**
 * @} (end addtogroup r_sci_uart_ep)
 **********************************************************************************************************************/

