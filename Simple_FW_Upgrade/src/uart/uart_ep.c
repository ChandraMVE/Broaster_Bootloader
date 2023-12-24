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
#include "FreeRTOS.h"
#include "task.h"

/*******************************************************************************************************************//**
 * @addtogroup r_sci_uart_ep
 * @{
 **********************************************************************************************************************/

/* Counter to update g_temp_buffer index */
static volatile uint8_t g_counter_var = RESET_VALUE;

/* Flag to check whether data is received or not */
static volatile uint8_t g_data_received_flag = false;

/* Flag for user callback */
static volatile uint8_t g_uart_event = RESET_VALUE;

uint8_t g_temp_buffer[DATA_LENGTH] = {RESET_VALUE};
bool key_event = false;
uint8_t message1[] = "\n\rEntered key = ";
uint8_t message2[] = "\n\r";

/*****************************************************************************************************************
 *  @brief       UART Example project to demonstrate the functionality
 *  @param[in]   None
 *  @retval      FSP_SUCCESS     Upon success
 *  @retval      Any Other Error code apart from FSP_SUCCESS
 ****************************************************************************************************************/
fsp_err_t uart_ep(void)
{
    fsp_err_t err = FSP_SUCCESS;

    while (true)
    {
        if(g_data_received_flag)
        {
            g_data_received_flag  = false;
            key_event = true;

            err = R_SCI_UART_Write (&g_uart0_ctrl, message1, sizeof(message1));
            vTaskDelay (10);
            err = R_SCI_UART_Write (&g_uart0_ctrl, g_temp_buffer, sizeof(g_temp_buffer));
            vTaskDelay (10);
            err = R_SCI_UART_Write (&g_uart0_ctrl, message2, sizeof(message2));
            vTaskDelay (10);
            err = R_SCI_UART_Write (&g_uart0_ctrl, message2, sizeof(message2));
            vTaskDelay (10);
            if (FSP_SUCCESS != err)
            {
                APP_ERR_PRINT ("\r\n **  UART WRITE FAILED  ** \r\n");
                return err;
            }
        } 
    }
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

/*****************************************************************************************************************
 *  @brief      UART user callback
 *  @param[in]  p_args
 *  @retval     None
 ****************************************************************************************************************/
void uart2RxInterrupt(uart_callback_args_t *p_args)
{
#if 0
    if(UART_EVENT_RX_CHAR == p_args->event)
    {
        _port.rxBuf[_port.rxIn ++] = (uint8_t ) p_args->data;
        if(_port.rxIn >= RX_LEN)
        {
            _port.rxIn = 0;
        }
    }
    else if(UART_EVENT_TX_DATA_EMPTY == p_args->event)
    {
//        _port.txIn = 0;         // Reset tx in index for next tx
        xSemaphoreGiveFromISR(xUartTxSemaphore, NULL);
    }
#endif

    /* Logged the event in global variable */
    g_uart_event = (uint8_t)p_args->event;

    if(UART_EVENT_RX_CHAR == p_args->event)
    {
        g_temp_buffer[g_counter_var++] = (uint8_t ) p_args->data;
    }

    /* Reset g_temp_buffer index if it exceeds than buffer size */
    if(DATA_LENGTH == g_counter_var)
    {
        g_counter_var = RESET_VALUE;
        g_data_received_flag  = true;
    }
}

/********************************************************************
* Function Name: uart0SendPacket
* Description  : Send data in TX buffer to serial port
* Arguments    : None
* Return Value : None
*********************************************************************/
void uart0SendPacket(uint8_t * pS,uint32_t length)
{
//    vTaskDelay(1000 / portTICK_PERIOD_MS);       // Wait 600ms
    if(length != 0)
    {
        R_SCI_UART_Write (&g_uart0_ctrl, pS, length);
    }
}

/********************************************************************
* Function Name: uart0GetRxByte
* Description  : return one byte from rx buffer
* Arguments    : None
* Return Value : data
*********************************************************************/
uint8_t uart0GetRxByte(void)
{
    uint8_t Data = _port.rxBuf[_port.rxOut ++];

    if(_port.rxOut >= RX_LEN)
    {
        _port.rxOut = 0;
    }
    return Data;
}
/*******************************************************************************************************************//**
 * @} (end addtogroup r_sci_uart_ep)
 **********************************************************************************************************************/
