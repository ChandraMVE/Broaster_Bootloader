/***********************************************************************************************************************
 * File Name    : uart_ep.h
 * Description  : Contains function declaration of uart_ep.c and Macros.
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

#ifndef UART_EP_H_
#define UART_EP_H_

#include "common_utils.h"
#include "../firmwareupgrade.h"

/* Macro definition */
#define DATA_LENGTH               (1u)      /* Expected Input Data length */
#define UART_ERROR_EVENTS         (UART_EVENT_BREAK_DETECT | UART_EVENT_ERR_OVERFLOW | UART_EVENT_ERR_FRAMING | \
                                    UART_EVENT_ERR_PARITY)    /* UART Error event bits mapped in registers */

#define RX_LEN  1024
#define TX_LEN  1024

/********************************************************************
* Function Name: SPort_t
* Description  : Definition for port
*********************************************************************/
typedef struct
{
    uint16_t rxIn;
    uint16_t rxOut;
    uint8_t rxBuf[RX_LEN];

    //uint16_t txIn;
    //uint16_t txOut;
    //uint8_t txBuf[TX_LEN];
}SPort_t;

static SPort_t _port = {0};

extern uint8_t g_temp_buffer[DATA_LENGTH];
extern bool key_event;
extern SemaphoreHandle_t xUartTxSemaphore;

/* Function declaration */
fsp_err_t uart_ep(void);
fsp_err_t uart_initialize(void);
void deinit_uart(void);

#endif /* UART_EP_H_ */