#include "communication_thread0.h"
#include "uart/uart_ep.h"
#include "common_utils.h"
#include "watchdog.h"
#include "flashProgram.h"
#include "firmwareupgrade.h"

SemaphoreHandle_t xUartTxSemaphore = NULL;
StaticSemaphore_t xUartTxSemaphoreBuffer;

/* Com Thread entry function */
/* pvParameters contains TaskHandle_t */
void communication_thread0_entry(void *pvParameters)
{
    FSP_PARAMETER_NOT_USED (pvParameters);

    fsp_err_t err = FSP_SUCCESS;
    uint8_t Data;

    // Init and start watchdog with 5.3seconds using PCLKB 25mHz
     R_WDT_Open(&g_wdt0_ctrl, &g_wdt0_cfg);
     R_WDT_Refresh(&g_wdt0_ctrl);


     /* Create a binary semaphore without using any dynamic memory
         allocation.  The semaphore's data structures will be saved into
         the xSemaphoreBuffer variable. */
     xUartTxSemaphore = xSemaphoreCreateBinaryStatic( &xUartTxSemaphoreBuffer );

     xSemaphoreGive(xUartTxSemaphore);

     /* Initializing UART */
     err = uart_initialize();

     if (FSP_SUCCESS != err)
     {
         APP_ERR_TRAP(err);
     }

#if 1
     /* User defined function to demonstrate UART functionality */
     err = uart_ep();
     if (FSP_SUCCESS != err)
     {
         deinit_uart();
         APP_ERR_TRAP(err)
     }
#endif

     while (1)
     {
         R_WDT_Refresh(&g_wdt0_ctrl);
#if 0
         Data = uart0GetRxByte();
         if(Data == STX)
         {
             FirmwareUpgrade();
         }
#endif
         vTaskDelay (1);
     }
}
