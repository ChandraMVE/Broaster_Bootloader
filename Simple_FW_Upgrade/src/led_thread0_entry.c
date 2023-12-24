#include "led_thread0.h"
#include "watchdog.h"
/* Led Thread entry function */
/* pvParameters contains TaskHandle_t */
void led_thread0_entry(void *pvParameters)
{
    bool value = true;
    FSP_PARAMETER_NOT_USED (pvParameters);

    /* TODO: add your own code here */
    while (1)
    {
        R_WDT_Refresh(&g_wdt0_ctrl);
        R_BSP_PinAccessEnable();
        R_BSP_PinWrite(BSP_IO_PORT_04_PIN_15,value);
        R_BSP_PinWrite(BSP_IO_PORT_04_PIN_04,value);
        R_BSP_PinWrite(BSP_IO_PORT_01_PIN_14,value);
        R_BSP_PinWrite(BSP_IO_PORT_04_PIN_05,value);
        R_BSP_PinWrite(BSP_IO_PORT_06_PIN_08,value);
        R_BSP_PinWrite(BSP_IO_PORT_01_PIN_15,value);
        R_BSP_PinAccessDisable();
        R_BSP_SoftwareDelay(50U, BSP_DELAY_UNITS_MILLISECONDS);
        value = !value;
        vTaskDelay (10);
    }
}
