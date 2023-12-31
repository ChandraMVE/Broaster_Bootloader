#include "hal_data.h"
#include <flashProgram.h>
#include "uart/uart_ep.h"

FSP_CPP_HEADER
void R_BSP_WarmStart(bsp_warm_start_event_t event);
FSP_CPP_FOOTER

//#define APPLICATION_START_ADDRESS 0x2000

void JumpToapp (void);
void start_app(uint32_t pc, uint32_t sp) __attribute__((naked, no_instrument_function, \
        no_profile_instrument_function));

/* This function jumps to the application image. */
void start_app (uint32_t pc __attribute__((unused)), uint32_t sp __attribute__((unused))) {
    /* This is a naked/stackless function. Do not pass arguments to the inline assembly when the GCC compiler is
     * used. */
    __asm volatile (

        /* Set stack pointer. */
        "MSR     MSP, R1                         \n"
        "DSB                                     \n"
        "ISB                                     \n"

        /* Branch to application. */
        "BX      R0                              \n"
        );
}

void JumpToapp (void) {
   uint32_t * app_image = (uint32_t *) APPLICATION_START_ADDRESS;
   uint32_t   app_sp    = app_image[0];
   uint32_t   app_pc    = app_image[1];


   /* Set the applications vector table. */
   SCB->VTOR = APPLICATION_START_ADDRESS;

   /* Disable MSP monitoring. */
#if BSP_FEATURE_TZ_HAS_TRUSTZONE
   __set_MSPLIM(0);
#else
   R_MPU_SPMON->SP[0].CTL = 0;
#endif

   /* Set SP and branch to PC. */
   start_app(app_pc, app_sp);
}

bool jumpToApp = false;
/*******************************************************************************************************************//**
 * main() is generated by the RA Configuration editor and is used to generate threads if an RTOS is used.  This function
 * is called by main() when no RTOS is used.
 **********************************************************************************************************************/
void hal_entry(void)
{
    bool value = true;
    /* TODO: add your own code here */
    // Init watchdog to 5.3 seconds with PCLKB 25mHz
    R_WDT_Open(&g_wdt0_ctrl, &g_wdt0_cfg);
    // Start watchdog
    R_WDT_Refresh(&g_wdt0_ctrl);

    R_FLASH_HP_Open(&g_flash0_ctrl, &g_flash0_cfg);

    for(int i = 0; i<10; i++)
    {
        R_BSP_PinAccessEnable();
        R_BSP_PinWrite(BSP_IO_PORT_04_PIN_15,value);
        R_BSP_PinWrite(BSP_IO_PORT_04_PIN_04,value);
        R_BSP_PinWrite(BSP_IO_PORT_01_PIN_14,value);
        R_BSP_PinWrite(BSP_IO_PORT_04_PIN_05,value);
        R_BSP_PinWrite(BSP_IO_PORT_06_PIN_08,value);
        R_BSP_PinWrite(BSP_IO_PORT_01_PIN_15,value);
        R_BSP_PinAccessDisable();
        R_BSP_SoftwareDelay(250U, BSP_DELAY_UNITS_MILLISECONDS);
        value = !value;
    }

    uart_initialize();
    void sckFastlanV5Init(void);
    uart_ep();

    //flashProgramCheckUpdate();

    JumpToapp();
#if BSP_TZ_SECURE_BUILD
    /* Enter non-secure code */
    R_BSP_NonSecureEnter();
#endif
}

/*******************************************************************************************************************//**
 * This function is called at various points during the startup process.  This implementation uses the event that is
 * called right before main() to set up the pins.
 *
 * @param[in]  event    Where at in the start up process the code is currently at
 **********************************************************************************************************************/
void R_BSP_WarmStart(bsp_warm_start_event_t event)
{
    if (BSP_WARM_START_RESET == event)
    {
#if BSP_FEATURE_FLASH_LP_VERSION != 0

        /* Enable reading from data flash. */
        R_FACI_LP->DFLCTL = 1U;

        /* Would normally have to wait tDSTOP(6us) for data flash recovery. Placing the enable here, before clock and
         * C runtime initialization, should negate the need for a delay since the initialization will typically take more than 6us. */
#endif
    }

    if (BSP_WARM_START_POST_C == event)
    {
        /* C runtime environment and system clocks are setup. */

        /* Configure pins. */
        R_IOPORT_Open (&g_ioport_ctrl, g_ioport.p_cfg);
    }
}

#if BSP_TZ_SECURE_BUILD

BSP_CMSE_NONSECURE_ENTRY void template_nonsecure_callable ();

/* Trustzone Secure Projects require at least one nonsecure callable function in order to build (Remove this if it is not required to build). */
BSP_CMSE_NONSECURE_ENTRY void template_nonsecure_callable ()
{

}
#endif
