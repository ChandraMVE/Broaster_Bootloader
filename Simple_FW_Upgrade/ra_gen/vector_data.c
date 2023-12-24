/* generated vector source file - do not edit */
#include "bsp_api.h"
/* Do not build these data structures if no interrupts are currently allocated because IAR will have build errors. */
#if VECTOR_DATA_IRQ_COUNT > 0
        BSP_DONT_REMOVE const fsp_vector_t g_vector_table[BSP_ICU_VECTOR_MAX_ENTRIES] BSP_PLACE_IN_SECTION(BSP_SECTION_APPLICATION_VECTORS) =
        {
                        [0] = sci_uart_rxi_isr, /* SCI0 RXI (Receive data full) */
            [1] = sci_uart_txi_isr, /* SCI0 TXI (Transmit data empty) */
            [2] = sci_uart_tei_isr, /* SCI0 TEI (Transmit end) */
            [3] = sci_uart_eri_isr, /* SCI0 ERI (Receive error) */
            [4] = sci_i2c_txi_isr, /* SCI4 TXI (Transmit data empty) */
            [5] = sci_i2c_tei_isr, /* SCI4 TEI (Transmit end) */
            [6] = sci_uart_rxi_isr, /* SCI2 RXI (Received data full) */
            [7] = sci_uart_txi_isr, /* SCI2 TXI (Transmit data empty) */
            [8] = sci_uart_tei_isr, /* SCI2 TEI (Transmit end) */
            [9] = sci_uart_eri_isr, /* SCI2 ERI (Receive error) */
            [10] = adc_scan_end_isr, /* ADC0 SCAN END (A/D scan end interrupt) */
            [11] = adc_scan_end_b_isr, /* ADC0 SCAN END B (A/D scan end interrupt for group B) */
            [12] = adc_window_compare_isr, /* ADC0 WINDOW A (Window A Compare match) */
            [13] = rtc_alarm_periodic_isr, /* RTC ALARM (Alarm interrupt) */
            [14] = rtc_alarm_periodic_isr, /* RTC PERIOD (Periodic interrupt) */
            [15] = rtc_carry_isr, /* RTC CARRY (Carry interrupt) */
            [16] = agt_int_isr, /* AGT1 INT (AGT interrupt) */
            [17] = agt_int_isr, /* AGT0 INT (AGT interrupt) */
            [18] = agt_int_isr, /* AGT2 INT (AGT interrupt) */
            [19] = fcu_frdyi_isr, /* FCU FRDYI (Flash ready interrupt) */
            [20] = fcu_fiferr_isr, /* FCU FIFERR (Flash access error interrupt) */
        };
        const bsp_interrupt_event_t g_interrupt_event_link_select[BSP_ICU_VECTOR_MAX_ENTRIES] =
        {
            [0] = BSP_PRV_IELS_ENUM(EVENT_SCI0_RXI), /* SCI0 RXI (Receive data full) */
            [1] = BSP_PRV_IELS_ENUM(EVENT_SCI0_TXI), /* SCI0 TXI (Transmit data empty) */
            [2] = BSP_PRV_IELS_ENUM(EVENT_SCI0_TEI), /* SCI0 TEI (Transmit end) */
            [3] = BSP_PRV_IELS_ENUM(EVENT_SCI0_ERI), /* SCI0 ERI (Receive error) */
            [4] = BSP_PRV_IELS_ENUM(EVENT_SCI4_TXI), /* SCI4 TXI (Transmit data empty) */
            [5] = BSP_PRV_IELS_ENUM(EVENT_SCI4_TEI), /* SCI4 TEI (Transmit end) */
            [6] = BSP_PRV_IELS_ENUM(EVENT_SCI2_RXI), /* SCI2 RXI (Received data full) */
            [7] = BSP_PRV_IELS_ENUM(EVENT_SCI2_TXI), /* SCI2 TXI (Transmit data empty) */
            [8] = BSP_PRV_IELS_ENUM(EVENT_SCI2_TEI), /* SCI2 TEI (Transmit end) */
            [9] = BSP_PRV_IELS_ENUM(EVENT_SCI2_ERI), /* SCI2 ERI (Receive error) */
            [10] = BSP_PRV_IELS_ENUM(EVENT_ADC0_SCAN_END), /* ADC0 SCAN END (A/D scan end interrupt) */
            [11] = BSP_PRV_IELS_ENUM(EVENT_ADC0_SCAN_END_B), /* ADC0 SCAN END B (A/D scan end interrupt for group B) */
            [12] = BSP_PRV_IELS_ENUM(EVENT_ADC0_WINDOW_A), /* ADC0 WINDOW A (Window A Compare match) */
            [13] = BSP_PRV_IELS_ENUM(EVENT_RTC_ALARM), /* RTC ALARM (Alarm interrupt) */
            [14] = BSP_PRV_IELS_ENUM(EVENT_RTC_PERIOD), /* RTC PERIOD (Periodic interrupt) */
            [15] = BSP_PRV_IELS_ENUM(EVENT_RTC_CARRY), /* RTC CARRY (Carry interrupt) */
            [16] = BSP_PRV_IELS_ENUM(EVENT_AGT1_INT), /* AGT1 INT (AGT interrupt) */
            [17] = BSP_PRV_IELS_ENUM(EVENT_AGT0_INT), /* AGT0 INT (AGT interrupt) */
            [18] = BSP_PRV_IELS_ENUM(EVENT_AGT2_INT), /* AGT2 INT (AGT interrupt) */
            [19] = BSP_PRV_IELS_ENUM(EVENT_FCU_FRDYI), /* FCU FRDYI (Flash ready interrupt) */
            [20] = BSP_PRV_IELS_ENUM(EVENT_FCU_FIFERR), /* FCU FIFERR (Flash access error interrupt) */
        };
        #endif
