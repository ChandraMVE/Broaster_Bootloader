/* generated vector header file - do not edit */
#ifndef VECTOR_DATA_H
#define VECTOR_DATA_H
#ifdef __cplusplus
        extern "C" {
        #endif
/* Number of interrupts allocated */
#ifndef VECTOR_DATA_IRQ_COUNT
#define VECTOR_DATA_IRQ_COUNT    (21)
#endif
/* ISR prototypes */
void sci_uart_rxi_isr(void);
void sci_uart_txi_isr(void);
void sci_uart_tei_isr(void);
void sci_uart_eri_isr(void);
void sci_i2c_txi_isr(void);
void sci_i2c_tei_isr(void);
void adc_scan_end_isr(void);
void adc_scan_end_b_isr(void);
void adc_window_compare_isr(void);
void rtc_alarm_periodic_isr(void);
void rtc_carry_isr(void);
void agt_int_isr(void);
void fcu_frdyi_isr(void);
void fcu_fiferr_isr(void);

/* Vector table allocations */
#define VECTOR_NUMBER_SCI0_RXI ((IRQn_Type) 0) /* SCI0 RXI (Receive data full) */
#define SCI0_RXI_IRQn          ((IRQn_Type) 0) /* SCI0 RXI (Receive data full) */
#define VECTOR_NUMBER_SCI0_TXI ((IRQn_Type) 1) /* SCI0 TXI (Transmit data empty) */
#define SCI0_TXI_IRQn          ((IRQn_Type) 1) /* SCI0 TXI (Transmit data empty) */
#define VECTOR_NUMBER_SCI0_TEI ((IRQn_Type) 2) /* SCI0 TEI (Transmit end) */
#define SCI0_TEI_IRQn          ((IRQn_Type) 2) /* SCI0 TEI (Transmit end) */
#define VECTOR_NUMBER_SCI0_ERI ((IRQn_Type) 3) /* SCI0 ERI (Receive error) */
#define SCI0_ERI_IRQn          ((IRQn_Type) 3) /* SCI0 ERI (Receive error) */
#define VECTOR_NUMBER_SCI4_TXI ((IRQn_Type) 4) /* SCI4 TXI (Transmit data empty) */
#define SCI4_TXI_IRQn          ((IRQn_Type) 4) /* SCI4 TXI (Transmit data empty) */
#define VECTOR_NUMBER_SCI4_TEI ((IRQn_Type) 5) /* SCI4 TEI (Transmit end) */
#define SCI4_TEI_IRQn          ((IRQn_Type) 5) /* SCI4 TEI (Transmit end) */
#define VECTOR_NUMBER_SCI2_RXI ((IRQn_Type) 6) /* SCI2 RXI (Received data full) */
#define SCI2_RXI_IRQn          ((IRQn_Type) 6) /* SCI2 RXI (Received data full) */
#define VECTOR_NUMBER_SCI2_TXI ((IRQn_Type) 7) /* SCI2 TXI (Transmit data empty) */
#define SCI2_TXI_IRQn          ((IRQn_Type) 7) /* SCI2 TXI (Transmit data empty) */
#define VECTOR_NUMBER_SCI2_TEI ((IRQn_Type) 8) /* SCI2 TEI (Transmit end) */
#define SCI2_TEI_IRQn          ((IRQn_Type) 8) /* SCI2 TEI (Transmit end) */
#define VECTOR_NUMBER_SCI2_ERI ((IRQn_Type) 9) /* SCI2 ERI (Receive error) */
#define SCI2_ERI_IRQn          ((IRQn_Type) 9) /* SCI2 ERI (Receive error) */
#define VECTOR_NUMBER_ADC0_SCAN_END ((IRQn_Type) 10) /* ADC0 SCAN END (A/D scan end interrupt) */
#define ADC0_SCAN_END_IRQn          ((IRQn_Type) 10) /* ADC0 SCAN END (A/D scan end interrupt) */
#define VECTOR_NUMBER_ADC0_SCAN_END_B ((IRQn_Type) 11) /* ADC0 SCAN END B (A/D scan end interrupt for group B) */
#define ADC0_SCAN_END_B_IRQn          ((IRQn_Type) 11) /* ADC0 SCAN END B (A/D scan end interrupt for group B) */
#define VECTOR_NUMBER_ADC0_WINDOW_A ((IRQn_Type) 12) /* ADC0 WINDOW A (Window A Compare match) */
#define ADC0_WINDOW_A_IRQn          ((IRQn_Type) 12) /* ADC0 WINDOW A (Window A Compare match) */
#define VECTOR_NUMBER_RTC_ALARM ((IRQn_Type) 13) /* RTC ALARM (Alarm interrupt) */
#define RTC_ALARM_IRQn          ((IRQn_Type) 13) /* RTC ALARM (Alarm interrupt) */
#define VECTOR_NUMBER_RTC_PERIOD ((IRQn_Type) 14) /* RTC PERIOD (Periodic interrupt) */
#define RTC_PERIOD_IRQn          ((IRQn_Type) 14) /* RTC PERIOD (Periodic interrupt) */
#define VECTOR_NUMBER_RTC_CARRY ((IRQn_Type) 15) /* RTC CARRY (Carry interrupt) */
#define RTC_CARRY_IRQn          ((IRQn_Type) 15) /* RTC CARRY (Carry interrupt) */
#define VECTOR_NUMBER_AGT1_INT ((IRQn_Type) 16) /* AGT1 INT (AGT interrupt) */
#define AGT1_INT_IRQn          ((IRQn_Type) 16) /* AGT1 INT (AGT interrupt) */
#define VECTOR_NUMBER_AGT0_INT ((IRQn_Type) 17) /* AGT0 INT (AGT interrupt) */
#define AGT0_INT_IRQn          ((IRQn_Type) 17) /* AGT0 INT (AGT interrupt) */
#define VECTOR_NUMBER_AGT2_INT ((IRQn_Type) 18) /* AGT2 INT (AGT interrupt) */
#define AGT2_INT_IRQn          ((IRQn_Type) 18) /* AGT2 INT (AGT interrupt) */
#define VECTOR_NUMBER_FCU_FRDYI ((IRQn_Type) 19) /* FCU FRDYI (Flash ready interrupt) */
#define FCU_FRDYI_IRQn          ((IRQn_Type) 19) /* FCU FRDYI (Flash ready interrupt) */
#define VECTOR_NUMBER_FCU_FIFERR ((IRQn_Type) 20) /* FCU FIFERR (Flash access error interrupt) */
#define FCU_FIFERR_IRQn          ((IRQn_Type) 20) /* FCU FIFERR (Flash access error interrupt) */
#ifdef __cplusplus
        }
        #endif
#endif /* VECTOR_DATA_H */
