/* generated configuration header file - do not edit */
#ifndef BSP_PIN_CFG_H_
#define BSP_PIN_CFG_H_
#include "r_ioport.h"

/* Common macro for FSP header files. There is also a corresponding FSP_FOOTER macro at the end of this file. */
FSP_HEADER

#define TEMP_IN (BSP_IO_PORT_00_PIN_00) /* TEMP_IN ADC */
#define PRESSURE_IN (BSP_IO_PORT_00_PIN_01) /* PRESSURE_IN ADC */
#define Thermocouple_3_Low (BSP_IO_PORT_00_PIN_03) /* Thermocouple_3_Low ADC */
#define Thermocouple_3_High (BSP_IO_PORT_00_PIN_04) /* Thermocouple_3_High ADC */
#define Thermocouple_2_Low (BSP_IO_PORT_00_PIN_05) /* Thermocouple_2_Low ADC */
#define Thermocouple020High (BSP_IO_PORT_00_PIN_06) /* Thermocouple020High ADC */
#define Thermocouple010Low (BSP_IO_PORT_00_PIN_07) /* Thermocouple010Low ADC */
#define Thermocouple_1_High (BSP_IO_PORT_00_PIN_08) /* Thermocouple_1_High ADC */
#define AUDIO_SIGNAL (BSP_IO_PORT_00_PIN_14) /* AUDIO_SIGNAL DAC */
#define LIFT_RET (BSP_IO_PORT_01_PIN_02) /* LIFT_RET DO */
#define LIFT_EXT (BSP_IO_PORT_01_PIN_03) /* LIFT_EXT DO */
#define DISPOSAL_REV (BSP_IO_PORT_01_PIN_04) /* DISPOSAL_REV DO */
#define DISPOSAL_FWD (BSP_IO_PORT_01_PIN_05) /* DISPOSAL_FWD DO */
#define FILL_REV (BSP_IO_PORT_01_PIN_06) /* FILL_REV DO */
#define FILL_FWD (BSP_IO_PORT_01_PIN_07) /* FILL_FWD DO */
#define RELAY_DRIVER_3 (BSP_IO_PORT_01_PIN_12) /* RELAY_DRIVER_3 DO */
#define RELAY_DRIVER_4 (BSP_IO_PORT_01_PIN_13) /* RELAY_DRIVER_4 DO */
#define DL4 (BSP_IO_PORT_01_PIN_14) /* DL4 LED Red */
#define DL3 (BSP_IO_PORT_01_PIN_15) /* DL3 LED Red */
#define RELAY_DRIVER_2 (BSP_IO_PORT_02_PIN_05) /* RELAY_DRIVER_2 DO */
#define EEP_SCL (BSP_IO_PORT_02_PIN_06) /* EEP_SCL I2C */
#define EEP_SDA (BSP_IO_PORT_02_PIN_07) /* EEP_SDA I2C */
#define Thermocouple3_Ctrl (BSP_IO_PORT_02_PIN_09) /* Thermocouple3_Ctrl DO */
#define Thermocouple2_Ctrl (BSP_IO_PORT_02_PIN_10) /* Thermocouple2_Ctrl DO */
#define Thermocouple1_Ctrl (BSP_IO_PORT_02_PIN_11) /* Thermocouple1_Ctrl DO */
#define RELAY_DRIVER_1 (BSP_IO_PORT_02_PIN_14) /* RELAY_DRIVER_1 DO */
#define UART2_RXD (BSP_IO_PORT_03_PIN_01) /* UART2_RXD to RS485 */
#define UART2_TXD (BSP_IO_PORT_03_PIN_02) /* UART2_TXD to RS485 */
#define RS485_TE (BSP_IO_PORT_03_PIN_03) /* RS485_TE  */
#define Thermocouple3_Err (BSP_IO_PORT_03_PIN_05) /* Thermocouple3_Err DI */
#define Thermocouple2_Err (BSP_IO_PORT_03_PIN_06) /* Thermocouple2_Err DI */
#define Thermocouple1_Err (BSP_IO_PORT_03_PIN_07) /* Thermocouple1_Err DI */
#define SELA (BSP_IO_PORT_04_PIN_00) /* Temperature Probes SELA DO */
#define SELB (BSP_IO_PORT_04_PIN_01) /* Temperature Probes SELB DO */
#define SELC (BSP_IO_PORT_04_PIN_02) /* Temperature Probes SELC DO */
#define LIFTER_POS2_IN (BSP_IO_PORT_04_PIN_07) /* LIFTER_POS2_IN DI */
#define LIFTER_POS1_IN (BSP_IO_PORT_04_PIN_08) /* LIFTER_POS1_IN DI */
#define FILL_POS2_IN (BSP_IO_PORT_04_PIN_09) /* FILL_POS2_IN DI */
#define FILL_POS1_IN (BSP_IO_PORT_04_PIN_10) /* FILL_POS1_IN DI */
#define DRAIN_POS2_IN (BSP_IO_PORT_04_PIN_11) /* DRAIN_POS2_IN DI */
#define DRAIN_POS1_IN (BSP_IO_PORT_04_PIN_12) /* DRAIN_POS1_IN DI */
#define DISPOSAL_POS2_IN (BSP_IO_PORT_04_PIN_13) /* DISPOSAL_POS2_IN DI */
#define DISPOSAL_POS1_IN (BSP_IO_PORT_04_PIN_14) /* DISPOSAL_POS1_IN DI */
#define FRYER_COVER_IN (BSP_IO_PORT_04_PIN_15) /* FRYER_COVER_IN DI */
#define FILTER_CTRL (BSP_IO_PORT_05_PIN_00) /* FILTER_CTRL DO */
#define HEAT_CTRL (BSP_IO_PORT_05_PIN_01) /* HEAT_CTRL DO */
#define PUMP_CTRL (BSP_IO_PORT_05_PIN_02) /* PUMP_CTRL DO */
#define AC4_CTRL (BSP_IO_PORT_05_PIN_03) /* AC4_CTRL DO */
#define AC5_CTRL (BSP_IO_PORT_05_PIN_04) /* AC5_CTRL DO */
#define DRAIN_REV (BSP_IO_PORT_06_PIN_00) /* DRAIN_REV DO */
#define DRAIN_FWD (BSP_IO_PORT_06_PIN_01) /* DRAIN_FWD DO */
#define EXHAUST_REV (BSP_IO_PORT_06_PIN_02) /* EXHAUST_REV DO */
#define EXHAUST_FWD (BSP_IO_PORT_06_PIN_03) /* EXHAUST_FWD DO */
#define DL2 (BSP_IO_PORT_06_PIN_08) /* DL2 LED Red */
#define SOUND_SD (BSP_IO_PORT_06_PIN_09) /* SOUND_SD DO */
#define FILTER_COVER_IN (BSP_IO_PORT_07_PIN_08) /* FILTER_COVER_IN DI */
extern const ioport_cfg_t g_bsp_pin_cfg; /* R7FA4M2AD3CFP.pincfg */

void BSP_PinConfigSecurityInit();

/* Common macro for FSP header files. There is also a corresponding FSP_HEADER macro at the top of this file. */
FSP_FOOTER

#endif /* BSP_PIN_CFG_H_ */
