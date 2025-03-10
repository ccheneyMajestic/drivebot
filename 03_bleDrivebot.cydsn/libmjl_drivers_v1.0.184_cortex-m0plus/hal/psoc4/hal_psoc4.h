/***************************************************************************
*                                Majestic Labs © 2023
* File: hal_psoc4.h
* Workspace: MJL Hardware Abstraction Layer (HAL) Library 
* Version: v1.0.0
* Author: C. Cheney
* Target: PSoC4 CYBLE_222014-01 Module
*
* Brief: Contains wrappers for the MJL library from the Cypress PSoC4 libraries
*
* 2023.04.26  - Document Created
********************************************************************************/
/* Header Guard */
#ifndef HAL_PSOC4_H
  #define HAL_PSOC4_H
  /***************************************
  * Included files
  ***************************************/
  #include <stdint.h>
  #include "mjl_uart.h"
  /***************************************
  * Macro Definitions
  ***************************************/
  #define AUX_OFF             (0) /* Disable 3V3_AUX */
  #define AUX_ON              (1) /* Enable 3V3_AUX */  
  #define ADC_CHAN_INA        (0) /* INA output */
  #define ADC_CHAN_BATT_MON   (1) /* Battery monitory channel */
  #define BATT_MON_RH         (20000.0)/* High side battery monitor resistor */
  #define BATT_MON_RL         (5100.0) /* Low side battery monitor resistor */
  #define BATT_MON_SCALE      ((BATT_MON_RH + BATT_MON_RL) / (BATT_MON_RL)) /* Scaling factor for the battery */
  #define LEN_ROW 128

  #define SL_SPI_ID_DISPLAY   (0) /* SPI Slave ID of the Display */
  #define SL_SPI_ID_INA       (1) /* SPI Slave ID of the Instrumentation Amp */
  #define SL_SPI_ID_FLASH     (2) /* SPI Slave ID of the Flash */
  #define SPI_CS_ACTIVE       (0) /* Value to assert an active slave) */
  #define SPI_CS_INACTIVE     (1) /* Value to disassert a slave) */
  /***************************************
  * Enumerated types
  ***************************************/

  /***************************************
  * Structures 
  ***************************************/

  /***************************************
  * Function declarations 
  ***************************************/
  uint32_t uart_psoc4SCB_start(MLJ_UART_T *const state);
  uint32_t uart_psoc4SCB_stop(MLJ_UART_T *const state);
  uint32_t uart_psoc4SCB_writeArrayBlocking(const uint8_t *array, uint16_t len);
  uint32_t uart_psoc4SCB_read(uint8_t *data);
  uint32_t spi_scbWriteArrayBlocking(uint8_t slaveId, uint8_t * cmdArray, uint16_t len);
  uint32_t spi_scbReadArrayBlocking(uint8_t slaveId, uint8_t * buffer, uint16_t len);
  uint32_t spi_assertSlave(uint8_t slaveId);
  void spi_disassertSlave(void);
  void pin_oled_reset_write_dummy(uint8_t val);

    
#endif /* HAL_PSOC4_H */
/* [] END OF FILE */
