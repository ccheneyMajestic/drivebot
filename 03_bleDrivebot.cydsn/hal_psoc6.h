/***************************************************************************
*                                Majestic Labs © 2023
* File: hal_psoc6.h
* Workspace: MJL Hardware Abstraction Layer (HAL) Library 
* Version: v1.0.0
* Author: C. Cheney
* Target: PSoC6
*
* Brief: Contains wrappers for the MJL library from the Cypress PSoC4 libraries
*
* 2023.12.31  - Document Created
********************************************************************************/
/* Header Guard */
#ifndef HAL_PSOC6_H
  #define HAL_PSOC6_H
  /***************************************
  * Configurations
  ***************************************/
//  #define USE_SPI /* Comment this out to disable SPI */
    
  /***************************************
  * Included files
  ***************************************/
  #include <stdint.h>
  #include "mjl_uart.h"
  #include "mjl_spi.h"
  #include "rgb.h"
  
  /***************************************
  * Macro Definitions
  ***************************************/
  #define LED_ON (0)
  #define LED_OFF (1)
  #define ENCODER_OFFSET    (1u<<31)

  /* Motor Control Register */
  #define MOTOR_INDEX_LEFT   (0)
  #define MOTOR_INDEX_RIGHT  (1)
  #define MOTOR_DIRECTION_MASK      (0b0011)
  #define MOTOR_ENABLE_MASK         (0b1100)
  #define MOTOR_DIRECTION_LEFT_SHIFT  (0)
  #define MOTOR_DIRECTION_RIGHT_SHIFT (1)
  #define MOTOR_ENABLE_LEFT_SHIFT     (2)
  #define MOTOR_ENABLE_RIGHT_SHIFT    (3)
  #define MOTOR_EFFORT_MAX            (1000)
  #define MOTOR_EFFORT_MIN            (-1000)

  
  /* Flash - emulated eeprom */
  /* ID Field */
  #define FLASH_ID_ADDR               (0) /* Offset address of the ID value in flash */
  #define FLASH_ID_LEN                (1) /* Length of the flash ID */
  #define FLASH_RGB_ADDR              (FLASH_ID_ADDR + FLASH_ID_LEN) /* Offset address of the RGB Color */
  #define FLASH_RGB_LEN               (sizeof(rgb_s))
  /***************************************
  * Macro-like definitions
  ***************************************/
  #define CLAMP(value, min, max) ((value) < (min) ? (min) : ((value) > (max) ? (max) : (value)))

  /***************************************
  * Forward Declarations
  ***************************************/
  extern MLJ_UART_S usb; /* Defined in main_cm0p.c */

  

  /***************************************
  * Function declarations 
  ***************************************/
  uint32_t uart_psoc6SCB_start(MLJ_UART_T *const state);
  uint32_t uart_psoc6SCB_stop(MLJ_UART_T *const state);
  uint32_t uart_psoc6SCB_writeArrayBlocking(const uint8_t *array, uint16_t len);
  uint32_t uart_psoc6SCB_read(uint8_t *data);

  uint32_t spi_psoc6SCB_start(MJL_SPI_T *const state);
  uint32_t spi_psoc6SCB_stop(MJL_SPI_T *const state);
  uint32_t spi_psoc6SCB_writeArray_blocking(const uint8_t *array, uint16_t len);
  uint32_t spi_psoc6SCB_read(uint8_t *result);
  uint32_t spi_psoc6SCB_setActive(uint8_t id);
  uint32_t spi_psoc6SCB_getRxBufferNum(void);
  uint32_t spi_psoc6SCB_getTxBufferNum(void);
  uint32_t spi_psoc6SCB_clearRxBuffer(void);
  uint32_t spi_psoc6SCB_clearTxBuffer(void);
  
  void hal_reset_device(MLJ_UART_S *const uart);
  /* Flash operations */
  uint32_t hal_flash_id_read(uint8_t* id);
  uint32_t hal_flash_id_write(uint8_t id);
  uint32_t hal_flash_rgb_read(rgb_s * rgb);
  uint32_t hal_flash_rgb_write(const rgb_s * rgb);
  /* RGB */
  uint32_t hal_led_pin_write(bool state);
  uint32_t hal_rgb_set_color(const rgb_s* rgb);
  uint32_t hal_rgb_set_alpha(uint8_t alpha);
  /* Motors and encoders */
  uint32_t hal_encoder_read_left(int32_t* encoder_val);
  uint32_t hal_encoder_read_right(int32_t* encoder_val);
  uint32_t hal_motors_enable(bool left, bool right);
  uint32_t hal_motors_set_effort(int16_t left, int16_t right);

  
    
#endif /* HAL_PSOC6_H */
/* [] END OF FILE */
