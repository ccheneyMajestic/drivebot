/***************************************************************************
*                                Majestic Labs © 2023
* File: mjl_spi.h
* Workspace: MJL Driver Library 
* Version: v1.0.0
* Author: C. Cheney
*
* Brief: Serial Peripheral Interface middleware driver 
*   
*
* 2023.04.26  - Document Created
********************************************************************************/
/* Header Guard */
#ifndef MJL_SPI_H
  #define MJL_SPI_H
  /***************************************
  * Included files
  ***************************************/
  #include <stdbool.h>
  #include <stdint.h>
  /***************************************
  * Macro Definitions
  ***************************************/

  /***************************************
  * Enumerated types
  ***************************************/

  /***************************************
  * Structures 
  ***************************************/
   /* Forward declare struct */
  typedef struct MJL_SPI_S MJL_SPI_T;
  /* Configuration Structure */
  typedef struct {
    uint32_t (*req_hal_writeArray_blocking)(const uint8_t *array, uint16_t len);  /* Write data into the TX buffer */
    uint32_t (*req_hal_read)(uint8_t *result);                            /* Move data from the RX buffer to the result */
    uint32_t (*req_hal_setActive) (uint8_t id);                          /* Set the Slave with given ID active */
    uint32_t (*req_hal_getRxBufferNum) (void);                /* Get the number of elements in the Receive Buffer */
    uint32_t (*req_hal_getTxBufferNum) (void);                /* Get the number of elements in the Transmit Buffer */
    uint32_t (*req_hal_clearRxBuffer) (void);                            /* Clear the Receive Buffer */
    uint32_t (*req_hal_clearTxBuffer) (void);                            /* Clear the Transmit Buffer */
    uint32_t (*opt_hal_externalStart)(MJL_SPI_T *const);                 /* Optional External start function */
    uint32_t (*opt_hal_externalStop)(MJL_SPI_T *const);                  /* Optional External stop function */
  } MJL_SPI_CFG_S;

  /* Serial State Object   */
  typedef struct {
    uint32_t (*req_hal_writeArray_blocking)(const uint8_t *array, uint16_t len);  /* Write data into the TX buffer */
    uint32_t (*req_hal_read)(uint8_t *result);                           /* Move data from the RX buffer to the result */
    uint32_t (*req_hal_setActive) (uint8_t id);                          /* Set the Slave with given ID active */
    uint32_t (*req_hal_getRxBufferNum) (void);                /* Get the number of elements in the Receive Buffer */
    uint32_t (*req_hal_getTxBufferNum) (void);                /* Get the number of elements in the Transmit Buffer */
    uint32_t (*req_hal_clearRxBuffer) (void);                            /* Clear the Receive Buffer */
    uint32_t (*req_hal_clearTxBuffer) (void);                            /* Clear the Transmit Buffer */
    uint32_t (*opt_hal_externalStart)(MJL_SPI_T *const);                 /* Optional External start function */
    uint32_t (*opt_hal_externalStop)(MJL_SPI_T *const);                  /* Optional External stop function */

    bool _init;
    bool _running;
  } MJL_SPI_S;

  /* Default config struct */
  extern const MJL_SPI_CFG_S spi_cfg_default;
  /***************************************
  * Function declarations 
  ***************************************/
  /* State Operations */
  uint32_t spi_init(MJL_SPI_S *const state, MJL_SPI_CFG_S *const cfg);
  uint32_t spi_start(MJL_SPI_S *const state);
  uint32_t spi_stop(MJL_SPI_S *const state);
  uint32_t spi_writeArray_blocking(MJL_SPI_S *const state, uint8_t id, uint8_t * array, uint16_t len);
  uint32_t spi_readArray(MJL_SPI_S *const state,  uint8_t id, uint8_t * array, uint16_t len);
  uint32_t spi_write(MJL_SPI_S *const state, uint8_t id, uint8_t data);
  uint32_t spi_read(MJL_SPI_S *const state,  uint8_t id, uint8_t * data);
    
#endif /* MJL_SPI_H */
/* [] END OF FILE */