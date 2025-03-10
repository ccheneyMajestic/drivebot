/***************************************************************************
*                                Majestic Labs © 2023
* File: FLASH_IS25.h
* Workspace: MJL Driver Library 
* Version: v1.0.0
* Author: C. Cheney
*
* Brief: Driver file for the IS25 Flash module from ISSI
*
* 2023.05.25 - Document Created 
********************************************************************************/
/* Header Guard */
#ifndef FLASH_IS25_H
  #define FLASH_IS25_H
  /***************************************
  * Included files
  ***************************************/
  #include <stdbool.h>
  #include <mjl_errors.h>
  /***************************************
  * Macro Definitions
  ***************************************/
  #define IS25_CMD_WRSR   (0x01) /* Write status register command */
  #define IS25_CMD_RDSR   (0x05) /* Read status register command */
  #define IS25_CMD_WREN   (0x06) /* Write enable operation */
  #define IS25_CMD_RSTREN (0x66) /* Reset Enable command */
  #define IS25_CMD_RDMDID (0x90) /* Read manufacturer and Device ID command */
  #define IS25_CMD_RST    (0x99) /* Reset Command */
  
  
  #define IS25_US_RESET         (50) /* Software Reset time [µs] */
  #define IS25_RDMDID_LEN       (6) /* Length of the Read ID command  */
  #define IS25_RDMDID_POS_MFG   (4) /* Position of the manufacturer ID */   
  #define IS25_RDMDID_POS_DEV   (5) /* Position of the device ID */   

  #define IS25_ID_MFG           (0x9D)  /* Manufacturer ID */
  #define IS25_ID_DEV           (0x14)  /* Device ID */

  /***************************************
  * Enumerated types
  ***************************************/

  /***************************************
  * Structures 
  ***************************************/
  /* Configuration Structure */
  typedef struct {
    void (*fn_delayUs)(uint16_t microsecond);
    uint32_t (*fn_spi_writeArrayBlocking) (uint8_t slaveId, const uint8_t * cmdArray, uint16_t len);
    uint32_t (*fn_spi_readArrayBlocking) (uint8_t slaveId, uint8_t * buffer, uint16_t len);
    uint8_t slaveId;

  } FLASH_IS25_CFG_S;

  /* Serial State Object   */
  typedef struct {
    void (*fn_delayUs)(uint16_t microsecond);
    uint32_t (*fn_spi_writeArrayBlocking) (uint8_t slaveId, const uint8_t * cmdArray, uint16_t len);
    uint32_t (*fn_spi_readArrayBlocking) (uint8_t slaveId, uint8_t * buffer, uint16_t len);
    uint8_t slaveId;


    bool _init;
    bool _running;
  } FLASH_IS25_S;

  /* Default config struct */
  extern const FLASH_IS25_CFG_S flash_is25_cfg_default;
  /***************************************
  * Function declarations 
  ***************************************/
  /* State Operations */
  uint32_t flash_is25_init(FLASH_IS25_S *const state, FLASH_IS25_CFG_S *const cfg);
  uint32_t flash_is25_start(FLASH_IS25_S *const state);
  // uint32_t flash_is25_stop(FLASH_IS25_S *const state);

#endif /* FLASH_IS25_H */
/* [] END OF FILE */
