/***************************************************************************
*                                       MICA
* File: Sflash.h
* Workspace: micaComponents
* Project Name: libMica
* Version: v1.0
* Author: Craig Cheney
*
* Brief:
*   Header for interacting with the Supervisory flash
* 
* Authors:
*   Craig Cheney
*
* Change Log:
*   2018.03.03 CC - Document created
********************************************************************************/
/* Header Guard */
#ifndef SFLASH_H
    #define SFLASH_H
    /***************************************
    * Included Files
    ***************************************/
    #include "CyFlash.h"
    #include "cytypes.h"
    /***************************************
    * Macro Definitions
    ***************************************/
    #define SFLASH_BASE_ADDR       (CY_SFLASH_USERBASE) /**< Base location in memory of the SFLASH for 256k devices  */
    #define SFLASH_SIZE_ROW        (CY_SFLASH_SIZEOF_USERROW) /**< The size in bytes of the SFlash row  */

    #define SFLASH_ROW_0  (0)     /* Row 0 - Bootloader information */
    #define SFLASH_ROW_1  (1)     /* Row 1 - Max history */
  
  
    #define SFLASH_NUM_ROWS        (4u)    /**< Number of rows available for user data  */
    #define SFLASH_INDEX_ROW_NAME  (1u)    /**< Row index that contains the BLE Device Name  */
    
    #define SFLASH_NAME_LEN        (26u)   /**< Length of the name data */
    #define SFLASH_NAME_ROW        (1u)    /**< Row that contains the name data */  
    #define SFLASH_NAME_OFFSET     (0u)    /**< Offset in the row of the name data */  
    
    #define SFLASH_ERR_OK          (0u)   /**< Operation was successful */
    #define SFLASH_ERR_INDEX_RANGE (1u)   /**< Value passed in was outside the valid range */

    #define SFLASH_IDX_APP_ID         (0)
    #define SFLASH_IDX_APP_VERSION    (2)
    #define SFLASH_IDX_TIMEOUT_COUNTER (4)
    #define SFLASH_IDX_BOOTLOAD_MODE  (6)
    
    #define SL_BOOTLOAD_TIMEOUT_MAX  (5) /* Maximum number of times to timeout */
    #define SL_BOOTLOADABLE_APP_ID    (0) /* ID of the bootloadable app */


    typedef enum{
      SL_APPFLOW_MODE_APP = 0x00,
      SL_APPFLOW_MODE_BOOTLOAD = 0x01,
      SL_APPFLOW_MODE_EXIT = 0x02,
    } SL_APPFLOW_MODE_T;
    /***************************************
    * Function declarations 
    ***************************************/
    uint32 SFLASH_ReadRow(uint8 rowNum, uint8* dataBuffer);
    uint32 SFLASH_WriteRow(uint8 rowNum, const uint8* rowData);
    uint32 SFLASH_ReadLocalName(char* nameArray);
    uint32 SFLASH_WriteLocalName(uint8 len, char* nameArray);
    
#endif /* SFLASH_H */
/* [] END OF FILE */
