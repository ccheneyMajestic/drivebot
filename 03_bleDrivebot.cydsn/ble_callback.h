/***************************************************************************
*                                Majestic Labs © 2025
* File: ble_callback.h
* Workspace: bleDriveboth
* Version: v1.0.0
* Author: C. Cheney
*
* Brief: Bluetooth application for Drivebot
*
* 2025.03.10  - Document Created
********************************************************************************/
/* Header Guard */
#ifndef BLE_DRIVEBOT_H
  #define BLE_DRIVEBOT_H
  /***************************************
  * Included files
  ***************************************/
  #include <stdbool.h>
  #include <mjl_errors.h>
  #include "project.h"
  #include "hal_psoc6.h"
  
  /***************************************
  * Macro Definitions
  ***************************************/
  #define LEN_CHAR_LED    (1) /* Length of the LED control characteristic */
  #define LEN_CHAR_RGB    (3) /* Length of the (R, G, B) led */
  
  /***************************************
  * Enumerated types
  ***************************************/

  /***************************************
  * Structures 
  ***************************************/
  typedef struct {
    cy_stc_ble_conn_handle_t connHandle; /* Connection Handle */
    bool print_stack_events; /* Flag indicating if stack events should print */
    bool print_write_events; /* Flag indicating if printing events should occur */ 
    
    /* Offboard LED control */
    cy_stc_ble_gatt_handle_value_pair_t ledHandle;
    uint8_t ledState[LEN_CHAR_LED];
    char* ledName;
    /* Onboard RGB Control */
    cy_stc_ble_gatt_handle_value_pair_t rgbHandle;
    uint8_t rgbState[LEN_CHAR_RGB];
    char* rgbName;



    
  } BLE_APP_S;
  
  /***************************************
  * Forward declarations
  ***************************************/
  extern BLE_APP_S bleState; /* Defined in ble_callback.c */

  /***************************************
  * Function declarations 
  ***************************************/
  /* State Operations */
  void bleApp_eventCallback(uint32_t eventCode, void * eventParam);
  uint32_t bleApp_updateGattDb(cy_stc_ble_gatt_handle_value_pair_t* handleValuePair, cy_stc_ble_gatt_value_t* write_value);
  /* Write request handlers */
  uint32_t bleApp_led_write_handler(cy_stc_ble_gatt_value_t* write_value);
  uint32_t bleApp_rgb_write_handler(cy_stc_ble_gatt_value_t* write_value);
  
  

#endif /* BLE_DRIVEBOT_H */
/* [] END OF FILE */
