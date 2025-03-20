/***************************************************************************
*                                Majestic Labs © 2025
* File: ble_app.h
* Workspace: bleDriveboth
* Version: v1.0.0
* Author: C. Cheney
*
* Brief: Bluetooth application for Drivebot
*
* 2025.03.10  - Document Created
********************************************************************************/
/* Header Guard */
#ifndef BLE_APP_H
  #define BLE_APP_H
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
  #define LEN_CHAR_LED                      (1) /* Length of the LED control characteristic */
  #define LEN_CHAR_RGB                      (3) /* Length of the (R, G, B) led */
  #define LEN_CHARACTERISTIC_ENCODER_POS    (2) /* Number of elements in the encoder position */
  #define LEN_CHARACTERISTIC_MOTOR_ENABLE   (2) /* Number of elements in the motor enable word */
  #define LEN_CHARACTERISTIC_MOTOR_EFFORT   (2) /* Number of elements in the control effort  */
  /* Scan Response packet */
  #define SCAN_RESPONSE_RGB_INDEX_RED       (22) /* Index of the RED led in the scan response packet */
  #define SCAN_RESPONSE_RGB_INDEX_GREEN     (23) /* Index of the GREEN led in the scan response packet */
  #define SCAN_RESPONSE_RGB_INDEX_BLUE      (24) /* Index of the BLUE led in the scan response packet */
  
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
    bool print_hardware_events; /* Flag indicating if printing hardware driven events should occur */
    
    /* Offboard LED control */
    cy_stc_ble_gatt_handle_value_pair_t ledHandle;
    uint8_t ledState[LEN_CHAR_LED];
    char* ledName;
    /* Onboard RGB Control */
    cy_stc_ble_gatt_handle_value_pair_t rgbHandle;
    uint8_t rgbState[LEN_CHAR_RGB];
    char* rgbName;
    /* RGB Advertisement */
    uint8_t rgbAdv_index;
    /* Encoder Position */
    cy_stc_ble_gatt_handle_value_pair_t encoderPosHandle;
    int32_t encoderPosState[LEN_CHARACTERISTIC_ENCODER_POS];
    char* encoderPos_name;
    bool encoderPos_notifications_enabled;
    /* Control Mode */
    cy_stc_ble_gatt_handle_value_pair_t motorEnable_handle;
    uint8_t motorEnableState[LEN_CHARACTERISTIC_MOTOR_ENABLE];
    char* motorEnable_name;
    /* Motor Control Effort */
    cy_stc_ble_gatt_handle_value_pair_t motorEffort_handle;
    int16_t motorEffortState[LEN_CHARACTERISTIC_MOTOR_EFFORT];
    char* motorEffort_name;
    
    
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
  uint32_t bleApp_advertise(void);
  uint32_t bleApp_updateBleDatabase(cy_stc_ble_gatt_handle_value_pair_t* handle, bool was_locally_initiated);
  void bleApp_printWrite(uint32_t write_error, char* name, char* message);
  /* Write request handlers */
  uint32_t bleApp_led_write_request_handler(cy_stc_ble_gatt_value_t* write_request);
  uint32_t bleApp_rgb_write_request_handler(cy_stc_ble_gatt_value_t* write_request);
  uint32_t bleApp_motorEnable_write_request_handler(cy_stc_ble_gatt_value_t* write_request);
  uint32_t bleApp_motorEffort_write_request_handler(cy_stc_ble_gatt_value_t* write_request);
  /* State Updaters */
  uint32_t bleApp_advertise_cycle_rgb(void);
  uint32_t bleApp_led_update_state(bool led, bool was_locally_initiated);
  uint32_t bleApp_rgb_update_state(uint8_t red, uint8_t green, uint8_t blue, bool was_locally_initiated);
  uint32_t bleApp_motorEnable_update_state(bool left, bool right, bool was_locally_initiated);
  uint32_t bleApp_motorEffort_update_state(int16_t left, int16_t right, bool was_locally_initiated);
  /* State Updaters — Read-only */
  uint32_t bleApp_encoder_update_state(int32_t left, int32_t right);
  
  

#endif /* BLE_APP_H */
/* [] END OF FILE */
