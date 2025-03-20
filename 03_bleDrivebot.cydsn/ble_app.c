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
#include "ble_app.h"
#include <stdio.h>

/* BLE State Intialization */
BLE_APP_S bleState = {
  /* Printing state */
  .print_stack_events = true,
  .print_write_events = true,
  .print_hardware_events = true,
  /*********** Characteristic Config ***********/
  /* LED control */
  .ledHandle.attrHandle = CY_BLE_DRIVEBOT_SERVICE_LED_CHAR_HANDLE,
  .ledHandle.value.val = bleState.ledState,
  .ledHandle.value.len = LEN_CHAR_LED,
  .ledName = "Offboard LED",
  /* RGB control */
  .rgbHandle.attrHandle = CY_BLE_DRIVEBOT_SERVICE_RGB_CHAR_HANDLE,
  .rgbHandle.value.val = bleState.rgbState,
  .rgbHandle.value.len = LEN_CHAR_RGB,
  .rgbName = "Onboard RGB",
  /* RGB Advertisement LED */
  .rgbAdv_index = 0,
  /* Encoder Reading */
  .encoderPosHandle.attrHandle = CY_BLE_DRIVEBOT_SERVICE_ENCODER_POSITION_CHAR_HANDLE, 
  .encoderPosHandle.value.val = (uint8_t *) bleState.encoderPosState,
  .encoderPosHandle.value.len = LEN_CHARACTERISTIC_ENCODER_POS * sizeof(int32_t),
  .encoderPos_name = "Encoder Position",
  .encoderPos_notifications_enabled = false,
  /* Motor Enable */
  .motorEnable_handle.attrHandle = CY_BLE_DRIVEBOT_SERVICE_MOTOR_ENABLE_CHAR_HANDLE,
  .motorEnable_handle.value.val = bleState.motorEnableState,
  .motorEnable_handle.value.len = LEN_CHARACTERISTIC_MOTOR_ENABLE,
  .motorEnable_name = "Motor Enable",
  /* Control effort */
  .motorEffort_handle.attrHandle = CY_BLE_DRIVEBOT_SERVICE_MOTOR_CONTROL_EFFORT_CHAR_HANDLE,
  .motorEffort_handle.value.val = (uint8_t *) bleState.motorEffortState,
  .motorEffort_handle.value.len = LEN_CHARACTERISTIC_MOTOR_EFFORT * sizeof(int16_t),
  .motorEffort_name = "Motor Effort",
  
};



/*******************************************************************************
* Function Name: bleApp_eventCallback()
********************************************************************************
* \brief
*   Bluetooth callback function. Handle all BLE functionality  
*
* \param eventCode [in]
* Code of the latest event that occured 
*
* \param eventParam [in]
*   Pointer to the struct (type depends on eventCode)
*
* \return
*  None
*******************************************************************************/
void bleApp_eventCallback(uint32_t eventCode, void * eventParam){
  /* Act depending on the event*/
  switch(eventCode){
    /**********************************************************
    *                       General Events
    ***********************************************************/
    /* Stack initialized; ready for advertisement*/
    case CY_BLE_EVT_STACK_ON: {
      /* Start advertising */ 
      bleApp_advertise();
      break;
    }
    /* Restart Advertisement on timeout */
    case CY_BLE_EVT_GAPP_ADVERTISEMENT_START_STOP: {
      /* See the state of the advertisement */
      cy_en_ble_adv_state_t advState = Cy_BLE_GetAdvertisementState();
      if(advState == CY_BLE_ADV_STATE_STOPPED){
        if(bleState.print_stack_events){uart_println(&usb, "Advertising Stopped");}
        /* Restart advertisement */
        bleApp_advertise();
      }
      else if (advState == CY_BLE_ADV_STATE_ADV_INITIATED) {
        if(bleState.print_stack_events){uart_println(&usb, "Advertising Start Initiated");}
      }
      else if (advState == CY_BLE_ADV_STATE_ADVERTISING){
        if(bleState.print_stack_events){uart_println(&usb, "Advertising");}
      }
      else if (advState == CY_BLE_ADV_STATE_STOP_INITIATED){
        if(bleState.print_stack_events){uart_println(&usb, "Advertising Stop Initiated");}
      }

      break;
    }
    /**********************************************************
    *                       GAP Events
    ***********************************************************/
    /* Start adversiting when disconnected */
    case CY_BLE_EVT_GAP_DEVICE_DISCONNECTED: {
      if(bleState.print_stack_events){uart_println(&usb, "BLE Device Disconnected");}
      /* Disable the motors */
      bleApp_motorEnable_update_state(false, false, true);
      /* Start advertising */
      bleApp_advertise();
      break;
    }
    /**********************************************************
    *                       GATT Events
    ***********************************************************/
    /* This event is generated at the GAP peripheral end after connection
    is completed with the peer central device */
    case CY_BLE_EVT_GATT_CONNECT_IND: {
      bleState.connHandle = * (cy_stc_ble_conn_handle_t *) eventParam;
      /* Update the LED */
      bleApp_rgb_update_state(0, 50, 0, true);
      if(bleState.print_stack_events){uart_println(&usb, "BLE Device Connected");}
      break;
    }
    
    /***************************************
    *           Write Request
    *****************************************/
    /* Peer device is requesting to write to a characteristic */
    case CY_BLE_EVT_GATTS_WRITE_REQ: {
      /* Unpack the write parameters */
      cy_stc_ble_gatt_write_param_t writeParam = *(cy_stc_ble_gatt_write_param_t *) eventParam;
      cy_stc_ble_gatt_value_t* write_request = &writeParam.handleValPair.value;      
      cy_ble_gatt_db_attr_handle_t attributeHandle = writeParam.handleValPair.attrHandle;
      /* Send a write response. NOTE: A write response does NOT echo back data. A Read request is required to readback data */
      Cy_BLE_GATTS_WriteRsp(bleState.connHandle);
      /**************** Handle the Specific characteristic *****************/
      /* Control the offboard LED */  
      if (attributeHandle == bleState.ledHandle.attrHandle){
        bleApp_led_write_request_handler(write_request);
      }
      /* Set the RGB led */
      else if (attributeHandle == bleState.rgbHandle.attrHandle){
        bleApp_rgb_write_request_handler(write_request);
      }
      /* Motor enable state */
      else if (attributeHandle == bleState.motorEnable_handle.attrHandle){
        bleApp_motorEnable_write_request_handler(write_request); 
      }
      /* Motor Effort */
      else if (attributeHandle == bleState.motorEffort_handle.attrHandle){
        bleApp_motorEffort_write_request_handler(write_request); 
      }
      break;
    }
    /* Write without response */
    case CY_BLE_EVT_GATTS_WRITE_CMD_REQ: {
      cy_stc_ble_gatts_write_cmd_req_param_t writeParam = *(cy_stc_ble_gatts_write_cmd_req_param_t *) eventParam;
      cy_stc_ble_gatt_value_t* write_request = &writeParam.handleValPair.value;      
      cy_ble_gatt_db_attr_handle_t attributeHandle = writeParam.handleValPair.attrHandle;
      if (attributeHandle == bleState.motorEffort_handle.attrHandle){
        bleApp_motorEffort_write_request_handler(write_request); 
      }
      break;
    }
          
    /* Unhandled event */
    default: {
      break;
    }
    /* End EventCode Switch */
  }
}

/*******************************************************************************
* Function Name: bleApp_advertise()
********************************************************************************
* \brief
*   Initiate Advertising
*
* \return
*  Error code of the operation 
*******************************************************************************/
uint32_t bleApp_advertise(){
  uint32_t error = 0;
  /* Get the state of the stack */ 
  cy_en_ble_state_t ble_state = Cy_BLE_GetState();
  if(ble_state != CY_BLE_STATE_ON){error |= 1;}
  
  if(!error){
    /* Get the current state */
    cy_en_ble_adv_state_t advState = Cy_BLE_GetAdvertisementState();
    /* Was not advertising, start now */
    if(advState == CY_BLE_ADV_STATE_STOPPED){

      /* Start Advertising */
      if(bleState.print_stack_events){uart_println(&usb, "Starting Advertisement");}
      error = Cy_BLE_GAPP_StartAdvertisement(CY_BLE_ADVERTISING_FAST, CY_BLE_PERIPHERAL_CONFIGURATION_0_INDEX);
      /* Get the next color  */
      error = bleApp_advertise_cycle_rgb();
    }
  }
  if(error){
    uart_printlnf(&usb, "Error in bleApp_advertise():%u", error);
  }
  return error;
}

/*******************************************************************************
* Function Name: bleApp_advertise_cycle_rgb()
********************************************************************************
* \brief
*   Advance to the next advertising color for the RGB and update the scan response
*   packet with the new color
*
* \return
*  Error code of the operation 
*******************************************************************************/
uint32_t bleApp_advertise_cycle_rgb(void){
  uint32_t error = 0;
  /* Get the state of the stack */ 
  if(Cy_BLE_GetState() != CY_BLE_STATE_ON){error |= 1;}
  if(CY_BLE_ADV_STATE_STOPPED == Cy_BLE_GetAdvertisementState()){error |= 2;}

  if(!error){
    /* Update the state */
    uint8_t index = bleState.rgbAdv_index;
    uart_printlnf(&usb, "Cycling advertising RGB %u", index);
    uint32_t rgbWord = advertisingColors[index];
    bleState.rgbAdv_index++;
    if(bleState.rgbAdv_index >= ADVERTISING_COLORS_LEN){bleState.rgbAdv_index = 0;}
    /* Update the hardware */
    uint8_t red, green, blue;
    hal_rgb_get_duty_word(rgbWord, &red, &green, &blue);
    bleApp_rgb_update_state(red, green, blue, true);
    /* Update the Scan response data to include the color */ 
    cy_stc_ble_gapp_disc_mode_info_t advScanParams;
    cy_ble_scanRspData->scanRspData[SCAN_RESPONSE_RGB_INDEX_RED]=red;
    cy_ble_scanRspData->scanRspData[SCAN_RESPONSE_RGB_INDEX_GREEN]=green;
    cy_ble_scanRspData->scanRspData[SCAN_RESPONSE_RGB_INDEX_BLUE]=blue;
    advScanParams.scanRspData = cy_ble_scanRspData;
    /* Don't change the existing Advertisement data */
    advScanParams.advData = cy_ble_discoveryData; 
    /* Execute the update */
    error = Cy_BLE_GAPP_UpdateAdvScanData(&advScanParams);
 
  }
  if(error && bleState.print_hardware_events){
    uart_printlnf(&usb, "Bad Cycle Advertise RGB error: %u", error);
  }
    
  return error;
}


/*******************************************************************************
* Function Name: bleApp_updateBleDatabase()
********************************************************************************
* \brief
*   Update the BLE database 
*
* \param handle [in]
* Pointer to the handle of the characteristic to update
*
* \param was_locally_initiated [in]
*   True if this was set locally, False if initiated by the peer 
*
* \return
*  Error code of the operation 
*******************************************************************************/
inline uint32_t bleApp_updateBleDatabase(cy_stc_ble_gatt_handle_value_pair_t* handle, bool was_locally_initiated){
  uint32_t error = 0; 
  cy_en_ble_state_t ble_state = Cy_BLE_GetState();
  if(CY_BLE_STATE_STOPPED == ble_state){error |= 3;}
  
  if(!error){
    /* Locally initiated */
    if(was_locally_initiated){
      error |= Cy_BLE_GATTS_WriteAttributeValueLocal(handle);
    }
    /* Peer initiated */
    else {
      error |= Cy_BLE_GATTS_WriteAttributeValuePeer(&bleState.connHandle, handle); 
    }
  }
  return error;
}


/*******************************************************************************
* Function Name: bleApp_printWrite()
********************************************************************************
* \brief
*   Prints the message on success, or the error if failed
*
* \param write_error [in]
*  The error code of the write operation
*
* \param name [in]
*   Pointer to the name of the characteristic
*
* \param message [in]
*   Pointer to the message to display on succcess
*
* \return
*  Error code of the operation 
*******************************************************************************/
void bleApp_printWrite(uint32_t write_error, char* name, char* message){
  if(bleState.print_write_events){
    if(write_error){
      uart_printlnf(&usb, "Error writing %s: 0x%x", name, write_error); 
    } else {
      uart_printlnf(&usb, "%s", message); 
    }
  }
}


/*========================== Write Request Handlers ==========================*/

/*******************************************************************************
* Function Name: bleApp_led_write_request_handler()
********************************************************************************
* \brief
*   Set the state of the off-board LED
*
* \param write_request [in]
*  Pointer to the write request
*
* \return
*  Error code of the operation 
*******************************************************************************/
uint32_t bleApp_led_write_request_handler(cy_stc_ble_gatt_value_t* write_request){
  uint32_t error = 0; 
    /* Validate Request */
  if(LEN_CHAR_LED != write_request->len){error |= 1;}
  /* Unpack the request and update the state */
  if(!error){
    bool led = write_request->val[0];
    /* Call the State Updater */
    error |= bleApp_led_update_state(led, false);
  }
  return error;
}

/*******************************************************************************
* Function Name: bleApp_rgb_write_request_handler()
********************************************************************************
* \brief
*   Set the state of the on-board RGB
*
* \param write_request [in]
*  Pointer to the write request
*
* \return
*  Error code of the operation 
*******************************************************************************/
uint32_t bleApp_rgb_write_request_handler(cy_stc_ble_gatt_value_t* write_request){
  uint32_t error = 0; 
  /* Validate Request */
  if(LEN_CHAR_RGB != write_request->len){error |= 1;}
  /* Unpack the request and update the state */
  if(!error){
    uint8_t red = write_request->val[RGB_INDEX_RED];
    uint8_t green = write_request->val[RGB_INDEX_GREEN];
    uint8_t blue = write_request->val[RGB_INDEX_BLUE];
    /* Call the State Updater */
    error |= bleApp_rgb_update_state(red, green, blue, false);
  }
  return error;
}

/*******************************************************************************
* Function Name: bleApp_motorEnable_write_request_handler()
********************************************************************************
* \brief
*   Process a write request for the Motor Enable Characteristic
*
* \param write_request [in]
* Pointer to the write request
*
* \return
*  Error code of the operation 
*******************************************************************************/
uint32_t bleApp_motorEnable_write_request_handler(cy_stc_ble_gatt_value_t* write_request){
  uint32_t error = 0;
  /* Validate Request */
  if(LEN_CHARACTERISTIC_MOTOR_ENABLE != write_request->len){error |= 1;}
  /* Unpack the request and update the state */
  if(!error){
    bool left = write_request->val[MOTOR_INDEX_LEFT];
    bool right = write_request->val[MOTOR_INDEX_RIGHT];
    /* Call the State Updater */
    error |= bleApp_motorEnable_update_state(left, right, false);
  }
  if(error && bleState.print_write_events){
    uart_printlnf(&usb, "Bad Motor Enable write request, received len:%u expected:%u", write_request->len, LEN_CHARACTERISTIC_MOTOR_ENABLE); 
  }
  return error;
}

/*******************************************************************************
* Function Name: bleApp_motorEffort_write_request_handler()
********************************************************************************
* \brief
*   Process a write request for the Motor Control Effort
*
* \param write_request [in]
* Pointer to the write request
*
* \return
*  Error code of the operation 
*******************************************************************************/
uint32_t bleApp_motorEffort_write_request_handler(cy_stc_ble_gatt_value_t* write_request){
  uint32_t error = 0;
  /* Validate Request */
  uint8_t expected_len = LEN_CHARACTERISTIC_MOTOR_EFFORT * sizeof(int16_t);
  if(expected_len != write_request->len){error |= 1;}
  /* Unpack the request and update the state */
  if(!error){
    int16_t left;
    int16_t right;
    memcpy(&left, &write_request->val[MOTOR_INDEX_LEFT * sizeof(int16_t)], sizeof(int16_t));
    memcpy(&right, &write_request->val[MOTOR_INDEX_RIGHT * sizeof(int16_t)], sizeof(int16_t));
    /* Call the State Updater */
    error |= bleApp_motorEffort_update_state(left, right, false);
  }
  if(error && bleState.print_write_events){
    uart_printlnf(&usb, "Bad Motor Effort write request, received len:%u expected:%u", write_request->len, expected_len); 
  }
  return error;
}


/*========================== State Updaters ==========================*/

/*******************************************************************************
* Function Name: bleApp_led_update_state()
********************************************************************************
* \brief
*  Set the state of the off-board LED and update both the BLE and State databases
*
* \param red [in]
*   Is the led on of off
*
* \param was_locally_initiated [in]
*   True if this was set locally, False if initiated by the peer 
*
* \return
*  Error code of the operation 
*******************************************************************************/
uint32_t bleApp_led_update_state(bool led, bool was_locally_initiated){
  uint32_t error = 0;
  /* Update the Hardware */
  error |= hal_led_pin_write(led);
  /* Update the Databases */
  if(!error){
    /***** Update the State Database *****/
    bleState.ledState[0] = led;
    /***** Update the BLE Databases *****/
    error |= bleApp_updateBleDatabase(&bleState.ledHandle, was_locally_initiated);
    /***** Print the message *****/
    char message[40];
    sprintf(message, "Setting LED state to: %d", led);
    bleApp_printWrite(error, bleState.ledName, message);
  }
  return error;
}

/*******************************************************************************
* Function Name: bleApp_rgb_update_state()
********************************************************************************
* \brief
*  Set the state of the on-board RGB and update both the BLE and State databases
*
* \param red [in]
*   Set the PWM values for the red LED [0-100]
*
* \param green [in]
*   Set the PWM values for the red LED [0-100]
*
* \param blue [in]
*   Set the PWM values for the red LED [0-100]
*
* \param was_locally_initiated [in]
*   True if this was set locally, False if initiated by the peer 
*
* \return
*  Error code of the operation 
*******************************************************************************/
uint32_t bleApp_rgb_update_state(uint8_t red, uint8_t green, uint8_t blue, bool was_locally_initiated){
  uint32_t error = 0; 
  /* Update the Hardware */
  error |= hal_rgb_set_duty(red, green, blue);
  /* Update the Databases */
  if(!error){
    /***** Update the State Database *****/
    bleState.rgbState[RGB_INDEX_RED] = red;
    bleState.rgbState[RGB_INDEX_GREEN] = green;
    bleState.rgbState[RGB_INDEX_BLUE] = blue;
    /***** Update the BLE Databases *****/
    error |= bleApp_updateBleDatabase(&bleState.rgbHandle, was_locally_initiated);
    /***** Print the message *****/
    char message[40];
    sprintf(message, "Writing RGB: %d, %d, %d", red, green, blue);
    bleApp_printWrite(error, bleState.rgbName, message);
  }
  return error;
}

/*******************************************************************************
* Function Name: bleApp_motorEnable_update_state()
********************************************************************************
* \brief
*  Update the state of the Databases with the latest encoder values
*
* \param left [in]
* True for enable, false for disable
* 
* \param right [in]
* True for enable, false for disable
*
* \param was_locally_initiated [in]
*   True if this was set locally, False if initiated by the peer 
*
* \return
*  Error code of the operation 
*******************************************************************************/
uint32_t bleApp_motorEnable_update_state(bool left, bool right, bool was_locally_initiated){
  uint32_t error = 0; 
  /* Update the onboard led to match the motor enable state */
  bool motors_enabled = left || right;
  error = bleApp_led_update_state(motors_enabled, was_locally_initiated);
  /* Update the Hardware */
  error |= hal_motors_enable(left, right);
  /* Update the Databases */
  if(!error){
    /***** Update the State Database *****/
    bleState.motorEnableState[MOTOR_INDEX_LEFT] = left;
    bleState.motorEnableState[MOTOR_INDEX_RIGHT] = right;
    /***** Update the BLE Databases *****/
    error |= bleApp_updateBleDatabase(&bleState.motorEnable_handle, was_locally_initiated);
    /***** Print the message *****/
    char message[40];
    sprintf(message, "Motor Enable: (%d, %d)", left, right);
    bleApp_printWrite(error, bleState.rgbName, message);
  }
  return error;
}

/*******************************************************************************
* Function Name: bleApp_motorEffort_update_state()
********************************************************************************
* \brief
*  Update the control effort for the motors
*
* \param left [in]
* control effort [-1000, 1000]
* 
* \param right [in]
* control effort [-1000, 1000]
*
* \param was_locally_initiated [in]
*   True if this was set locally, False if initiated by the peer 
*
* \return
*  Error code of the operation 
*******************************************************************************/
uint32_t bleApp_motorEffort_update_state(int16_t left, int16_t right, bool was_locally_initiated){
   uint32_t error = 0; 
  /* Clip range */
  int16_t left_clipped = CLAMP(left, MOTOR_EFFORT_MIN, MOTOR_EFFORT_MAX);
  int16_t right_clipped = CLAMP(right, MOTOR_EFFORT_MIN, MOTOR_EFFORT_MAX);;
  /* Update the Hardware */
  error |= hal_motors_set_effort(left_clipped, right_clipped);
  /* Update the Databases */
  if(!error){
    /***** Update the State Database *****/
    bleState.motorEffortState[MOTOR_INDEX_LEFT] = left_clipped;
    bleState.motorEffortState[MOTOR_INDEX_RIGHT] = right_clipped;
    /***** Update the BLE Databases *****/
    error |= bleApp_updateBleDatabase(&bleState.motorEffort_handle, was_locally_initiated);
    /***** Print the message *****/
    char message[40];
    sprintf(message, "Motor Effort: (%d, %d)", left_clipped, right_clipped);
    bleApp_printWrite(error, bleState.rgbName, message);
  }
  return error; 
}

/*******************************************************************************
* Function Name: bleApp_encoder_update_state()
********************************************************************************
* \brief
*  Update the state of the Databases with the latest encoder values
*
* \param left [in]
* Value of the left motor encoder
* 
* \param right [in]
* Value fo the right motor encoder
*
* \return
*  Error code of the operation 
*******************************************************************************/
uint32_t bleApp_encoder_update_state(int32_t left, int32_t right){
  uint32_t error = 0;
  /***** Update the State Database *****/
  bleState.encoderPosState[0] = left;
  bleState.encoderPosState[1] = right;
  /***** Update the BLE Databases *****/
  error |= bleApp_updateBleDatabase(&bleState.encoderPosHandle, true);
  /***** Don't Print the message *****/
  
  return error;
}


/* [] END OF FILE */