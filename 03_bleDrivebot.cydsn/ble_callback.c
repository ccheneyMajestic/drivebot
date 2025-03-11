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
#include "ble_callback.h"

/* BLE State */
BLE_APP_S bleState = {
  /* Printing state */
  .print_stack_events = true,
  .print_write_events = true,
  /*********** Characteristic Config ***********/
  /* LED control */
  .ledHandle.attrHandle = CY_BLE_DRIVEBOT_SERVICE_LED_CHARACTERISTIC_CHAR_HANDLE,
  .ledHandle.value.val = bleState.ledState,
  .ledHandle.value.len = LEN_CHAR_LED,
  .ledName = "Offboard LED",
  /* RGB control */
  .rgbHandle.attrHandle = CY_BLE_DRIVEBOT_SERVICE_RGB_CHARACTERISTIC_CHAR_HANDLE,
  .rgbHandle.value.val = bleState.rgbState,
  .rgbHandle.value.len = LEN_CHAR_RGB,
  .rgbName = "Onboard RGB"
  
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
      Cy_BLE_GAPP_StartAdvertisement(CY_BLE_ADVERTISING_FAST, CY_BLE_PERIPHERAL_CONFIGURATION_0_INDEX);
      if(bleState.print_stack_events){uart_println(&usb, "BLE Stack ON");}
      break;
    }
    /* Restart Advertisement on timeout */
    case CY_BLE_EVT_GAPP_ADVERTISEMENT_START_STOP: {
      if(Cy_BLE_GetState() == CY_BLE_STATE_STOPPED) {
        Cy_BLE_GAPP_StartAdvertisement(CY_BLE_ADVERTISING_FAST, CY_BLE_PERIPHERAL_CONFIGURATION_0_INDEX);
        if(bleState.print_stack_events){uart_println(&usb, "BLE Stack Restarted");}
      }
      break;
    }
    /**********************************************************
    *                       GAP Events
    ***********************************************************/
    /* Start adversiting when disconnected */
    case CY_BLE_EVT_GAP_DEVICE_DISCONNECTED: {
      Cy_BLE_GAPP_StartAdvertisement(CY_BLE_ADVERTISING_FAST, CY_BLE_PERIPHERAL_CONFIGURATION_0_INDEX);
      if(bleState.print_stack_events){uart_println(&usb, "BLE Device Disconnected");}
      break;
    }
    /**********************************************************
    *                       GATT Events
    ***********************************************************/
    /* This event is generated at the GAP peripheral end after connection
    is completed with the peer central device */
    case CY_BLE_EVT_GATT_CONNECT_IND: {
      bleState.connHandle = * (cy_stc_ble_conn_handle_t *) eventParam;
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
      cy_stc_ble_gatt_value_t* write_value = &writeParam.handleValPair.value;      
      cy_ble_gatt_db_attr_handle_t attributeHandle = writeParam.handleValPair.attrHandle;
      /* Send a write response. NOTE: Write response does NOT echo back data. A Read request is required to echo data */
      Cy_BLE_GATTS_WriteRsp(bleState.connHandle);
      /**************** Handle the Specific characteristic *****************/
      /* Control the offboard LED */  
      if (attributeHandle == bleState.ledHandle.attrHandle){
        bleApp_led_write_handler(write_value);
      }
      /* Set the RGB led */
      else if (attributeHandle == bleState.rgbHandle.attrHandle){
        bleApp_rgb_write_handler(write_value);
      }
      break;
    }
          
    /* Unhandled event */
    default: {
      break;
    }
    /* End Switch */
  }
}


/*******************************************************************************
* Function Name: bleApp_led_write_handler()
********************************************************************************
* \brief
*   Set the state of the off-board LED
*
* \param state [in]
* Boolean state of the LED
*
* \return
*  Error code of the operation 
*******************************************************************************/
uint32_t bleApp_led_write_handler(cy_stc_ble_gatt_value_t* write_value){
  uint32_t error = 0; 
  /* Update the hardware state */
  if(LEN_CHAR_LED != write_value->len){error |= 1;}
  /* Update the hardware */
  if(!error){
    error |= hal_led_pin_write(write_value->val[0]);
  }
  /* Update the BLE server */
  if(!error){    
    error |= bleApp_updateGattDb(&bleState.ledHandle, write_value);
  }
  /* Print update info */
  if(bleState.print_write_events){
    if(error){
      uart_printlnf(&usb, "Error writing %s: 0x%x", bleState.ledName, error); 
      
    } else {
      uart_printlnf(&usb, "Setting LED state to: %b", write_value->val[0]); 
    }
  }
  
  return error;
}

/*******************************************************************************
* Function Name: bleApp_rgb_write_handler()
********************************************************************************
* \brief
*   Set the state of the on-board RGB
*
* \param state [in]
* Boolean state of the LED
*
* \return
*  Error code of the operation 
*******************************************************************************/
uint32_t bleApp_rgb_write_handler(cy_stc_ble_gatt_value_t* write_value){
  uint32_t error = 0; 
  /* Update the hardware state */
  if(LEN_CHAR_RGB != write_value->len){error |= 1;}
  /* Update the hardware */
  if(!error){
    error |= hal_set_rgb_duty(write_value->val[0], write_value->val[1], write_value->val[2]);
  }
  /* Update the BLE database */
  if(!error){
    error |= bleApp_updateGattDb(&bleState.rgbHandle, write_value);
  }
  /* Print update info */
  if(bleState.print_write_events){
    if(error){
      uart_printlnf(&usb, "Error writing %s: 0x%x", bleState.rgbName, error); 
    }else  {
      uart_printlnf(&usb, "Setting RGB: %d, %d, %d", write_value->val[0], write_value->val[1], write_value->val[2]);  
    }
  }
  
  return error;
}

/*******************************************************************************
* Function Name: bleApp_updateGattDb()
****************************************************************************//**
* \brief
*  Update the given handle-value pair in the GATT DB.
*
* \param handleValuePair [in]
*   The GATT DB entry to update. Pairs a DB handle with the new value it should store.
*   Generally a field of the bleState struct.
*
* \return
*  None
*******************************************************************************/
uint32_t bleApp_updateGattDb(cy_stc_ble_gatt_handle_value_pair_t* handleValuePair, cy_stc_ble_gatt_value_t* write_value) {
  uint32_t error = 0;
  /* Update the local memory object */
  memcpy(handleValuePair->value.val, write_value->val, write_value->len);
  /* Create the update parameters */
  cy_stc_ble_gatts_db_attr_val_info_t param;
  param.handleValuePair = *handleValuePair;
  param.offset = 0;
  param.connHandle = bleState.connHandle;
  param.flags = CY_BLE_GATT_DB_PEER_INITIATED; // TODO: Don't hardcode PEER
  /* Process update in local BLE server */
  error = Cy_BLE_GATTS_WriteAttributeValue(&param);
  return error;
}

/* [] END OF FILE */