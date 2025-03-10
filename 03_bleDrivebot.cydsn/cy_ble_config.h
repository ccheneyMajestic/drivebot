/***************************************************************************//**
* \file cy_ble_config.h
* \version 2.90
*
* \brief
*  The user BLE configuration file. Allows redefining the configuration #define(s)
*  generated by the BLE customizer.
*
********************************************************************************
* \copyright
* Copyright 2017-2023, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#ifndef CY_BLE_CONF_H
#define CY_BLE_CONF_H

#include "ble/cy_ble_defines.h"

/**
 * The BLE_config.h file is generated by the BLE customizer and includes all common
 * configuration defines (CY_BLE_CONFIG_***).
 */
#include "BLE_config.h"

#include <cy_device_headers.h>
#ifndef CY_IP_MXBLESS
    #error "The BLE middleware is not supported on this device"
#endif

/**
 * The BLE Interrupt Notification Feature - Exposes BLE interrupt notifications
 * to an application that indicates a different link layer and radio state
 * transition to the user from the BLESS interrupt context.
 * This callback is triggered at the beginning of a received BLESS interrupt
 * (based on the registered interrupt mask). After this feature is enabled,
 * the following APIs are available:
 * Cy_BLE_RegisterInterruptCallback() and Cy_BLE_UnRegisterInterruptCallback().
 *
 * The valid value: 1u - enable / 0u - disable.
 *
 * BLE Dual mode requires an additional define IPC channel and IPC Interrupt
 * structure to send notification from the controller core to host core.
 * Use the following defines:
 * #define CY_BLE_INTR_NOTIFY_IPC_CHAN             (9..15)
 * #define CY_BLE_INTR_NOTIFY_IPC_INTR             (9..15)
 * #define CY_BLE_INTR_NOTIFY_IPC_INTR_PRIOR       (0..7)
 */
#define CY_BLE_INTR_NOTIFY_FEATURE_ENABLE          (0u)


/**
 * To redefine the config #define(s) generated by the BLE customizer,
 * use the construction #undef... #define.
 *
 * #undef CY_BLE_CONFIG_ENABLE_LL_PRIVACY
 * #define CY_BLE_CONFIG_ENABLE_LL_PRIVACY         (1u)
 *
 */


#endif /* !defined(CY_BLE_CONF_H)*/

/* [] END OF FILE */
