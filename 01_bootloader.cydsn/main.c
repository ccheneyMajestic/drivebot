/***************************************************************************
*                           Majestic Labs  © 2023
* File: main.c
* Workspace: Creator Template
* Version: v1.0
* Author: Craig Cheney
*
* PCB: 
*
* Brief:
*   Template for the PSoC 4200 Main 
*
* Change Log:
*   2024.05.05 - Document created
********************************************************************************/
#include "project.h" /* Cypress project */
#include "mjl_errors.h"
#include "mjl_uart.h"
#include "hal_psoc4.h"

/* ####################### BEGIN PROGRAM CONFIGURATION ###################### */

//#define MJL_DEBUG
/* ---------------- DEBUG CASE ----------------
* Uncomment ONE of the following
* Debugging will only occur when MJL_DEBUG is defined
*/
#ifdef MJL_DEBUG
//    #define MJL_DEBUG_LED            /* Test the battery charger LED */
    #define MJL_DEBUG_UART            /* Test the UART Connection */
#endif
/* -------------- END DEBUG CASE --------------  */

/* ############################# BEGIN PROGRAM ############################## */
uint32_t initHardware(void);

#define BOOTLOADABLE_APP_ID    (0) /* ID of the bootloadable App */

/* Global Variables */
MLJ_UART_S usb;
const uint8_t text_is_valid_image[27] = "Valid bootloadable image: ";
const uint8_t text_starting_bootloader [29] = " *** Starting Bootloader ***";

/* Interrupt Service Routines */
void isr_button(void);

/* Main Program */
#ifndef MJL_DEBUG
/*******************************************************************************
* Function Name: main()
********************************************************************************
* Summary:
*   The top-level application function for the project.
*
*******************************************************************************/
int main(void) {
    initHardware();
    /* Set the LED */ 
    
    /* Test the supervisory flash */
    uart_printHeader(&usb, "MJL_DEBUG_BOOTLOADER", __DATE__, __TIME__);
    uart_println(&usb, "* Press the user button to exit the bootloader");
    uart_println(&usb, "");
    /* Check if the bootloadable image is valid */
    bool is_image_valid = (CYRET_SUCCESS == Bootloader_ValidateBootloadable(BOOTLOADABLE_APP_ID));
    uart_printlnf(&usb, "%s%b", text_is_valid_image, is_image_valid);  
    CyDelay(10);
    if(is_image_valid){
        bool is_user_app_active = Bootloader_GET_RUN_TYPE == Bootloader_SCHEDULE_BTLDB;
        /* Launch the user app */
        if(is_user_app_active) {
            uart_println(&usb, " * APP mode - Launching application"); 
            CyDelay(10);
            /* This will never return */
            Bootloader_Exit(Bootloader_EXIT_TO_BTLDB);
        }
        /* User app was not active */
        else {
            /* Start the bootloader -- this will never return */
            uart_printlnf(&usb, "%s", text_starting_bootloader);              
            CyDelay(100);
            Bootloader_Start();
        }
    }
    /* No valid bootloadable image, start the bootloader */
    else {
        uart_printlnf(&usb, "%s", text_starting_bootloader);  
        CyDelay(100);
        /* This will never return */
        Bootloader_Start();
    }
    for(;;) {}
  
}
#endif /* ifndef MJL_DEBUG*/
/* End Main Program */


/* ############################ BEGIN DEBUGGER ############################## */
#ifdef MJL_DEBUG
/*******************************************************************************
* DEBUGGER: main()
********************************************************************************
* Summary:
*   Debugging function for the PCB hardware
*******************************************************************************/
int main(void){
    #warning "MJL_DEBUG is enabled"

    initHardware();
    
    /* Test Cases */
    #ifdef MJL_DEBUG_LED
        /* Set pin_CHRG_STAT to Open Drain & Input */
        /* NOTE: USB must be plugged in, with no battery attached for this to work */
        /* blink the LED */
        for(;;) {
            pin_CHRG_STAT_Write(0);
            CyDelay(500);
            pin_CHRG_STAT_Write(1);
            CyDelay(500);
        }
        
    /* End MJL_DEBUG_LED */
    #elif defined MJL_DEBUG_UART           
        /* Test the UART Connection */
        uart_printHeader(&usb, "MJL_DEBUG_UART", __DATE__, __TIME__);
        CyDelay(10);
        uart_println(&usb, "");
        for(;;) {
            uint8_t readVal = 0;
            uint32_t readError = uart_read(&usb, &readVal);
            if(!readError) {
                /* Echo UART */
                uart_write(&usb, readVal);                
                /* Reset on Enter */
                if('\r' == readVal) {
                    uart_print(&usb, "\r\nResetting...");
                    CyDelay(1000);
                    CySoftwareReset();  
                }
            }
        }
    #endif 
    /* Fall through Infinite loop */
    for(;;){}
}
#endif /* MJL_DEBUG */


/* ####################### BEGIN FUNCTION DEFINITIONS ####################### */

/*******************************************************************************
* Function Name: initHardware()
********************************************************************************
* \brief
*   Configures the hardware to start
*
* \return
*  Error code of the operation
*******************************************************************************/
uint32_t initHardware(void) {
    uint32_t error = 0;
    /* Start the UART */
    MJL_UART_CFG_S uartCfg = uart_cfg_default;
    uartCfg.hal_req_writeArray = uart_psoc4SCB_writeArrayBlocking;
    uartCfg.hal_req_read = uart_psoc4SCB_read;
    uartCfg.hal_opt_externalStart = uart_psoc4SCB_start;
    uartCfg.hal_opt_externalStop = uart_psoc4SCB_stop;
    error |= uart_init(&usb, &uartCfg);
    error |= uart_start(&usb);
    /* Register the Button ISR with the interrupt */
    interrupt_btn_StartEx(isr_button);
    /* Enable global interrutps */
    CyGlobalIntEnable;
    return error;
}

/*******************************************************************************
* Function Name: isr_button()
********************************************************************************
* \brief
*   Interrupt service routine for when the button is pressed/relased
*
* \return
*  None
*******************************************************************************/
void isr_button(void){
    pin_BTN_ClearInterrupt();
    /* Disable interrupts */
    CyGlobalIntDisable;
    /* Check if the current image is valid */
    bool is_image_valid = (CYRET_SUCCESS == Bootloader_ValidateBootloadable(BOOTLOADABLE_APP_ID));
    uart_printlnf(&usb, "%s%b", text_is_valid_image, is_image_valid);  
    CyDelay(10);
    if(is_image_valid){
        uart_println(&usb, "\nExiting to the user app...");  
        /* Exit to the user application */
        Bootloader_Exit(Bootloader_EXIT_TO_BTLDB);
    }
    /* No valid application */
    else {
        /* Restore interrupts */
        CyGlobalIntEnable;   
    }
}

/* [] END OF FILE */