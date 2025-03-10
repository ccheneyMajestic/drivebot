/***************************************************************************
*                             Majestic Labs © 2022
*
* File: DRV8244.h
* Workspace: mjl_drivers
* Version: v1.0.0
* Author: C. Cheney
*
* Brief: Driver functions for the DRV8244 Full bridge driver 
*   
*
* 2022.04.02  - Document Created
********************************************************************************/

/* Header Guard */
#ifndef DRV8244_H
  #define DRV8244_H
  /***************************************
  * Included files
  ***************************************/
  #include <stdint.h>
  #include <stdbool.h>
  /***************************************
  * Macro Definitions
  ***************************************/
  #define DRV8244_VAL_AWAKE             (1) /* Write to the nSLEEP pin on the DRV8244 to wake up */
  #define DRV8244_VAL_ASLEEP            (0) /* Write to the nSLEEP pin on the DRV8244 to go to sleep*/
  #define DRV8244_VAL_DISABLED           (1) /* Write to the DRVOFF pin on the DRV8244 to disable */
  #define DRV8244_VAL_ENABLED            (0) /* Write to the DRVOFF pin on the DRV8244 to go to enabled*/
  #define DRV8244_VAL_FAULT             (0)  /* read  val on nFAULT pin if there is an error */
  #define DRV8244_PERIOD_RESET          (30) /* Time in [µs] to clear the fault */
  #define DRV8244_PERIOD_SLEEP          (80) /* Time in [µs] to sleep the device */
  #define DRV8244_PERIOD_WAKE          (1000) /* Time in [µs] to sleep the device */


  /***************************************
  * Enumerated Types
  ***************************************/

  /***************************************
  * Structures
  ***************************************/
  /* Configuration Structure */
  typedef struct {
    /* Function pointers */
    void (*fn_pin_sleep_write) (uint32_t); /* Function to control the SLEEP pin */
    void (*fn_pin_drvoff_write) (uint32_t); /* Function to control the DRVFOFF pin */
    void (*fn_delay_us) (uint16_t); /* Function to delay in [µs] */
    void (*fn_criticalSection_exit)(uint32_t);   /* Exit critical timing code block */
    uint32_t (*fn_pin_fault_read) (void); /* Function to read the state of the fault pin */
    uint32_t (*fn_criticalSection_enter) (void); /* Enter critcal timing code block */
  } drv8244_cfg_s;


  
  /* State Structure */
  typedef struct {
    bool _init;
    bool _running;
    /* State Variables */
    bool isEnabled;
    /* Hardware Abstaction Layer Functions */
    void (*fn_pin_sleep_write) (uint32_t); /* Function to control the SLEEP pin */
    void (*fn_pin_drvoff_write) (uint32_t); /* Function to control the DRVFOFF pin */
    void (*fn_delay_us) (uint16_t); /* Function to delay in [µs] */
    void (*fn_criticalSection_exit)(uint32_t);   /* Exit critical timing code block */
    uint32_t (*fn_pin_fault_read) (void); /* Function to read the state of the fault pin */
    uint32_t (*fn_criticalSection_enter) (void); /* Enter critcal timing code block */
    /* Outputs */
    uint32_t output;
  } drv8244_state_s;
  
  extern const drv8244_cfg_s drv8244_config_default;
  /***************************************
  * Function declarations 
  ***************************************/
  uint32_t drv8244_init(drv8244_state_s *const state, drv8244_cfg_s *const cfg);
  uint32_t drv8244_start(drv8244_state_s *const state);
  
  uint32_t drv8244_stop(drv8244_state_s *const state);
  uint32_t drv8244_enable(drv8244_state_s *const state);
  uint32_t drv8244_disable(drv8244_state_s *const state);
  


#endif /* DRV8244_H*/
/* [] END OF FILE */