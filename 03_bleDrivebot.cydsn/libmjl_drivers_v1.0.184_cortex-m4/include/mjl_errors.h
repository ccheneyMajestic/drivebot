/***************************************************************************
*                             Majestic Labs © 2023
*
* File: mjl_errors.h
* Workspace: MJL Driver Library 
* Version: v1.0.0
* Author: C. Cheney
*
* Brief: Common errors for the MJL library 
*   
*
* 2023.04.17  - Document Created
********************************************************************************/

/* Header Guard */
#ifndef MJL_ERRORS_H
  #define MJL_ERRORS_H
  /***************************************
  * Included files
  ***************************************/
  #include <stdint.h>
  #include <stdbool.h>
  /***************************************
  * Macro Definitions
  ***************************************/
  #ifndef NULL
    #define NULL ((void*) 0)
  #endif

  #define ERROR_SHIFT_POINTER       (0) /* Shift of a null pointer */
  #define ERROR_SHIFT_INIT          (1) /* Shift of an uninitialized structure */
  #define ERROR_SHIFT_RUNNING       (2) /* Shift of an object already running  */
  #define ERROR_SHIFT_STOPPED       (3) /* Shift of an object stopped */
  #define ERROR_SHIFT_VAL           (4) /* Shift of a bad value error */
  #define ERROR_SHIFT_TIMEOUT       (5) /* Shift of a timeout */
  #define ERROR_SHIFT_INVALID       (6) /* Shift of an invalid operation */
  #define ERROR_SHIFT_MODE          (7) /* Shift of an invalid modeWhat */
  #define ERROR_SHIFT_PARAM         (8) /* Shift of A bad parameter was included */
  #define ERROR_SHIFT_UNAVAILABLE   (9) /* Shift of no data available */
  #define ERROR_SHIFT_STATE         (10) /* Shift of bad state to perform operation */

  #define ERROR_NONE                (0)   /* No error */
  #define ERROR_POINTER             (1 << ERROR_SHIFT_POINTER) 
  #define ERROR_INIT                (1 << ERROR_SHIFT_INIT) 
  #define ERROR_RUNNING             (1 << ERROR_SHIFT_RUNNING) 
  #define ERROR_STOPPED             (1 << ERROR_SHIFT_STOPPED) 
  #define ERROR_VAL                 (1 << ERROR_SHIFT_VAL) 
  #define ERROR_TIMEOUT             (1 << ERROR_SHIFT_TIMEOUT) 
  #define ERROR_INVALID             (1 << ERROR_SHIFT_INVALID) 
  #define ERROR_MODE                (1 << ERROR_SHIFT_MODE) 
  #define ERROR_PARAM               (1 << ERROR_SHIFT_PARAM) 
  #define ERROR_UNAVAILABLE         (1 << ERROR_SHIFT_UNAVAILABLE) 
  #define ERROR_STATE               (1 << ERROR_SHIFT_STATE) 
    
  /***************************************
  * Enumerated Types
  ***************************************/
  typedef void (* MJL_DELAY_T) (uint32_t); /* Delay function type */
  /***************************************
  * Structures
  ***************************************/

  /***************************************
  * External variables
  ***************************************/

  /***************************************
  * Function declarations 
  ***************************************/



#endif /* MJL_ERRORS_H*/
/* [] END OF FILE */