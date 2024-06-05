/***************************************************************************
*                                Majestic Labs © 2023
* File: mjl_button.h
* Workspace: MJL Driver Library 
* Version: v1.0.0
* Author: C. Cheney
*
* Brief: Button Library 
*
* 2023.04.27  - Document Created
********************************************************************************/
/* Header Guard */
#ifndef MJL_BUTTON_H
  #define MJL_BUTTON_H
  /***************************************
  * Included files
  ***************************************/
  #include <stdbool.h>
  #include <mjl_errors.h>
  /***************************************
  * Macro Definitions
  ***************************************/
  #define BUTTON_LONG_PRESS     (1000) /* Duration in [ms] for a long press */
  /***************************************
  * Enumerated types
  ***************************************/

  /***************************************
  * Structures 
  ***************************************/

  /* Configuration Structure */
  typedef struct {
    bool (*fn_isBtnPressed)(void);  /* Function point to if the button is pressed */
  } MJL_BUTTON_CFG_S;

  /* Serial State Object   */
  typedef struct {
    bool (*fn_isBtnPressed)(void);  /* Function point to if the button is pressed */
    bool isBtnPressed;
    bool isBtnPressed_prev;
    bool flag_wasButtonPressed;
    bool flag_wasButtonHeld;        /* Long press */
    bool flag_wasButtonHeld_handled;
    bool flag_wasButtonReleased;
    uint64_t startMillis;
    uint32_t elapsedMillis;
    bool _init;
  } MJL_BUTTON_S;

  /* Default config struct */
  extern const MJL_BUTTON_CFG_S button_cfg_default;
  /***************************************
  * Function declarations 
  ***************************************/
  /* State Operations */
  uint32_t button_init(MJL_BUTTON_S *const state, MJL_BUTTON_CFG_S *const cfg);
  uint32_t button_updateButton(MJL_BUTTON_S *const state, uint64_t currentMillis);
  uint32_t button_clearState(MJL_BUTTON_S *const state);


#endif /* MJL_BUTTON_H */
/* [] END OF FILE */
