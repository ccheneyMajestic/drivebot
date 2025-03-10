/***************************************************************************
*                              Majestic Labs  © 2022
*                               
*
* File: DRV8244_fsm.h
* Workspace: Cloud Chamber
* Project: finite state machine (fsm)
* Version: 1.0.0
* Authors: C. Cheney
*
* Brief:
*   Header for DRV8244_fsm.cpp, the template for the DRV fsm
*
* 2022.04.04  - Document Created
********************************************************************************/

/* Header Guard */
#ifndef FSM_DRV8244_H
  #define FSM_DRV8244_H
  /***************************************
  * Included files
  ***************************************/
  #include "DRV8244.h"
  /***************************************
  * Macro Definitions
  ***************************************/


  /***************************************
  * Error Definitions
  ***************************************/


  /***************************************
  * Enumerated Types
  ***************************************/

  /***************************************
  * Structures
  ***************************************/



  /***************************************
  * Function declarations 
  ***************************************/
  uint32_t fsm_template_processState(fsm_template_state_s *const state);
  uint32_t fsm_template_cmd_wake(fsm_template_state_s *const state);
  uint32_t fsm_template_cmd_enabled(fsm_template_state_s *const state);



#endif /* FSM_DRV8244_H */
/* [] END OF FILE */
