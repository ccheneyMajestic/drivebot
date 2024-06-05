/***************************************************************************
*                                Majestic Labs © 2024
* File: mjl_encoder.h
* Workspace: MJL Driver Library 
* Version: v1.0.0
* Author: C. Cheney
*
* Brief: Read from a quadrature decoder
*
* 2024.05.28  - Document Created
********************************************************************************/
#include "mjl_encoder.h"

/* Default config struct */
const mjl_encoder_cfg_s mjl_encoder_cfg_default = {
    .fn_get_encoder_count = NULL,
    .midpoint = 0,
};


/*******************************************************************************
* Function Name: mjl_encoder_init()
********************************************************************************
* \brief
*   Initializes the state struct from a configuration struct 
*
* \param state [in/out]
* Pointer to the state struct
* 
* \param cfg [in]
* Pointer to the configuration struct
*
* \return
*  Error code of the operation
*******************************************************************************/
uint32_t mjl_encoder_init(mjl_encoder_s *const state, mjl_encoder_cfg_s *const cfg){
    uint32_t error = 0;
    /* Verify required functions */
    error |= (NULL == state) ? ERROR_POINTER : ERROR_NONE;
    error |= (NULL == cfg) ? ERROR_POINTER : ERROR_NONE;
    error |= (NULL == cfg->fn_get_encoder_count) ? ERROR_POINTER : ERROR_NONE;
    /* Valid Inputs */
    if(!error) {
        /* Copy params */
        state->fn_get_encoder_count = cfg->fn_get_encoder_count;
        state->midpoint = cfg->midpoint;
        /* Mark as initialized */
        state->_init = true;
        state->_running = false;
    }
    if(error){state->_init=false;}
    return error;
}


/*******************************************************************************
* Function Name: mjl_encoder_start()
********************************************************************************
* \brief
*   Reset the encoder 
*
* \param state [in/out]
* Pointer to the state struct
*
* \return
*  Error code of the operation
*******************************************************************************/
uint32_t mjl_encoder_start(mjl_encoder_s *const state){
    uint32_t error = 0;
    if(!state->_init){error|=ERROR_INIT;}

    if(!error) {
        /* Pre-mark as running */
        state->_running = true;
        state->_overflows = 0;
        error |= mjl_encoder_read(state);
    }
    return error;
}


/*******************************************************************************
* Function Name: mjl_encoder_read()
********************************************************************************
* \brief
*   Read from hardware and update the state structure 
*
* \param state [in/out]
* Pointer to the state struct
*
* \return
*  Error code of the operation
*******************************************************************************/
uint32_t mjl_encoder_stop(mjl_encoder_s *const state){
    uint32_t error = 0;
    if(!state->_init){error|=ERROR_INIT;}
    if(!state->_running){error|=ERROR_STOPPED;}

    if(!error){
        state->_running = false;
    }
    return error;
}

/*******************************************************************************
* Function Name: mjl_encoder_read()
********************************************************************************
* \brief
*   Read from hardware and update the state structure 
*
* \param state [in/out]
* Pointer to the state struct
*
* \return
*  Error code of the operation
*******************************************************************************/
uint32_t mjl_encoder_read(mjl_encoder_s *const state){
    uint32_t error = 0;
    if(!state->_init){error|=ERROR_INIT;}
    if(!state->_running){error|=ERROR_STOPPED;}

    if(!error) {
        state->_count = state->fn_get_encoder_count();
        state->position = (int32_t) (state->_overflows * state->midpoint + (state->_count - state->midpoint));
    }
    return error;
}


/* [] END OF FILE */