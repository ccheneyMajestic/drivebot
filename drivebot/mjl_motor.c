/***************************************************************************
*                                Majestic Labs © 2024
* File: mjl_motor.h
* Workspace: MJL Driver Library 
* Version: v1.0.0
* Author: C. Cheney
*
* Brief: Control a brushed motor
*
* 2024.05.28  - Document Created
********************************************************************************/
#include "mjl_motor.h"

/* Default config struct */
const mjl_motor_cfg_s motor_cfg_default = {
    .fn_motor_duty_write = NULL,
    .fn_motor_enable_write = NULL,
    .fn_motor_dir_write = NULL,
};


/*******************************************************************************
* Function Name: mjl_motor_init()
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
uint32_t mjl_motor_init(mjl_motor_s *const state, mjl_motor_cfg_s *const cfg){
    uint32_t error = 0;
    /* Verify required functions */
    error |= (NULL == state) ? ERROR_POINTER : ERROR_NONE;
    error |= (NULL == cfg) ? ERROR_POINTER : ERROR_NONE;
    error |= (NULL == cfg->fn_motor_duty_write) ? ERROR_POINTER : ERROR_NONE;
    error |= (NULL == cfg->fn_motor_enable_write) ? ERROR_POINTER : ERROR_NONE;
    error |= (NULL == cfg->fn_motor_dir_write) ? ERROR_POINTER : ERROR_NONE;

    /* Valid Inputs */
    if(!error) {
        /* Copy params */
        state->fn_motor_duty_write = cfg->fn_motor_duty_write;
        state->fn_motor_enable_write = cfg->fn_motor_enable_write;
        state->fn_motor_dir_write = cfg->fn_motor_dir_write;

        /* Mark as initialized */
        state->_init = true;
        state->_running = false;
    }
    if(error){state->_init=false;}
    return error;
}


/*******************************************************************************
* Function Name: mjl_motor_start()
********************************************************************************
* \brief
*   Starts the Object and call the external start function, if present 
*
* \param state [in/out]
* Pointer to the state struct
*
* \return
*  Error code of the operation
*******************************************************************************/
uint32_t mjl_motor_start(mjl_motor_s *const state){
    uint32_t error = 0;
    if(!state->_init){error|=ERROR_INIT;}

    if(!error){
        /* Pre-mark as running */
        state->_running = true;
        error|= mjl_motor_enable_set(state, false);
        error|= mjl_motor_duty_set(state, 0);
    }
    return error;
}

/*******************************************************************************
* Function Name: mjl_motor_stop()
********************************************************************************
* \brief
*   Stops the object
*
* \param state [in/out]
* Pointer to the state struct
*
* \return
*  Error code of the operation
*******************************************************************************/
uint32_t mjl_motor_stop(mjl_motor_s *const state){
    uint32_t error = 0;
    if(!state->_init){error|=ERROR_INIT;}
    if(!state->_running){error|=ERROR_STOPPED;}

    if(!error){
        error |= mjl_motor_enable_set(state, false);
        error |= mjl_motor_duty_set(state, 0);
        state->_running = false;
    }
    return error;
}


/*******************************************************************************
* Function Name: mjl_motor_duty_set()
********************************************************************************
* \brief
*   Set the duty cycle of the motor 
*
* \param state [in/out]
* Pointer to the state struct
*
* \param duty [in]
*   The duty cycle to set 
* 
* \return
*  Error code of the operation
*******************************************************************************/
uint32_t mjl_motor_duty_set(mjl_motor_s *const state, int16_t duty){
    uint32_t error = 0;
    if(!state->_init){error|=ERROR_INIT;}
    if(!state->_running){error|=ERROR_STOPPED;}

    if(!error){
        state->duty = duty;
        mjl_motor_dir_t dir = forward;
        if(duty < 0){
            dir = reverse;
            duty *= -1;
        }
        state->fn_motor_dir_write(dir);
        state->fn_motor_duty_write( (uint16_t) duty);
    }

    return error;
}

/*******************************************************************************
* Function Name: mjl_motor_enable_set()
********************************************************************************
* \brief
*   Set the enable status of the motor
*
* \param state [in/out]
* Pointer to the state struct
*
* \param duty [in]
*   The duty cycle to set 
* 
* \return
*  Error code of the operation
*******************************************************************************/
uint32_t mjl_motor_enable_set(mjl_motor_s *const state, bool will_motor_enable){
    uint32_t error = 0;
    if(!state->_init){error|=ERROR_INIT;}
    if(!state->_running){error|=ERROR_STOPPED;}

    if(!error){
        state->fn_motor_enable_write(will_motor_enable);
        state->is_enabled = will_motor_enable;
    }

    return error;
}

/* [] END OF FILE */
