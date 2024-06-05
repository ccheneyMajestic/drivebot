/***************************************************************************
*                                Majestic Labs © 2024
* File: drivebot.h
* Workspace: MJL Driver Library 
* Version: v1.0.0
* Author: C. Cheney
*
* Brief: Header file for Drivebot
*
* 2024.05.27  - Document Created
********************************************************************************/
#include "drivebot.h"

/* Default config struct */
const drivebot_cfg_s drivebot_cfg_default = {
    .encoder_l_cfg = NULL,
    .encoder_r_cfg = NULL,
    .motor_l_cfg = NULL,
    .motor_r_cfg = NULL,
};


/*******************************************************************************
* Function Name: drivebot_init()
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
uint32_t drivebot_init(drivebot_state_s *const state, drivebot_cfg_s *const cfg){
    uint32_t error = 0;
    /* Verify required functions */
    error |= (NULL == state) ? ERROR_POINTER : ERROR_NONE;
    error |= (NULL == cfg) ? ERROR_POINTER : ERROR_NONE;
    error |= (NULL == cfg->encoder_l_cfg) ? ERROR_POINTER : ERROR_NONE;
    error |= (NULL == cfg->encoder_r_cfg) ? ERROR_POINTER : ERROR_NONE;
    error |= (NULL == cfg->motor_l_cfg) ? ERROR_POINTER : ERROR_NONE;
    error |= (NULL == cfg->motor_r_cfg) ? ERROR_POINTER : ERROR_NONE;

    /* Valid Inputs */
    if(!error) {
        /* Copy params */
        error |= mjl_encoder_init(&state->left.encoder, cfg->encoder_l_cfg);
        error |= mjl_encoder_init(&state->right.encoder, cfg->encoder_r_cfg);
        error |= mjl_motor_init(&state->left.motor, cfg->motor_l_cfg);
        error |= mjl_motor_init(&state->right.motor, cfg->motor_r_cfg);
        /* Mark as initialized */
        state->_init = true;
        state->_running = false;
    }
    if(error){state->_init=false;}
    return error;
}

/*******************************************************************************
* Function Name: drivebot_start()
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
uint32_t drivebot_start(drivebot_state_s *const state){
    uint32_t error = 0;
    if(!state->_init){error|=ERROR_INIT;}

    if(!error){
        /* Pre-mark as running */
        state->_running = true;
        /* Start everything */
        mjl_encoder_start(&state->left.encoder);
        mjl_encoder_start(&state->right.encoder);
        mjl_motor_start(&state->left.motor);
        mjl_motor_start(&state->right.motor);
    }
    return error;
}

/*******************************************************************************
* Function Name: drivebot_stop()
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
uint32_t drivebot_stop(drivebot_state_s *const state){
    uint32_t error = 0;
    if(!state->_init){error|=ERROR_INIT;}
    if(!state->_running){error|=ERROR_STOPPED;}

    if(!error){
        mjl_encoder_stop(&state->left.encoder);
        mjl_encoder_stop(&state->right.encoder);
        mjl_motor_stop(&state->left.motor);
        mjl_motor_stop(&state->right.motor);
        state->_running = false;
    }
    return error;
}

/*******************************************************************************
* Function Name: drivebot_read_encoders()
********************************************************************************
* \brief
*   Reads the encoders and calculates the new positions. Takes into account
*   the psoc quad decoder midpoint behavior.
*
* \param state [in/out]
* Pointer to the state struct
*
* \return
*  Error code of the operation
*******************************************************************************/
uint32_t drivebot_read_encoders(drivebot_state_s *const state){
    uint32_t error = 0;
    if(!state->_init){error|=ERROR_INIT;}
    if(!state->_running){error|=ERROR_STOPPED;}

    if(!error){
        error |= mjl_encoder_read(&state->left.encoder);
        error |= mjl_encoder_read(&state->right.encoder);
    }
    return error;
}


/*******************************************************************************
* Function Name: drivebot_set_duty()
********************************************************************************
* \brief
*   Set the duty cycle for both of the motors
*
* \param state [in/out]
* Pointer to the state struct
*
* \param duty_l [in]
*   Left duty cycle
*
* \param duty_r [in]
*   Right duty cycle
*
* \return
*  Error code of the operation
*******************************************************************************/
uint32_t drivebot_set_duty(drivebot_state_s *const state, int16_t duty_l, int16_t duty_r){
    uint32_t error = 0;
    if(!state->_init){error|=ERROR_INIT;}
    if(!state->_running){error|=ERROR_STOPPED;}

    if(!error){
        error |= mjl_motor_duty_set(&state->left.motor, duty_l);
        error |= mjl_motor_duty_set(&state->right.motor, duty_r);
    }
    return error;
}

/*******************************************************************************
* Function Name: drivebot_set_enable()
********************************************************************************
* \brief
*   Enable or disable both motors
*
* \param state [in/out]
* Pointer to the state struct
*
* \param en_l [in]
*   Enable bit for the left motor 
*
* \param en_r [in]
*   Enable bit for the right motor 
*
* \return
*  Error code of the operation
*******************************************************************************/
uint32_t drivebot_set_enable(drivebot_state_s *const state, bool en_l, bool en_r){
    uint32_t error = 0;
    if(!state->_init){error|=ERROR_INIT;}
    if(!state->_running){error|=ERROR_STOPPED;}

    if(!error){
        error |= mjl_motor_enable_set(&state->left.motor, en_l);
        error |= mjl_motor_enable_set(&state->right.motor, en_r);
    }
    return error;
}




/* [] END OF FILE */