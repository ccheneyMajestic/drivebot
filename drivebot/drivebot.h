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
/* Header Guard */
#ifndef DRIVEBOT_H
    #define DRIVEBOT_H
    /***************************************
     * Included files
     ***************************************/
    #include <stdbool.h>
    #include <stdint.h>
    #include <mjl_errors.h>
    #include "mjl_motor.h"
    #include "mjl_encoder.h"
    /***************************************
     * Macro Definitions
     ***************************************/
    #define DRIVEBOT_CTR_REG_SHIFT_DIR_R        (0) /* Shift of the right motor direction pin */
    #define DRIVEBOT_CTR_REG_SHIFT_DIR_L        (1) /* Shift of the Left motor direction pin */
    #define DRIVEBOT_CTR_REG_SHIFT_EN_R         (2) /* Shift of the right motor enable pin */
    #define DRIVEBOT_CTR_REG_SHIFT_EN_L         (3) /* Shift of the Left motor enable pin */

    #define DRIVEBOT_CTR_REG_MASK_DIR_R         (1 << DRIVEBOT_CTR_REG_SHIFT_DIR_R) /* MASK of the right motor direction pin */
    #define DRIVEBOT_CTR_REG_MASK_DIR_L         (1 << DRIVEBOT_CTR_REG_SHIFT_DIR_L) /* MASK of the Left motor direction pin */
    #define DRIVEBOT_CTR_REG_MASK_EN_R          (1 << DRIVEBOT_CTR_REG_SHIFT_EN_R) /* MASK of the right motor enable pin */
    #define DRIVEBOT_CTR_REG_MASK_EN_L          (1 << DRIVEBOT_CTR_REG_SHIFT_EN_L) /* MASK of the Left motor enable pin */

    #define DRIVEBOT_CTR_REG_MASK_DIR           (DRIVEBOT_CTR_REG_MASK_DIR_R | DRIVEBOT_CTR_REG_MASK_DIR_L)   /* Mask of the direction pins for drivebot motor hardware control*/
    #define DRIVEBOT_CTR_REG_MASK_EN            (DRIVEBOT_CTR_REG_MASK_EN_R | DRIVEBOT_CTR_REG_MASK_EN_L) /* Mask of the motor enable pins for drivebot motor hardware control*/
    #define DRIVEBOT_CTR_REG_MASK               (DRIVEBOT_CTR_REG_MASK_DIR | DRIVEBOT_CTR_REG_MASK_EN) /* Mask of the motor enable pins for drivebot motor hardware control*/


    #define DRIVEBOT_WHEEL_DIAMETER_MM          (40.0) /* Drivebot v5 wheel diameter in [mm] */

    /***************************************
     * Enumerated types
     ***************************************/
    typedef float float32_t;

    /***************************************
     * Structures 
     ***************************************/


    /* Drivebot motor struct */
    typedef struct{
        mjl_encoder_s encoder;
        mjl_motor_s motor;
    } drivebot_motor_s;

    /* Drivebot configuration structure */
    typedef struct {
        mjl_encoder_cfg_s *encoder_l_cfg;
        mjl_encoder_cfg_s *encoder_r_cfg;
        mjl_motor_cfg_s *motor_l_cfg;
        mjl_motor_cfg_s *motor_r_cfg;
    } drivebot_cfg_s;

    /* Drivebot state struct */
    typedef struct{
        bool _init;
        bool _running;
        drivebot_motor_s left;
        drivebot_motor_s right;
    } drivebot_state_s;

    /* Default config struct */
    extern const drivebot_cfg_s drivebot_cfg_default;
    /***************************************
     * Function declarations 
     ***************************************/
    /* State Operations */
    uint32_t drivebot_init(drivebot_state_s *const state, drivebot_cfg_s *const cfg);
    uint32_t drivebot_start(drivebot_state_s *const state);
    uint32_t drivebot_stop(drivebot_state_s *const state);
    uint32_t drivebot_read_encoders(drivebot_state_s *const state);
    uint32_t drivebot_set_duty(drivebot_state_s *const state, int16_t duty_l, int16_t duty_r);
    uint32_t drivebot_set_enable(drivebot_state_s *const state, bool en_l, bool en_r);




#endif /* DRIVEBOT_H */
/* [] END OF FILE */
