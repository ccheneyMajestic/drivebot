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
/* Header Guard */
#ifndef MLJ_MOTOR_H
    #define MLJ_MOTOR_H
    /***************************************
     * Included files
     ***************************************/
    #include <stdbool.h>
    #include <stdint.h>
    #include <mjl_errors.h>
    /***************************************
     * Macro Definitions
     ***************************************/

    /***************************************
     * Enumerated types
     ***************************************/
    typedef enum {
        forward = 0x0,
        reverse = 0x1,
    } mjl_motor_dir_t;

    /***************************************
     * Structures 
     ***************************************/

    typedef struct {
        void (*fn_motor_duty_write)(uint32_t);
        void (*fn_motor_enable_write)(bool);
        void (*fn_motor_dir_write)(mjl_motor_dir_t);
    } mjl_motor_cfg_s;

    /* Drivebot motor struct */
    typedef struct{
        bool _init;
        bool _running;
        bool is_enabled;
        int16_t duty;
        void (*fn_motor_duty_write)(uint32_t);
        void (*fn_motor_enable_write)(bool);
        void (*fn_motor_dir_write)(mjl_motor_dir_t);
    } mjl_motor_s;


    /* Default config struct */
    extern const mjl_motor_cfg_s motor_cfg_default;
    /***************************************
     * Function declarations 
     ***************************************/
    uint32_t mjl_motor_init(mjl_motor_s *const state, mjl_motor_cfg_s *const cfg);
    uint32_t mjl_motor_start(mjl_motor_s *const state);
    uint32_t mjl_motor_stop(mjl_motor_s *const state);

    uint32_t mjl_motor_duty_set(mjl_motor_s *const state, int16_t duty);
    uint32_t mjl_motor_enable_set(mjl_motor_s *const state, bool will_motor_enable);    

#endif /* MLJ_MOTOR_H */
/* [] END OF FILE */
