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
/* Header Guard */
#ifndef MLJ_ENCODER_H
    #define MLJ_ENCODER_H
    /***************************************
     * Included files
     ***************************************/
    #include <stdbool.h>
    #include <stdint.h>
    #include "mjl_types.h"
    #include "mjl_errors.h"
    /***************************************
     * Macro Definitions
     ***************************************/

    /***************************************
     * Enumerated types
     ***************************************/


    /***************************************
     * Structures 
     ***************************************/
    typedef struct{
        uint32_t (*fn_get_encoder_count)(void);
        uint16_t midpoint; 
    } mjl_encoder_cfg_s;

    typedef struct{
        bool _init;
        bool _running;
        int32_t position;
        uint16_t _count;
        volatile int32_t _overflows;
        uint16_t midpoint; 
        uint32_t (*fn_get_encoder_count)(void);
    } mjl_encoder_s;

    /* Default config struct */
    extern const mjl_encoder_cfg_s mjl_encoder_cfg_default;
    /***************************************
     * Function declarations 
     ***************************************/
    uint32_t mjl_encoder_init(mjl_encoder_s *const state, mjl_encoder_cfg_s *const cfg);
    uint32_t mjl_encoder_start(mjl_encoder_s *const state);
    uint32_t mjl_encoder_stop(mjl_encoder_s *const state);
    uint32_t mjl_encoder_read(mjl_encoder_s *const state);


#endif /* MLJ_ENCODER_H */
/* [] END OF FILE */
