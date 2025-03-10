/***************************************************************************
*                                Majestic Labs © 2023
* File: mjl_force.h
* Workspace: MJL Drivers
* Version: v1.0.0
* Author: C. Cheney
*
* Brief: Calculating force from a strain gauge
*
* 2023.06.22 - Document Created
********************************************************************************/
/* Header Guard */
#ifndef MJL_FORCE_H
  #define MJL_FORCE_H
  /***************************************
  * Included files
  ***************************************/
  #include <stdint.h>
  #include <stdbool.h>
  /***************************************
  * Macro Definitions
  ***************************************/

  
  /***************************************
  * Enumerated types
  ***************************************/
  typedef enum{
    MJL_UNITS_KG,
    MJL_UNITS_LBS,
  }mjl_force_units_s;
  /***************************************
  * Structures 
  ***************************************/
  /* Configuration Structure */
  typedef struct {
    uint32_t k_force_gf_per_v;     /* Force constant [gf/V] */
    uint32_t k_adc_uv_per_count;    /* ADC constant [nV/count] */
    uint32_t k_dac_na_per_count;    /* DAC constant [nA/count] */
    uint32_t r_scale_ohm;           /* DAC resistor [Ω] */
  } mjl_force_cfg_s;

  /* Force Structure */
  typedef struct{
    bool _init;                     /* Initialization variable*/
    uint16_t adcWord;               /* ADC [counts] */
    uint16_t gainVal;               /* Gain [V/V] */
    uint8_t dacWord;                /* DAC [counts] */
    uint32_t k_force_gf_per_v;     /* Force constant [gf/V] */
    uint32_t k_adc_uv_per_count;    /* ADC constant [nV/count] */
    uint32_t k_dac_na_per_count;    /* DAC constant [nA/count] */
    uint32_t r_scale_ohm;           /* DAC resistor [Ω] */
    uint32_t offset_gf;             /* Zero point offset [g] */
    uint32_t out_force_gf;          /* Calculated force [gf] */

  } mjl_force_s;
  
  extern const mjl_force_cfg_s mjl_force_cfg_default;
  /***************************************
  * Function declarations 
  ***************************************/
  uint32_t mjl_force_init(mjl_force_s *const state, mjl_force_cfg_s *const cfg);
  uint32_t mjl_force_update(mjl_force_s *const state);

  

#endif /* MJL_FORCE_H */
/* [] END OF FILE */
