/***************************************************************************
*                                Majestic Labs © 2023
* File: butterworth.h
* Workspace: MJL Driver Library 
* Version: v1.0.0
* Author: C. Cheney
*
* Brief: Header for an IIR Butterworth Filter  
*
* 2023.06.04  - Document Created
********************************************************************************/
/* Header Guard */
#ifndef BUTTERWORTH_H
  #define BUTTERWORTH_H
  /***************************************
  * Included files
  ***************************************/
  #include <stdbool.h>
  #include <mjl_errors.h>
  /***************************************
  * Macro Definitions
  ***************************************/

  /***************************************
  * Enumerated types
  ***************************************/

  /***************************************
  * Structures 
  ***************************************/
  /* Forward declare struct */
  typedef struct MLJ_TEMPLATE_S MLJ_TEMPLATE_T;
  /* Configuration Structure */
  typedef struct {
    int16_t cornerFreq;  /* Corner frequency in [Hz] */
    int16_t qFactor;     /* Q-factor */
    int32_t sampleRate;  /* Sample rate in [Hz] */
  } BUTTERWORTH_CFG_S;

  /* Serial State Object   */
  typedef struct {
    int16_t cornerFreq;  /* Corner frequency in [Hz] */
    int16_t qFactor;     /* Q-factor */
    int32_t sampleRate;  /* Sample rate in [Hz] */
    int32_t numerator[5];        /* Numerator coefficients */
    int32_t denominator[5];        /* Denominator coefficients */
    int32_t inputLine[4];        /* Input delay line */
    int32_t outputLine[4];        /* Output delay line */
    int16_t result;             /* Result of the latest sample*/

    bool _init;
  } BUTTERWORTH_S;

  /* Default config struct */
  extern const BUTTERWORTH_CFG_S template_cfg_default;
  /***************************************
  * Function declarations 
  ***************************************/
  /* State Operations */
  uint32_t butterworth_init(BUTTERWORTH_S *const state, BUTTERWORTH_CFG_S *const cfg);
  uint32_t butterworth_applyInput(BUTTERWORTH_S *const state, int16_t sample);

#endif /* BUTTERWORTH_H */
/* [] END OF FILE */
