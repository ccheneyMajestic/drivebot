/***************************************************************************
*                                Majestic Labs © 2023
* File: LTC6915.h
* Workspace: MJL Driver Library 
* Version: v1.0.0
* Author: C. Cheney
*
* Brief: Driver file for the LTC6915 Variable gain instrumentation amplifier 
*   
*
* 2023.04.25  - Document Created
********************************************************************************/
/* Header Guard */
#ifndef LTC6915_H
  #define LTC6915_H
  /***************************************
  * Included files
  ***************************************/
  #include <stdbool.h>
  #include "mjl_spi.h"
  /***************************************
  * Macro Definitions
  ***************************************/
  #define LTC6915_MASK_D0           (1u << 0)
  #define LTC6915_MASK_D1           (1u << 1)
  #define LTC6915_MASK_D2           (1u << 2)
  #define LTC6915_MASK_D3           (1u << 3)

  /***************************************
  * Enumerated types
  ***************************************/
  typedef enum {
    LTC6915_MODE_SERIAL,
    LTC6915_MODE_PARALLEL,
  } LTC6915_MODE_T;

  
  typedef enum {
    LTC6915_GAIN_0    = 0x00,
    LTC6915_GAIN_1    = 0x01,
    LTC6915_GAIN_2    = 0x02,
    LTC6915_GAIN_4    = 0x03,
    LTC6915_GAIN_8    = 0x04,
    LTC6915_GAIN_16   = 0x05,
    LTC6915_GAIN_32   = 0x06,
    LTC6915_GAIN_64   = 0x07,
    LTC6915_GAIN_128  = 0x08,
    LTC6915_GAIN_256  = 0x09,
    LTC6915_GAIN_512  = 0x0A,
    LTC6915_GAIN_1024 = 0x0B,
    LTC6915_GAIN_2048 = 0x0C,
    LTC6915_GAIN_4096 = 0x0D,
  } LTC6915_GAIN_T;
  /***************************************
  * Structures 
  ***************************************/
  typedef struct {
    void (*fn_pin_D0_Write) (uint8_t val);    /* Generic function to write D0 pin state in parallel mode  */
    void (*fn_pin_D1_Write) (uint8_t val);    /* Generic function to write D1 pin state in parallel mode */
    void (*fn_pin_D2_Write) (uint8_t val);    /* Generic function to write D2 pin state in parallel mode */
    void (*fn_pin_D3_Write) (uint8_t val);    /* Generic function to write D3 pin state in parallel mode */
    MJL_SPI_S *spi;  /* SPI communication struct */
    LTC6915_MODE_T mode;    /* The mode of the device */
    LTC6915_GAIN_T gainWord;   /* Gain word of the amp */
    uint8_t slaveId; 
  } LTC6915_cfg_s;


  /* State Object */
  typedef struct {
    void (*fn_pin_D0_Write) (uint8_t val);    /* Generic function to write D0 pin state in parallel mode  */
    void (*fn_pin_D1_Write) (uint8_t val);    /* Generic function to write D1 pin state in parallel mode */
    void (*fn_pin_D2_Write) (uint8_t val);    /* Generic function to write D2 pin state in parallel mode */
    void (*fn_pin_D3_Write) (uint8_t val);    /* Generic function to write D3 pin state in parallel mode */
    MJL_SPI_S *spi;  /* SPI communication struct */

    LTC6915_MODE_T mode;    /* The mode of the device */
    LTC6915_GAIN_T gainWord;   /* Gain word of the amp */
    uint8_t slaveId;      /* ID of the slave in serial mode*/
    bool _init; /* Initialization error */
    bool _running;  /* Is the device running */
  } LTC6915_S;

  extern const LTC6915_cfg_s ltc6915_cfg_default;
  /***************************************
  * Function declarations 
  ***************************************/
  /* State operations */
  uint32_t ltc6915_init(LTC6915_S *const state, LTC6915_cfg_s *const cfg);
  uint32_t ltc6915_start(LTC6915_S *const state);
  uint32_t ltc6915_stop(LTC6915_S *const state);
  uint32_t ltc6915_setGainWord(LTC6915_S *const state, LTC6915_GAIN_T gainWord);
  /* Utils */
  uint32_t ltc6915_valueFromWord(LTC6915_GAIN_T gainWord, uint16_t *const result);
  uint32_t ltc6915_wordFromValue(uint16_t gain, LTC6915_GAIN_T *const gainWord);
  bool ltc6915_isValidGainWord(uint8_t gainWord);
  bool ltc6915_isValidGainVal(uint8_t gainValue);
  uint32_t ltc6915_getNextGainWord(LTC6915_S *const state, LTC6915_GAIN_T *const nextWord);
  uint32_t ltc6915_getPreviousGainWord(LTC6915_S *const state, LTC6915_GAIN_T *const prevWord);

#endif /* LTC6915_H */
    

/* [] END OF FILE */
