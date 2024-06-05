/***************************************************************************
*                              Majestic Labs  © 2024
*
* File: max31856.h
* Workspace: mjl_drivers
* Version: 1.0.0
* Authors: C. Cheney
*
* Brief:
*   Driver header for max31856.c thermocouple amplifier 
*
* 2019.06.20 - Document Created by CC
* 2024.01.07 - Ported into MJL Drivers
********************************************************************************/

/* Header Guard */
#ifndef MAX31856_H
  #define MAX31856_H
  /***************************************
  * Included files
  ***************************************/
  #include <stdint.h>
  #include "mjl_spi.h"
  /***************************************
  * Macro Definitions
  ***************************************/

  /* Temperature Conversions  */
  #define MAX31856_TEMP_SHIFT_LSB         (5)     /* Position of the least significant bit (32-bit format) */
  #define MAX31856_TEMP_SHIFT_MSB         (22)    /* Position of the most significant bit (32-bit format) */
  #define MAX31856_TEMP_SHIFT_SIGN        (23)    /* Position of the sign bit (32-bit format) */

  #define MAX31856_TEMP_MASK_SIGN         (1u << MAX31856_TEMP_SHIFT_SIGN)   /* Position of the sign bit (32-bit format) */
  
  #define MAX31856_TEMP_START_POWER       (-7)    /* Resolution starts at a power of 2^-7 */
  #define MAX31856_TEMP_NEG               (2048.0)  /* Offset for the sign bit */

  #define MAX31856_TEMP_TEST1_BIN         (0b011001000000000000000000) /* Temperature test value in Binary */
  #define MAX31856_TEMP_TEST1_F           (1600.0) /* Temperature test value in °C */
  #define MAX31856_TEMP_TEST2_BIN         (0b000001100100111100000000) /* Temperature test value in Binary */
  #define MAX31856_TEMP_TEST2_F           (100.9375) /* Temperature test value in °C */
  #define MAX31856_TEMP_TEST3_BIN         (0b000000000000000100000000) /* Temperature test value in Binary */
  #define MAX31856_TEMP_TEST3_F           (0.0625) /* Temperature test value in °C */
  #define MAX31856_TEMP_TEST4_BIN         (0b111111111111111100000000) /* Temperature test value in Binary */
  #define MAX31856_TEMP_TEST4_F           (-0.0625) /* Temperature test value in °C */
  #define MAX31856_TEMP_TEST5_BIN         (0b111111111111000000000000) /* Temperature test value in Binary */
  #define MAX31856_TEMP_TEST5_F           (-1.00) /* Temperature test value in °C */
  #define MAX31856_TEMP_TEST6_BIN         (0b111100000110000000000000) /* Temperature test value in Binary */
  #define MAX31856_TEMP_TEST6_F           (-250.00) /* Temperature test value in °C */


  /* SPI */
  #define MAX31856_MASK_WRITEADDR            (0x80)     /* Mask for writing to addresses */
  /* Configuration Register 0 (CR0) */
  #define MAX31856_CR0_ADDR               (0x00)     /* Address of Configuration 0 Register */
  #define MAX31856_CR0_VAL                (0x00)     /* Default value of Configuration 0 Register */
  #define MAX31856_CR0_SHIFT_50HZ         (0u)        /* Position of 50/60 HZ Noise rejection filter*/
  #define MAX31856_CR0_SHIFT_FAULTCLR     (1u)        /* Position of Fault status Clear */
  #define MAX31856_CR0_SHIFT_FAULT        (2u)        /* Position of Fault mode */
  #define MAX31856_CR0_SHIFT_CJ           (3u)        /* Position of Cold-junction sensor disable */
  #define MAX31856_CR0_SHIFT_OCFAULT0     (4u)        /* Position of Open circuit fault detection bit 0  */
  #define MAX31856_CR0_SHIFT_OCFAULT1     (5u)        /* Position of Open circuit fault detection bit 1 */
  #define MAX31856_CR0_SHIFT_1SHOT        (6u)        /* Position of One shot mode */
  #define MAX31856_CR0_SHIFT_CMODE        (7u)        /* Position of Conversion Mode */

  #define MAX31856_CR0_MASK_50HZ          (1u << MAX31856_CR0_SHIFT_50HZ    )        /* Mask of 50/60 HZ Noise rejection filter*/
  #define MAX31856_CR0_MASK_FAULTCLR      (1u << MAX31856_CR0_SHIFT_FAULTCLR)        /* Mask of Fault status Clear */
  #define MAX31856_CR0_MASK_FAULT         (1u << MAX31856_CR0_SHIFT_FAULT   )        /* Mask of Fault mode */
  #define MAX31856_CR0_MASK_CJ            (1u << MAX31856_CR0_SHIFT_CJ      )        /* Mask of Cold-junction sensor disable */
  #define MAX31856_CR0_MASK_OCFAULT0      (1u << MAX31856_CR0_SHIFT_OCFAULT0)        /* Mask of Open circuit fault detection MASK 0  */
  #define MAX31856_CR0_MASK_OCFAULT1      (1u << MAX31856_CR0_SHIFT_OCFAULT1)        /* Mask of Open circuit fault detection MASK 1 */
  #define MAX31856_CR0_MASK_1SHOT         (1u << MAX31856_CR0_SHIFT_1SHOT   )        /* Mask of One shot mode */
  #define MAX31856_CR0_MASK_CMODE         (1u << MAX31856_CR0_SHIFT_CMODE   )        /* Mask of Conversion Mode */

  
  /* Configuration Register 1 (CR1) */
  #define MAX31856_CR1_ADDR               (0x01)      /* Address of Configuration 1 Register */
  #define MAX31856_CR1_VAL                (0x03)      /* Default value of Configuration 1 Register */
  #define MAX31856_CR1_SHIFT_TCTYPE       (0)         /* Position of the LSB of the Thermocouple type */
  #define MAX31856_CR1_SHIFT_AVGSEL       (4)         /* Position of the LSB of the Average mode selection */
  
  #define MAX31856_CR1_MASK_TC_B          (0b0000)    /* Thermocouple type B */
  #define MAX31856_CR1_MASK_TC_E          (0b0001)    /* Thermocouple type E */
  #define MAX31856_CR1_MASK_TC_J          (0b0010)    /* Thermocouple type J */
  #define MAX31856_CR1_MASK_TC_K          (0b0011)    /* Thermocouple type K */
  #define MAX31856_CR1_MASK_TC_N          (0b0100)    /* Thermocouple type N */
  #define MAX31856_CR1_MASK_TC_R          (0b0101)    /* Thermocouple type R */
  #define MAX31856_CR1_MASK_TC_S          (0b0110)    /* Thermocouple type S */                  
  #define MAX31856_CR1_MASK_TC_T          (0b0111)    /* Thermocouple type T */

  #define MAX31856_CR1_MASK_AVG1          (0b000 << MAX31856_CR1_SHIFT_AVGSEL)    /* Average of 1 sample */
  #define MAX31856_CR1_MASK_AVG2          (0b001 << MAX31856_CR1_SHIFT_AVGSEL)    /* Average of 2 samples */
  #define MAX31856_CR1_MASK_AVG4          (0b010 << MAX31856_CR1_SHIFT_AVGSEL)    /* Average of 4 samples */
  #define MAX31856_CR1_MASK_AVG8          (0b011 << MAX31856_CR1_SHIFT_AVGSEL)    /* Average of 8 samples */
  #define MAX31856_CR1_MASK_AVG16         (0b100 << MAX31856_CR1_SHIFT_AVGSEL)    /* Average of 16 samples */

  /* Linearized Temperature Low Fault Threshold MSB (LTLFTH) */
  #define MAX31856_LTLFTH_ADDR            (0x07)      /* Address of Linearized Temperature Low Fault Threshold MSB */
  #define MAX31856_LTLFTH_VAL             (0x80)      /* Default Value of Linearized Temperature Low Fault Threshold MSB */
  
  /* Linearize TC Temp Byte 2 (LTCBH) */
  #define MAX31856_LTCBH_ADDR             (0x0C)      /* Address of TC Temp high byte */
  /* Linearize TC Temp Byte 1 (LTCBM) */
  #define MAX31856_LTCBM_ADDR             (0x0D)      /* Address of TC Temp mid byte */
  /* Linearize TC Temp Byte 0 (LTCBL) */
  #define MAX31856_LTCBL_ADDR             (0x0E)      /* Address of TC Temp high byte */

  /* Fault Status RegisterB (SR) */    
  #define MAX31856_SR_ADDR        (0x0F)      /* Fault Status Register */
  #define MAX31856_SR_VAL         (0x00)      /* Fault Status Register */
  
  
  /***************************************
  * Enumerated Types
  ***************************************/

  
  /***************************************
  * Structures
  ***************************************/
  typedef struct {
    uint8_t slaveId;    /* ID of the slave */    
    MJL_SPI_S* spi;    /* SPI comms struct */
  } max31856_cfg_s;

  typedef struct {
    bool _init;     /* Has state struct been initizlied? */
    bool _running;  
    uint32_t error;     /* Passing error codes out */
    uint8_t slaveId;    /* ID of the slave */    
    MJL_SPI_S* spi;    /* SPI comms struct */
    float temp;         /* latest temperature reading */
  } max31856_state_s;
  
  extern const max31856_cfg_s max31856_config_default;
  /***************************************
  * Function declarations 
  ***************************************/
  uint32_t max31856_init(max31856_state_s *const state, max31856_cfg_s *const cfg);
  uint32_t max31856_start(max31856_state_s *const state);

  uint32_t max31856_readReg(max31856_state_s* state, uint8_t addr, uint8_t *result);
  uint32_t max31856_writeReg(max31856_state_s* state, uint8_t addr, uint8_t val);
  uint32_t max31856_readArray(max31856_state_s* state, uint8_t *array, uint8_t len);
  uint32_t max31856_writeArray(max31856_state_s* state, uint8_t *array, uint8_t len);

  uint32_t max31856_readTemp(max31856_state_s* state);
  
  float max31856_convertTemp(uint32_t temp);

    

#endif /* MAX31856_H */
/* [] END OF FILE */
