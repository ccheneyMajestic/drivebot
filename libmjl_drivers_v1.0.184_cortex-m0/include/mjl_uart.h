/***************************************************************************
*                                Majestic Labs © 2023
* File: mjl_uart.h
* Workspace: MJL Driver Library 
* Version: v1.0.0
* Author: C. Cheney
*
* Brief: Serial communication middleware for the MJL driver library 
*   
*
* 2023.04.26  - Document Created
********************************************************************************/
/* Header Guard */
#ifndef MJL_UART_H
  #define MJL_UART_H
  /***************************************
  * Included files
  ***************************************/
  #include <stdbool.h>
  #include <stdint.h>
  /***************************************
  * Macro Definitions
  ***************************************/
  #define CLEAR_SCREEN_CMD    "\033[2J\033[1;1H"
  #define CLEAR_LINE_CMD      "\033[K\r"
  #define CMD_LINE_UP         "\033[1A"
  #define CMD_CLEAR_LINEUP    "\r\033[1A\033[K"
  #define HEX_VAL_MAX                         (16)  /* Maximum value of a hex digit */
  #define BYTES_PER_FLOAT                     (4) /* Number of bytes in a float */
  #define BYTES_PER_UINT16                    (2) /* Number of bytes in a uint16_t */
  #define BYTES_PER_INT16                     (2) /* Number of bytes in a int16_t */
  #define BYTES_PER_UINT32                    (4) /* Number of bytes in a uint32_t */
  #define UART_FLOAT_DEFAULT_PREC   (6u)
  #define UART_FLOAT_BUFFER_SIZE    (32u)
  #define UART_MAX_FLOAT            (1e9)

  /***************************************
  * Enumerated types
  ***************************************/

  /***************************************
  * Structures 
  ***************************************/
  /* Forward declare struct */
  typedef struct MLJ_UART_S MLJ_UART_T;
  /* Configuration Structure */
  typedef struct {
    uint32_t (*hal_req_writeArray)(const uint8_t *array, uint16_t len);  /* Write data into the TX buffer */
    uint32_t (*hal_req_read)(uint8_t *result);         /* Move data from the RX buffer to the result */
    uint32_t (*hal_opt_externalStart)(MLJ_UART_T *const);              /* Optional External start function */
    uint32_t (*hal_opt_externalStop)(MLJ_UART_T *const);                        /* Optional External stop function */
    uint32_t opt_baud; /* Baud rate */ 
  } MJL_UART_CFG_S;

  /* Serial State Object   */
  typedef struct {
    uint32_t (*hal_req_writeArray)(const uint8_t *array, uint16_t len);  /* Write data into the TX buffer */
    uint32_t (*hal_req_read)(uint8_t *result);         /* Move data from the RX buffer to the result */
    uint32_t (*hal_opt_externalStart)(MLJ_UART_T *const);              /* Optional External start function */
    uint32_t (*hal_opt_externalStop)(MLJ_UART_T *const);                        /* Optional External stop function */
    uint32_t baud;

    bool _init;
    bool _running;
    bool isLoggingEnabled;
  } MLJ_UART_S;

  /* Default config struct */
  extern const MJL_UART_CFG_S uart_cfg_default;
  /***************************************
  * Function declarations 
  ***************************************/
  /* State Operations */
  uint32_t uart_init(MLJ_UART_S *const state, MJL_UART_CFG_S *const cfg);
  uint32_t uart_start(MLJ_UART_S *const state);
  uint32_t uart_stop(MLJ_UART_S *const state);

  uint32_t uart_write(MLJ_UART_S *const state, uint8_t data);
  uint32_t uart_read(MLJ_UART_S *const state, uint8_t * data);
  uint32_t uart_writeArray(MLJ_UART_S *const state, uint8_t * array, uint16_t len);
  uint32_t uart_readArray(MLJ_UART_S *const state, uint8_t * array, uint16_t len);
  uint32_t uart_write_reverse(MLJ_UART_S *const state, uint8_t * array, uint16_t len);
  uint32_t uart_print(MLJ_UART_S *const state, const char * pszFmt);
  uint32_t uart_println(MLJ_UART_S *const state, const char * pszFmt);
  uint32_t uart_printf(MLJ_UART_S* state, const char *pszFmt,...);
  // uint32_t uart_printlnf(MLJ_UART_S* state, const char *pszFmt,...);
  #define uart_printlnf(state,...)do{uart_printf(state,__VA_ARGS__); uart_println(state,"");}while(0)

  /* Utility Operations */
  uint32_t uart_printHeader(MLJ_UART_S* state, const char* name, const char *date, const char* time);
  uint32_t uart_hex2Ascii(uint8_t hex, uint8_t* ascii);
  uint32_t uart_printError(MLJ_UART_S* state, const char *description, uint32_t code);
  


  // uint32_t uart_printHeaderDTB(MLJ_UART_S* state, const char* name, const char *date, const char* time);
  // uint32_t uart_printErrorStatus(MLJ_UART_S* state, const char* name, uint32_t error, uint32_t subError);
  // uint32_t uart_compareReg(MLJ_UART_S* state, const char* name, uint16_t actual, uint16_t expected);
  // uint32_t uart_getInputFloat(MLJ_UART_S* state, float* result);

  // uint32_t parseAsciiFloat(uint8_t* buf, uint16_t len, float* result);

    
#endif /* MJL_UART_H */
/* [] END OF FILE */
