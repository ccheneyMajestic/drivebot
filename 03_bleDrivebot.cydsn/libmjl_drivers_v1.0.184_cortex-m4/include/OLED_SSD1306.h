/***************************************************************************
*                             Majestic Labs © 2023
*
* File: OLED_SSD1306.h
* Workspace: MJL Driver Library 
* Version: v1.0.0
* Author: C. Cheney
*
* Brief: Driver library for the SSD1306 OLED Display 128x64
*   
*
* 2023.02.18  - Document Created
********************************************************************************/

/* Header Guard */
#ifndef SSD1306_H
  #define SSD1306_H
  /***************************************
  * Included files
  ***************************************/
  #include <stdint.h>
  #include <stdbool.h>
  #include <stddef.h>
  #include "mjl_ringBuffer.h"
  /***************************************
  * Macro Definitions
  ***************************************/
  #define SSD1306_RESET_DISASSERT             (1) /* Deassert the reset pin */
  #define SSD1306_RESET_ASSERT                (0) /* Assert the reset pin */
  #define SSD1306_PAGE_HEIGHT                 (8) /* Pixels per page */
  #define SSD1306_NUM_PAGE                    (8)  /* Number of pages in the display */
  #define SSD1306_NUM_COLS                    (128)  /* Number of cols in the display */
  #define SSD1306_NUM_ROWS                    (64)  /* Number of Rows in the display */
  #define SSD1306_DC_DATA                     (1) /* Write data */
  #define SSD1306_DC_COMMAND                  (0) /* Write command */
  #define SSD1306_DC_DATA                     (1) /* Write data */
  #define SSD1306_DC_COMMAND                  (0) /* Write command */

  #define SSD1306_CMD_ALL_ON                  (0xA5) /* Command to turn all LEDs on */
  #define SSD1306_CMD_ALL_RAM                 (0xA4) /* Command to turn all LEDs to match what's in RAM */
  #define SSD1306_CMD_CHARGE_PUMP             (0x8D) /* Command to address the charge pump */
  #define SSD1306_CMD_CHARGE_PUMP_ON          (0x14) /* Enable Charge pump */
  #define SSD1306_CMD_CHARGE_PUMP_OFF         (0x10) /* Disable Charge pump */
  #define SSD1306_CMD_ON                      (0xAF) /* Enable the Display (wakeup) */
  #define SSD1306_CMD_SLEEP                   (0xAE) /* Disable the Display (sleep) */

  #define SSD1306_CMD_ADDRESS_MODE            (0x20) /* Set the addressing mode */
  #define SSD1306_CMD_ADDRESS_MODE_HORIZONTAL (0x00) /* Horizontal Addressing mode */
  #define SSD1306_CMD_ADDRESS_MODE_VERTICAL   (0x01) /* Vertical Addressing mode */
  #define SSD1306_CMD_ADDRESS_MODE_PAGE       (0x02) /* Page Addressing mode (default) */

  #define SSD1306_CMD_COLUMN_ADDR             (0x21) /* Set the column address */
  #define SSD1306_CMD_PAGE_ADDR               (0x22) /* Set the page address */

  #define SSD1306_DELAY_US_RESET              (100) /* Reset time in [µs]*/
  #define SSD1306_DELAY_US_DC                 (1) /* Time in [µs] for data command*/



  #define DISPLAY_LEN_16x32                   (64)
  #define CHAR_NUM_COLS                       (8) /* Number of columns in a character*/
  #define ROW_NUM_CHARS                       (SSD1306_NUM_COLS/CHAR_NUM_COLS) /* Maximum number of characters per row*/
  #define ASCII_OFFSET_LETTER_CAP             ('A') /* Use to map capital ascii to alphabet offset */
  #define ASCII_OFFSET_LETTER_LOWER           ('a') /* Use to map lowercase ascii to alphabet offset */
  #define ASCII_OFFSET_DIGIT                  ('0') /* Use to map ascii to alphabet offset */
  
  #define BITS_PER_BYTE (8)


  /***************************************
  * Enumerated Types
  ***************************************/
  /* Enumerated State Types */
  typedef enum {
    SSD1306_STATE_OFF,
    SSD1306_STATE_ON,
  } ssd1306_state_t;

  /* Addressing mode */
  typedef enum{
    SSD1306_ADDRESSING_HORIZONTAL = SSD1306_CMD_ADDRESS_MODE_HORIZONTAL,
    SSD1306_ADDRESSING_VERTICAL = SSD1306_CMD_ADDRESS_MODE_VERTICAL,
    SSD1306_ADDRESSING_PAGE = SSD1306_CMD_ADDRESS_MODE_PAGE
  } ssd1306_addressing_mode_t;
  
  /***************************************
  * Structures
  ***************************************/


  /* Control the display window */
  typedef struct {
    uint8_t pageStart;
    uint8_t pageEnd;
    uint8_t colStart;
    uint8_t colEnd;  
  } display_window_s;

  typedef struct {
    uint8_t origin_col;
    uint8_t origin_row;
    uint8_t size_cols;
    uint8_t size_rows;
    uint8_t repeat_num;
    uint8_t repeat_spacing;
  } display_position_s;

  /* Icons */
  typedef struct {
    display_position_s pos;
    const uint8_t * data;
    bool wasIconUpdated;
    bool shouldIconRender;
    bool isInverted;
  }display_icon_s;

  /* Strings */
  typedef struct {
    display_position_s pos;
    char data[ROW_NUM_CHARS];
    bool wasTextUpdated;
    bool shouldTextRender;
    bool isInverted;
  }display_text_s;

  /* Lines */
  typedef struct{
    uint8_t rowStart;
    uint8_t colStart;
    uint8_t numCol;
    uint8_t numRow;
  } display_line_cfg_s;

  typedef struct {
    uint8_t colStart;
    uint8_t rowStart;
    uint8_t numCol;
    uint8_t numRow;


    uint8_t rowEnd;
    uint8_t colEnd;
    uint8_t pageStart;
    uint8_t pageEnd;
    uint8_t numPage;
    bool _isInit;
    bool shouldRender;
  } display_line_s;
  extern const display_line_cfg_s display_line_cfg_default;


  /* Graph */
  typedef struct{
    uint8_t rowStart;
    uint8_t colStart;
    uint8_t numCol;
    uint8_t numRow;
  } display_graph_cfg_s;

  typedef struct {
    uint8_t rowStart;
    uint8_t colStart;
    uint8_t numCol;
    uint8_t numRow;

    uint8_t rowEnd;
    uint8_t colEnd;
    uint8_t pageStart;
    uint8_t pageEnd;
    uint8_t numPage;
    display_line_s axis_horizontal;
    display_line_s axis_vertical;
    bool _isInit;
    bool shouldGraphRender;
  } display_graph_s;
  extern const display_graph_cfg_s display_graph_cfg_default;

  /* Object Configuration Structure */
  typedef struct{
    /* HAL Function Pointers */
    uint32_t (*fn_spi_writeArrayBlocking) (uint8_t slaveId, const uint8_t * cmdArray, uint16_t len);
    void (*fn_pin_reset_write) (uint8_t val);
    void (*fn_pin_dataCommand_write) (uint8_t val);
    void (*fn_delayUs)(uint16_t microsecond);
    display_window_s fullWindow;
    uint8_t spi_slaveId;
    /* Object function pointer */
  } ssd1306_cfg_s;


  /* Object State Structure */
  typedef struct {
    /* Required Variables */
    bool _isInitialized;
    bool _isLocked;
    uint32_t output;
    uint32_t error;
    /* Input parameters */
    uint32_t (*fn_spi_writeArrayBlocking) (uint8_t slaveId, const uint8_t * cmdArray, uint16_t len);
    void (*fn_pin_reset_write) (uint8_t val);
    void (*fn_pin_dataCommand_write) (uint8_t val);
    void (*fn_delayUs)(uint16_t microsecond);
    display_window_s fullWindow;
    uint8_t spi_slaveId;
    /* Nested objects */

    /* State variables */
    ssd1306_addressing_mode_t addressMode;
    uint32_t stateCount;
    ssd1306_state_t state_current;
    ssd1306_state_t state_next;
    ssd1306_state_t state_previous;
    ssd1306_state_t state_requested;
    /* Flags and status */
  } ssd1306_state_s;

  /***************************************
  * External variables
  ***************************************/

  /***************************************
  * Function declarations 
  ***************************************/
  /* Required Functions */
  uint32_t SSD1306_init(ssd1306_state_s *const state, ssd1306_cfg_s *const cfg);
  uint32_t SSD1306_start(ssd1306_state_s *const state);
  uint32_t SSD1306_stop(ssd1306_state_s *const state);
  uint32_t SSD1306_writeCommandArray(ssd1306_state_s *const state, uint8_t * cmdArray, uint8_t len);
  uint32_t SSD1306_writeDataArray(ssd1306_state_s *const state, const uint8_t * dataArray, uint16_t len);
  uint32_t SSD1306_setWindow(ssd1306_state_s *const state, display_window_s *const window);
  uint32_t SSD1306_setAddressingMode(ssd1306_state_s *const state, ssd1306_addressing_mode_t mode);
  uint32_t SSD1306_clearScreen(ssd1306_state_s *const state);
  uint32_t SSD1306_drawDigit_8x16(ssd1306_state_s *const state, uint8_t num);
  uint32_t SSD1306_setDigits(ssd1306_state_s *const state, uint8_t* digits, display_position_s *const pos);
  uint32_t SSD1306_setDigits_16x32(ssd1306_state_s *const state, const uint8_t **digits, display_position_s *const pos);
  uint32_t SSD1306_setLetters(ssd1306_state_s *const state, const uint8_t **letters, display_position_s *const pos);
  uint32_t SSD1306_renderString(ssd1306_state_s *const state, display_text_s *const text);
  uint32_t SSD1306_setIcon(ssd1306_state_s *const state, display_icon_s *const icon);
  uint32_t SSD1306_clearIcon(ssd1306_state_s *const state, display_icon_s *const icon);
  uint32_t SSD1306_setIcon_inverse(ssd1306_state_s *const state, display_icon_s *const icon);

  uint32_t create16x32(const uint8_t * inObject, uint8_t * outObject);
  uint32_t tokenizeNumber(uint16_t val, uint8_t *outArray);

  uint32_t SSD1306_drawLine(ssd1306_state_s *const state, display_line_s *const line);

  /* Display objects */
  uint8_t reverseBits(uint8_t byte);
  uint32_t windowFromPos(display_position_s *const pos, uint8_t instanceNum, display_window_s *const window);
  uint32_t display_graph_init(display_graph_s *const state, display_graph_cfg_s *const cfg);
  uint32_t display_updateGraphColumn(ssd1306_state_s *const state, display_graph_s *const graph, uint8_t colIdx, uint8_t val);
  uint32_t display_updateGraphScroll(ssd1306_state_s *const state, display_graph_s *const graph, mjl_ring_s *const ring);
  uint32_t display_line_init(display_line_s *const state, display_line_cfg_s *const cfg);



  /* Actions */


#endif /* SSD1306_H*/
/* [] END OF FILE */