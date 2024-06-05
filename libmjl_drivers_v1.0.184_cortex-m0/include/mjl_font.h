/***************************************************************************
*                             Majestic Labs © 2023
*
* File: mjl_font.h
* Workspace: MJL Driver Library 
* Version: v1.0.0
* Author: C. Cheney
*
* Brief: The majestic lab font
*   
*
* 2023.04.17  - Document Created
********************************************************************************/

/* Header Guard */
#ifndef MJL_FONT_H
  #define MJL_FONT_H
  /***************************************
  * Included files
  ***************************************/
  #include <stdint.h>
  #include <stdbool.h>
  /***************************************
  * Macro Definitions
  ***************************************/
  #define UI_TEXT_8x16_LEN                    (16) /* Number of byte in an 8x16 character */
  #define UI_ICON_BATTERY_LEN                 (16) /* number of bytes in the battery icon */
  #define UI_ICON_HISTORY_LEN                 (48) /* number of bytes in the history icon */
  #define UI_ICON_RESET_LEN                   (48) /* number of bytes in the reset icon */

  #define UI_CHARS_IDX_SPACE                  (0) /* Index of the space character in specialChars[]*/
  #define UI_CHARS_IDX_DASH                   (1) /* Index of the space character in specialChars[]*/
  #define FONT_LEN_DECIMAL              (4) /* Length of the decimal, colon, etc. */

  #define FONT_BATT_SINGLE_LINE (0x7E)
  #define FONT_BATT_MIN_WIDTH (7)
  #define FONT_BATT_SOC_MAX (100)
  #define FONT_BATT_BORDER_LEFT   (2)
  #define FONT_BATT_BORDER_RIGHT  (1)

  /***************************************
  * External Vars
  ***************************************/
  extern const uint8_t* digits_8x16[10];
  extern const uint8_t* alphabet_8x16[26];
  extern const uint8_t* specialChars_8x16[10];

  extern const uint8_t icon_batteryOutline[UI_ICON_BATTERY_LEN];
  extern const uint8_t icon_battery_1bar[UI_ICON_BATTERY_LEN];
  extern const uint8_t icon_blueTooth[16]; 
  extern const uint8_t icon_charging[16]; 
  extern const uint8_t icon_downArrow[16];  
  extern const uint8_t icon_upArrow[16]; 
  extern const uint8_t icon_colon[4];   
  extern const uint8_t icon_decimalPoint[4];
  extern const uint8_t icon_history[UI_ICON_HISTORY_LEN];
  extern const uint8_t icon_reset[UI_ICON_RESET_LEN];
  extern const uint8_t icon_back[UI_ICON_RESET_LEN];

  /***************************************
  * Structures
  ***************************************/

  /***************************************
  * External variables
  ***************************************/

  /***************************************
  * Function declarations 
  ***************************************/
  uint32_t font_generateBatteryIcon(uint8_t stateOfCharge, uint8_t width, uint8_t* outputBuffer);


#endif /* MJL_FONT_H*/
/* [] END OF FILE */