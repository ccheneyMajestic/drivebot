/***************************************************************************
*                                Majestic Labs © 2025
* File: rgb.h
* Version: v1.0.0
* Author: C. Cheney
*
* Brief: RGB  control
*
* 2025.03.30  - Document Created
********************************************************************************/
/* Header Guard */
#ifndef RGB_H
  #define RGB_H

  /***************************************
  * Included files
  ***************************************/
  #include <stdint.h>
  #include <stdbool.h>
  /***************************************
  * Structures 
  ***************************************/
  typedef struct {
    uint8_t red;
    uint8_t green;
    uint8_t blue;
  } rgb_s;

  /***************************************
  * Macros 
  ***************************************/
  /* Advertising colors */
  #define BASE_COLORS_LEN      (8)
  #define RGB_INDEX_RED               (0)
  #define RGB_INDEX_GREEN             (1)
  #define RGB_INDEX_BLUE              (2)
  #define RGB_SHIFT_RED               (RGB_INDEX_RED   * sizeof(uint8_t))
  #define RGB_SHIFT_GREEN             (RGB_INDEX_GREEN * sizeof(uint8_t))
  #define RGB_SHIFT_BLUE              (RGB_INDEX_BLUE  * sizeof(uint8_t))
  #define ALPHA_MAX 255
  
  /***************************************
  * External constants 
  ***************************************/
  extern const rgb_s* rgb_base_colors[BASE_COLORS_LEN]; 
  extern const rgb_s color_off;
  extern const rgb_s color_red;
  extern const rgb_s color_blue;
  extern const rgb_s color_green;
  extern const rgb_s color_yellow;
  extern const rgb_s color_magenta;
  extern const rgb_s color_cyan;
  extern const rgb_s color_orange;
  extern const rgb_s color_purple;
  
  /***************************************
  * Function declarations 
  ***************************************/
  bool rgb_is_equal(const rgb_s* a, const rgb_s* b);
  uint32_t rgb_get_base_color_index(const rgb_s* color, uint8_t* out);
  uint32_t rgb_word_to_struct(uint32_t rgbWord, rgb_s* rgb);  
  
#endif /* RGB_H */
/* [] END OF FILE */
