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
#include "rgb.h"
#include <stddef.h>

/* Color profiles */
const rgb_s color_off     = {.red = 0x00, .green=0x00, .blue=0x00};
const rgb_s color_red     = {.red = 0x80, .green=0x00, .blue=0x00};
const rgb_s color_blue    = {.red = 0x00, .green=0x00, .blue=0x80};
const rgb_s color_green   = {.red = 0x00, .green=0x80, .blue=0x00};
const rgb_s color_yellow  = {.red = 0x80, .green=0x80, .blue=0x00};
const rgb_s color_magenta = {.red = 0x80, .green=0x00, .blue=0x80};
const rgb_s color_cyan    = {.red = 0x00, .green=0x80, .blue=0x80};
const rgb_s color_orange  = {.red = 0x60, .green=0x40, .blue=0x00};
const rgb_s color_purple  = {.red = 0x40, .green=0x00, .blue=0x60};
/* Player colors for the button */
const rgb_s* rgb_base_colors[BASE_COLORS_LEN] = {
  &color_red,
  &color_blue,
  &color_green,
  &color_yellow,
  &color_magenta,
  &color_cyan,
  &color_orange,
  &color_purple,
};

/*******************************************************************************
* Function Name: rgb_word_to_struct()
********************************************************************************
* \brief
*   Converts a 32-bit packed RGB word into individual red, green, and blue 
*   components and stores them in the provided rgb_s struct.
*
* \param rgbWord [in]
*   32-bit RGB value with packed red, green, and blue components.
*
* \param rgb [out]
*   Pointer to the rgb_s struct to store the unpacked RGB components.
*
* \return
*   Bitmask error code:
*     - 0: Success
*     - 1: Output pointer was NULL
*******************************************************************************/  
uint32_t rgb_word_to_struct(uint32_t rgbWord, rgb_s* rgb){
  uint32_t error = 0;
  if(rgb == NULL) {error |= 1;}
  if(!error) {
    rgb->red = (uint8_t) (rgbWord >> RGB_SHIFT_RED);
    rgb->green = (uint8_t) (rgbWord >> RGB_SHIFT_GREEN);
    rgb->blue = (uint8_t) (rgbWord >> RGB_SHIFT_BLUE);
  }
  return error;
}


/*******************************************************************************
* Function Name: rgb_is_equal()
********************************************************************************
* \brief
*   Compares two rgb_s structs for equality based on their red, green, and blue values.
*   Returns false if either input is NULL.
*
* \param a [in]
*   Pointer to the first rgb_s struct.
*
* \param b [in]
*   Pointer to the second rgb_s struct.
*
* \return
*   true if all color components are equal and inputs are valid; false otherwise.
*******************************************************************************/  

bool rgb_is_equal(const rgb_s* a, const rgb_s* b){
  if(a == NULL || b == NULL) {
    return false;
  }
  return (a->red == b->red) &&
         (a->green == b->green) &&
         (a->blue == b->blue);
}

/*******************************************************************************
* Function Name: rgb_get_base_color_index()
********************************************************************************
* \brief
*   Searches for the given color in the rgb_base_colors array and outputs its index.
*   Returns an error code if the input color is NULL or if no match is found.
*
* \param color [in]
*   Pointer to the rgb_s struct representing the color to search for.
*
* \param out [out]
*   Pointer to a variable where the index of the matching base color will be stored.
*
* \return
*   Bitmask error code:
*     - 0: Success
*     - 1: Input color was NULL
*     - 2: No matching color found
*******************************************************************************/  
uint32_t rgb_get_base_color_index(const rgb_s* color, uint8_t* out){
  uint32_t error = 0;
  if(color == NULL) {error |= 1;}
  if(out == NULL) {error |= 1;}
  
  if(!error) {
    bool was_index_found = false;
    /* Search for the colors in the base list */
    for(uint8_t i = 0; i < BASE_COLORS_LEN; i++){
      if(rgb_is_equal(rgb_base_colors[i], color)){
        *out = i;
        was_index_found = true;
        break;
      }
    }
    if(!was_index_found){error |= 2;}
  }
  return error;
}

/* [] END OF FILE */
