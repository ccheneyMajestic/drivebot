/***************************************************************************
*                                Majestic Labs © 2023
* File: hal_psoc6.h
* Workspace: MJL Hardware Abstraction Layer (HAL) Library 
* Version: v1.0.0
* Author: C. Cheney
* Target: PSoC6
*
* Brief: Contains wrappers for the MJL library from the Cypress PSoC6 libraries
*
* 2023.12.31  - Document Created
********************************************************************************/
#include "hal_psoc6.h"
#include "mjl_errors.h"
#include "project.h"    /* Cypress files*/

/*******************************************************************************
* Function Name: uart_psoc6SCB_start()
********************************************************************************
* \brief
*   Start wrapper for a SCB based UART on PSoC6
*
* \return
*  Error code of the operation
*******************************************************************************/
uint32_t uart_psoc6SCB_start(MLJ_UART_T *const state){   
    uint32_t error = 0;   
    (void) state;
    uartUsb_Start();
    return error;
}

/*******************************************************************************
* Function Name: uart_psoc6SCB_stop()
********************************************************************************
* \brief
*   Stop wrapper for a SCB based UART on PSoC6
*
* \return
*  Error code of the operation
*******************************************************************************/
uint32_t uart_psoc6SCB_stop(MLJ_UART_T *const state){   
  uint32_t error = 0;   
  (void) state;
  Cy_SCB_UART_Disable(uartUsb_HW, &uartUsb_context);
  return error;
}

/*******************************************************************************
* Function Name: uart_psoc6SCB_writeArrayBlocking()
********************************************************************************
* \brief
*   Wrapper for an SCB Based UART on PSoC6
*   Write an array of data via UART using the SCB api. This is a blocking function
*
* \return
*  Error code of the operation
*******************************************************************************/
uint32_t uart_psoc6SCB_writeArrayBlocking(const uint8_t *array, uint16_t len){
  uint32_t error = 0;   
  Cy_SCB_UART_PutArrayBlocking(uartUsb_HW, (void *) array, len);
  return error;
}

/*******************************************************************************
* Function Name: uart_psoc6SCB_read()
********************************************************************************
* \brief
*   Wrapper for an SCB Based UART on PSoC6
*   Read a single element from the RX buffer
*
* \return
*  Error code of the operation
*******************************************************************************/
uint32_t uart_psoc6SCB_read(uint8_t *data){
  uint32_t error = 0;   
  if(0 == Cy_SCB_UART_GetNumInRxFifo(uartUsb_HW)){error|= ERROR_UNAVAILABLE;}
  else {*data = Cy_SCB_UART_Get(uartUsb_HW);}
  return error;
}



/*******************************************************************************
* Function Name: spi_psoc6SCB_start()
********************************************************************************
* \brief
*   Wrapper for an SCB Based SPI on PSoC6
*   Start the SCB Block
*
* \return
*  Error code of the operation
*******************************************************************************/
uint32_t spi_psoc6SCB_start(MJL_SPI_T *const state){
  uint32_t error = 0;
  (void) state;
  #ifdef USE_SPI
    SPI_Start();
  #endif /* USE_SPI */
  return error;
}

/*******************************************************************************
* Function Name: spi_psoc6SCB_stop()
********************************************************************************
* \brief
*   Wrapper for an SCB Based SPI on PSoC6
*   Stop the SCB blcok
*
* \return
*  Error code of the operation
*******************************************************************************/
uint32_t spi_psoc6SCB_stop(MJL_SPI_T *const state){
  (void) state;
  uint32_t error = 0;
  #ifdef USE_SPI
    Cy_SCB_SPI_Disable(SPI_HW, &SPI_context);
  #endif /* USE_SPI */
  return error;
}

/*******************************************************************************
* Function Name: spi_psoc6SCB_writeArray_blocking()
********************************************************************************
* \brief
*   Wrapper for an SCB Based SPI on PSoC6
*   Write an array of elements
*
* \return
*  Error code of the operation
*******************************************************************************/
uint32_t spi_psoc6SCB_writeArray_blocking(const uint8_t *array, uint16_t len){
  uint32_t error = 0;
  /* Write out the data */
  #ifdef USE_SPI
    Cy_SCB_SPI_WriteArrayBlocking(SPI_HW, (void *) array, len);
  #else 
    (void) array;
    (void) len;
  #endif /* USE_SPI */

  return error;
}

/*******************************************************************************
* Function Name: spi_psoc6SCB_read()
********************************************************************************
* \brief
*   Wrapper for an SCB Based SPI on PSoC6
*   Place one element from the RX FIFO into the results buffer 
*
* \return
*  Error code of the operation
*******************************************************************************/
uint32_t spi_psoc6SCB_read(uint8_t *result) {
  uint32_t error = 0;
  #ifdef USE_SPI
      /* Move data from RX FIFO to results */
      *result = Cy_SCB_SPI_Read(SPI_HW);
  #else 
    (void) result;
  #endif /* USE_SPI */

  
  return error;
}

/*******************************************************************************
* Function Name: spi_psoc6SCB_setActive()
********************************************************************************
* \brief
*   Wrapper for an SCB Based SPI on PSoC6
*   Sets an active slave 
*
* \return
*  Error code of the operation
*******************************************************************************/
uint32_t spi_psoc6SCB_setActive(uint8_t id){
  uint32_t error = 0;
  #ifdef USE_SPI
  /* Write out the data */
  Cy_SCB_SPI_SetActiveSlaveSelect(SPI_HW, id);
  #else 
    (void) id;
  #endif /* USE_SPI */
  return error;
}

/*******************************************************************************
* Function Name: spi_psoc6SCB_getRxBufferNum()
********************************************************************************
* \brief
*   Wrapper for an SCB Based SPI on PSoC6
*   Return the number of elements in the receive buffer
*
* \return
*  Error code of the operation
*******************************************************************************/
uint32_t spi_psoc6SCB_getRxBufferNum(void){
    uint32_t num = 0;
  #ifdef USE_SPI
    num = Cy_SCB_SPI_GetNumInRxFifo(SPI_HW);
  #endif /* USE_SPI */
  return num;
}

/*******************************************************************************
* Function Name: spi_psoc6SCB_getTxBufferNum()
********************************************************************************
* \brief
*   Wrapper for an SCB Based SPI on PSoC6
*   Return the number of elements in the transmit buffer
*
* \return
*  Error code of the operation
*******************************************************************************/
uint32_t spi_psoc6SCB_getTxBufferNum(void){
  uint32_t num = 0;
  #ifdef USE_SPI
    num = Cy_SCB_SPI_GetNumInTxFifo(SPI_HW);
  #endif /* USE_SPI */
  return num;
}

/*******************************************************************************
* Function Name: spi_psoc6SCB_clearRxBuffer()
********************************************************************************
* \brief
*   Wrapper for an SCB Based SPI on PSoC6
*   Clear all of the elements in the receive buffer 
*
* \return
*  Error code of the operation
*******************************************************************************/
uint32_t spi_psoc6SCB_clearRxBuffer(void) {
  uint32_t error = 0;
  #ifdef USE_SPI
  Cy_SCB_SPI_ClearRxFifo(SPI_HW);
  #endif /* USE_SPI */

  return error;
}

/*******************************************************************************
* Function Name: spi_psoc6SCB_clearTxBuffer()
********************************************************************************
* \brief
*   Wrapper for an SCB Based SPI on PSoC6
*   Clear all of the elements in the transmit buffer 
*
* \return
*  Error code of the operation
*******************************************************************************/
uint32_t spi_psoc6SCB_clearTxBuffer(void) {
  uint32_t error = 0;
  #ifdef USE_SPI
  Cy_SCB_SPI_ClearTxFifo(SPI_HW);
  #endif /* USE_SPI */

  return error;
}

/*******************************************************************************
* Function Name: hal_reset_device()
********************************************************************************
* \brief
*   Resets the microcontroller after printing to the uart, if present. This
*   function will never return.
*
* \return
*  This function will never return
*******************************************************************************/
void hal_reset_device(MLJ_UART_S *const uart){
  /* Print to uart if present */
  if(NULL != uart) {
    uart_print(&usb, "\r\nResetting...");
    CyDelay(1000);
  }
  /* Reset both cores */
  NVIC_SystemReset();
}



/*******************************************************************************
* Function Name: hal_led_pin_write()
********************************************************************************
* \brief
*  Set the state of the offboard LED
* 
* \param state [in]
* The desired state to place the pin into 
*
* \return
*  Error code of the operation
*******************************************************************************/
uint32_t hal_led_pin_write(bool state) {
  Cy_GPIO_Write(pin_LED_EXT_0_PORT, pin_LED_EXT_0_NUM, state);
  return 0;
}

/*******************************************************************************
* Function Name: hal_rgb_set_duty()
********************************************************************************
* \brief
*   Set the RGB led pwm values [0-255]
*
*******************************************************************************/
uint32_t hal_rgb_set_duty(uint8_t red, uint8_t green, uint8_t blue){
  Cy_TCPWM_PWM_SetCompare0(pwm_led_R_HW, pwm_led_R_CNT_NUM, red);
  Cy_TCPWM_PWM_SetCompare0(pwm_led_G_HW, pwm_led_G_CNT_NUM, green);
  Cy_TCPWM_PWM_SetCompare0(pwm_led_B_HW, pwm_led_B_CNT_NUM, blue);
  return 0;
}

/*******************************************************************************
* Function Name: hal_rgb_set_duty()
********************************************************************************
* \brief
*   Set the RGB led pwm values from a single uint32_t [0-255]
*
*******************************************************************************/
uint32_t hal_rgb_set_duty_word(uint32_t rgbWord){
  uint32_t error = 0;
  uint8_t red, green, blue;
  error |= hal_rgb_get_duty_word(rgbWord, &red, &green, &blue);
  error |= hal_rgb_set_duty(red, green, blue);
  return error;
}

/*******************************************************************************
* Function Name: hal_rgb_get_duty_word()
********************************************************************************
* \brief
*   Map and RGB word into the red, green, blue values
*
* \param rgbWord [in]
* RGB Word to use
*
* \param red [out]
* Pointer to place the unpacked value into
*
* \param green [out]
* Pointer to place the unpacked value into
*
* \param blue [out]
* Pointer to place the unpacked value into
*
* \return 
* Error code of the operation
*******************************************************************************/
uint32_t hal_rgb_get_duty_word(uint32_t rgbWord, uint8_t* red, uint8_t* green, uint8_t* blue){
  *red = (uint8_t) (rgbWord >> RGB_SHIFT_RED);
  *green = (uint8_t) (rgbWord >> RGB_SHIFT_GREEN);
  *blue = (uint8_t) (rgbWord >> RGB_SHIFT_BLUE);
  return 0;
}


/*******************************************************************************
* Function Name: hal_encoder_read_left()
********************************************************************************
* \brief
*   Read the current position of the left encoder
* 
* \param encoder_val [out]
* Pointer to the return value
* 
* \returns
* Error code of the operation
*******************************************************************************/
uint32_t hal_encoder_read_left(int32_t* encoder_val){
  *encoder_val = (int32_t) (Cy_TCPWM_Counter_GetCounter(encoder_left_HW, encoder_left_CNT_NUM) - ENCODER_OFFSET);
  return 0;
}

/*******************************************************************************
* Function Name: hal_encoder_read_right()
********************************************************************************
* \brief
*   Read the current position of the right encoder
* 
* \param encoder_val [out]
* Pointer to the return value
* 
* \returns
* Error code of the operation
*******************************************************************************/
uint32_t hal_encoder_read_right(int32_t* encoder_val){
  *encoder_val = (int32_t) (Cy_TCPWM_Counter_GetCounter(encoder_right_HW, encoder_right_CNT_NUM) - ENCODER_OFFSET);
  return 0;
}

/*******************************************************************************
* Function Name: hal_motors_enable()
********************************************************************************
* \brief
*   Enable or disable the motor drivers
* 
* \param left [in]
*  Enable or disable the left motor 
* 
* \param right [in]
*  Enable or disable the right motor 
* 
* \returns
* Error code of the operation
*******************************************************************************/
uint32_t hal_motors_enable(bool left, bool right){
  /* Read the current state of the control register */
  uint8_t control_word = motor_control_reg_Read();
  /* Clear the motor bits */
  control_word &= ~MOTOR_ENABLE_MASK;
  /* Set the motor bits to the new state */
  control_word |= ((uint8_t) left) << MOTOR_ENABLE_LEFT_SHIFT;
  control_word |= ((uint8_t) right) << MOTOR_ENABLE_RIGHT_SHIFT;
  /* Write the control word to the register */
  motor_control_reg_Write(control_word);
  return 0;
}

/*******************************************************************************
* Function Name: hal_motors_set_effort()
********************************************************************************
* \brief
*   Set the PWM duty ratio for each motor [-1000, 1000]
* 
* \param left [in]
*  Pwm value for the left motor
* 
* \param right [in]
*  Pwm value for the right motor
* 
* \returns
* Error code of the operation
*******************************************************************************/
uint32_t hal_motors_set_effort(int16_t left, int16_t right){
  uint32_t error = 0;
  /* Determine directions */
  bool is_left_reverse = left < 0;
  bool is_right_reverse = right < 0;
  /* Take the absolute value of the control word */
  uint16_t left_control = is_left_reverse ? -left : left;
  uint16_t right_control = is_right_reverse ? -right : right;
  /* Read the current state of the control register */
  uint8_t control_word = motor_control_reg_Read();
  /* Clear the direction bits */
  control_word &= ~MOTOR_DIRECTION_MASK;
  control_word |= ((uint8_t) is_left_reverse) << MOTOR_DIRECTION_LEFT_SHIFT;
  control_word |= ((uint8_t) is_right_reverse) << MOTOR_DIRECTION_RIGHT_SHIFT;
  /* Set the PWM values */
  pwm_left_SetCompare0(left_control);
  pwm_right_SetCompare0(right_control);
  /* Write the control word to the register */
  motor_control_reg_Write(control_word);
  return error;
}

/* Colors */
const uint32_t advertisingColors[ADVERTISING_COLORS_LEN] = {
  0x000080, /* Red */
  0x800000, /* Blue */
  0x008080, /* Yellow */
  0x800080, /* Magenta */
  0x808000, /* Cyan */
  0x004060, /* Orange */
  0x600040, /* Purple */
};
