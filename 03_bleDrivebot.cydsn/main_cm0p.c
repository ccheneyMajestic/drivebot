/***************************************************************************
*                           Majestic Labs  © 2023
* File: template_main_psoc6.c
* Workspace: MJL Driver Templates
* Version: v1.0
* Author: Craig Cheney
*
* PCB: 
*
* Brief:
*   Template for the PSoC6 Main for the CM0+ Core
*
* Change Log:
*   2023.12.31 - Document created
********************************************************************************/
#include "project.h" /* Cypress project */
#include "mjl_errors.h"
#include "mjl_uart.h"
#include "hal_psoc6.h"
#include "LTC6915.h"
#include "DRV8244.h"
#include "max31856.h"
#include "ble_app.h"


/* ####################### BEGIN PROGRAM CONFIGURATION ###################### */

//#define MJL_DEBUG
/* ---------------- DEBUG CASE ----------------
* Uncomment ONE of the following
* Debugging will only occur when MJL_DEBUG is defined
*/
#ifdef MJL_DEBUG
//  #define MJL_DEBUG_LED             /* Test the battery charger LED */
//  #define MJL_DEBUG_UART            /* Test the UART Connection */
//  #define MJL_DEBUG_BTN             /* Test the button */
//  #define MJL_DEBUG_ENCODERS        /* Test the encoders */
//  #define MJL_DEBUG_MOTORS          /* Press the button to toggle motors */
//  #define MJL_DEBUG_COLORS          /* Cycle through all of the colors */
//  #define MJL_DEBUG_FLASH           /* Test reading and writing from flash */
//  #define MJL_DEBUG_BLE             /* Test the basic ble application */
  #define MJL_DEBUG_BREATHE         /* Breathe the RGB LED*/
//  #define MJL_DEBUG_ADC               /* TODO: Read the ADC */
//  #define MJL_DEBUG_ADC_INT          /* TODO: Read the ADC using the interrupt flag */
//  #define MJL_DEBUG_ADC_AVG          /* TODO: Read the ADC and average in software */
//  #define MJL_DEBUG_SYSTEM_COUNTER      /* TODO: Test using the microsecond counter */
#endif
/* -------------- END DEBUG CASE --------------  */

/* Enable SPI */

/* ############################# BEGIN PROGRAM ############################## */
/* Macro Definitions */
#define PWM_DRV_PERIOD (999)
#define PWM_DRV_DUTY_50  (PWM_DRV_PERIOD+1)/2
#define PWM_DRV_DUTY_100  (PWM_DRV_PERIOD+1)
#define ADC_CHAN_DRV_IPROP (0) /* Current measure of the DRV */
#define ADC_SAMPLES_PER_SEC  (939) /* Rate of the ADC in [Hz] */
#define TEMP_SENSE_ID  (0)
#define DRV_R_IPROPI   (1000.0) /* Shunt resistor value in [Ω] */
#define DRV8244_GAIN_IPROPI_QFN       (4750.0) /* Current gain in [A/A] */
#define DRV_VIPROPI_TO_I  (DRV8244_GAIN_IPROPI_QFN / DRV_R_IPROPI) /* [1/Ω] - Multiply V_ipropi by this to get the current in [A]  */

/* System timer */
typedef struct {
  uint32_t second;
  uint32_t microsecond;
} system_time_s;

uint32_t get_system_time_atomic(system_time_s *const state);

uint32_t drv8244_current_from_voltage(drv8244_state_s *const, float32_t voltage); 


/* Global Variables */
MLJ_UART_S usb;
alpha_dimmer_s dimmer_rgb;

//volatile bool flag_timer_second = false;
//volatile bool flag_adc = false;
//system_time_s time;

/* ISR Declarations */
void isr_rgb_breathe(void);
void isr_adc(void);
void isr_second(void);

/* Function Definitions */
uint32_t initHardware(void);
uint32_t process_rgb_breathing(alpha_dimmer_s* state);


/* Main Program */
#ifndef MJL_DEBUG
/*******************************************************************************
* Function Name: main()
********************************************************************************
* Summary:
*   The top-level application function for the project.
  
  // Enable CM4.  CY_CORTEX_M4_APPL_ADDR must be updated if CM4 memory layout is changed.
  //  Cy_SysEnableCM4(CY_CORTEX_M4_APPL_ADDR); 
*
*******************************************************************************/
int main(void) {  
  initHardware();
    /* Start ble */
  uart_printHeader(&usb, "MJL_DRIVEBOT_MAIN", __DATE__, __TIME__);
  CyDelay(10);
  uart_println(&usb, "");
  uart_println(&usb,"* Press 'Enter' to reset");
  uart_println(&usb, "");
  
  
  /* Configure the state object */
  bleState.breathing = &dimmer_rgb;
  /* Start the BLE component */
  Cy_BLE_Start(bleApp_eventCallback);
  CyDelay(10);
  /* Delay to wait for the BLE stack to finish startup Flash operations, which take ~2 ms.
     This is an inelegant way to do so, but I can't find a way of reading the current flash state.
     Presumably this is because the BLE stack is using async flash operations, which don't play well
     the synchronous flash operations. No BLE stack event is generated that clearly indicates Flash
     operations are complete, so a blocking delay appears to be the simplest solution. */
  
  /********* Set the initial states *********/
  /* Read the ID from flash -- Input parameter is ignored */
  bleApp_id_update_state(0, true);
  /* LED */
  bleApp_led_update_state(false, true);
  /* RGB */
  rgb_s flash_color = color_off;
  hal_flash_rgb_read(&flash_color);
  bleApp_rgb_update_state(&flash_color, true, false);
  /* Motor Enable */
  bleApp_motorEnable_update_state(false, false, true);
  /* Motor Effort */
  bleApp_motorEffort_update_state(500, 500, true);
  
  /* Encoder */
  int32_t encoder_left = 0;
  int32_t encoder_left_prev = 0;
  int32_t encoder_right = 0;
  int32_t encoder_right_prev = 0;
  
  /* Read the button state */
  bool stateBtn = false;
  bool stateBtn_prev = false;
  
  for(;;) {
    /* Handle the UART */
    uint8_t readVal = 0;
    uint32_t readError = uart_read(&usb, &readVal);
    if(!readError) {              
      /* Reset on Enter */
      if('\r' == readVal) {hal_reset_device(&usb);}
    }
    /* Handle the encoders */
    hal_encoder_read_left(&encoder_left);
    hal_encoder_read_right(&encoder_right);      
    if((encoder_left != encoder_left_prev) || (encoder_right != encoder_right_prev)){
      encoder_left_prev = encoder_left;
      encoder_right_prev = encoder_right;
      bleApp_encoder_update_state(encoder_left, encoder_right);
    }
    /* Handle the button */
    stateBtn = Cy_GPIO_Read(pin_BTN_0_PORT, pin_BTN_0_NUM);
    if(stateBtn != stateBtn_prev){
      stateBtn_prev = stateBtn;
      /* Toggle on release */
      if(stateBtn == false){
        uart_println(&usb, "Button release");
        /* Change the RGB color */
        bleApp_cycle_rgb();
      }
    }    
    /* Handle the rgb dimming */
    process_rgb_breathing(bleState.breathing);
    
    /* Handle ble events */
    Cy_BLE_ProcessEvents();
  }
}
#endif /* ifndef MJL_DEBUG*/
/* End Main Program */


/* ############################ BEGIN DEBUGGER ############################## */
#ifdef MJL_DEBUG
/*******************************************************************************
* DEBUGGER: main()
********************************************************************************
* Summary:
*   Debugging function for the PCB hardware
*******************************************************************************/
int main(void){
  #warning "MJL_DEBUG is enabled"
  initHardware();
  
  /* Test Cases */
  #ifdef MJL_DEBUG_LED
    /* Cycle the RGB LED */   
    rgb_s rgb ={
      .alpha = 25 
    };
    for(;;) {
      /* Red */
      rgb.red = 0xff;
      rgb.green = 0;
      rgb.blue = 0;
      hal_rgb_set_color(&rgb);
      CyDelay(500);
      /* Green */
      rgb.red = 0;
      rgb.green = 0xff;
      rgb.blue = 0;
      hal_rgb_set_color(&rgb);
      CyDelay(500);
      /* Blue */
      rgb.red = 0;
      rgb.green = 0;
      rgb.blue = 0xff;
      hal_rgb_set_color(&rgb);
      CyDelay(500);   
      /* Increase the brightness */
      rgb.alpha += 25;
    }
  /* End MJL_DEBUG_LED */
  #elif defined MJL_DEBUG_UART             
  /* Test the UART Connection */
  uart_printHeader(&usb, "MJL_DEBUG_UART", __DATE__, __TIME__);
  CyDelay(10);
  uart_println(&usb, "");
  uart_println(&usb,"* Press 'Enter' to reset");
  uart_println(&usb, "");
  hal_rgb_set_color(&color_red);

  for(;;) {
//    uint8_t readVal = 0;
//    uint32_t readError = uart_read(&usb, &readVal);
//    if(!readError) {
//      /* Echo UART */
//      uart_write(&usb, readVal);                
//      /* Reset on Enter */
//      if('\r' == readVal) {hal_reset_device(&usb);}
//    }
  }
  #elif defined MJL_DEBUG_BTN
    /* Test the button */
    uart_printHeader(&usb, "MJL_DEBUG_BTN", __DATE__, __TIME__);
    CyDelay(10);
    uart_println(&usb, "");
    uart_println(&usb,"* Press 'Enter' to reset");
    uart_println(&usb, "");
    hal_rgb_set_color(&color_blue);

    /* Read the button state */
    bool stateBtn = false;
    bool stateBtn_prev = true;
    
    for(;;) {
      uint8_t readVal = 0;
      uint32_t readError = uart_read(&usb, &readVal);
      if(!readError) {
        /* Echo UART */
        uart_write(&usb, readVal);                
        /* Reset on Enter */
        if('\r' == readVal) {hal_reset_device(&usb);}
      }

      stateBtn = Cy_GPIO_Read(pin_BTN_0_PORT, pin_BTN_0_NUM);
      if(stateBtn != stateBtn_prev){
        stateBtn_prev = stateBtn;
        uart_printlnf(&usb, "Btn: %b", stateBtn);
        Cy_GPIO_Write(pin_LED_EXT_0_PORT, pin_LED_EXT_0_NUM, stateBtn);
      }
    }
  #elif defined MJL_DEBUG_ENCODERS
    /* Test the encoders */
    uart_printHeader(&usb, "MJL_DEBUG_ENCODERS", __DATE__, __TIME__);
    CyDelay(10);
    uart_println(&usb, "");
    uart_println(&usb,"* Press 'Enter' to reset");
    uart_println(&usb, "");
    hal_rgb_set_color(&color_blue);
        
    int32_t encoder_left = 0;
    int32_t encoder_left_prev = 0;
    int32_t encoder_right = 0;
    int32_t encoder_right_prev = 0;
    
    for(;;) {
      /* Service the UART */
      uint8_t readVal = 0;
      uint32_t readError = uart_read(&usb, &readVal);
      if(!readError) {               
        /* Reset on Enter */
        if('\r' == readVal) {hal_reset_device(&usb);}
      }
      /* Read the encoder value */
      hal_encoder_read_left(&encoder_left);
      hal_encoder_read_right(&encoder_right);      
      if((encoder_left != encoder_left_prev) || (encoder_right != encoder_right_prev)){
        encoder_left_prev = encoder_left;
        encoder_right_prev = encoder_right;
        uart_print(&usb, CMD_CLEAR_LINEUP);
        uart_printlnf(&usb, "L:%d R:%d", encoder_left, encoder_right);
      }
    }
  #elif defined MJL_DEBUG_MOTORS 
    /* Press the button to toggle motors */
      uart_printHeader(&usb, "MJL_DEBUG_MOTORS", __DATE__, __TIME__);
    CyDelay(10);
    uart_println(&usb, "");
    uart_println(&usb,"* Press 'Enter' to reset");
    uart_println(&usb,"* Press 'Space' to Cycle through the motor states");
    uart_println(&usb, "");
    hal_rgb_set_color(&color_red);
    
    #define MOTOR_STATE_DISABLED    (0)
    #define MOTOR_STATE_FORWARD     (1)
    #define MOTOR_STATE_REVERSE     (2)
    uint8_t motor_state = MOTOR_STATE_DISABLED;
    uint8_t motor_state_prev = MOTOR_STATE_FORWARD;
        
    for(;;) {
      /*************** UART ***************/
      uint8_t readVal = 0;
      uint32_t readError = uart_read(&usb, &readVal);
      if(!readError) {               
        /* Reset on Enter */
        if('\r' == readVal) {hal_reset_device(&usb);}
        /* Advance the state */
        else if (' ' == readVal){
          motor_state++;
          if(motor_state > MOTOR_STATE_REVERSE){
            motor_state = MOTOR_STATE_DISABLED; 
          }
        }
      }
      /*************** Motor ***************/
      if(motor_state != motor_state_prev){
        motor_state_prev = motor_state;
        if(MOTOR_STATE_DISABLED == motor_state){
          uart_println(&usb, "Motors Disabled"); 
          hal_motors_enable(false, false);
          hal_rgb_set_color(&color_red);
        }
        else if (MOTOR_STATE_FORWARD == motor_state){
          uart_println(&usb, "Motors Forward");
          hal_motors_enable(true, true);
          hal_motors_set_effort(1000, 1000);
          hal_rgb_set_color(&color_green);
        }
        else if (MOTOR_STATE_REVERSE == motor_state){
          uart_println(&usb, "Motors Reverse");
          hal_motors_enable(true, true);
          hal_motors_set_effort(-1000, -1000);          
          hal_rgb_set_color(&color_blue);
        }
      }
    }
  #elif defined MJL_DEBUG_COLORS 
    /* Cycle through all of the colors */
    uart_printHeader(&usb, "MJL_DEBUG_COLORS", __DATE__, __TIME__);
    CyDelay(10);
    uart_println(&usb, "");
    uart_println(&usb,"* Press 'Enter' to reset");
    uart_println(&usb,"* Press 'Space' to cycle through colors");
    uart_println(&usb, "");

    uint8_t color_index = 0;
    uint8_t color_index_prev = 1;
    
    for(;;) {
      /* Service the UART */
      uint8_t readVal = 0;
      uint32_t readError = uart_read(&usb, &readVal);
      if(!readError) {               
        /* Reset on Enter */
        if('\r' == readVal) {hal_reset_device(&usb);}
        /* Cycle through the colors */
        else if (' ' == readVal) {
          color_index++;
          if(BASE_COLORS_LEN <= color_index){
            color_index = 0; 
          }
        }
      }
      /* Handle the colors */
      if(color_index != color_index_prev){
        color_index_prev = color_index;
        const rgb_s* color = rgb_base_colors[color_index];
        hal_rgb_set_color(color);
        uart_printlnf(&usb, "Color index: %u", color_index);
      }

    }
  #elif defined MJL_DEBUG_BREATHE        
    /* Breathe the RGB LED*/
    uart_printHeader(&usb, "MJL_DEBUG_BREATHE", __DATE__, __TIME__);
    CyDelay(10);
    uart_println(&usb, "");
    uart_println(&usb,"* Press 'Enter' to reset");
    uart_println(&usb,"* Press 'c' to cycle through colors");
    uart_println(&usb,"* Press 'Space' to toggle breathing");
    uart_println(&usb, "");
    
    rgb_s color;
    uint8_t base_index = 0;
    /* Read the color stored in flash */
    hal_flash_rgb_read(&color);
    /* Set the new color */
    hal_rgb_set_color(&color);
    /* Find the index of the color */
    uint8_t match_index;
    uint32_t match_error = rgb_get_base_color_index(&color, &match_index);
    if(!match_error) {
      uart_printlnf(&usb, "Match index found: %u", match_index);    
      base_index = match_index +1;
    } else {
      uart_printlnf(&usb, "Match error: 0x%x", match_error);    
    }
    
    for(;;) {
      /* Service the UART */
      uint8_t readVal = 0;
      uint32_t readError = uart_read(&usb, &readVal);
      if(!readError) {               
        /* Reset on Enter */
        if('\r' == readVal) {hal_reset_device(&usb);}
        /* Toggle breathing */
        else if (' ' == readVal) {
          dimmer_rgb.is_active = !dimmer_rgb.is_active;
        }
        /* Cycle through the colors */
        else if ('c' == readVal) {
          uart_printlnf(&usb, "Color index: %u", base_index);
          color = *rgb_base_colors[base_index];
          /* Update the RGB */
          hal_rgb_set_color(&color);
          /* Save to flash */
          hal_flash_rgb_write(&color);
          /* Increment the index */
          base_index++;
          if(base_index >= BASE_COLORS_LEN){
            base_index = 0;
          }
        }
      }
      /* Handle the breathing in software */
      process_rgb_breathing(&dimmer_rgb);
    }
    
  #elif defined MJL_DEBUG_FLASH   
    /* Read and write to flash */
    uart_printHeader(&usb, "MJL_DEBUG_FLASH", __DATE__, __TIME__);
    uart_println(&usb, "");
    uart_println(&usb,"* Press 'enter'to reset the device");
    uart_println(&usb,"* Press 'space' to increment the control word in RAM");
    uart_println(&usb,"* Press 'w' to Write the control word to flash");
    uart_println(&usb,"* Press 'r' to Read the control word from flash");
    uart_println(&usb,"* Press 'i' to Read the robot ID from flash");
    uart_println(&usb,"* Press 's' to increment and write the robot ID from flash");
    uart_println(&usb, "");
    hal_rgb_set_color(&color_blue);
    
    
    #define FLASH_CONTROL_WORD_ADDR     (0)/* Address of the control word in EM Flash */
    #define FLASH_CONTROL_WORD_LEN      (1) /* Length of the data of the control word */
    
    uint8_t controlWord;
    uint8_t id;
    
    cy_en_em_eeprom_status_t returnVal;
    returnVal = eeprom_Read(FLASH_CONTROL_WORD_ADDR, &controlWord, FLASH_CONTROL_WORD_LEN);
    uart_printlnf(&usb, "Read returnVal: 0x%x", returnVal);
    uart_printlnf(&usb, "Control word at boot: 0x%x", controlWord);
    
    for(;;) {
      /* Handle the UART */
      uint8_t readVal = 0;
      uint32_t readError = uart_read(&usb, &readVal);
      if(!readError) {
        /* Reset on Enter */
        if('\r' == readVal) {
          hal_reset_device(&usb);
        }
        else if (' ' == readVal){
          controlWord +=1;
          uart_printlnf(&usb, "New control word: 0x%x", controlWord);
        }
        else if ('w' == readVal){
          returnVal = eeprom_Write(FLASH_CONTROL_WORD_ADDR, &controlWord, FLASH_CONTROL_WORD_LEN);
          uart_printlnf(&usb, "Write: 0x%x, returnVal: 0x%x", controlWord, returnVal);
        }
        else if ('r' == readVal){
          returnVal = eeprom_Read(FLASH_CONTROL_WORD_ADDR, &controlWord, FLASH_CONTROL_WORD_LEN);
          uart_printlnf(&usb, "Read: 0x%x, returnVal: 0x%x", controlWord, returnVal);
        }
        else if ('i' == readVal){
          uint32_t error = hal_flash_id_read(&id);
          uart_printlnf(&usb, "Read ID: 0x%x, returnVal: 0x%x", id, error);
        }
        else if ('s' == readVal) {
          uint32_t error = hal_flash_id_write(++id);
          uart_printlnf(&usb, "Writing ID: 0x%x, returnVal: 0x%x", id, error); 
        }
      }
    }

  #elif defined MJL_DEBUG_BLE   
    /* Start ble */
    uart_printHeader(&usb, "MJL_DEBUG_BLE", __DATE__, __TIME__);
    CyDelay(10);
    uart_println(&usb, "");
    uart_println(&usb,"* Press 'Enter' to reset");
    uart_println(&usb, "");
    
    /* Start the BLE component */
    Cy_BLE_Start(bleApp_eventCallback);
    /********* Set the initial states *********/
    /* LED */
    bleApp_led_update_state(false, true);
    /* RGB */
    bleApp_rgb_update_state(&color_blue, true);
    /* Motor Enable */
    bleApp_motorEnable_update_state(false, false, true);
    /* Motor Effort */
    bleApp_motorEffort_update_state(500, 500, true);
    
    int32_t encoder_left = 0;
    int32_t encoder_left_prev = 0;
    int32_t encoder_right = 0;
    int32_t encoder_right_prev = 0;
    
    for(;;) {
      /* Handle the UART */
      uint8_t readVal = 0;
      uint32_t readError = uart_read(&usb, &readVal);
      if(!readError) {              
        /* Reset on Enter */
        if('\r' == readVal) {hal_reset_device(&usb);}
      }
      /* Update the encoders */
      hal_encoder_read_left(&encoder_left);
      hal_encoder_read_right(&encoder_right);      
      if((encoder_left != encoder_left_prev) || (encoder_right != encoder_right_prev)){
        encoder_left_prev = encoder_left;
        encoder_right_prev = encoder_right;
        bleApp_encoder_update_state(encoder_left, encoder_right);
      }
      
      /* Process ble events */
      Cy_BLE_ProcessEvents();
    }
   
  #elif defined MJL_DEBUG_TIMERS 
    /* Test the timers */
        /* Basic usage the DRV driver chip */
    uart_printHeader(&usb, "MJL_DEBUG_DRV", __DATE__, __TIME__);
    CyDelay(10);
    uart_println(&usb, "");
    uart_println(&usb, "");
         
    for(;;) {
      /* Handle Print timer */
      if(flag_timer_second) {
        flag_timer_second = false;
        uart_println(&usb, "a");
      }
      /* Handle temperature error */
      uint8_t readVal = 0;
      uint32_t readError = uart_read(&usb, &readVal);
      if(!readError) {  
        /* Reset on Enter */
        if('\r' == readVal) {
          uart_print(&usb, "\r\nResetting...");
          CyDelay(1000);
          /* Reset both cores */
          NVIC_SystemReset();
        }
      }
    }
  #elif defined MJL_DEBUG_ADC          
    /* Start and read the ADC */
    uart_printHeader(&usb, "MJL_DEBUG_ADC", __DATE__, __TIME__);
    CyDelay(10);
    uart_println(&usb, "\r\n");
    
    ADC_Start();
    ADC_StartConvert();
    
    for(;;) {
        ADC_IsEndConversion(CY_SAR_WAIT_FOR_RESULT);
        uint16_t val = ADC_GetResult16(ADC_CHAN_DRV_IPROP);
        uart_printlnf(&usb, "ADC: 0x%x (%d)", val, val);
        CyDelay(1000);
    }
        
    /* End MJL_DEBUG_ADC */
    #elif defined MJL_DEBUG_ADC_INT          
      /* Read the ADC using the interrupt flag */
      uart_printHeader(&usb, "MJL_DEBUG_ADC_INT", __DATE__, __TIME__);
      CyDelay(10);
      uart_println(&usb, "\r\n");
      
      /* Start the ADC Conversions */
      Cy_SAR_StartConvert(ADC_SAR__HW, CY_SAR_START_CONVERT_CONTINUOUS);
      
      uint16_t adcVal = 0;
      for(;;) {
          /* ADC is available for servicing */
          if(flag_adc) {
              flag_adc = false;
              adcVal = ADC_GetResult16(ADC_CHAN_DRV_IPROP);
          }
      }
    /* End MJL_DEBUG_ADC */ 
  #elif defined MJL_DEBUG_ADC_AVG          
    /* Read the ADC and average */
    uart_printHeader(&usb, "MJL_DEBUG_ADC_AVG", __DATE__, __TIME__);
    CyDelay(10);
    uart_println(&usb, "\r\n");
      
    uint32_t ongoingSum = 0;
    uint16_t iteration = 0;
    const uint16_t samplesPerSecond = 939; 
    
    /* Start the ADC Conversions */
    Cy_SAR_StartConvert(ADC_SAR__HW, CY_SAR_START_CONVERT_CONTINUOUS);

    for(;;) {
      /* ADC is available for servicing */
      if(flag_adc) {
        flag_adc = false;
        uint16_t adcVal = ADC_GetResult16(ADC_CHAN_DRV_IPROP);
        ongoingSum += adcVal;
        iteration++;
        if(samplesPerSecond <=  iteration) {
          Cy_GPIO_Write(pin_DEBUG_ADC_SUM_PORT, pin_DEBUG_ADC_SUM_NUM, 1);
          uint32_t mean = ongoingSum/iteration;
          ongoingSum = 0;
          iteration = 0;
          uart_printlnf(&usb, "%d", mean);
          Cy_GPIO_Write(pin_DEBUG_ADC_SUM_PORT, pin_DEBUG_ADC_SUM_NUM, 0);
        }
      }
    } 
    /* End MJL_DEBUG_ADC_AVG */ 
  #elif defined MJL_DEBUG_SYSTEM_COUNTER      
    /* Test using the minute counter */
    uart_printHeader(&usb, "MJL_DEBUG_SYSTEM_COUNTER", __DATE__, __TIME__);
    CyDelay(10);
    uart_println(&usb, "");
    uart_println(&usb,"* Press 'Enter' to reset");
    uart_println(&usb, "");
    
    for(;;) {
      /* Triggered every second */
      if(flag_timer_second){
        flag_timer_second = false;
        get_system_time_atomic(&time);
        uart_printlnf(&usb, "sec:%d, micro:%d", time.second, time.microsecond);
      }
      /* Handle the UART */
      uint8_t readVal = 0;
      uint32_t readError = uart_read(&usb, &readVal);
      if(!readError) {
        /* Echo UART */
        //uart_write(&usb, readVal);                
        /* Reset on Enter */
        if('\r' == readVal) {
          uart_print(&usb, "\r\nResetting...");
          CyDelay(1000);
          /* Reset both cores */
          NVIC_SystemReset();
        }
      }
    }
  #endif 
  /* Fall through Infinite loop */
  for(;;){}
}
#endif /* MJL_DEBUG */


/* ####################### BEGIN FUNCTION DEFINITIONS ####################### */

/*******************************************************************************
* Function Name: initHardware()
********************************************************************************
* \brief
*   Configures the hardware to start
*
* \return
*  Error code of the operation
*******************************************************************************/
uint32_t initHardware(void) {
  uint32_t error = 0;
  /* Disable the CM4 — The CM4 Core MUST be either explicitly Enabled or Disabled to read from EEPROM */
  Cy_SysDisableCM4();
  /* Start the rgb pwm blocks */
  pwm_led_R_Start();
  pwm_led_G_Start();
  pwm_led_B_Start();
  pwm_led_alpha_Start();
  
  /* Start the Breathing timer */
  Cy_SysInt_Init(&interrupt_rgb_breathe_cfg, isr_rgb_breathe);
  NVIC_ClearPendingIRQ(interrupt_rgb_breathe_cfg.intrSrc);
  NVIC_EnableIRQ(interrupt_rgb_breathe_cfg.intrSrc);
  timer_rgb_breathe_Start();
  dimmer_rgb = dimmer_default;

  /* Start the UART */
  MJL_UART_CFG_S uartCfg = uart_cfg_default;
  uartCfg.hal_req_writeArray = uart_psoc6SCB_writeArrayBlocking;
  uartCfg.hal_req_read = uart_psoc6SCB_read;
  uartCfg.hal_opt_externalStart = uart_psoc6SCB_start;
  uartCfg.hal_opt_externalStop = uart_psoc6SCB_stop;
  error |= uart_init(&usb, &uartCfg);
  error |= uart_start(&usb);
  
  /* Start the encoders */  
  encoder_left_Start();
  encoder_right_Start();
  
  /* Start the motor PWM Block */
  pwm_left_Start();
  pwm_right_Start();    
  pwm_left_SetCompare0(1000);
  pwm_right_SetCompare0(1000);
  
  /* Initialize EEPROM */
  /* Must update the CM4 linker script (cy8c6xx7_cm4_dual.ld) and remove the '.cy_em_eeprom' entry to 
    allow the CM0+ core exclusive access to the eeprom storage */
  error |= eeprom_Init((uint32_t)eeprom_em_EepromStorage); /* Argument has no effect on PSoC6 when using Emulated EEPROM */
   
//  /*Connect the microsecond timer to the second interrupt */
//  Cy_SysInt_Init(&interrupt_second_cfg, isr_second);
//  NVIC_ClearPendingIRQ(interrupt_second_cfg.intrSrc);
//  NVIC_EnableIRQ(interrupt_second_cfg.intrSrc);
//  counter_microsecond_Start();
//  counter_second_Start();
//  
//  /* Enable the ADC  */
//  Cy_SysInt_Init(&interrupt_adc_cfg, isr_adc);
//  NVIC_ClearPendingIRQ(interrupt_adc_cfg.intrSrc);
//  NVIC_EnableIRQ(interrupt_adc_cfg.intrSrc);
//  ADC_Start();
//  NVIC_EnableIRQ(ADC_IRQ_cfg.intrSrc);
  
  /* Start global interrupts */
  __enable_irq(); /* Enable global interrupts. */
  
  return error;
}


/*******************************************************************************
* Function Name: process_rgb_breathing()
********************************************************************************
* \brief
*   Handles software-based RGB breathing effect by adjusting the alpha value 
*   over time. Reverses direction when min/max bounds are reached.
*
* \param state [in,out]
*   Pointer to the alpha dimmer state structure containing breathing parameters.
*
* \return
*   Bitmask error code (0 for success; non-zero indicates failure).
*******************************************************************************/  
uint32_t process_rgb_breathing(alpha_dimmer_s* state) { 
  uint32_t error = 0;
  if(state==NULL){error|=1;}
  
  if(!error){
    /* Handle the updated breathing state */
    if(state->is_active != state->is_active_prev){
      state->is_active_prev = state->is_active;
      uart_printlnf(&usb, "Breathing: %b", state->is_active);
      /* Full alpha when not breathing */
      if(!state->is_active) {hal_rgb_set_alpha(ALPHA_MAX);}
    }
    /* Handle the breathing in software */
    if(state->is_active && state->flag_timer_breathe){
      state->flag_timer_breathe = false;
      /* Update the current alpha value */
      hal_rgb_set_alpha(state->val);
      /* Step alpha forward */
      state->val += state->delta;
      /* Clip to range */
      if(state->val >= ALPHA_MAX){
        state->val = ALPHA_MAX;
        state->delta *= -1;
      }
      else if (state->val <= 0) {
        state->val = 0;
        state->delta *= -1;
      }
    }
  }
  return error;
}

/*******************************************************************************
* Function Name: isr_rgb_breathe()
********************************************************************************
* \brief
*   Breath the RGB by updating the led
*
*******************************************************************************/
void isr_rgb_breathe(void){
  dimmer_rgb.flag_timer_breathe = true;
  timer_rgb_breathe_ClearInterrupt(CY_TCPWM_INT_ON_TC);
  NVIC_ClearPendingIRQ(interrupt_rgb_breathe_cfg.intrSrc);
}

///*******************************************************************************
//* Function Name: isr_second()
//********************************************************************************
//* \brief
//*   Triggers after 1 second
//*
//*******************************************************************************/
//void isr_second(void){
//  flag_timer_second = true;
//  counter_microsecond_ClearInterrupt(CY_TCPWM_INT_ON_TC);
//  NVIC_ClearPendingIRQ(interrupt_second_cfg.intrSrc);
//}


///*******************************************************************************
//* Function Name: isr_adc()
//********************************************************************************
//* \brief
//*   Triggers when the ADC End of Scan is complete (EOS)
//*
//*******************************************************************************/
//void isr_adc(void) {
//  flag_adc = true;
//  NVIC_ClearPendingIRQ(interrupt_adc_cfg.intrSrc);
//}

///*******************************************************************************
//* Function Name: get_system_time_atomic()
//********************************************************************************
//* \brief
//*   Atomically updates the system time state structure passed in 
//*
//*******************************************************************************/
//uint32_t get_system_time_atomic(system_time_s *const state){
// uint32_t error = 0;
//  /* Get atomically */
//  uint8_t intr_state = Cy_SysLib_EnterCriticalSection();
//  state->microsecond = Cy_TCPWM_Counter_GetCounter(counter_microsecond_HW, counter_microsecond_CNT_NUM);
//  state->second = Cy_TCPWM_Counter_GetCounter(counter_second_HW, counter_second_CNT_NUM);
//  Cy_SysLib_ExitCriticalSection(intr_state);
//  return error;
//}




/* [] END OF FILE */
