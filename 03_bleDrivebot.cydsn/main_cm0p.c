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

#define MJL_DEBUG
/* ---------------- DEBUG CASE ----------------
* Uncomment ONE of the following
* Debugging will only occur when MJL_DEBUG is defined
*/
#ifdef MJL_DEBUG
//  #define MJL_DEBUG_LED            /* Test the battery charger LED */
//  #define MJL_DEBUG_UART            /* Test the UART Connection */
//  #define MJL_DEBUG_BTN /* Test the button */
//  #define MJL_DEBUG_ENCODERS /* Test the encoders */
//  #define MJL_DEBUG_MOTORS /* Press the button to toggle motors */
  #define MJL_DEBUG_BLE   /* Test the basic ble application */
//  #define MJL_DEBUG_PWM /*  Test PWM  */
//  #define MJL_DEBUG_TIMERS /* Test the timers */
//  #define MJL_DEBUG_ADC               /* Read the ADC */
//  #define MJL_DEBUG_ADC_INT          /* Read the ADC using the interrupt flag */
//  #define MJL_DEBUG_ADC_AVG          /* Read the ADC and average in software */
//  #define MJL_DEBUG_SYSTEM_COUNTER      /* Test using the microsecond counter */
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
volatile bool flag_timer_second;
volatile bool flag_adc;
MLJ_UART_S usb;
MJL_SPI_S spi;
max31856_state_s tempSense;
drv8244_state_s drv;
system_time_s time;

/* ISR Declarations */
void isr_adc(void);
void isr_second(void);

/* Function Definitions */
uint32_t initHardware(void);
void breatheLed_setState(bool state);

/* Main Program */
#ifndef MJL_DEBUG
/*******************************************************************************
* Function Name: main()
********************************************************************************
* Summary:
*   The top-level application function for the project.
*
*******************************************************************************/
int main(void) {

  initHardware();
  uart_printHeader(&usb, "MJL_TEMPLATE_MAIN", __DATE__, __TIME__);
  CyDelay(10);
  uart_println(&usb, "");
  uart_println(&usb,"* Press 'Enter' to reset");
  /* Enable CM4.  CY_CORTEX_M4_APPL_ADDR must be updated if CM4 memory layout is changed. */
  //  Cy_SysEnableCM4(CY_CORTEX_M4_APPL_ADDR); 
  
  for(;;){
    /* Handle the UART */
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
    uint8_t brightness = 25;
    for(;;) {
        hal_rgb_set_duty(brightness, 0, 0);
        CyDelay(500);
        hal_rgb_set_duty(0, brightness, 0);
        CyDelay(500);
        hal_rgb_set_duty(0, 0, brightness);
        CyDelay(500);   
        /* Increase the brightness */
        brightness += 25;
        if(brightness > 100){
            brightness = 25;
        }
    }
  /* End MJL_DEBUG_LED */
  #elif defined MJL_DEBUG_UART             
  /* Test the UART Connection */
  uart_printHeader(&usb, "MJL_DEBUG_UART", __DATE__, __TIME__);
  CyDelay(10);
  uart_println(&usb, "");
  uart_println(&usb,"* Press 'Enter' to reset");
  uart_println(&usb, "");
  hal_rgb_set_duty(0, 0, 50);

  for(;;) {
    uint8_t readVal = 0;
    uint32_t readError = uart_read(&usb, &readVal);
    if(!readError) {
      /* Echo UART */
      uart_write(&usb, readVal);                
      /* Reset on Enter */
      if('\r' == readVal) {
        uart_print(&usb, "\r\nResetting...");
        CyDelay(1000);
        /* Reset both cores */
        NVIC_SystemReset();
      }
    }
  }
  #elif defined MJL_DEBUG_BTN
    /* Test the button */
    uart_printHeader(&usb, "MJL_DEBUG_BTN", __DATE__, __TIME__);
    CyDelay(10);
    uart_println(&usb, "");
    uart_println(&usb,"* Press 'Enter' to reset");
    uart_println(&usb, "");
    hal_rgb_set_duty(0, 0, 50);

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
        if('\r' == readVal) {
          uart_print(&usb, "\r\nResetting...");
          CyDelay(1000);
          /* Reset both cores */
          NVIC_SystemReset();
        }
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
    hal_rgb_set_duty(0, 0, 50);
        
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
        if('\r' == readVal) {
          uart_print(&usb, "\r\nResetting...");
          CyDelay(1000);
          /* Reset both cores */
          NVIC_SystemReset();
        }
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
    hal_rgb_set_duty(50,0,0);
    
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
        if('\r' == readVal) {
          uart_print(&usb, "\r\nResetting...");
          CyDelay(1000);
          /* Reset both cores */
          NVIC_SystemReset();
        }
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
          hal_rgb_set_duty(50,0,0);
        }
        else if (MOTOR_STATE_FORWARD == motor_state){
          uart_println(&usb, "Motors Forward");
          hal_motors_enable(true, true);
          hal_motors_set_effort(1000, 1000);
          hal_rgb_set_duty(0,50,0);
        }
        else if (MOTOR_STATE_REVERSE == motor_state){
          uart_println(&usb, "Motors Reverse");
          hal_motors_enable(true, true);
          hal_motors_set_effort(-1000, -1000);          
          hal_rgb_set_duty(0,0,50);
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
    bleApp_rgb_update_state(0, 50, 0, true);
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
        /* Echo UART */
        uart_write(&usb, readVal);                
        /* Reset on Enter */
        if('\r' == readVal) {
          uart_print(&usb, "\r\nResetting...");
          CyDelay(1000);
          /* Reset both cores */
          NVIC_SystemReset();
        }
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
    

  #elif defined MJL_DEBUG_PWM           
    /* Basic usage the DRV driver chip */
    uart_printHeader(&usb, "MJL_DEBUG_DRV", __DATE__, __TIME__);
    CyDelay(10);
    uart_println(&usb, "");
    uart_println(&usb, "* Press ' ' to toggle PWM");
    uart_println(&usb, "");
          
    bool pwmState = false;
    bool prevPwmState = true;
        
    for(;;) {
      /* Check for UART input */
      uint8_t readVal = 0;
      uint32_t readError = uart_read(&usb, &readVal);
      if(readVal == ' ') {
          pwmState = !pwmState;
      }
      /* Detect State Change */
      if(pwmState != prevPwmState) {
          prevPwmState = pwmState;
          uart_println(&usb, CMD_CLEAR_LINEUP);
          if(pwmState) {
              PWM_DRV_Enable();
              Cy_TCPWM_TriggerStart(PWM_DRV_HW, PWM_DRV_CNT_MASK);
              breatheLed_setState(true);
          }
          else {
              PWM_DRV_Disable();
              breatheLed_setState(false);
          }
          uart_printlnf(&usb, "PWM Enabled: %b", pwmState);
        }
    }
  /* End MJL_DEBUG_PWM */
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
    
  /* Start the RGB led */
  pwm_led_R_Start();
  pwm_led_G_Start();
  pwm_led_B_Start();

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
* Function Name: isr_second()
********************************************************************************
* \brief
*   Triggers after 1 second
*
*******************************************************************************/
void isr_second(void){
  flag_timer_second = true;
  counter_microsecond_ClearInterrupt(CY_TCPWM_INT_ON_TC);
  NVIC_ClearPendingIRQ(interrupt_second_cfg.intrSrc);
}


/*******************************************************************************
* Function Name: isr_adc()
********************************************************************************
* \brief
*   Triggers when the ADC End of Scan is complete (EOS)
*
*******************************************************************************/
void isr_adc(void) {
  flag_adc = true;
  NVIC_ClearPendingIRQ(interrupt_adc_cfg.intrSrc);
}

/*******************************************************************************
* Function Name: get_system_time_atomic()
********************************************************************************
* \brief
*   Atomically updates the system time state structure passed in 
*
*******************************************************************************/
uint32_t get_system_time_atomic(system_time_s *const state){
 uint32_t error = 0;
  /* Get atomically */
  uint8_t intr_state = Cy_SysLib_EnterCriticalSection();
  state->microsecond = Cy_TCPWM_Counter_GetCounter(counter_microsecond_HW, counter_microsecond_CNT_NUM);
  state->second = Cy_TCPWM_Counter_GetCounter(counter_second_HW, counter_second_CNT_NUM);
  Cy_SysLib_ExitCriticalSection(intr_state);
  return error;
}




/* [] END OF FILE */
