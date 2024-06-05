/***************************************************************************
*                           Majestic Labs  © 2024
* File: main.c
* Workspace: Drivebot_v5
* Version: v1.0
* Author: Craig Cheney
*
* PCB: mcuPSoC4
*
* Brief:
*   Tests for the Drivebot microcontroller 
*
* Change Log:
*   2024.05.05 - Document created
********************************************************************************/
#include "project.h" /* Cypress project */
#include "mjl_errors.h"
#include "mjl_uart.h"
#include "hal_psoc4.h"
#include "drivebot.h"

/* ####################### BEGIN PROGRAM CONFIGURATION ###################### */

//#define MJL_DEBUG
/* ---------------- DEBUG CASE ----------------
* Uncomment ONE of the following
* Debugging will only occur when MJL_DEBUG is defined
*/
#ifdef MJL_DEBUG
//    #define MJL_DEBUG_LED            /* Test the battery charger LED */
//    #define MJL_DEBUG_UART            /* Test the UART Connection */
//    #define MJL_DEBUG_BTN_POLL           /* Polling the button status */
//    #define MJL_DEBUG_BTN_INTERRUPT        /* Act on the button status from an interrupt */
//    #define MJL_DEBUG_FLASH             /* Read and write to Flash */
//    #define MJL_DEBUG_ENCODER           /* Read the encoders */
//    #define MJL_DEBUG_PWM                   /* Test the PWM */
//    #define MJL_DEBUG_MOTORS            /* Test the motors */
#endif
/* -------------- END DEBUG CASE --------------  */
   


/* ############################# BEGIN PROGRAM ############################## */
/* Local functions */
uint32_t initHardware(void);
float32 encoder_calculate_position(uint16_t current_count, int32_t overflows);
void hal_motor_left_enable(bool will_enable);
void hal_motor_right_enable(bool will_enable);
void hal_motor_left_dir(mjl_motor_dir_t dir);
void hal_motor_right_dir(mjl_motor_dir_t dir);


/* Interrupt service routines */
void isr_button(void);
void isr_encoder_left(void);
void isr_encoder_right(void);

/* Macro Definitions */
#define BUTTON_PRESSED     (1) /* Value of the button pin when the button is pressed */
#define BUTTON_RELEASED    (0) /* Value of the button pin when the button is released */
#define LED_ON             (0) /* Value of leds being ON */
#define LED_OFF            (1) /* Value of leds being ON */
#define ENCODER_MIDPOINT  (0x8000)  /* Reset value of the encoders */
#define ENCODER_MAX       (0xFFFF)  /* Maximum value of the encoder */
#define ENCODER_CAPTURE_OVERFLOW    (0xFFFF) /* Value of the capture register after OVERflow has occurred */
#define ENCODER_CAPTURE_UNDERFLOW   (0x0000) /* Value of the capture register after UNDERflow has occurred */
#define PWM_MAX             (999)   /* maximum value of the PWM */

/* Global Variables */
MLJ_UART_S usb;
volatile bool flag_button_pressed = false;
volatile bool flag_button_released = false;
drivebot_state_s state;

/* Flash EEPROM */
const uint8_t flash_eeprom[flash_PHYSICAL_SIZE]__ALIGNED(CY_FLASH_SIZEOF_ROW) = {0u};
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

    for(;;){
        uart_printHeader(&usb, "MJL_DRIVEBOT_DEV", __DATE__, __TIME__);
        uart_println(&usb, "Press 'Enter' to reset device");
        uart_println(&usb, "Press the button to start/stop driving");
        uart_println(&usb, "");

        int16_t speed = 0;
        bool is_drivebot_driving = false;
        bool is_drivebot_driving_prev = false;
        for(;;) {
            /* Handle the uart */
            uint8_t readVal = 0;
            uint32_t readError = uart_read(&usb, &readVal);
            if(!readError) {          
                /* Reset on Enter */
                if('\r' == readVal) {
                    uart_print(&usb, "\r\nResetting...");
                    CyDelay(1000);
                    CySoftwareReset();  
                }
            }
            /* Wait for button press */
            if(flag_button_pressed){
                flag_button_pressed = false;
                is_drivebot_driving = !is_drivebot_driving;
            }
            
            /* React to button state changes */
            if(is_drivebot_driving != is_drivebot_driving_prev){
                is_drivebot_driving_prev = is_drivebot_driving;
                speed = is_drivebot_driving ? PWM_MAX : 0;
                drivebot_set_duty(&state, speed, speed);
                drivebot_set_enable(&state, is_drivebot_driving, is_drivebot_driving);
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
        /* blink the LED */
        #define DELAY 500
        for(;;) {
            led_control_reg_Write(0b001);
            CyDelay(DELAY);
            led_control_reg_Write(0b010);
            CyDelay(DELAY);
            led_control_reg_Write(0b100);
            CyDelay(DELAY);
            led_control_reg_Write(0b011);
            CyDelay(DELAY);
            led_control_reg_Write(0b101);
            CyDelay(DELAY);
            led_control_reg_Write(0b110);
            CyDelay(DELAY);
            led_control_reg_Write(0b111);
            CyDelay(DELAY);
            led_control_reg_Write(0b000);
            CyDelay(DELAY);
        }
    /* End MJL_DEBUG_LED */
    #elif defined MJL_DEBUG_UART           
        /* Test the UART Connection */
        uart_printHeader(&usb, "MJL_DEBUG_UART", __DATE__, __TIME__);
        uart_println(&usb, "Press 'Enter' to reset device");
        uart_println(&usb, "");
        
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
                    CySoftwareReset();  
                }
            }
        }
    #elif defined MJL_DEBUG_BTN_POLL
     /* Polling the button status */   
        uart_printHeader(&usb, "MJL_DEBUG_BTN_POLL", __DATE__, __TIME__);
        uart_println(&usb, "Press 'Enter' to reset device");
        uart_println(&usb, "");
        led_control_reg_Write(0b001);
        
        bool was_button_pressed = false;
        bool was_button_pressed_prev = false;
        for(;;) {
            uint8_t readVal = 0;
            uint32_t readError = uart_read(&usb, &readVal);
            if(!readError) {               
                /* Reset on Enter */
                if('\r' == readVal) {
                    uart_print(&usb, "\r\nResetting...");
                    CyDelay(1000);
                    CySoftwareReset();  
                }
            }
            /* Check the button status */
            was_button_pressed = pin_BTN_Read() == BUTTON_PRESSED ? true : false;
            if(was_button_pressed != was_button_pressed_prev){
                was_button_pressed_prev = was_button_pressed;
                if(was_button_pressed) {
                    uart_print(&usb, "Pressed..");
                    led_control_reg_Write(0b010);
                } else{
                    uart_println(&usb, "Released");
                    led_control_reg_Write(0b100);
                }
            }
        }
    #elif defined MJL_DEBUG_BTN_INTERRUPT
        /* Act on the button status from an interrupt */
        uart_printHeader(&usb, "MJL_DEBUG_BTN_INTERRUPT", __DATE__, __TIME__);
        uart_println(&usb, "Press 'Enter' to reset device");
        uart_println(&usb, "Press the user button");
        
        uart_println(&usb, "");


        for(;;) {
            uint8_t readVal = 0;
            uint32_t readError = uart_read(&usb, &readVal);
            if(!readError) {               
                /* Reset on Enter */
                if('\r' == readVal) {
                    uart_print(&usb, "\r\nResetting...");
                    CyDelay(1000);
                    CySoftwareReset();  
                }
            }
            /* Check the button status */
            if(flag_button_pressed) {
                flag_button_pressed = false;
                uart_print(&usb, "Pressed..");
                led_control_reg_Write(0b010);
            } 
            if(flag_button_released){
                flag_button_released = false;
                uart_println(&usb, "Released");
                led_control_reg_Write(0b100);
            }
        }
    #elif defined MJL_DEBUG_FLASH             
        /* Read and write to Flash */
        uart_printHeader(&usb, "MJL_DEBUG_FLASH", __DATE__, __TIME__);
        uart_println(&usb, "");
        uart_println(&usb,"* Press 'enter'to reset the device");
        uart_println(&usb,"* Press 'space' to increment the control word in RAM");
        uart_println(&usb,"* Press 'w' to Write the control word to flash");
        uart_println(&usb,"* Press 'r' to Read the control word from flash");
        uart_println(&usb, "");
        #define FLASH_CONTROL_WORD_ADDR     (0)/* Address of the control word in EM Flash */
        #define FLASH_CONTROL_WORD_LEN      (1) /* Length of the data of the control word */
        
        uint8_t controlWord;
        cy_en_em_eeprom_status_t returnVal;
        returnVal = flash_Read(FLASH_CONTROL_WORD_ADDR, &controlWord, FLASH_CONTROL_WORD_LEN);
        uart_printlnf(&usb, "Read returnVal: 0x%x", returnVal);
        uart_printlnf(&usb, "Control word at boot: 0x%x", controlWord);
        
        for(;;) {
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
                else if (' ' == readVal){
                  controlWord +=1;
                  uart_printlnf(&usb, "New control word: 0x%x", controlWord);
                }
                else if ('w' == readVal){
                  returnVal = flash_Write(FLASH_CONTROL_WORD_ADDR, &controlWord, FLASH_CONTROL_WORD_LEN);
                  uart_printlnf(&usb, "Write: 0x%x, returnVal: 0x%x", controlWord, returnVal);
                }
                else if ('r' == readVal){
                  returnVal = flash_Read(FLASH_CONTROL_WORD_ADDR, &controlWord, FLASH_CONTROL_WORD_LEN);
                  uart_printlnf(&usb, "Read: 0x%x, returnVal: 0x%x", controlWord, returnVal);
                }
              }
        }
    #elif defined MJL_DEBUG_ENCODER           
        /* Polling the button status */   
        uart_printHeader(&usb, "MJL_DEBUG_ENCODER", __DATE__, __TIME__);
        uart_println(&usb, "Press 'Enter' to reset device");        
        uart_println(&usb, "\n");
        
        for(;;) {
            uint8_t readVal = 0;
            uint32_t readError = uart_read(&usb, &readVal);
            if(!readError) {               
                /* Reset on Enter */
                if('\r' == readVal) {
                    uart_print(&usb, "\r\nResetting...");
                    CyDelay(1000);
                    CySoftwareReset();  
                }
            }
            /* Process the encoders */
            drivebot_read_encoders(&state);

            /* Display positions */
            uart_print(&usb, CMD_CLEAR_LINEUP);
            uart_printlnf(&usb, "PosL:%d, PosR:%d", state.left.encoder.position, state.right.encoder.position);
            CyDelay(50);
        }
    #elif defined MJL_DEBUG_PWM                   
        /* Test the PWM */
        uart_printHeader(&usb, "MJL_DEBUG_PWM", __DATE__, __TIME__);
        uart_println(&usb, "Press 'Enter' to reset device");
        uart_println(&usb, "Press '0' to set the PWM to zero");
        uart_println(&usb, "Press '1' to set the PWM to half");
        uart_println(&usb, "Press '2' to set the PWM to full");
        uart_println(&usb, "Press 'space' to toggle the direction");
        uart_println(&usb, "Press 'e' to set the toggle the enable");
        uart_println(&usb, "");
        
        for(;;) {
            uint8_t readVal = 0;
            uint32_t readError = uart_read(&usb, &readVal);
            if(!readError) {             
                /* Reset on Enter */
                if('\r' == readVal) {
                    uart_print(&usb, "\r\nResetting...");
                    CyDelay(1000);
                    CySoftwareReset();  
                }
                else if ('0' == readVal){
                    int16_t pwm_control = 0;
                    uart_printlnf(&usb, "* PWM Set to %d", 0);
                    drivebot_set_duty(&state, pwm_control, pwm_control);
                }
                else if ('1' == readVal){
                    int16_t pwm_control = 499;
                    uart_printlnf(&usb, "* PWM Set to %d", pwm_control);
                    drivebot_set_duty(&state, pwm_control, pwm_control);
                }
                else if ('2' == readVal){
                    int16_t pwm_control = PWM_MAX;
                    uart_printlnf(&usb, "* PWM Set to %d", pwm_control);
                    drivebot_set_duty(&state, pwm_control, pwm_control);
                }
                else if (' ' ==readVal){
                    int16_t pwm_control_l = -1 * state.left.motor.duty;
                    int16_t pwm_control_r = -1 * state.right.motor.duty;
                    drivebot_set_duty(&state, pwm_control_l, pwm_control_r);
                    uart_printlnf(&usb, "* PWM Set to %d, %d", pwm_control_l, pwm_control_r);
                }
                else if ('e' == readVal){
                    bool enable_l = !state.left.motor.is_enabled;
                    bool enable_r = !state.right.motor.is_enabled;
                    drivebot_set_enable(&state, enable_l, enable_r);
                    uart_printlnf(&usb, "* Enable left:%b, right:%b", enable_l, enable_r);
                }
            }
        }
    #elif defined MJL_DEBUG_MOTORS            
        /* Test the motors */
        uart_printHeader(&usb, "MJL_DEBUG_MOTORS", __DATE__, __TIME__);
        uart_println(&usb, "Press 'Enter' to reset device");
        uart_println(&usb, "Press '0' to enable/disable motors");
        uart_println(&usb, "Press '1' to set Left  motor to FORWARD");
        uart_println(&usb, "Press '2' to set Left  motor to OFF");
        uart_println(&usb, "Press '3' to set Left  motor to REVERSE");
        uart_println(&usb, "Press '4' to set Right motor to FORWARD");
        uart_println(&usb, "Press '5' to set Right motor to OFF");
        uart_println(&usb, "Press '6' to set Right motor to REVERSE");
        uart_println(&usb, "");

        for(;;) {
            uint8_t readVal = 0;
            uint32_t readError = uart_read(&usb, &readVal);
            if(!readError) {               
                /* Reset on Enter */
                if('\r' == readVal) {
                    uart_print(&usb, "\r\nResetting...");
                    CyDelay(1000);
                    CySoftwareReset();  
                }
                /* Enable/Disable motors */
                else if ('0'== readVal) {
                    bool are_motors_enabled = state.motor_left.is_enabled || state.motor_right.is_enabled;
                    bool will_motors_enable = !are_motors_enabled;
                    drivebot_motors_enable(&state, will_motors_enable, will_motors_enable);
                    uart_printlnf(&usb, " * Motors enabled? %b", will_motors_enable);
                }
                /* Left Motor Forward */
                else if ('1'== readVal) {
                    drivebot_motors_enable_left(&state, true);
                    // TODO: Pickup here
//                    drivebot_motor_set_velocity(&state.motor_left, -400);
                    uart_printlnf(&usb, " * L Motor Forward");
                }
                /* Left Motor Disable */
                else if ('2'== readVal) {
                    drivebot_motors_enable_left(&state, false);
                    uart_printlnf(&usb, " * L Motor disabled");
                    
                }
                /* Left Motor Reverse */
                else if ('3'== readVal) {
                }
                
                /* Right Motor Forward */
                else if ('4'== readVal) {
                    drivebot_motors_enable_right(&state, true);
                    uart_printlnf(&usb, " * R Motor Forward");
                }
                /* Left Motor Disable */
                else if ('5'== readVal) {
                    drivebot_motors_enable_right(&state, false);
                    uart_printlnf(&usb, " * R Motor disabled");
                    
                }
                
            }
        }
    #endif 
    /* Fall through Infinite loop - Should never reach here */
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
    /* Start the UART */
    MJL_UART_CFG_S uartCfg = uart_cfg_default;
    uartCfg.hal_req_writeArray = uart_psoc4SCB_writeArrayBlocking;
    uartCfg.hal_req_read = uart_psoc4SCB_read;
    uartCfg.hal_opt_externalStart = uart_psoc4SCB_start;
    uartCfg.hal_opt_externalStop = uart_psoc4SCB_stop;
    error |= uart_init(&usb, &uartCfg);
    error |= uart_start(&usb);
    
    /* Register the Button ISR with the interrupt */
    interrupt_btn_StartEx(isr_button);
    
    /* Start the encoders */
    encoder_left_Start();
    interrupt_encoder_left_StartEx(isr_encoder_left);
    encoder_right_Start();
    interrupt_encoder_right_StartEx(isr_encoder_right);
    
    /* Configure the state drivebot structure */
    /* Encoders */
    mjl_encoder_cfg_s encoder_l_cfg = mjl_encoder_cfg_default;
    encoder_l_cfg.midpoint = ENCODER_MIDPOINT;
    encoder_l_cfg.fn_get_encoder_count = (uint32_t(*)(void)) encoder_left_ReadCounter;
    mjl_encoder_cfg_s encoder_r_cfg = mjl_encoder_cfg_default;
    encoder_r_cfg.midpoint = ENCODER_MIDPOINT;
    encoder_r_cfg.fn_get_encoder_count = (uint32_t(*)(void)) encoder_right_ReadCounter;
    /* Motors */
    mjl_motor_cfg_s motor_l_cfg = motor_cfg_default;
    motor_l_cfg.fn_motor_duty_write = (void(*)(uint32_t)) pwm_left_WriteCompare;
    motor_l_cfg.fn_motor_enable_write = hal_motor_left_enable;
    motor_l_cfg.fn_motor_dir_write = hal_motor_left_dir;
    mjl_motor_cfg_s motor_r_cfg = motor_cfg_default;
    motor_r_cfg.fn_motor_duty_write = (void(*)(uint32_t)) pwm_right_WriteCompare;
    motor_r_cfg.fn_motor_enable_write = hal_motor_right_enable;
    motor_r_cfg.fn_motor_dir_write = hal_motor_right_dir;

    
    /* Start the drivebot function */
    drivebot_cfg_s db_cfg = drivebot_cfg_default;
    db_cfg.encoder_l_cfg = &encoder_l_cfg;
    db_cfg.encoder_r_cfg = &encoder_r_cfg;
    db_cfg.motor_l_cfg = &motor_l_cfg;
    db_cfg.motor_r_cfg = &motor_r_cfg;
    error |= drivebot_init(&state, &db_cfg);
    error |= drivebot_start(&state);

    /* Start the PWM */
    pwm_left_Start();
    pwm_right_Start();
    pwm_left_WriteCompare(0);
    pwm_right_WriteCompare(0);
    
    /* Set the inital rgb state */
    led_control_reg_Write(0b100);
    
    /* Enable global interrutps */
    CyGlobalIntEnable;
    
    /* Initialize EEPROM */
    error |= flash_Init((uint32_t)flash_eeprom); 
 
    return error;
}

/*******************************************************************************
* Function Name: isr_button()
********************************************************************************
* \brief
*   Interrupt service routine for when the button is pressed/relased
*
* \return
*  None
*******************************************************************************/
void isr_button(void){
    pin_BTN_ClearInterrupt();
    bool btn_state = pin_BTN_Read();
    if(btn_state == BUTTON_PRESSED){
        flag_button_pressed = true;
    } else {
        flag_button_released = true;
    }
}

/*******************************************************************************
* Function Name: isr_encoder_left()
********************************************************************************
* \brief
*   Interrupt service routine for when the left encoder under/over flows. Check
*   the capture register to determine which event occured. 
*
* \return
*  None
*******************************************************************************/
void isr_encoder_left(void){
    encoder_left_ClearInterrupt(encoder_left_INTR_MASK_CC_MATCH);
    /* Read the capture register to determine the direction of the over/underflow */
    uint16_t capture = (uint16_t) encoder_left_ReadCapture();
    if(ENCODER_CAPTURE_OVERFLOW == capture){
        state.left.encoder._overflows++;
    }
    else if(ENCODER_CAPTURE_UNDERFLOW == capture){
        state.left.encoder._overflows--;
    }
}

/*******************************************************************************
* Function Name: isr_encoder_right()
********************************************************************************
* \brief
*   Interrupt service routine for when the right encoder hits its terminal count
*
* \return
*  None
*******************************************************************************/
void isr_encoder_right(void){
    encoder_right_ClearInterrupt(encoder_right_INTR_MASK_CC_MATCH);
    /* Read the capture register to determine the direction of the over/underflow */
    uint16_t capture = (uint16_t) encoder_right_ReadCapture();
    if(ENCODER_CAPTURE_OVERFLOW == capture){
        state.right.encoder._overflows++;
        
    }
    else if(ENCODER_CAPTURE_UNDERFLOW == capture){
        state.right.encoder._overflows--;
        
    }
}


/*******************************************************************************
* Function Name: hal_motor_left_enable()
********************************************************************************
* \brief
*   Set the enable state of the left motor
*
* \param will_enable [in]
* The state to immediatley set the left motor
*
* \return
*  None
*******************************************************************************/
void hal_motor_left_enable(bool will_enable){
    /* Check the current direction pin status from the hardware*/
    uint8_t current_word =  motor_control_reg_Read();
    uint8_t current_dir = DRIVEBOT_CTR_REG_MASK_DIR & current_word;
    uint8_t current_en = DRIVEBOT_CTR_REG_MASK_EN & current_word;
    /* Pack the new enable word */
    uint8_t new_en = (will_enable << DRIVEBOT_CTR_REG_SHIFT_EN_L) | (current_en & DRIVEBOT_CTR_REG_MASK_EN_R);
    /* Pack the full word*/
    uint8_t new_word = (current_dir | new_en) & DRIVEBOT_CTR_REG_MASK;
    /* Write out new word */
    motor_control_reg_Write(new_word);
}

/*******************************************************************************
* Function Name: hal_motor_right_enable()
********************************************************************************
* \brief
*   Set the enable state of the right motor
*
* \param will_enable [in]
* The state to immediatley set the right motor
*
* \return
*  None
*******************************************************************************/
void hal_motor_right_enable(bool will_enable){
    /* Check the current direction pin status from the hardware*/
    uint8_t current_word =  motor_control_reg_Read();
    uint8_t current_dir = DRIVEBOT_CTR_REG_MASK_DIR & current_word;
    uint8_t current_en = DRIVEBOT_CTR_REG_MASK_EN & current_word;
    /* Pack the new enable word */
    uint8_t new_en = (will_enable << DRIVEBOT_CTR_REG_SHIFT_EN_R) | (current_en & DRIVEBOT_CTR_REG_MASK_EN_L);
    /* Pack the full word*/
    uint8_t new_word = (current_dir | new_en) & DRIVEBOT_CTR_REG_MASK;
    /* Write out new word */
    motor_control_reg_Write(new_word);
}

/*******************************************************************************
* Function Name: hal_motor_left_dir()
********************************************************************************
* \brief
*   Set the direction of the left motor 
*
* \param dir [in]
* The direction to set 
*
* \return
*  None
*******************************************************************************/
void hal_motor_left_dir(mjl_motor_dir_t dir){
    /* Check the current direction pin status from the hardware*/
    uint8_t current_word =  motor_control_reg_Read();
    uint8_t current_dir = DRIVEBOT_CTR_REG_MASK_DIR & current_word;
    uint8_t current_en = DRIVEBOT_CTR_REG_MASK_EN & current_word;
    /* Pack the new enable word */
    uint8_t new_dir = (dir << DRIVEBOT_CTR_REG_SHIFT_DIR_L) | (current_dir & DRIVEBOT_CTR_REG_MASK_DIR_R);
    /* Pack the full word*/
    uint8_t new_word = (new_dir | current_en) & DRIVEBOT_CTR_REG_MASK;
    /* Write out new word */
    motor_control_reg_Write(new_word);
}

/*******************************************************************************
* Function Name: hal_motor_right_dir()
********************************************************************************
* \brief
*   Set the direction of the right motor 
*
* \param dir [in]
* The direction to set 
*
* \return
*  None
*******************************************************************************/
void hal_motor_right_dir(mjl_motor_dir_t dir){
    /* Check the current direction pin status from the hardware*/
    uint8_t current_word =  motor_control_reg_Read();
    uint8_t current_dir = DRIVEBOT_CTR_REG_MASK_DIR & current_word;
    uint8_t current_en = DRIVEBOT_CTR_REG_MASK_EN & current_word;
    /* Pack the new enable word */
    uint8_t new_dir = (dir << DRIVEBOT_CTR_REG_SHIFT_DIR_R) | (current_dir & DRIVEBOT_CTR_REG_MASK_DIR_L);
    /* Pack the full word*/
    uint8_t new_word = (new_dir | current_en) & DRIVEBOT_CTR_REG_MASK;
    /* Write out new word */
    motor_control_reg_Write(new_word);
}

/* [] END OF FILE */