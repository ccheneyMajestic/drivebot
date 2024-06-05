
/***************************************************************************
*                                Majestic Labs © 2023
* File: hal_psoc4.c
* Workspace: MJL Hardware Abstraction Layer (HAL) Library 
* Version: v1.0.0
* Author: C. Cheney
* Target: PSoC4 CYBLE_222014-01 Module
*
* Brief: Serial communication middleware for the MJL driver library 
*
* 2023.04.26  - Document Created
********************************************************************************/
#include "hal_psoc4.h"
#include "mjl_errors.h"
#include "project.h"    /* Cypress files*/

/*******************************************************************************
* Function Name: uart_psoc4SCB_start()
********************************************************************************
* \brief
*   Start wrapper for a SCB based UART on PSoC4
*
* \return
*  Error code of the operation
*******************************************************************************/
uint32_t uart_psoc4SCB_start(MLJ_UART_T *const state){   
    uint32_t error = 0;   
    (void) state;
    uartUsb_Start();
    return error;
}

/*******************************************************************************
* Function Name: uart_psoc4SCB_stop()
********************************************************************************
* \brief
*   Stop wrapper for a SCB based UART on PSoC4
*
* \return
*  Error code of the operation
*******************************************************************************/
uint32_t uart_psoc4SCB_stop(MLJ_UART_T *const state){   
    uint32_t error = 0;   
    (void) state;
    uartUsb_Stop();
    return error;
}

/*******************************************************************************
* Function Name: uart_psoc4SCB_writeArrayBlocking()
********************************************************************************
* \brief
*   Wrapper for an SCB Based UART on PSoC4
*   Write an array of data via UART using the SCB api. This is a blocking function
*
* \return
*  Error code of the operation
*******************************************************************************/
uint32_t uart_psoc4SCB_writeArrayBlocking(const uint8_t *array, uint16_t len){
    uint32_t error = 0;   
    uartUsb_SpiUartPutArray(array, len);
    return error;
}

/*******************************************************************************
* Function Name: uart_psoc4SCB_read()
********************************************************************************
* \brief
*   Wrapper for an SCB Based UART on PSoC4
*   Write an array of data via UART using the SCB api. This is a blocking function
*
* \return
*  Error code of the operation
*******************************************************************************/
uint32_t uart_psoc4SCB_read(uint8_t *data){
    uint32_t error = 0;   
    if(0 == uartUsb_SpiUartGetRxBufferSize()){error|= ERROR_UNAVAILABLE;}
    else {*data = uartUsb_SpiUartReadRxData();}
    return error;
}

#ifdef USE_SPI
    /*******************************************************************************
    * Function Name: spi_scbWriteArrayBlocking()
    ********************************************************************************
    * \brief
    *   Write an array of data via SPI using the SCB api. This is a blocking function
    *
    * \return
    *  Error code of the operation
    *******************************************************************************/
    uint32_t spi_scbWriteArrayBlocking(uint8_t slaveId, uint8_t * cmdArray, uint16_t len) {
        uint32_t error = 0;
        /* Bit bang CS line */
        error |= spi_assertSlave(slaveId);
        /* Place the array into the TX buffer */
        SPI_SpiUartPutArray(cmdArray, len);
        if(!error){
            /* Wait for the write buffer to complete */
            uint16_t count = 0;
            /* Wait until data has been shifted in */
            while(!SPI_SpiIsBusBusy()){
                if(++count == 0){
                    error=ERROR_TIMEOUT;
                    // printLn(&usb, "SPI 'Done' Timeout");
                    break;
                }    
            }
            /* Wait until data has been shifted out */
            if(!error){
                count = 0;
                while(SPI_SpiIsBusBusy()){
                    if(++count == 0){
                        error=ERROR_TIMEOUT;
                        // printLn(&usb, "SPI 'Done' Timeout");
                        break;
                    }    
                }   
            }
        }
        /* Bit bang CS line */
        spi_disassertSlave();
        
        return error;
    }

    /*******************************************************************************
    * Function Name: spi_scbReadArrayBlocking()
    ********************************************************************************
    * \brief
    *   Read an array of data via SPI using the SCB api. Clears the RX buffer before 
    *   reading. This is a blocking function.
    *
    * \param slaveId [in]
    *   ID of the slave to read from
    *   
    * \param buffer [out]
    *   Pointer to the buffer that the results will be placed into
    *
    * \param len [in]
    *   Number of elements to be read 
    * 
    * \return
    *  Error code of the operation
    *******************************************************************************/
    uint32_t spi_scbReadArrayBlocking(uint8_t slaveId, uint8_t * buffer, uint16_t len) {
        uint32_t error = 0;
        /* Clear the RX buffer */
        SPI_SpiUartClearRxBuffer();
        /* Use the buffer as temp storage for the dummy write*/
        memset(buffer, 0, len);
        spi_scbWriteArrayBlocking(slaveId, buffer, len);
        for(uint8_t i = 0; i<len; i++){
            if(SPI_SpiUartGetRxBufferSize()){
                buffer[i] = SPI_SpiUartReadRxData();
            }
            else{
                error |= ERROR_INVALID;
                break;
            }
        }
        return error;
    }


    /*******************************************************************************
    * Function Name: spi_assertSlave()
    ********************************************************************************
    * \brief
    *  Bit banged - Set the active slave for the SPI
    *
    * \return
    *  None
    *******************************************************************************/
    uint32_t spi_assertSlave(uint8_t slaveId){
        uint32_t error = 0;
        /* OLED Display */
        if(slaveId == SL_SPI_ID_DISPLAY){
            pin_SPI_CS_DISP_Write(SPI_CS_ACTIVE);   
        }
        /* Instrumentation Amp */
        else if (slaveId == SL_SPI_ID_INA) {
            pin_SPI_CS_INA_Write(SPI_CS_ACTIVE);   
        }
        /* Flash Memory */
        else if (slaveId == SL_SPI_ID_FLASH) {
            pin_SPI_CS_FLASH_Write(SPI_CS_ACTIVE);   
        }
        /* Invalid ID */
        else{
            error |= ERROR_SHIFT_VAL;   
        }
        return error;
    }

    /*******************************************************************************
    * Function Name: spi_disassertSlave()
    ********************************************************************************
    * \brief
    *  Bit banged - Remove the active slave
    *
    * \return
    *  None
    *******************************************************************************/
    void spi_disassertSlave(void){
        pin_SPI_CS_DISP_Write(SPI_CS_INACTIVE);
        pin_SPI_CS_INA_Write(SPI_CS_INACTIVE);
        pin_SPI_CS_FLASH_Write(SPI_CS_INACTIVE);
    }
#endif /* USE_SPI */

