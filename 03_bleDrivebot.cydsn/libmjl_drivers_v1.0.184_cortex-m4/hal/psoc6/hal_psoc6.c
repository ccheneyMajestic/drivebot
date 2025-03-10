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
  SPI_Start();
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
  uint32_t error = 0;
  (void) state;
  Cy_SCB_SPI_Disable(SPI_HW, &SPI_context);
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
  Cy_SCB_SPI_WriteArrayBlocking(SPI_HW, (void *) array, len);
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
  /* Move data from RX FIFO to results */
  *result = Cy_SCB_SPI_Read(SPI_HW);
  
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
  /* Write out the data */
  Cy_SCB_SPI_SetActiveSlaveSelect(SPI_HW, id);
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
  return Cy_SCB_SPI_GetNumInRxFifo(SPI_HW);
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
  return Cy_SCB_SPI_GetNumInTxFifo(SPI_HW);
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
  Cy_SCB_SPI_ClearRxFifo(SPI_HW);
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
  Cy_SCB_SPI_ClearTxFifo(SPI_HW);
  return error;
}



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
//    /* Bit bang CS line */
//    error |= spi_assertSlave(slaveId);
//    /* Place the array into the TX buffer */
//    SPI_SpiUartPutArray(cmdArray, len);
//    if(!error){
//        /* Wait for the write buffer to complete */
//        uint16_t count = 0;
//        /* Wait until data has been shifted in */
//        while(!SPI_SpiIsBusBusy()){
//            if(++count == 0){
//                error=ERROR_TIMEOUT;
//                // printLn(&usb, "SPI 'Done' Timeout");
//                break;
//            }    
//        }
//        /* Wait until data has been shifted out */
//        if(!error){
//            count = 0;
//            while(SPI_SpiIsBusBusy()){
//                if(++count == 0){
//                    error=ERROR_TIMEOUT;
//                    // printLn(&usb, "SPI 'Done' Timeout");
//                    break;
//                }    
//            }   
//        }
//    }
//    /* Bit bang CS line */
//    spi_disassertSlave();
//    
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
//    /* Clear the RX buffer */
//    SPI_SpiUartClearRxBuffer();
//    /* Use the buffer as temp storage for the dummy write*/
//    memset(buffer, 0, len);
//    spi_scbWriteArrayBlocking(slaveId, buffer, len);
//    for(uint8_t i = 0; i<len; i++){
//        if(SPI_SpiUartGetRxBufferSize()){
//            buffer[i] = SPI_SpiUartReadRxData();
//        }
//        else{
//            error |= ERROR_INVALID;
//            break;
//        }
//    }
    return error;
}

