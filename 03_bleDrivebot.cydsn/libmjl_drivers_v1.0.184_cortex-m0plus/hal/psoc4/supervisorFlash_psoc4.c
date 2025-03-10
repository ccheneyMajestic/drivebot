  /***************************************************************************
*                                       MICA
* File: Sflash.c
* Workspace: micaCompnents
* Project Name: libMica
* Version: v1.0
* Author: Craig Cheney
*
* Brief:
*   API for interacting with the Supervisory flash
* 
* Authors:
*   Craig Cheney
*
* Change Log:
*   2018.03.03 CC - Document created
********************************************************************************/
#include "supervisorFlash_psoc4.h"


/*******************************************************************************
* Function Name: SFLASH_ReadRow()
****************************************************************************//**
*
* \brief Places the SFlash data of the specified row into the array passed in. 
    Array must be of length SFLASH_SIZE_ROW.
*
* \param rowNum
*   The row number of the SFlash to read from 
*   
* \param dataBuffer
*   Buffer to place data into. Must be of size SFLASH_SIZE_ROW 
*
* \return
* uint32: An error code with the result of the Read procedure. 
* The possible error codes are:
*
*  Errors codes                             | Description
*   ------------                            | -----------
*   SFLASH_ERR_OK                 | Read was successful
*   SFLASH_ERR_INDEX_RANGE        | Invalid row number was passed in
*******************************************************************************/
uint32 SFLASH_ReadRow(uint8 rowNum, uint8* dataBuffer){
    /* Ensure a valid row number */
    if(rowNum > (SFLASH_NUM_ROWS - 1) ) {
        return SFLASH_ERR_INDEX_RANGE;
    }
    uint16 idx = 0;
    /* iterate through all of the row */
    for(idx = 0; idx < SFLASH_SIZE_ROW ; idx++){
        /* Calculate the address of the data cell */
        uint32 addr = SFLASH_BASE_ADDR + (rowNum * SFLASH_SIZE_ROW) + idx;
        /* Copy the data from SFlash into the buffer */
        dataBuffer[idx] = *(uint8 *) addr;
    }
    /* Return success */
    return SFLASH_ERR_OK;
}

/*******************************************************************************
* Function Name: SFLASH_WriteRow()
****************************************************************************//**
*
* \brief Copies the data in 'rowData' and writes it into the Supervisory Flash 
*   (SFlash) section of code. Wrapper of the Cypress function CySysSFlashWriteUserRow().
*
* \param rowNum
*   The row number of the SFlash to write to. 
*   
* \param rowData
*   Buffer copy data from
*
*\return 
* uint32: An error code with the result of the Write procedure. 
* The possible error codes are:
*
*  Errors codes                             | Description
*   ------------                            | -----------
*   ERR_NO_ERROR                            | Write was successful
*   <Cypress generated Error>                 | See CySysSFlashWriteUserRow()
*
*******************************************************************************/
uint32 SFLASH_WriteRow(uint8 rowNum, const uint8* rowData){
    /* Write the data and return the result */
    return CySysSFlashWriteUserRow(rowNum, rowData);
}


/*******************************************************************************
* Function Name: SFLASH_ReadLocalName()
****************************************************************************//**
*
* \brief Copies the name stored in SFLASH into the array passed in. 
*
* \param nameArray
*   Array the name will be passed into. Must be of length SFLASH_NAME_LEN
*   or longer.
*   
*
*\return 
* uint32: An error code with the result of the Read procedure. 
* The possible error codes are:
*
*  Errors codes                             | Description
*   ------------                            | -----------
*   SFLASH_ERR_OK                 | Read was successful
*******************************************************************************/
uint32 SFLASH_ReadLocalName(char* nameArray){
    uint8 i;
    /* Start of the name location in SFlash */
    uint32 nameBaseAddr = SFLASH_BASE_ADDR + (SFLASH_NAME_ROW * SFLASH_SIZE_ROW) + SFLASH_NAME_OFFSET;
    /* Move the longest possible name into the array */
    for(i = 0; i < SFLASH_NAME_LEN; i++) {
        /* Copy to the array passed in */
        uint32 addr = nameBaseAddr + i;
        /* Copy the character */
        nameArray[i] = *(char *) addr;
    }
    /* Return success */
    return SFLASH_ERR_OK;
}

/*******************************************************************************
* Function Name: SFLASH_WriteLocalName()
****************************************************************************//**
*
* \brief Stores the name passed in into SFLASH.
*
* \param len 
*   Length of the name. Must be less than or equal to SFLASH_NAME_LEN
*
* \param nameArray
*   Array which contains the Name - does not need to be null-terminated.
*
*\return 
* uint32: An error code with the result of the Read procedure. 
* The possible error codes are:
*
*  Errors codes                             | Description
*   ------------                            | -----------
*   SFLASH_ERR_OK                 | Writing the name was successful
*   SFLASH_ERR_INDEX_RANGE        | Name was too long to write
*******************************************************************************/
uint32 SFLASH_WriteLocalName(uint8 len, char* nameArray){
    /* Ensure data is proper length */
    if(len > SFLASH_NAME_LEN) {
        return SFLASH_ERR_INDEX_RANGE;
    }
    /* Read the current row into rowData[] */
    uint8 rowData[SFLASH_SIZE_ROW];
    SFLASH_ReadRow(SFLASH_NAME_ROW, rowData);
    /* Iterate through the new array */
    uint8 i;
    for(i = 0; i < len; i++) {
        /* Position of data  */   
        uint8 addr = SFLASH_NAME_OFFSET + i;
        /* Copy new character */
        rowData[addr] = nameArray[i];
    }
    /* Append a null octect unless there is no space for one */
    if( len != SFLASH_NAME_LEN) {
        /* Append a null octet to end of string */
        rowData[SFLASH_NAME_OFFSET + len] ='\0';
    }

    /* Write the new Row to SFlash */
    return SFLASH_WriteRow(SFLASH_NAME_ROW, rowData);
}
  
/* [] END OF FILE */
