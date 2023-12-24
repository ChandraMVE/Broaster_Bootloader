/*
 * flashTest.c
 *
 *  Created on: Nov 25, 2022
 *      Author: xbie
 */
#include "hal_data.h"
#include <flashProgram.h>

#define BLOCK_NUM   2

typedef struct
{
    uint32_t State;
    uint16_t totalPage;
    uint16_t Crc;
}SBootInfor_t;

//static SBootInfor_t _bootInfor = {0};
//SBootInfor_t b1 =
//{
// .State = 12111999,
// .totalPage = 201,
// .Crc = 22,
//};
//
//SBootInfor_t b2 =
//{
// .State = 1,
// .totalPage = 3,
// .Crc = 2,
//};

//typedef struct
//{
//    uint32_t Version;
//    uint16_t setTemp1;
//    uint16_t setTemp2;
//}SEepFeature_t;

//static SEepFeature_t _eepFeature = {0};
//SEepFeature_t eepFeature1 =
//{
// .Version = 1,
// .setTemp1 = 1608,
// .setTemp2 = 1708,
//};
//
//SEepFeature_t eepFeature2 =
//{
// .Version = 2,
// .setTemp1 = 1208,
// .setTemp2 = 1308,
//};

/* Flags, set from Callback function */
static volatile _Bool g_b_flash_event_not_blank = false;
static volatile _Bool g_b_flash_event_blank = false;
static volatile _Bool g_b_flash_event_erase_complete = false;
static volatile _Bool g_b_flash_event_write_complete = false;

static fsp_err_t blankcheck_event_flag(void);
//static void _readDataFlash(uint32_t dataAddress,uint32_t dataSize,uint32_t flashAddress);
//static fsp_err_t _saveDataFlash(uint32_t dataAddress,uint32_t dataSize, uint32_t flashAddress,uint16_t numFlashBlock);

//void writeBootInfor(void);
//void readBootInfor(void);
//void writeEepFeature(SEepFeature_t * pEepFeature);
//void readEepFeature(SEepFeature_t * pEepFeature);
fsp_err_t writeFlash(uint32_t dataAddress,uint32_t dataSize, uint32_t flashAddress,uint16_t numFlashBlock);
void readFlash(uint32_t dataAddress,uint32_t dataSize,uint32_t flashAddress);
//static fsp_err_t _eraseCodeFlash(uint32_t flashAddress,uint32_t numFlashBlock);
//static fsp_err_t _copyCodeImage(void);


//void bootInforTest(void)
//{
//    readBootInfor();
//    _bootInfor = b1;
//    writeBootInfor();
//
//    _bootInfor = b2;
//    readBootInfor();
//
//    readEepFeature();
//    _eepFeature = eepFeature1;
//    writeEepFeature();
//
//    _eepFeature = eepFeature2;
//    readEepFeature();
//}

//void flashProgramCheckUpdate(void)
//{
//    // Check boot state in boot information
//    readBootInfor();
//
//    if(_bootInfor.State == BOOT_LOADER_STATE_COPY_CODE)
//    {
//        // erase application area
//        if(_eraseCodeFlash(APPLICATION_START_ADDRESS,TOTAL_CODE_FLASH_BLOCK) == FSP_SUCCESS)
//        {
//            // copy code image
//            if(_copyCodeImage() == FSP_SUCCESS)
//            {
//                // Clear boot state and reboot the MCU
//                _bootInfor.State = 0;
//                writeBootInfor();
//            }
//        }
//
//        // No matter copy success or failed, we need reset mcu at this point
//        NVIC_SystemReset();
//    }
//}

//fsp_err_t flash_hp_code_flash_operations(void)
//{
//    fsp_err_t err = FSP_SUCCESS;
//    flash_result_t blank_check_result = FLASH_RESULT_BLANK;
//    uint8_t write_buffer[BLOCK_SIZE] = {RESET_VALUE};
//    uint8_t read_buffer[BLOCK_SIZE] = {RESET_VALUE};
//
//    /* Set write buffer, clear read buffer */
//    for (uint8_t index = 0; index < BLOCK_SIZE; index++)
//    {
//        write_buffer[index] = index;
//    }
//
//    /* Disable interrupts to prevent vector table access while code flash is in P/E mode. */
//    __disable_irq();
//
//    /* Erase Block */
//    err = R_FLASH_HP_Erase(&g_flash_ctrl, FLASH_HP_CF_BLOCK_8, BLOCK_NUM);
//    /* Error Handle */
//    if (FSP_SUCCESS != err)
//    {
//        APP_ERR_PRINT("\r\nErase API failed, Restart the Application");
//        return err;
//    }
//    APP_PRINT("\r\nErase successful");
//
//    /* Blank Check */
//    err = R_FLASH_HP_BlankCheck(&g_flash_ctrl, FLASH_HP_CF_BLOCK_8, FLASH_HP_CF_BLOCK_SIZE_8KB, &blank_check_result);
//    /* Error Handle */
//    if (FSP_SUCCESS != err)
//    {
//        APP_ERR_PRINT("\r\nBlankCheck API failed, Restart the Application");
//        return err;
//    }
//    APP_PRINT("\r\nBlankCheck API Successful");
//
//    /* Validate the blank check result */
//    if (FLASH_RESULT_BLANK == blank_check_result)
//    {
//        APP_PRINT("\r\n FLASH is blank ");
//    }
//    else if (FLASH_RESULT_NOT_BLANK == blank_check_result)
//    {
//        APP_ERR_PRINT("\r\n Flash is not Blank, not to write the data. Restart the application");
//        return (fsp_err_t)FLASH_RESULT_NOT_BLANK;
//    }
//
//    /* Write code flash data*/
//    err = R_FLASH_HP_Write(&g_flash_ctrl, (uint32_t) write_buffer,FLASH_HP_CF_BLOCK_8, BLOCK_SIZE);
//    /* Error Handle */
//    if (FSP_SUCCESS != err)
//    {
//        APP_ERR_PRINT("\r\nWrite API failed, Restart the Application");
//        return err;
//    }
//    APP_PRINT("\r\nWriting flash data is successful\r\n");
//
//    /*Read code flash data */
//    memcpy(read_buffer, (uint8_t *) FLASH_HP_CF_BLOCK_8, BLOCK_SIZE);
//
//    /* comparing the write_buffer and read_buffer */
//    if (RESET_VALUE == memcmp(read_buffer, write_buffer, BLOCK_SIZE))
//    {
//        APP_PRINT("\r\nRead and Write buffer is verified and successful");
//        /* Print the read data on the RTT terminal */
//        APP_PRINT("\r\nRead Data : \r\n");
//        READ_DATA_PRINT(read_buffer);
//    }
//    else
//    {
//        APP_PRINT("Read and Write buffer is verified and not successful");
//        return FSP_ERR_WRITE_FAILED;
//    }
//
//    /* Erase block again */
//    APP_PRINT("\r\nErase block again");
//    err = R_FLASH_HP_Erase(&g_flash_ctrl, FLASH_HP_CF_BLOCK_5, BLOCK_NUM);
//    if(FSP_SUCCESS != err)
//    {
//        APP_ERR_PRINT("\r\nErase API failed, Restart the Application");
//        return err;
//    }
//    APP_PRINT("\r\nErase successful");
//
////#if !(defined (BOARD_RA6M5_EK) || defined (BOARD_RA6M4_EK) || defined (BOARD_RA4M3_EK)||defined(BOARD_RA4M2_EK)) //Not supported for this MCU
////    /* Set Access window.
////     * CAUTION: Highly recommended not to use this function if not aware of consequences OR
////     * use it with the accessWindowClear API at the end of application.
////     * This API locks the Code flash and the lock retains even after power cycle.
////     * Which means, even after power cycle, user will not be able to program the code to code flash if the
////     * access window is wrongly set.
////     *
////     * WORKAROUND: If uses uses accessWindowSet and locks the window. Flash can be unlocked by running
////     * different application performing just "open" call and "accessWindowClear()" running from the RAM.
////     */
////    err = R_FLASH_HP_AccessWindowSet(&g_flash_ctrl, FLASH_HP_CF_BLOCK_3, FLASH_HP_CF_BLOCK_7);
////    if (FSP_SUCCESS != err)
////    {
////        APP_ERR_PRINT("\r\nAccessWindowSet API failed, Restart the Application");
////        return err;
////    }
////    APP_PRINT("\r\nAccessWindowSet successful");
////
////    /* Write code flash data*/
////    err = R_FLASH_HP_Write(&g_flash_ctrl, (uint32_t) write_buffer,FLASH_HP_CF_BLOCK_5, BLOCK_SIZE);
////    /* Error Handle */
////    if (FSP_SUCCESS != err)
////    {
////        APP_ERR_PRINT("\r\nWrite API failed, Restart the Application");
////        return err;
////    }
////
////    APP_PRINT("\r\nWriting flash data is successful");
////
////    /* Clear Flash Access Window */
////    err = R_FLASH_HP_AccessWindowClear(&g_flash_ctrl);
////    /* Error Handle */
////    if (FSP_SUCCESS != err)
////    {
////        APP_ERR_PRINT("\r\nAccessWindoeClear API failed, Restart the Application");
////        return err;
////    }
////    APP_PRINT("\r\nAccess Window cleared ");
////#endif
//
//    /* Enable interrupts after code flash operations are complete. */
//    __enable_irq();
//
//    return err;
//}

/********************************************************************
* Function Name: fpEraseCodeFlash
* Description  : Erase code area. Can be used for application or code image.
* Arguments    : flashAddress - flash starting address
*                numFlashBlock - number of blocks to be erased. 32k each block.
* Return Value : fsp_err_t
*********************************************************************/
fsp_err_t fpEraseCodeFlash(uint32_t flashAddress,uint32_t numFlashBlock)
{
    fsp_err_t err = FSP_SUCCESS;
    flash_result_t blank_check_result = FLASH_RESULT_BLANK;
//        uint8_t write_buffer[BLOCK_SIZE] = {RESET_VALUE};
//        uint8_t read_buffer[BLOCK_SIZE] = {RESET_VALUE};
//
//        /* Set write buffer, clear read buffer */
//        for (uint8_t index = 0; index < BLOCK_SIZE; index++)
//        {
//            write_buffer[index] = index;
//        }

    //TODO: Add line here to feed watchdog

    /* Disable interrupts to prevent vector table access while code flash is in P/E mode. */
    __disable_irq();

    // Feed the dog
    R_WDT_Refresh(&g_wdt0_ctrl);

    /* Erase Block */
    //err = R_FLASH_HP_Erase(&g_flash0_ctrl, FLASH_HP_CF_BLOCK_8, BLOCK_NUM);
    err = R_FLASH_HP_Erase(&g_flash0_ctrl, flashAddress, numFlashBlock);
    /* Error Handle */
    if (FSP_SUCCESS != err)
    {
        //Erase API failed, Restart the Application
        return err;
    }
    //Erase successful

    // Feed the dog
    R_WDT_Refresh(&g_wdt0_ctrl);

    /* Blank Check */
    //err = R_FLASH_HP_BlankCheck(&g_flash0_ctrl, FLASH_HP_CF_BLOCK_8, FLASH_HP_CF_BLOCK_SIZE_8KB, &blank_check_result);
    err = R_FLASH_HP_BlankCheck(&g_flash0_ctrl, flashAddress, TOTAL_CODE_FLASH_BYTES, &blank_check_result);
    /* Error Handle */
    if (FSP_SUCCESS != err)
    {
        //BlankCheck API failed, Restart the Application
        return err;
    }
    //BlankCheck API Successful

    /* Validate the blank check result */
    if (FLASH_RESULT_BLANK == blank_check_result)
    {
        //FLASH is blank
    }
    else if (FLASH_RESULT_NOT_BLANK == blank_check_result)
    {
        //Flash is not Blank, not to write the data. Restart the application
        return (fsp_err_t)FLASH_RESULT_NOT_BLANK;
    }

        /* Write code flash data*/
//        err = R_FLASH_HP_Write(&g_flash_ctrl, (uint32_t) write_buffer,FLASH_HP_CF_BLOCK_8, BLOCK_SIZE);
//        /* Error Handle */
//        if (FSP_SUCCESS != err)
//        {
//            APP_ERR_PRINT("\r\nWrite API failed, Restart the Application");
//            return err;
//        }
//        APP_PRINT("\r\nWriting flash data is successful\r\n");

        /*Read code flash data */
//        memcpy(read_buffer, (uint8_t *) FLASH_HP_CF_BLOCK_8, BLOCK_SIZE);
//
//        /* comparing the write_buffer and read_buffer */
//        if (RESET_VALUE == memcmp(read_buffer, write_buffer, BLOCK_SIZE))
//        {
//            APP_PRINT("\r\nRead and Write buffer is verified and successful");
//            /* Print the read data on the RTT terminal */
//            APP_PRINT("\r\nRead Data : \r\n");
//            READ_DATA_PRINT(read_buffer);
//        }
//        else
//        {
//            APP_PRINT("Read and Write buffer is verified and not successful");
//            return FSP_ERR_WRITE_FAILED;
//        }

        /* Erase block again */
//        APP_PRINT("\r\nErase block again");
//        err = R_FLASH_HP_Erase(&g_flash_ctrl, FLASH_HP_CF_BLOCK_5, BLOCK_NUM);
//        if(FSP_SUCCESS != err)
//        {
//            APP_ERR_PRINT("\r\nErase API failed, Restart the Application");
//            return err;
//        }
//        APP_PRINT("\r\nErase successful");

    /* Enable interrupts after code flash operations are complete. */
    __enable_irq();

    return err;
}

/********************************************************************
* Function Name: _copyCodeImage
* Description  : Copy code from code image area to application area. Code
*                is already download by application. total 224 k 7 block.
*                This should be called after _eraseCodeFlash returns "FSP_SUCCESS"
* Arguments    : None
* Return Value : fsp_err_t
*********************************************************************/
fsp_err_t fpWriteCodePage(uint32_t desAddress,uint8_t * pCode)
{
    fsp_err_t err = FSP_SUCCESS;

    /* Disable interrupts to prevent vector table access while code flash is in P/E mode. */
    __disable_irq();

//    for(Page = 0; Page < _bootInfor.totalPage; Page ++)
//    {
        //TODO: Add line here to feed watchdog

        // Read page of code from code image
//        memcpy(_codeBuf, (uint8_t *) sourceAddress, CODE_PAGE_SIZE);

    // Feed the dog
    R_WDT_Refresh(&g_wdt0_ctrl);

    // Write code flash data
    err = R_FLASH_HP_Write(&g_flash0_ctrl, (uint32_t) pCode,desAddress, CODE_PAGE_SIZE);
    // Error Handle
    if (FSP_SUCCESS != err)
    {
        //Write API failed, Restart the Application
        return err;
    }

//        sourceAddress += CODE_PAGE_SIZE;
//        desAddress += CODE_PAGE_SIZE;

        //Writing flash data is successful

        /*Read code flash data */
//        memcpy(read_buffer, (uint8_t *) FLASH_HP_CF_BLOCK_8, BLOCK_SIZE);

        /* comparing the write_buffer and read_buffer */
//        if (RESET_VALUE == memcmp(read_buffer, write_buffer, BLOCK_SIZE))
//        {
//            // Read and Write buffer is verified and successful
//            // Print the read data on the RTT terminal
//            // Read Data : \r\n");
////            READ_DATA_PRINT(read_buffer);
//        }
//        else
//        {
//            // Read and Write buffer is verified and not successful
//            return FSP_ERR_WRITE_FAILED;
//        }
//    }

    /* Enable interrupts after code flash operations are complete. */
    __enable_irq();

    return err;
}

/********************************************************************
* Function Name: fpSaveDataFlash
* Description  : Save data to data flash
* Arguments    : flashAddress - flash starting address
*                numFlashBlock - number of blocks to be erased. 32k each block.
* Return Value : fsp_err_t
*********************************************************************/
fsp_err_t fpSaveDataFlash(uint32_t dataAddress,uint32_t dataSize, uint32_t flashAddress,uint16_t numFlashBlock)
{
    fsp_err_t err = FSP_SUCCESS;
    flash_result_t blank_check_result = FLASH_RESULT_BLANK;

    if(dataSize % 4)    // We want to make sure the dataSize is the multiple of 4. XB on 6/8/2023
    {
        dataSize = (dataSize / 4 + 1) * 4;
    }
    if(dataSize > FLASH_HP_DF_BLOCK_SIZE)
    {   // We want to make sure the data size is not greater than 64 bytes. XB on 6/8/2023
        dataSize = FLASH_HP_DF_BLOCK_SIZE;
    }

    // Feed the dog
    R_WDT_Refresh(&g_wdt0_ctrl);

    /* Erase Block */
    //err = R_FLASH_HP_Erase(&g_flash0_ctrl, FLASH_HP_DF_BLOCK_0, BLOCK_NUM);
    err = R_FLASH_HP_Erase(&g_flash0_ctrl, flashAddress, numFlashBlock);
    /* Error Handle */
    if (FSP_SUCCESS != err)
    {
        //Erase API failed, Restart the Application
        return err;
    }

    /* Wait for the erase complete event flag, if BGO is SET  */
    if (true == g_flash0_cfg.data_flash_bgo)
    {
        //BGO has enabled
        while (!g_b_flash_event_erase_complete);
        g_b_flash_event_erase_complete = false;
    }
    //Erase successful

    /* Data flash blank check */
    //err = R_FLASH_HP_BlankCheck(&g_flash0_ctrl, FLASH_HP_DF_BLOCK_0,FLASH_HP_DF_BLOCK_SIZE, &blank_check_result);
    err = R_FLASH_HP_BlankCheck(&g_flash0_ctrl, flashAddress,FLASH_HP_DF_BLOCK_SIZE * numFlashBlock, &blank_check_result);
    /* Error Handle */
    if (FSP_SUCCESS != err)
    {
        //BlankCheck API failed, Restart the Application
        return err;
    }

    /* Validate the blank check result */
    if (FLASH_RESULT_BLANK == blank_check_result)
    {
        //BlankCheck is successful
    }
    else if (FLASH_RESULT_NOT_BLANK == blank_check_result)
    {
        //BlankCheck is not blank,not to write the data. Restart the application
        return (fsp_err_t)FLASH_RESULT_NOT_BLANK;
    }
    else if (FLASH_RESULT_BGO_ACTIVE == blank_check_result)
    {
        /* BlankCheck will update in Callback */
        /* Event flag will be updated in the blank check function when BGO is enabled */
        err = blankcheck_event_flag();
        if(FSP_SUCCESS != err)
        {
            return err;
        }
    }
    else
    {
        /* No Operation */
    }

    // Feed the dog
    R_WDT_Refresh(&g_wdt0_ctrl);

    /* Write code flash data*/
    err = R_FLASH_HP_Write(&g_flash0_ctrl, (uint32_t) dataAddress,flashAddress, dataSize);
    /* Error Handle */
    if (FSP_SUCCESS != err)
    {
        //Write API failed, Restart the Application
        return err;
    }
    /* Wait for the write complete event flag, if BGO is SET  */
    if (true == g_flash0_cfg.data_flash_bgo)
    {
        while (!g_b_flash_event_write_complete);
        g_b_flash_event_write_complete = false;
    }

//        APP_PRINT("\r\nWriting flash data is successful\r\n");
//
//        /*Read code flash data */
//        memcpy(read_buffer, (uint8_t *) FLASH_HP_DF_BLOCK_1, BLOCK_SIZE);
//
//        /* comparing the write_buffer and read_buffer */
//        if (RESET_VALUE == memcmp(read_buffer, write_buffer, BLOCK_SIZE))
//        {
//            APP_PRINT("\r\nRead and Write buffer is verified and successful");
//            /* Print the read data on the RTT terminal */
//            APP_PRINT("\r\nRead Data : \r\n");
//            READ_DATA_PRINT(read_buffer);
//        }
//        else
//        {
//            APP_PRINT("Read and Write buffer is verified and not successful");
//            return FSP_ERR_WRITE_FAILED;
//        }

    return err;
}


void fpReadDataFlash(uint32_t dataAddress,uint32_t dataSize,uint32_t flashAddress)
{
    memcpy((uint8_t *)dataAddress, (uint8_t *)flashAddress, dataSize);
}

/*******************************************************************************************************************//**
 * @brief Callback function for FLASH HP HAL
 **********************************************************************************************************************/
void bgo_callback(flash_callback_args_t *p_args)
{
    if (FLASH_EVENT_NOT_BLANK == p_args->event)
    {
        g_b_flash_event_not_blank = true;
    }
    else if (FLASH_EVENT_BLANK == p_args->event)
    {
        g_b_flash_event_blank = true;
    }
    else if (FLASH_EVENT_ERASE_COMPLETE == p_args->event)
    {
        g_b_flash_event_erase_complete = true;
    }
    else if (FLASH_EVENT_WRITE_COMPLETE == p_args->event)
    {
        g_b_flash_event_write_complete = true;
    }
    else
    {
        /*No operation */
    }

}

/*******************************************************************************************************************//**
 * @brief This function is called to set the blank check event flags
 * @param[IN]   None
 * @retval      FSP_SUCCESS             Upon successful Flash_HP is blank
 * @retval      Any Other Error code    Upon unsuccessful Flash_HP is not blank
 **********************************************************************************************************************/
static fsp_err_t blankcheck_event_flag(void)
{
    fsp_err_t err = FSP_SUCCESS;
    /* Wait for callback function to set flag */
    while (!(g_b_flash_event_not_blank || g_b_flash_event_blank));

    if (g_b_flash_event_not_blank)
    {
        /* Reset Flag */
        g_b_flash_event_not_blank = false;
        return (fsp_err_t)FLASH_EVENT_NOT_BLANK;
    }
    else
    {
        /* Reset Flag */
        g_b_flash_event_blank = false;
    }
    return err;
}
