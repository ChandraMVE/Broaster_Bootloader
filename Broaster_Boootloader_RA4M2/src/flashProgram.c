/*
 * flashTest.c
 *
 *  Created on: Nov 25, 2022
 *      Author: xbie
 */
#include <flashProgram.h>
#include "hal_data.h"

#define BLOCK_NUM   2

typedef struct
{
    uint32_t State;
    uint16_t totalPage;
    uint16_t Crc;
}SBootInfor_t;

static SBootInfor_t _bootInfor = {0};
SBootInfor_t b1 =
{
 .State = 12111999,
 .totalPage = 201,
 .Crc = 22,
};

SBootInfor_t b2 =
{
 .State = 1,
 .totalPage = 3,
 .Crc = 2,
};

typedef struct
{
    uint32_t Version;
    uint16_t setTemp1;
    uint16_t setTemp2;
}SEepFeature_t;

static SEepFeature_t _eepFeature = {0};
SEepFeature_t eepFeature1 =
{
 .Version = 1,
 .setTemp1 = 1608,
 .setTemp2 = 1708,
};

SEepFeature_t eepFeature2 =
{
 .Version = 2,
 .setTemp1 = 1208,
 .setTemp2 = 1308,
};

/* Flags, set from Callback function */
static volatile _Bool g_b_flash_event_not_blank = false;
static volatile _Bool g_b_flash_event_blank = false;
static volatile _Bool g_b_flash_event_erase_complete = false;
static volatile _Bool g_b_flash_event_write_complete = false;

static fsp_err_t blankcheck_event_flag(void);
void writeBootInfor(void);
void readBootInfor(void);
void writeEepFeature(void);
void readEepFeature(void);
fsp_err_t writeFlash(uint32_t dataAddress,uint32_t dataSize, uint32_t flashAddress,uint16_t numFlashBlock);
void readFlash(uint32_t dataAddress,uint32_t dataSize,uint32_t flashAddress);
static fsp_err_t _eraseCodeFlash(uint32_t flashAddress,uint32_t numFlashBlock);
static fsp_err_t _copyCodeImage(void);


void flashProgramCheckUpdate(void)
{
    // Check boot state in boot information
    readBootInfor();

    if(_bootInfor.State == BOOT_LOADER_STATE_COPY_CODE)
    {
        // Feed the dog
        R_WDT_Refresh(&g_wdt0_ctrl);

        // erase application area
        if(_eraseCodeFlash(APPLICATION_START_ADDRESS,TOTAL_CODE_FLASH_BLOCK) == FSP_SUCCESS)
        {
            // Feed the dog
            R_WDT_Refresh(&g_wdt0_ctrl);

            // copy code image
            if(_copyCodeImage() == FSP_SUCCESS)
            {
                // Feed the dog
                R_WDT_Refresh(&g_wdt0_ctrl);

                // Clear boot state and reboot the MCU
                _bootInfor.State = 0;
                writeBootInfor();
            }
        }

        // No matter copy success or failed, we need reset mcu at this point
        NVIC_SystemReset();
    }
}

/********************************************************************
* Function Name: _eraseCodeFlash
* Description  : Erase code area. Can be used for application or code image.
* Arguments    : flashAddress - flash starting address
*                numFlashBlock - number of blocks to be erased. 32k each block.
* Return Value : fsp_err_t
*********************************************************************/
static fsp_err_t _eraseCodeFlash(uint32_t flashAddress,uint32_t numFlashBlock)
{
    fsp_err_t err = FSP_SUCCESS;
    flash_result_t blank_check_result = FLASH_RESULT_BLANK;

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
static uint8_t _codeBuf[CODE_PAGE_SIZE] = {0};
static fsp_err_t _copyCodeImage(void)
{
    fsp_err_t err = FSP_SUCCESS;
    uint32_t sourceAddress;
    uint32_t desAddress;
    uint32_t Page;

    if(_bootInfor.totalPage > MAX_CODE_PAGES)
    {   // something is wrong
        return FSP_ERR_WRITE_FAILED;
    }

    // Feed the dog
    R_WDT_Refresh(&g_wdt0_ctrl);

    // Preparing addresses
    sourceAddress = CODE_IMAGE_START_ADDRESS;
    desAddress = APPLICATION_START_ADDRESS;

    /* Disable interrupts to prevent vector table access while code flash is in P/E mode. */
    __disable_irq();

    for(Page = 0; Page < _bootInfor.totalPage; Page ++)
    {
        //TODO: Add line here to feed watchdog
        // Feed the dog
        R_WDT_Refresh(&g_wdt0_ctrl);

        // Read page of code from code image
        memcpy(_codeBuf, (uint8_t *) sourceAddress, CODE_PAGE_SIZE);

        // Feed the dog
        R_WDT_Refresh(&g_wdt0_ctrl);

        // Write code flash data
        err = R_FLASH_HP_Write(&g_flash0_ctrl, (uint32_t) _codeBuf,desAddress, CODE_PAGE_SIZE);
        // Error Handle
        if (FSP_SUCCESS != err)
        {
            //Write API failed, Restart the Application
            return err;
        }

        sourceAddress += CODE_PAGE_SIZE;
        desAddress += CODE_PAGE_SIZE;
    }

    /* Enable interrupts after code flash operations are complete. */
    __enable_irq();

    return err;
}

fsp_err_t writeFlash(uint32_t dataAddress,uint32_t dataSize, uint32_t flashAddress,uint16_t numFlashBlock)
{
    fsp_err_t err = FSP_SUCCESS;
    flash_result_t blank_check_result = FLASH_RESULT_BLANK;

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

    return err;
}

void readFlash(uint32_t dataAddress,uint32_t dataSize,uint32_t flashAddress)
{
    memcpy((uint8_t *)dataAddress, (uint8_t *)flashAddress, dataSize);
}


void writeBootInfor(void)
{
    writeFlash((uint32_t)&_bootInfor,sizeof(SBootInfor_t),FLASH_HP_DF_BLOCK_0,1);
}

void readBootInfor(void)
{
    //memcpy(&_bootInfor, (uint8_t *) FLASH_HP_DF_BLOCK_0, sizeof(SBootInfor_t));
    readFlash((uint32_t)&_bootInfor,sizeof(SBootInfor_t),FLASH_HP_DF_BLOCK_0);
}

void writeEepFeature(void)
{
    writeFlash((uint32_t)&_eepFeature,sizeof(SEepFeature_t),FLASH_HP_DF_BLOCK_1,16);
}

void readEepFeature(void)
{
    readFlash((uint32_t)&_eepFeature,sizeof(SEepFeature_t),FLASH_HP_DF_BLOCK_1);
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
