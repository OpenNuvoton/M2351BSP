/**************************************************************************//**
 * @file     ota_api.c
 * @version  V1.00
 * @brief    OTA porting API demo code
 *
 * @note
 * Copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "M2351.h"
#include "ota.h"
#include "ota_transfer.h"

#define printf(...)

#if (OTA_UPGRADE_FROM_SD)
#include "diskio.h"
#include "ff.h"
#endif
/*---------------------------------------------------------------------------------------------------------*/
/* Macro, type and constant definitions                                                                    */
/*---------------------------------------------------------------------------------------------------------*/

#if (OTA_UPGRADE_FROM_SD)
extern uint8_t g_u8R3Flag;
extern uint8_t volatile g_u8SDDataReadyFlag;
char g_au8NuBL32FileName[] = "NuBL32Fw.bin";
char g_au8NuBL33FileName[] = "NuBL33Fw.bin";
FIL    g_FileObject;
#endif

extern uint32_t CyclesPerUs;
extern volatile uint8_t g_u8SendbytesFlag;
extern volatile uint8_t g_u8ResetFlag;
extern volatile uint8_t g_u8DisconnFlag;
extern uint8_t g_au8SendBuf[BUF_SIZE];
extern volatile uint32_t g_u32SendbytesLen;

void OTA_SysValueInit(uint32_t u32HSI);
unsigned long get_fattime (void);

#if (OTA_UPGRADE_FROM_SD)
void SDH0_IRQHandler(void);
#endif

/* Init NuBL2 global variables for library */
/**
  * @brief      Init function for any hardware requirements(optional)
  * @param[in]  u32HSI        PLL Output Clock Frequency
  * @return     None
  * @details    This funcfion is to init function for any hardware requirements.
  */
void OTA_SysValueInit(uint32_t u32HSI)
{
    uint8_t au8SendBuf[]="CONNECT0\r\n";
#if (OTA_UPGRADE_FROM_SD)
    char au8NuBL32FileName[] = "NuBL32Fw.bin";
    char au8NuBL33FileName[] = "NuBL33Fw.bin";
#endif

    CyclesPerUs = (u32HSI / 1000000UL);
    g_u8SendbytesFlag = 1;
    g_u8ResetFlag = 0;
    g_u8DisconnFlag = 0;
    memcpy(g_au8SendBuf, au8SendBuf, sizeof(au8SendBuf));
    g_u32SendbytesLen = sizeof(au8SendBuf);

#if (OTA_UPGRADE_FROM_SD)
    memcpy(g_au8NuBL32FileName, au8NuBL32FileName, sizeof(au8NuBL32FileName));
    memcpy(g_au8NuBL33FileName, au8NuBL33FileName, sizeof(au8NuBL33FileName));
#endif
}

/**
  * @brief        Set Reset flag for transfer task
  * @param        None
  * @return       None
  * @details      The function is used to set reset flag for transfer task.
  */
void OTA_API_SetResetFlag(void)
{
    Transfer_SetResetFlag();
}

/**
  * @brief      OTA task routine
  * @param      None
  * @retval     0             Success
  * @retval     others        Failed
  * @details    Transfer task routine
  */
int8_t OTA_API_TaskProcess(void)
{
    /* Process transfer task */
    Transfer_Process();

    return 0;
}

/**
  * @brief        Disconnect transfer connection
  * @param        None
  * @return       None
  * @details      The function is used to disconnect transfer connection.
  */
void OTA_API_TransferConnClose(void)
{
    /* Set disconnect flag for transfer task */
    Transfer_SetDisconnFlag();
}

/**
  * @brief        Send frame data
  * @param[in]    pu8TxBuf        The buffer to send the data
  * @param[in]    u32Len          The data lengths
  * @return       None
  * @details      The function is to write frame data into send buffer to transmit data.
  */
void OTA_API_SendFrame(uint8_t* pu8Buff, uint32_t u32Len)
{
    /* Write data to send buffer */
    Transfer_SendBytes(pu8Buff, u32Len);
}

/**
  * @brief      Get flag of firmware upgrade done
  * @param      None
  * @return     Flag of firmware upgrade done
  * @details    This function is to get flag of firmware upgrade done.
  */
uint8_t OTA_API_GetFwUpgradeDone(void)
{
    return OTA_GetFwUpgradeDone();
}

/**
  * @brief      OTA commands handler
  * @param[in]  pu8Buff        The pointer of packet buffer
  * @param[in]  u32Len         Valind data length
  * @param[in]  u32StartIdx    Valind data index in packet buffer
  * @param[in]  u32ValidLen    Valind data length
  * @return     None
  * @details    OTA commands handler
  */
int8_t OTA_API_RecvCallBack(uint8_t* pu8Buff, uint32_t u32Len, uint32_t u32StartIdx, uint32_t u32ValidLen)
{
    OTA_CallBackHandler(pu8Buff, u32Len, u32StartIdx, u32ValidLen);

    return 0;
}

/**
  * @brief      Init function for any hardware requirements(optional)
  * @param[in]  u32HSI        PLL Output Clock Frequency
  * @return     None
  * @details    This funcfion is to init function for any hardware requirements.
  */
void OTA_API_Init(uint32_t u32HSI)
{
	/* Init some global variables of NuBL2 when now is running NuBL32 firmware. */
    OTA_SysValueInit(u32HSI);

    /* Init hardware for transfer task */
    Transfer_Init();
}

/**
  * @brief      Get page size of flash
  * @param      None
  * @retval     0             Success
  * @retval     others        Failed
  * @details    This function is to get page size of flash.
  */
uint32_t OTA_API_GetFlashPageSize()
{
    return (uint32_t)FMC_FLASH_PAGE_SIZE;
}

/**
  * @brief      Erase flash region
  * @param[in]  u32FlashAddr  Flash address
  * @retval     0             Success
  * @retval     others        Failed
  * @details    This function is to erase flash region
  */
uint8_t OTA_API_EraseFlash(uint32_t u32FlashAddr)
{
    uint8_t u8Status;

//    SYS_UnlockReg();
//    FMC_Open();
    FMC_ENABLE_AP_UPDATE();

    if (FMC_Erase(u32FlashAddr))
        u8Status = STATUS_FAILED;
    else
        u8Status = STATUS_SUCCESS;

//    FMC_DisableAPUpdate();
//    FMC_Close();
//    SYS_LockReg();

    return u8Status;
}

/**
  * @brief      Write flash data
  * @param[in]  u32FlashAddr  Flash address
  * @param[in]  u32Data       data
  * @retval     0             Success
  * @retval     others        Failed
  * @details    This function is to write flash data
  */
uint8_t OTA_API_WriteFlash(uint32_t u32FlashAddr, uint32_t u32Data)
{
    SYS_UnlockReg();
    FMC_Open();
    FMC_ENABLE_AP_UPDATE();

    //printf("FMC_Write(0x%x, 0x%x)\n", u32FlashAddr, u32Data);
    FMC_Write(u32FlashAddr, u32Data);

//    FMC_DISABLE_AP_UPDATE();
//    FMC_Close();
//    SYS_LockReg();

    return STATUS_SUCCESS;
}

/*---------------------------------------------------------*/
/* User Provided RTC Function for FatFs module             */
/*---------------------------------------------------------*/
/* This is a real time clock service to be called from     */
/* FatFs module. Any valid time must be returned even if   */
/* the system does not support an RTC.                     */
/* This function is not required in read-only cfg.         */

unsigned long get_fattime (void)
{
    unsigned long tmr;

    tmr=0x00000;

    return tmr;
}

#if (OTA_UPGRADE_FROM_SD)
/**
  * @brief       IRQ Handler for SDH Interrupt
  * @param       None
  * @return      None
  * @details     The SDH_Process function is used for SDH interrupt handler.
  */
void SDH_Process(void)
{
    uint32_t volatile u32Isr;

    // FMI data abort interrupt
    if (SDH0->GINTSTS & SDH_GINTSTS_DTAIF_Msk)
    {
        /* ResetAllEngine() */
        SDH0->GCTL |= SDH_GCTL_GCTLRST_Msk;
    }

    //----- SD interrupt status
    u32Isr = SDH0->INTSTS;
    if (u32Isr & SDH_INTSTS_BLKDIF_Msk)
    {
        // block down
        g_u8SDDataReadyFlag = TRUE;
        SDH0->INTSTS = SDH_INTSTS_BLKDIF_Msk;
    }

    if (u32Isr & SDH_INTSTS_CDIF_Msk)   // port 0 card detect
    {
        //----- SD interrupt status
        // it is work to delay 50 times for SD_CLK = 200KHz
        {
            int volatile i;         // delay 30 fail, 50 OK
            for (i=0; i<0x500; i++);  // delay to make sure got updated value from REG_SDISR.
            u32Isr = SDH0->INTSTS;
        }

        if (u32Isr & SDH_INTSTS_CDSTS_Msk)
        {
            printf("\n***** card remove !\n");
            SD0.IsCardInsert = FALSE;   // SDISR_CD_Card = 1 means card remove for GPIO mode
            memset(&SD0, 0, sizeof(SDH_INFO_T));
        }
        else
        {
            printf("***** card insert !\n");
            SDH_Open(SDH0, CardDetect_From_GPIO);
            SDH_Probe(SDH0);
        }

        SDH0->INTSTS = SDH_INTSTS_CDIF_Msk;
    }

    // CRC error interrupt
    if (u32Isr & SDH_INTSTS_CRCIF_Msk)
    {
        if (!(u32Isr & SDH_INTSTS_CRC16_Msk))
        {
            //printf("***** ISR sdioIntHandler(): CRC_16 error !\n");
            // handle CRC error
        }
        else if (!(u32Isr & SDH_INTSTS_CRC7_Msk))
        {
            if (!g_u8R3Flag)
            {
                //printf("***** ISR sdioIntHandler(): CRC_7 error !\n");
                // handle CRC error
            }
        }
        SDH0->INTSTS = SDH_INTSTS_CRCIF_Msk;      // clear interrupt flag
    }

    if (u32Isr & SDH_INTSTS_DITOIF_Msk)
    {
        printf("***** ISR: data in timeout !\n");
        SDH0->INTSTS |= SDH_INTSTS_DITOIF_Msk;
    }

    // Response in timeout interrupt
    if (u32Isr & SDH_INTSTS_RTOIF_Msk)
    {
        printf("***** ISR: response in timeout !\n");
        SDH0->INTSTS |= SDH_INTSTS_RTOIF_Msk;
    }
}

/**
  * @brief       IRQ Handler for SDH Interrupt
  * @param       None
  * @return      None
  * @details     The SDH0_IRQHandler is default IRQ of SDH, declared in startup_M2351.s.
  */
void SDH0_IRQHandler(void)
{
    SDH_Process();
}

/**
  * @brief      Init SD Host peripheral
  * @param      None
  * @retval     0           Success
  * @retval     others      Failed
  * @details    This function is used to configure SD Host peripheral.
  */
int8_t OTA_API_SDInit(void)
{
    TCHAR  sd_path[] = { '0', ':', 0 };    /* SD drive started from 0 */
    FRESULT eFresult;

    /* select multi-function pins */
    SYS->GPE_MFPL &= ~(SYS_GPE_MFPL_PE2MFP_Msk | SYS_GPE_MFPL_PE3MFP_Msk | SYS_GPE_MFPL_PE4MFP_Msk | SYS_GPE_MFPL_PE5MFP_Msk |
                       SYS_GPE_MFPL_PE6MFP_Msk | SYS_GPE_MFPL_PE7MFP_Msk);
    SYS->GPD_MFPH &= ~SYS_GPD_MFPH_PD13MFP_Msk;
    SYS->GPE_MFPL |= (SYS_GPE_MFPL_PE2MFP_SD0_DAT0 | SYS_GPE_MFPL_PE3MFP_SD0_DAT1 | SYS_GPE_MFPL_PE4MFP_SD0_DAT2 | SYS_GPE_MFPL_PE5MFP_SD0_DAT3 |
                      SYS_GPE_MFPL_PE6MFP_SD0_CLK | SYS_GPE_MFPL_PE7MFP_SD0_CMD);
    SYS->GPD_MFPH |= SYS_GPD_MFPH_PD13MFP_SD0_nCD;

    //SD_PWR: PF9 - it should be pulled low to enalbed the pull-high resistor for SDIO pins(NuTiny-M2351)
    SYS->GPF_MFPH = (SYS->GPF_MFPH & (~SYS_GPF_MFPH_PF9MFP_Msk));
    //GPIO_SetMode(PF, BIT9, GPIO_MODE_OUTPUT);
    PF->MODE = (PF->MODE & (~GPIO_MODE_MODE9_Msk)) | (GPIO_MODE_OUTPUT << GPIO_MODE_MODE9_Pos);
    PF9 = 0;

    /* Select IP clock source */
    CLK_SetModuleClock(SDH0_MODULE, CLK_CLKSEL0_SDH0SEL_PLL, CLK_CLKDIV0_SDH0(4));
    /* Enable IP clock */
    CLK_EnableModuleClock(SDH0_MODULE);
    /* Enable NVIC SDH0 IRQ */
    NVIC_EnableIRQ(SDH0_IRQn);
    /* Configure FATFS */
    SDH_Open_Disk(SDH0, CardDetect_From_GPIO);
    /* set default path */
    eFresult = f_chdrive(sd_path);

    return (int8_t)eFresult;
}

/**
  * @brief  Open new NuBL3x firmware package file
  * @param[in]  u8NuBLxSel  NuBL3x firmware package selection. Bit0 = 1: NuBL32, Bit1 = 1: NuBL33
  * @retval     0           Success
  * @retval     others      Failed
  * @details    Open new NuBL3x firmware package file from SD card.
  */
uint8_t OTA_API_SDFwPackWriteOpen(uint8_t u8NuBLxSel)
{
    FRESULT eFresult;
    char *ptr;

    /* Get file name of firmware package */
    if (u8NuBLxSel&BIT0)
        ptr = g_au8NuBL32FileName;
    else if (u8NuBLxSel&BIT1)
        ptr = g_au8NuBL33FileName;
    else
        return FR_INVALID_PARAMETER;

    FwPackOpen:
    /* New a file */
    eFresult = f_open(&g_FileObject, ptr, FA_CREATE_NEW);
    if(eFresult != FR_OK)
    {
        if(eFresult == FR_EXIST)
        {
            f_unlink(ptr);
            goto FwPackOpen;
        }
        else
        {
            printf("new NuBL3%d FW package error!(0x%x)\n",((u8NuBLxSel&BIT0)==0)?2:3, eFresult);
        }
    }

    /* Open file for writing */
    eFresult = f_open(&g_FileObject, ptr, FA_WRITE);
    if(eFresult != FR_OK)
    {
        printf("open NuBL3%d FW package error!(0x%x)\n",((u8NuBLxSel&BIT0)==0)?2:3, eFresult);
    }
    return (uint8_t)eFresult;
}

/**
  * @brief  Write file data into SD card
  * @param[in]  pu8Buffer     Pointer of read data buffer
  * @param[in]  u32BufferLen  Data buffer length
  * @retval     0             Success
  * @retval     others        Failed
  * @details    Write file data into SD card.
  */
uint8_t OTA_API_SDWrite(uint8_t * pu8Buffer, uint32_t u32BufferLen)
{
    FRESULT eFresult;
    uint32_t u32ByteWrite;

    /* Write data */
    eFresult = f_write(&g_FileObject, pu8Buffer, u32BufferLen, (UINT *)&u32ByteWrite);
    if(eFresult != FR_OK)
    {
        printf("Write data error!(0x%x)\n", eFresult);
    }
    /* Check writed data lentgh */
    if (u32BufferLen != u32ByteWrite)
    {
        /* Write data length does not match. */
        eFresult = FR_DISK_ERR;
        printf("Write legth error!(0x%x)\n", eFresult);
    }
    return (uint8_t)eFresult;
}

/**
  * @brief  Close NuBL3x firmware package file
  * @param[in]  u8NuBLxSel  NuBL3x firmware package selection. Bit0 = 1: NuBL32, Bit1 = 1: NuBL33
  * @retval     0           Success
  * @retval     others      Failed
  * @details    Close NuBL3x firmware package file from SD card.
  */
uint8_t OTA_API_SDClose(uint8_t u8NuBLxSel)
{
    FRESULT eFresult;
    char *ptr;

    /* Get file name of firmware package */
    if (u8NuBLxSel&BIT0)
        ptr = g_au8NuBL32FileName;
    else if (u8NuBLxSel&BIT1)
        ptr = g_au8NuBL33FileName;
    else
        return FR_INVALID_PARAMETER;

    /* Close firmware package */
    eFresult = f_close(&g_FileObject);
    if(eFresult != FR_OK)
    {
        printf("Close NuBL3%d FW package error!(0x%x)\n",((u8NuBLxSel&BIT0)==0)?2:3, eFresult);
    }

    return (uint8_t)eFresult;
}

/**
  * @brief  Open existed NuBL3x firmware package file
  * @param[in]  u8NuBLxSel  NuBL3x firmware package selection. Bit0 = 1: NuBL32, Bit1 = 1: NuBL33
  * @retval     0           Success
  * @retval     others      Failed
  * @details    Open existed NuBL3x firmware package file from SD card.
  */
uint8_t OTA_API_SDFwPackReadOpen(uint8_t u8NuBLxSel)
{
    FRESULT eFresult;
    char *ptr;

    /* Get file name of firmware package */
    if (u8NuBLxSel&BIT0)
        ptr = g_au8NuBL32FileName;
    else if (u8NuBLxSel&BIT1)
        ptr = g_au8NuBL33FileName;
    else
        return FR_INVALID_PARAMETER;

    /* Open existed firmware package file */
    eFresult = f_open(&g_FileObject, ptr, FA_OPEN_EXISTING | FA_READ);
    if(eFresult != FR_OK)
    {
        printf("open NuBL3%d FW package error!(0x%x)\n",((u8NuBLxSel&BIT0)==0)?2:3, eFresult);
    }
    return (uint8_t)eFresult;
}

/**
  * @brief  Read file data from SD card
  * @param[in]  pu8Buffer     Pointer of read data buffer
  * @param[in]  u32BufferLen  Data buffer length
  * @param[in]  pu32ReadLen   Read data length
  * @retval     0             Success
  * @retval     others        Failed
  * @details    Read file data from SD card.
  */
uint8_t OTA_API_SDRead(uint8_t *pu8Buffer, uint32_t u32BufferLen, uint32_t *pu32ReadLen)
{
    FRESULT eFresult;

    /* Read data */
    eFresult = f_read(&g_FileObject, pu8Buffer, u32BufferLen, pu32ReadLen);
    if(eFresult != FR_OK)
    {
        printf("Read data error!(0x%x)\n", eFresult);
    }

    return (uint8_t)eFresult;
}

#endif

/**
  * @brief      System tick handler for transfer task
  * @param[in]  u32Ticks  System ticks
  * @retval     0         success
  * @retval     Other     fail
  * @details    The function is the system tick handler for transfer task.
  */
uint8_t OTA_API_SysTickProcess(uint32_t u32Ticks)
{
    return Transfer_SysTickProcess(u32Ticks);
}

/**
  * @brief        Read received transfer data
  * @param        None
  * @return       None
  * @details      The function is used to read Rx data from WiFi module.
  */
void OTA_API_WiFiProcess(void)
{
    Transfer_WiFiProcess();
}

/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/
