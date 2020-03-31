/**************************************************************************//**
 * @file     main_ns.c
 * @version  V1.00
 * @brief    Show MKROM Non-secure API in Non-secure region.
 *
 * Copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <arm_cmse.h>
#include "NuMicro.h"


/*----------------------------------------------------------------------------
  NonSecure Callable Functions from Secure Region
 *----------------------------------------------------------------------------*/
extern uint32_t GetSystemCoreClock(void);


static volatile uint32_t s_au32FlashData[FMC_FLASH_PAGE_SIZE / 4] = {0}; /* Initial 2048 byte data */
static volatile uint32_t s_au32GetData[FMC_FLASH_PAGE_SIZE / 4] = {0};

/*----------------------------------------------------------------------------
  Main function
 *----------------------------------------------------------------------------*/
int main(void)
{
    uint32_t i;
    uint32_t u32Addr, u32Status, u32Data;

    
    /* Call secure API to get system core clock */
    SystemCoreClock = GetSystemCoreClock();
    
    printf("\n\n");
    printf("System core clock = %d.\n", SystemCoreClock);
    printf("+------------------------------------------------------------+\n");
    printf("|    MKROM Non-secure API Sample Code in Non-secure Region   |\n");
    printf("+------------------------------------------------------------+\n\n");


    /* Show basic chip information */
    printf("[Basic chip info]:\n");
    printf("Maximum APROM flash size = %d bytes.\n", BL_EnableFMC());
    printf("Non-secure flash base = %d (0x%08x).\n", BL_GetNSBoundary(), BL_GetNSBoundary());
    printf("PID:    0x%08x\n", BL_ReadPID());
    printf("UID-0:  0x%08x\n", BL_ReadUID(0));
    printf("UID-1:  0x%08x\n", BL_ReadUID(1));
    printf("UID-2:  0x%08x\n", BL_ReadUID(2));
    printf("UCID-0: 0x%08x\n", BL_ReadUCID(0));
    printf("UCID-1: 0x%08x\n", BL_ReadUCID(1));
    printf("UCID-2: 0x%08x\n", BL_ReadUCID(2));
    printf("UCID-3: 0x%08x\n", BL_ReadUCID(3));
    printf("\n");


    /* Check data on Non-secure 0x7_0000 */
    /* Only support Non-secure flash access */
    u32Addr = 0x10070000UL;


    /* BL_FlashRead API */
    u32Data = BL_FlashRead(u32Addr);
    printf("Check Non-secure 0x%08x data is 0x%x ... ", u32Addr, u32Data);
    if(u32Data == 0x12345678UL)
    {
        printf("Pass.\n");
    }
    else
    {
        printf("Fail!\n");
        while(1) {}
    }

    /* Check page data are not all one by BL_CheckFlashAllOne */
    printf("Check [0x%08x] one page are not all one ... ", u32Addr);
    u32Status = BL_CheckFlashAllOne(u32Addr, FMC_FLASH_PAGE_SIZE);
    if(u32Status == BL_FLASH_NOT_ALLONE)
    {
        printf("Pass.\n");
    }
    else
    {
        printf("Fail! (0x%0x)\n", u32Status);
        while(1) {}
    }

    
    /* Page erase then check page data should be all one
        by BL_FlashPageErase and BL_CheckFlashAllOne */
    printf("Page Erase [0x%08x] ... ", u32Addr);
    if(BL_FlashPageErase(u32Addr) == 0)
    {
        printf("Ok.\n");
    }
    else
    {
        printf("Fail!\n");
        while(1) {}
    }
    printf("Check [0x%08x] one page are all one ... ", u32Addr);
    u32Status = BL_CheckFlashAllOne(u32Addr, FMC_FLASH_PAGE_SIZE);
    if(u32Status == BL_FLASH_ALLONE)
    {
        printf("Pass.\n");
    }
    else
    {
        printf("Fail! (0x%08x)\n", u32Status);
        while(1) {}
    }

    
    /* Program/Read Non-secure flash by single read/write and multi-read/write API */
    printf("\n*** Perform flash Read/Write/Multi-Read/Multi-Write API ***\n");
    for(i = 0; i < (FMC_FLASH_PAGE_SIZE / 4); i++)
    {
        s_au32FlashData[i] = 0xA5A50000UL + i;
    }
    /* BL_FlashMultiWrite first 1024 bytes */
    printf("Multi-Write [0x%08x] 1024 bytes ... ", u32Addr);
    if(BL_FlashMultiWrite(u32Addr, (uint32_t *)(uint32_t)s_au32FlashData, (FMC_FLASH_PAGE_SIZE / 2)) == 0)
    {
        printf("Ok.\n");
    }
    else
    {
        printf("Fail! (0x%08x)\n", u32Status);
        while(1) {}
    }
    /* BL_FlashWrite last 1024 bytes */
    printf("Single Write [0x%08x] 1024 bytes ... ", (uint32_t)(u32Addr + (FMC_FLASH_PAGE_SIZE / 2)));
    for(i = 0; i < ((FMC_FLASH_PAGE_SIZE / 4) / 2); i++)
    {
        if(BL_FlashWrite(u32Addr + (FMC_FLASH_PAGE_SIZE / 2) + (i * 4), s_au32FlashData[((FMC_FLASH_PAGE_SIZE / 4) / 2) + i]) != 0)
        {
            printf("Fail! (0x%0x: 0x%08x)\n\n", (uint32_t)(u32Addr + (FMC_FLASH_PAGE_SIZE / 2) + (i * 4)), (uint32_t)(s_au32FlashData[((FMC_FLASH_PAGE_SIZE / 4) / 2) + i]));
            while(1) {}
        }
    }
    printf("Ok.\n");

    
    /* BL_FlashMultiRead one page data */
    printf("Multi-Read [0x%08x] one page ... ", u32Addr);
    if(BL_FlashMultiRead(u32Addr, (uint32_t *)(uint32_t)s_au32GetData, FMC_FLASH_PAGE_SIZE) == 0)
    {
        printf("Ok.\n");
    }
    else
    {
        printf("Fail! (0x%08x)\n", u32Status);
        while(1) {}
    }
    /* BL_FlashRead one page data and compare data */
    printf("Single read then compare all data ... ");
    for(i = 0; i < (FMC_FLASH_PAGE_SIZE / 4); i++)
    {
        uint32_t data_tmp;
        if(BL_FlashRead(u32Addr + (i * 4)) != (0xA5A50000UL + i))
        {
            printf("Fail! (0x%08x: 0x%08x)\n", (u32Addr + (i * 4)), BL_FlashRead(u32Addr + (i * 4)));
            while(1) {}
        }
        data_tmp = s_au32FlashData[i];
        if(data_tmp != s_au32GetData[i])
        {
            printf("Fail! (0x%08x. W:0x%08x, R:0x%08x)\n", (u32Addr + (i * 4)), data_tmp, s_au32GetData[i]);
            while(1) {}
        }
    }
    printf("Pass.\n");

    printf("\n[Non-secure code call MKROM Non-secure API sample code done.\n\n");

    while(1) {}
}

/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/
