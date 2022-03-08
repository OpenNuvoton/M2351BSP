/******************************************************************************
 * @file     main.c
 * @version  V3.00
 * $Revision: 3 $
 * $Date: 16/10/17 2:06p $
 * @brief    Demonstrate how to use FMC Read-All-One ISP command to verify APROM/LDROM pages are all 0xFFFFFFFF or not.
 * @note
 * Copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define PLL_CLOCK       64000000

void SYS_Init(void);

void SYS_Init(void)
{
    /* Set PF multi-function pins for XT1_OUT(PF.2) and XT1_IN(PF.3) */
    SYS->GPF_MFPL = (SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF2MFP_Msk)) | SYS_GPF_MFPL_PF2MFP_XT1_OUT;
    SYS->GPF_MFPL = (SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF3MFP_Msk)) | SYS_GPF_MFPL_PF3MFP_XT1_IN;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and HCLK source divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable HXT clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Wait for HXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Enable PLL */
    CLK->PLLCTL = CLK_PLLCTL_128MHz_HIRC;

    /* Waiting for PLL stable */
    CLK_WaitClockReady(CLK_STATUS_PLLSTB_Msk);

    /* Select HCLK clock source as PLL and HCLK source divider as 2 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_PLL, CLK_CLKDIV0_HCLK(2));

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART module clock source as HXT and UART module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HXT, CLK_CLKDIV0_UART0(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & (~(UART0_RXD_PB12_Msk | UART0_TXD_PB13_Msk))) | UART0_RXD_PB12 | UART0_TXD_PB13;
}

int32_t main(void)
{
    uint32_t    u32Ret;                   /* return value */
    uint32_t    u32Data;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Open(UART0, 115200);

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("+------------------------------------------+\n");
    printf("|        FMC_ReadAllOne Sample Demo        |\n");
    printf("+------------------------------------------+\n");

    SYS_UnlockReg();                   /* Unlock protected registers */

    FMC_Open();                        /* Enable FMC ISP function */

    u32Data = FMC_ReadCID();           /* Read company ID. Should be 0xDA. */
    if (g_FMC_i32ErrCode != 0)
    {
        printf("FMC_ReadCID failed!\n");
        goto lexit;
    }
    printf("  Company ID ............................ [0x%08x]\n", u32Data);

    u32Data = FMC_ReadPID();           /* Read product ID. */
    if (g_FMC_i32ErrCode != 0)
    {
        printf("FMC_ReadPID failed!\n");
        goto lexit;
    }
    printf("  Product ID ............................ [0x%08x]\n", u32Data);

    /* Read User Configuration CONFIG0 */
    printf("  User Config 0 ......................... [0x%08x]\n", FMC_Read(FMC_CONFIG_BASE));
    if (g_FMC_i32ErrCode != 0)
    {
        printf("FMC_Read(FMC_CONFIG_BASE) failed!\n");
        goto lexit;
    }

    /* Read User Configuration CONFIG1 */
    printf("  User Config 1 ......................... [0x%08x]\n", FMC_Read(FMC_CONFIG_BASE + 4));
    if (g_FMC_i32ErrCode != 0)
    {
        printf("FMC_Read(FMC_CONFIG_BASE+4) failed!\n");
        goto lexit;
    }

    FMC_ENABLE_LD_UPDATE();            /* Enable LDROM update. */

    if (FMC_Erase(FMC_LDROM_BASE) != 0)     /* Erase LDROM page 0. */
    {
        printf("FMC_Erase(FMC_LDROM_BASE) failed!\n");
        goto lexit;
    }

    /* Run and check flash contents are all 0xFFFFFFFF. */
    u32Ret = FMC_CheckAllOne(FMC_LDROM_BASE, FMC_FLASH_PAGE_SIZE);
    if (g_FMC_i32ErrCode != 0)
    {
        printf("FMC_CheckAllOne failed!\n");
        goto lexit;
    }

    if(u32Ret == READ_ALLONE_YES)                   /* return value READ_ALLONE_YES means all flash contents are 0xFFFFFFFF */
        printf("READ_ALLONE_YES success.\n");    /* FMC_CheckAllOne() READ_ALLONE_YES passed on LDROM page 0. */
    else
        printf("READ_ALLONE_YES failed!\n");     /* FMC_CheckAllOne() READ_ALLONE_YES failed on LDROM page 0. */

    if (FMC_Write(FMC_LDROM_BASE, 0) != 0)       /* program a 0 to LDROM to make it not all 0xFFFFFFFF. */
    {
        printf("FMC_Write FMC_LDROM_BASE failed!\n");
        goto lexit;
    }

    /* Run and check flash contents are not all 0xFFFFFFFF. */
    u32Ret = FMC_CheckAllOne(FMC_LDROM_BASE, FMC_FLASH_PAGE_SIZE);
    if (g_FMC_i32ErrCode != 0)
    {
        printf("FMC_CheckAllOne failed!\n");
        goto lexit;
    }
    if(u32Ret == READ_ALLONE_NOT)
        printf("READ_ALLONE_NOT success.\n");   /* FMC_CheckAllOne() READ_ALLONE_NOT passed on LDROM page 0. */
    else
        printf("READ_ALLONE_NOT failed!\n");    /* FMC_CheckAllOne() READ_ALLONE_NOT failed on LDROM page 0. */

    printf("\nFMC Read-All-One test done.\n");

lexit:
    FMC_Close();                       /* Disable FMC ISP function */

    SYS_LockReg();                     /* Lock protected registers */

    while(1);
}
/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/


