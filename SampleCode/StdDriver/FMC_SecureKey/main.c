
/******************************************************************************
 * @file     main.c
 * @version  V3.00
 * $Revision: 3 $
 * $Date: 16/10/17 2:06p $
 * @brief    This sample shows how to setup the KPROM key and how to perform KPROM key comparison.
 * @note
 * Copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define PLL_CLOCK       64000000

#define KPMAX_VAL      3               /* KPMAX setting on setup security key (1~15) */
#define KEMAX_VAL      7               /* KEMAX setting on setup security key (1~31) */

static uint32_t  s_au32GoodKey[3] = { 0xe29c0f71, 0x8af051ce, 0xae1f8392 };      /* Assumed correct key in this demo program. */
static uint32_t  s_au32BadKey[3] =  { 0x73422111, 0xac45663a, 0xf46ac321 };      /* Assumed wrong key in this demo program. */

void SYS_Init(void);
void dump_key_status(void);

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

    /* Select UART module clock source as HXT `and UART module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HXT, CLK_CLKDIV0_UART0(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & (~(UART0_RXD_PB12_Msk | UART0_TXD_PB13_Msk))) | UART0_RXD_PB12 | UART0_TXD_PB13;
}

void dump_key_status()
{
    printf("KPKEYSTS: 0x%x\n    ", FMC->KPKEYSTS);    /* FMC_KEYSTS register value */
    printf("%s ", (FMC->KPKEYSTS & FMC_KPKEYSTS_KEYLOCK_Msk) ? "[LOCK]" : "[UNLOCK]");  /* KEYLOCK(FMC_KEY_STS[1] */
    if(FMC->KPKEYSTS & FMC_KPKEYSTS_KEYBUSY_Msk)
        printf("[BUSY] ");                            /* KEYBUSY(FMC_KEYSTS[0]) */
    if(FMC->KPKEYSTS & FMC_KPKEYSTS_KEYMATCH_Msk)
        printf("[KEYMATCH] ");                        /* KEYMATCH(FMC_KEYSTS[2]) */
    if(FMC->KPKEYSTS & FMC_KPKEYSTS_FORBID_Msk)
        printf("[FORBID] ");                          /* FORBID(FMC_KEYSTS[3]) */
    if(FMC->KPKEYSTS & FMC_KPKEYSTS_KEYFLAG_Msk)
        printf("[KEY LOCK] ");                        /* KEYFLAG(FMC_KEYSTS[4]) */
    if(FMC->KPKEYSTS & FMC_KPKEYSTS_CFGFLAG_Msk)
        printf("[CONFIG LOCK] ");                     /* CFGFLAG(FMC_KEYSTS[5]) */

    printf("KPCNT: 0x%x, KPMAX: 0x%x\n", (unsigned)(FMC->KPCNT & FMC_KPCNT_KPCNT_Msk) >> FMC_KPCNT_KPCNT_Pos,    /* KPCNT(FMC_KPCNT[3:0])  */
           (unsigned)((FMC->KPCNT & FMC_KPCNT_KPMAX_Msk) >> FMC_KPCNT_KPMAX_Pos));   /* KPMAX(FMC_KPCNT[11:8]) */

    printf("KPKEYCNT: 0x%x, KEMAX: 0x%x\n", (unsigned)(FMC->KPKEYCNT & FMC_KPKEYCNT_KPKECNT_Msk) >> FMC_KPKEYCNT_KPKECNT_Pos,    /* KPKEYCNT(FMC_KPKEYCNT[5:0])  */
           (unsigned)((FMC->KPKEYCNT & FMC_KPKEYCNT_KPKEMAX_Msk) >> FMC_KPKEYCNT_KPKEMAX_Pos));   /* KEMAX(FMC_KECNT[13:8]) */

    printf("\n\nPress any key to continue...\n");     /* Wait user press any key on UART0 debug console */
    getchar();                                        /* block on gettong any one character from UART0 */
}

int32_t main(void)
{
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

    printf("+-----------------------------------------+\n");
    printf("|      FMC KPROM Key Sample Demo          |\n");
    printf("+-----------------------------------------+\n");

    SYS_UnlockReg();                   /* Unlock protected registers */

    FMC_Open();                        /* Enable FMC ISP function */

    /* Setup a new key */
    if(FMC_SetSPKey(s_au32GoodKey, KPMAX_VAL, KEMAX_VAL, 0, 0) < 0)
    {
        printf("Failed to setup key!\n");   /* error message */
        while(1);                      /* Failed to setup security key. Program aborted. */
    }

    printf("The security key status after key setup:\n");
    dump_key_status();                 /* Dump FMC security key status. */

    FMC_CompareSPKey(s_au32BadKey);         /* Enter a wrong key for key comparison. */
    printf("The security key status after enter a wrong key:\n");
    dump_key_status();                 /* Dump FMC security key status. */

    FMC_CompareSPKey(s_au32BadKey);         /* Enter a wrong key for key comparison. */
    printf("The security key status after enter a wrong key second time:\n");
    dump_key_status();                 /* Dump FMC security key status. */

    FMC_CompareSPKey(s_au32GoodKey);        /* Enter the right key for key comparison. */
    printf("The security key status after enter a good key.\n");
    dump_key_status();                 /* Dump FMC security key status. */

    printf("Erase KPROM key.\n");
    FMC_Erase(FMC_KPROM_BASE);

    printf("Test done.\n");
    while(1);
}
/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/


