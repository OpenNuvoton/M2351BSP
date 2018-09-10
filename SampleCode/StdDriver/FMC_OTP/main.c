/******************************************************************************
 * @file     main.c
 * @version  V3.00
 * $Revision: 3 $
 * $Date: 16/10/17 2:06p $
 * @brief    Demonstrate how to program, read, and lock OTP.
 * @note
 * Copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define PLL_CLOCK       64000000

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
    uint32_t    u32i, u32OtpHw, u32OtpLw;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Open(UART0, 115200);

    printf("+------------------------------------+\n");
    printf("|   M2351 FMC OTP Sample Demo        |\n");
    printf("+------------------------------------+\n");

    SYS_UnlockReg();                   /* Unlock protected registers */

    FMC_Open();                        /* Enable FMC ISP function */

    for(u32i = 0; u32i < FMC_OTP_ENTRY_CNT; u32i++)
    {
        if(FMC_Read_OTP(u32i, &u32OtpLw, &u32OtpHw) != 0)
        {
            printf("Read OTP%d failed!\n", u32i);
            goto lexit;
        }

        if((u32OtpLw == 0xFFFFFFFF) && (u32OtpHw == 0xFFFFFFFF))
        {
            printf("OTP%d is 0xFFFFFFFF-0xFFFFFFFF. It should be a free entry.\n", u32i);
            break;
        }
    }

    if(u32i == FMC_OTP_ENTRY_CNT)
    {
        printf("All OTP entries are used.\n");
        goto lexit;
    }

    printf("Program OTP%d with 0x%x-0x%x...\n", u32i, 0x5A5A0000 | u32i, 0x00005A5A | u32i);

    if(FMC_Write_OTP(u32i, 0x5A5A0000 | u32i, 0x00005A5A | u32i) != 0)
    {
        printf("Failed to program OTP%d!\n", u32i);
        goto lexit;
    }

    if(FMC_Read_OTP(u32i, &u32OtpLw, &u32OtpHw) != 0)
    {
        printf("Read OTP%d failed after programmed!\n", u32i);
        goto lexit;
    }

    printf("Read back OTP%d: 0x%x-0x%x.\n", u32i, u32OtpLw, u32OtpHw);

    if((u32OtpLw != (0x5A5A0000 | u32i)) || (u32OtpHw != (0x00005A5A | u32i)))
    {
        printf("OTP%d value is not matched with programmed value!\n", u32i);
        goto lexit;
    }

    printf("Lock OTP%d...\n", u32i);

    if(FMC_Lock_OTP(u32i) != 0)
    {
        printf("Failed to lock OTP%d!\n", u32i);
        goto lexit;
    }

    if(FMC_Read_OTP(u32i, &u32OtpLw, &u32OtpHw) != 0)
    {
        printf("Read OTP%d failed after programmed!\n", u32i);
        goto lexit;
    }

    printf("Read OTP%d after locked: 0x%x-0x%x.\n", u32i, u32OtpLw, u32OtpHw);

    if((u32OtpLw != (0x5A5A0000 | u32i)) || (u32OtpHw != (0x00005A5A | u32i)))
    {
        printf("OTP%d value is incorrect after locked!\n", u32i);
        goto lexit;
    }

    printf("OTP demo done.\n");

lexit:
    FMC_Close();                       /* Disable FMC ISP function */
    SYS_LockReg();                     /* Lock protected registers */

    while(1);
}
/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/


