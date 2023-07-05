/******************************************************************************
 * @file     main.c
 * @version  V3.00
 * $Revision: 3 $
 * $Date: 16/10/17 2:06p $
 * @brief    Show how to config/erase XOM region.
 * @note
 * Copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "xomapi.h"


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

    int32_t NumArray[10] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();


    /* Configure UART: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Open(UART0, 115200);

    /*
        This sample code is used to show how to build an XOM library.

        The location of XOM region is defined by scatter file: xom_scatter.scf
        The API header file is xomapi.h
        The XOM functions are implemented in xom.c

        This project is only used to build code for XOM region and test its functions.
        To enable XOM region, please use "NuMicro ICP Programming Tool".



        Example flow:
        1. Build XOMCode and test XOM functions
        2. Open "NuMicro ICP Programming Tool" to enable XOM region and
           according to xom_scatter.scf settings.
        3. Test XOM function with XOM enabled again.
        4. Review xomlib.c(Keil), xomlibIAR.c(IAR) and .\lib\xomlib.h
           to make sure all XOM function pointers are included.
        5. (Keil) Build final XOMCode. XOMAddr.exe will be executed to update
                  function pointer addresses after built.
           (IAR) Update xomlibIAR.c function pointers manually.
        6. Build XOMLib project to generate xomlib.lib or xomlib.a.
           It includes function pointers for XOM.
           The library (xomlib.lib or xomlib.a) and header (xomlib.h) is located at lib directory.
        7. Pass xomlib library & xomlib.h to the people who will call the functions in XOM.
    */

    printf("\n\n");
    printf("+----------------------------------------+\n");
    printf("|      FMC XOM Library Build Example     |\n");
    printf("+----------------------------------------+\n");

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable FMC ISP function and enable APROM active */
    FMC_Open();
    FMC_ENABLE_AP_UPDATE();

    /* Read User Configuration */
    printf("\n");
    printf("XOM Status = 0x%X\n", FMC->XOMSTS);

    /* Run XOM function */
    printf("\n");
    printf(" 100 + 200 = %d\n", XOM_Add(100, 200));
    printf(" 500 - 100 = %d\n", XOM_Sub(500, 100));
    printf(" 200 * 100 = %d\n", XOM_Mul(200, 100));
    printf("1000 / 250 = %d\n", XOM_Div(1000, 250));

    printf("\n");
    printf("1 + 2 +..+ 10 = %d\n", XOM_Sum(NumArray, 10));

    while(1);
}

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/


