/****************************************************************************
 * @file     main.c
 * @version  V3.00
 * $Revision: 2 $
 * $Date: 16/10/17 3:05p $
 * @brief    A sample code to show how to debug with semihost message print.
 * @note
 * Copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/

#include <stdio.h>
#include "NuMicro.h"

# if defined (__GNUC__)
extern void initialise_monitor_handles(void);
#endif
/*---------------------------------------------------------------------------------------------------------*/
/* Main Function                                                                                            */
/*---------------------------------------------------------------------------------------------------------*/

void UART0_Init(void)
{
    /* Select IP clock source */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & (~CLK_CLKSEL1_UART0SEL_Msk)) | CLK_CLKSEL1_UART0SEL_HIRC;

    /* Enable IP clock */
    CLK->APBCLK0 |= CLK_APBCLK0_UART0CKEN_Msk;

    UART0->LINE = UART_PARITY_NONE | UART_STOP_BIT_1 | UART_WORD_LEN_8;
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC, 115200);

    /* Set multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & (~(UART0_RXD_PB12_Msk | UART0_TXD_PB13_Msk))) | UART0_RXD_PB12 | UART0_TXD_PB13;
}


int32_t main()
{
    char ch;

    /*
        To enable semihost, user must define "DEBUG_ENABLE_SEMIHOST" constant when buildind sample code.
        If defined DEBUG_ENABLE_SEMIHOST = 1 or 2 and ICE connected, the message will output to ICE.
        if defined DEBUG_ENABLE_SEMIHOST = 1 and ICE off line, the message will re-direct to UART debug port.
        if defined DEBUG_ENABLE_SEMIHOST = 2 and ICE off line, no any debug message output.

        This sample code is used to show how to print message/getchar on IDE debug environment.
        It will echo all input character back on UART #1 of KEIL IDE.

        In KEIL MDK, user need to open "View->Serial Window->UART #1" windows in debug mode.
        In IAR Workbench, user need to open "View->Terminal I/O" in debug mode.

        NOTE1: Hardfault_Handler is used for semihost. User cannot overwrite it when using semihost.
               If it is necessary to process hardfault, user can append code to ProcessHardfault of retarget.c
        NOTE2: Semihost only works with Nuvoton NuLink ICE Dongle in debug mode.
        NOTE3: The message will output to debug port if Nuvoton NuLink ICE Dongle is not connected.


        Semihost On/Off | NuLink Connected | Output Path
       ==============================================================
              1         |         1        |  ICE
              1         |         0        |  UART Debug Port / NULL when DEBUG_ENABLE_SEMIHOST=2
              0         |         1        |  UART Debug Port
              0         |         0        |  UART Debug Port
       --------------------------------------------------------------


    */

    UART0_Init();
# if defined (__GNUC__)
    initialise_monitor_handles();
#endif
    /*
    PC->MODE = GPIO_MODE_OUTPUT << (14*2);
    PC14 = 0;
    */

    printf("\n Start SEMIHOST test: \n");

    while(1)
    {
        /* Get input character */
        ch = getchar();
        /*
        PC14 ^= 1;
        */

        /* Print input character back */
        printf("%c\n", ch);

    }
}

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/



