/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate the methods of detecting reset abnormalities.
 *
 * @copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

void SysTick_Handler(void);
void SYS_Init(void);
void UART0_Init(void);

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
static volatile uint32_t s_u32ResetFlagCount = 0;


/*--------------------------------------------------------------------------------------------------------*/
/*  SysTick IRQ Handler                                                                                   */
/*--------------------------------------------------------------------------------------------------------*/
void SysTick_Handler(void)
{
    if(s_u32ResetFlagCount >= 5)
    {
        printf("Reset abnormalities happened! The same reset event occurs repeatedly for a fixed period of time.\n");

        s_u32ResetFlagCount = 0;
    }
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and HCLK clock divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART module clock source as HIRC and UART module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & (~(UART0_RXD_PB12_Msk | UART0_TXD_PB13_Msk))) | UART0_RXD_PB12 | UART0_TXD_PB13;
}

void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART0 module */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    /* Generate interrupt each 1 s */
    SysTick_Config(SystemCoreClock / 1);

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);
    printf("+-------------------------------------+\n");
    printf("|   SYS Reset Detection Sample Code   |\n");
    printf("+-------------------------------------+\n\n");

    printf("Check UART message to confirm if any reset abnormalities have occurred.\n\n");

    while(1)
    {
        /* Get reset source is from reset pin reset */
        if(SYS_IS_RSTPIN_RST())
        {
            /* Clear reset source flag */
            SYS_CLEAR_RST_SOURCE(SYS_RSTSTS_PINRF_Msk);

            s_u32ResetFlagCount++;
        }

        /* Get reset sources are from Chip reset and CPU reset */
        while(SYS_IS_POR_RST() && SYS_IS_CPU_RST())
        {
            printf("Reset abnormalities happened! Different reset events occur at the same time.\n");
        }
    }
}

/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/
