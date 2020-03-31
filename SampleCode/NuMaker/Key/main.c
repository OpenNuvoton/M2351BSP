/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    A simple key demo for NuMaker board.
 *
 *
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "M2351.h"

#define LED_INIT()  (PA->MODE = (PA->MODE &(~(0xful << 10*2))) | (0x5ul << 10 * 2))
#define LED_YELLOW  PA10
#define LED_RED     PA11

void SYS_Init(void);
void UART0_Init(void);

void SYS_Init(void)
{

    /* Enable PLL */
    CLK->PLLCTL = CLK_PLLCTL_128MHz_HIRC;

    /* Waiting for PLL stable */
    while((CLK->STATUS & CLK_STATUS_PLLSTB_Msk) == 0);

    /* Set HCLK divider to 2 */
    CLK->CLKDIV0 = (CLK->CLKDIV0 & (~CLK_CLKDIV0_HCLKDIV_Msk)) | 1;

    /* Switch HCLK clock source to PLL */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLKSEL_Msk)) | CLK_CLKSEL0_HCLKSEL_PLL;

    /* Select IP clock source */
    CLK->CLKSEL1 = CLK_CLKSEL1_UART0SEL_HIRC;

    /* Enable IP clock */
    CLK->APBCLK0 |= CLK_APBCLK0_UART0CKEN_Msk | CLK_APBCLK0_TMR0CKEN_Msk;


    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    //SystemCoreClockUpdate();
    PllClock        = 128000000;           // PLL
    SystemCoreClock = 128000000 / 2;       // HCLK
    CyclesPerUs     = 64000000 / 1000000;  // For SYS_SysTickDelay()

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & (~(UART0_RXD_PB12_Msk | UART0_TXD_PB13_Msk))) | UART0_RXD_PB12 | UART0_TXD_PB13;

}

void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    UART0->LINE = UART_PARITY_NONE | UART_STOP_BIT_1 | UART_WORD_LEN_8;
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC, 115200);

}



int main()
{
    SYS_UnlockReg();

    SYS_Init();
    UART0_Init();

    printf("\n");
    printf("+------------------------------------------------------------------+\n");
    printf("|              Simple Key Demo                                     |\n");
    printf("+------------------------------------------------------------------+\n");

    /* Init GPIO for LED toggle. */
    LED_INIT();
    LED_YELLOW = 1;
    LED_RED = 1;

    /* Enable debounce of PB0~1 */
    PB->DBEN = 0x3;
    PB->DBCTL |= GPIO_DBCTL_DBCLKSRC_Msk | 0x3;


    LED_YELLOW = 1;
    LED_RED = 0;
    while(1)
    {
        CLK_SysTickLongDelay(200000);

        /* Toggle LED */
        LED_YELLOW ^= 1;
        LED_RED ^= 1;


        if(PB0 == 0)
        {
            printf("SW2\n");
            LED_YELLOW = 0;
            LED_RED = 1;
            while(PB0 == 0);
        }

        if(PB1 == 0)
        {
            printf("SW3\n");
            LED_YELLOW = 1;
            LED_RED = 0;
            while(PB1 == 0);
        }

    }


}

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
