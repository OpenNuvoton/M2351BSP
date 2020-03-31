/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Use MKROM Crypto library to generate random numbers.
 *
 * Copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"


#define BYTE_COUNT      32  /* Number of byte counts */

void SYS_Init(void);
void UART_Init(void);

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Set PLL frequency */
    CLK->PLLCTL = CLK_PLLCTL_128MHz_HIRC;

    /* Waiting for PLL stable */
    CLK_WaitClockReady(CLK_STATUS_PLLSTB_Msk);

    /* Select HCLK clock source as PLL and HCLK source divider as 2 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_PLL, CLK_CLKDIV0_HCLK(2));

    /* Set SysTick source to HCLK/2*/
    CLK_SetSysTickClockSrc(CLK_CLKSEL0_STCLKSEL_HCLK_DIV2);
    
    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Enable CRYPTO module clock */
    CLK_EnableModuleClock(CRPT_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & (~(UART0_RXD_PB12_Msk | UART0_TXD_PB13_Msk))) | UART0_RXD_PB12 | UART0_TXD_PB13;
}

void UART_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure UART and set UART Baudrate */
    DEBUG_PORT->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC, 115200);
    DEBUG_PORT->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}

/*----------------------------------------------------------------------------
  Main function
 *----------------------------------------------------------------------------*/
int main(void)
{
    int32_t  i;
    XTRNG_T rng;
    uint32_t au32RandVal[BYTE_COUNT/4];
    uint8_t *pu8Buf;
    
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART for printf */
    UART_Init();

    printf("\n\n");
    printf("+------------------------------------------+\n");
    printf("|    True Random Number Generation Demo    |\n");
    printf("+------------------------------------------+\n");
    printf("\n");

    
    /* Initial TRNG */
    XTRNG_RandomInit(&rng, XTRNG_PRNG | XTRNG_LIRC32K);
    
    pu8Buf = (uint8_t *)au32RandVal;
    
    /* Generate random number 1 */
    XTRNG_Random(&rng, pu8Buf, BYTE_COUNT);
    
    printf("Random numbers 1:\n");
    for(i=0; i<(BYTE_COUNT/4); i++)
    {
        printf("  0x%08x", au32RandVal[i]);
    }
    printf("\n");


    /* Generate random number 2 */
    XTRNG_Random(&rng, pu8Buf, BYTE_COUNT);
    
    printf("Random numbers 2:\n");
    for(i=0; i<(BYTE_COUNT/4); i++)
    {
        printf("  0x%08x", au32RandVal[i]);
    }
    printf("\n");
        

    /* Generate random number 3 */
    XTRNG_Random(&rng, pu8Buf, BYTE_COUNT);
    
    printf("Random numbers 3:\n");
    for(i=0; i<(BYTE_COUNT/4); i++)
    {
        printf("  0x%08x", au32RandVal[i]);
    }
    printf("\n");
        
    while(1) {}
}

/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/
