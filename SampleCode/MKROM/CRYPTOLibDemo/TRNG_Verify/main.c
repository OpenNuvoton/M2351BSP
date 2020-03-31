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
    DEBUG_PORT->LINE = UART_WORD_LEN_8 | UART_PARITY_EVEN | UART_STOP_BIT_1;
}

/*----------------------------------------------------------------------------
  Main function
 *----------------------------------------------------------------------------*/
int main(void)
{
    int32_t  i,j;
    XTRNG_T rng;
    uint32_t au32RandVal[8];
    uint8_t *pu8Buf;
    uint32_t u32Sum, u32Val;
    
    
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART for printf */
    UART_Init();

    /* Initial TRNG */
    XTRNG_RandomInit(&rng, XTRNG_PRNG | XTRNG_LIRC32K);
    
    pu8Buf = (uint8_t *)au32RandVal;
    
    /* Waiting for press any key */
    printf("Press any key to start ...\n");
    getchar();
    
    printf("//-- START --//\n");
    
    /* Checksum is used to double check if data transfer ok */
    u32Sum = 0;
    
    /* 512000 bits * 10 blocks is required for NIST SP800-22 analysis */
    /* Total generated bits are 20001*32*8 = 5120256 bits */
    for (i = 0; i < 20001; i++)
    {
        XTRNG_Random(&rng, pu8Buf, 8*4);
        
        for(j=0;j<8;j++)
        {
            u32Val = au32RandVal[j];
            u32Sum += u32Val;
            printf("%08x", u32Val);
        }
        
        /* A short delay could let data trasnfer be more stable for such husge data counts */
        if((i&0xff) == 0)
        {
            CLK_SysTickDelay(1000);
        }
        
    }
    printf("\n//-- END --//\n");
    printf("Random Check Sum = %u\n", u32Sum);

    while(1) {__WFI();}
}

/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/
