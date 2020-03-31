/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Show Crypto IP ECC P-192 key generation function.
 *
 * @note
 * Copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

#define KEY_LENGTH          192          /* Select ECC P-192 curve, 192-bits key length */

static char d[]  = "e5ce89a34adddf25ff3bf1ffe6803f57d0220de3118798ea";    /* private key */
static char Qx[] = "8abf7b3ceb2b02438af19543d3e5b1d573fa9ac60085840f";    /* expected answer: public key 1 */
static char Qy[] = "a87f80182dcd56a6a061f81f7da393e7cffd5e0738c6b245";    /* expected answer: public key 2 */

static char gKey1[168], gKey2[168];             /* temporary buffer used to keep output public keys */


void CRPT_IRQHandler(void);
void SYS_Init(void);
void DEBUG_PORT_Init(void);

void CRPT_IRQHandler()
{
    ECC_DriverISR(CRPT);
}

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
    CLK->CLKSEL3 = CLK_CLKSEL3_UART5SEL_HIRC;

    /* Enable IP clock */
    CLK->AHBCLK  |= CLK_AHBCLK_CRPTCKEN_Msk;
    CLK->APBCLK0 |= CLK_APBCLK0_UART0CKEN_Msk | CLK_APBCLK0_TMR0CKEN_Msk | CLK_APBCLK0_UART5CKEN_Msk;


    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    //SystemCoreClockUpdate();
    PllClock        = 128000000;           // PLL
    SystemCoreClock = 128000000 / 1;       // HCLK
    CyclesPerUs     = 64000000 / 1000000;  // For SYS_SysTickDelay()

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & (~(UART0_RXD_PB12_Msk | UART0_TXD_PB13_Msk))) | UART0_RXD_PB12 | UART0_TXD_PB13;


}

void DEBUG_PORT_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    DEBUG_PORT->LINE = UART_PARITY_NONE | UART_STOP_BIT_1 | UART_WORD_LEN_8;
    DEBUG_PORT->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC, 115200);

}
/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    int32_t i;

    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 for printf */
    DEBUG_PORT_Init();

    printf("+---------------------------------------------+\n");
    printf("|   Crypto ECC Public Key Generation Demo     |\n");
    printf("+---------------------------------------------+\n");

    /* Enable ECC interrupt */
    NVIC_EnableIRQ(CRPT_IRQn);
    ECC_ENABLE_INT(CRPT);

    /* Generate public key from private key d */
    if(ECC_GeneratePublicKey(CRPT, CURVE_P_192, d, gKey1, gKey2) < 0)
    {
        printf("ECC key generation failed!!\n");
        while(1);
    }
    
    /* Verify public key 1 */
    if(memcmp(Qx, gKey1, KEY_LENGTH / 8))
    {

        printf("Public key 1 [%s] is not matched with expected [%s]!\n", gKey1, Qx);

        if(memcmp(Qx, gKey1, KEY_LENGTH / 8) == 0)
            printf("PASS.\n");
        else
            printf("Error !!\n");


        for(i = 0; i < KEY_LENGTH / 8; i++)
        {
            if(Qx[i] != gKey1[i])
                printf("\n%d - 0x%x 0x%x\n", i, Qx[i], gKey1[i]);
        }
        while(1);
    }

    /* Verify public key 2 */
    if(memcmp(Qy, gKey2, KEY_LENGTH / 8))
    {
        printf("Public key 2 [%s] is not matched with expected [%s]!\n", gKey2, Qy);
        while(1);
    }

    printf("ECC key compared OK.\n");

    while(1);
}



