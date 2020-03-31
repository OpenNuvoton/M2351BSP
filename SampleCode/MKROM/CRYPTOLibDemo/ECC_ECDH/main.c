/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Use MKROM Crypto library to demonstrate how to calculate share key by A private key and B private key.
 *
 * Copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"


#define ECC_CURVE_TYPE      CURVE_P_256
#define ECC_KEY_SIZE        256     /* Select ECC P-256 curve, 256-bits key length */

#define B2C(c)    (((uint8_t)(c)<10)?((uint8_t)(c)+'0'):((uint8_t)(c)-10+'a'))

static char d1[68];                        /* private key */
static char d2[68];                        /* private key */
static char Qx[68], Qy[68];                /* temporary buffer used to keep output public keys */
static char Qx2[68], Qy2[68];              /* temporary buffer used to keep output public keys */
static char k[68];                         /* random integer k form [1, n-1]                */
static char k2[68];                        /* random integer k form [1, n-1]                */

void GenPrivateKey(XTRNG_T *rng, char *d, int32_t i32NBits);
void SYS_Init(void);
void UART_Init(void);

void GenPrivateKey(XTRNG_T *rng, char *d, int32_t i32NBits)
{
    int32_t i, j;
    uint8_t au8r[(ECC_KEY_SIZE + 7) / 8];

    do
    {
        /* Generate random number for private key */
        XTRNG_Random(rng, au8r, (uint32_t)((i32NBits + 7) / 8));

        for(i = 0, j = 0; i < (i32NBits + 7) / 8; i++)
        {
            d[j++] = B2C(au8r[i] & 0xf);
            d[j++] = B2C(au8r[i] >> 4);
        }
        d[(i32NBits + 3) / 4] = 0;
        //printf("Private key = %s\n", d);

        /* Check if the private key valid */
        if(XECC_IsPrivateKeyValid(XCRPT, ECC_CURVE_TYPE, d))
        {
            //printf("Private key check ok\n");
            break;
        }
        else
        {
            // Decrease 1 bit and try again
            d[(i32NBits + 2) / 4] = 0;
            if(XECC_IsPrivateKeyValid(XCRPT, ECC_CURVE_TYPE, d))
            {
                //printf("Private key check ok\n");
                if(((i32NBits & 0x3) != 0) && (((i32NBits - 1) & 0x3) == 0))
                {
                    // Need padding 1 nibble back
                    j = (i32NBits + 2) / 4;
                    for(i = j; i > 0; i--)
                        d[i] = d[i - 1];
                    d[i + 1] = 0;
                    d[0] = '0';
                }
                break;
            }
            else
            {
                /* Invalid key */
                //printf("Current private key is not valid. Need a new one.\n");
            }
        }
    }
    while(1);
}

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
    XTRNG_T rng;
    uint32_t u32Ticks;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART for printf */
    UART_Init();

    printf("\n\n");
    printf("+----------------------------------+\n");
    printf("|    Crypto ECC P-256 ECDH Demo    |\n");
    printf("+----------------------------------+\n");
    printf("\n");

    
    ECC_ENABLE_INT(CRPT);

    /* Initial TRNG */
    XTRNG_RandomInit(&rng, XTRNG_PRNG | XTRNG_LIRC32K);

//------------------------------------------------------------------------
    // Generate a private key A
    GenPrivateKey(&rng, d1, ECC_KEY_SIZE);
    printf("Private key A = %s\n", d1);

    // Generate public Key A
    /* Reset SysTick to measure time */
    SysTick->VAL = 0;
    if(XECC_GeneratePublicKey(XCRPT, ECC_CURVE_TYPE, d1, Qx, Qy) < 0)
    {
        printf("ECC key generation failed!!\n");
        while(1) {}
    }
    u32Ticks = 0xffffff - SysTick->VAL;
    printf("Pub Key    Ax = %s\n", Qx);
    printf("Pub Key    Ay = %s\n", Qy);
    printf("Elapsed time: %d.%d ms\n\n", u32Ticks / CyclesPerUs / 1000, u32Ticks / CyclesPerUs % 1000);

//------------------------------------------------------------------------
    // Generate a private key B
    GenPrivateKey(&rng, d2, ECC_KEY_SIZE);
    printf("Private key  B = %s\n", d2);

    // Generate public Key B
    /* Reset SysTick to measure time */
    SysTick->VAL = 0;
    if(XECC_GeneratePublicKey(XCRPT, ECC_CURVE_TYPE, d2, Qx2, Qy2) < 0)
    {
        printf("ECC key generation failed!!\n");
        while(1) {}
    }
    u32Ticks = 0xffffff - SysTick->VAL;
    printf("Pub Key     Bx = %s\n", Qx2);
    printf("Pub Key     By = %s\n", Qy2);
    printf("Elapsed time: %d.%d ms\n\n", u32Ticks / CyclesPerUs / 1000, u32Ticks / CyclesPerUs % 1000);

//------------------------------------------------------------------------
    // Calcualte Share Key by private key A and publick key B
    if(XECC_GenerateSecretZ(XCRPT, ECC_CURVE_TYPE, d1, Qx2, Qy2, k) < 0)
    {
        printf("ECC ECDH share key calculation fail!!\n");
        while(1) {}
    }
    printf("Share key calculated by A = %s\n", k);

    // Calcualte Share Key by private key B and publick key A
    if(XECC_GenerateSecretZ(XCRPT, ECC_CURVE_TYPE, d2, Qx, Qy, k2) < 0)
    {
        printf("ECC ECDH share key calculation fail!!\n");
        while(1) {}
    }
    printf("Share key calculated by B = %s\n", k2);

    if(strcmp(k, k2) != 0)
    {
        printf("Share key mismatch!!\n");
    }
    else
    {
        printf("Share key match. Test OK\n");
    }

    while(1) {}
}

/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/
