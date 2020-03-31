/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Use MKROM Crypto library to demonstrate the ECC P-256 signature generation and verification.
 *
 * Copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"


#define ECC_CURVE_TYPE      CURVE_P_256
#define ECC_KEY_SIZE        256     /* Select ECC P-256 curve, 256-bits key length */

static char d[68];                         /* private key */
static char Qx[68], Qy[68];                /* temporary buffer used to keep output public keys */
static char k[68];                         /* random integer k form [1, n-1]                */
static char R[68], S[68];                  /* temporary buffer used to keep digital signature (R,S) pair */
static char msg[] = "This is a message. It could be encypted.";

uint8_t Byte2Char(uint8_t c);
void SYS_Init(void);
void UART_Init(void);

uint8_t Byte2Char(uint8_t c)
{
    if(c < 10)
        return (c + '0');
    if(c < 16)
        return (c - 10 + 'a');

    return 0;
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
    int32_t i, j, m;
    int32_t i32NBits, i32Err;
    XTRNG_T rng;
    uint8_t au8r[ECC_KEY_SIZE / 8];
    uint32_t u32Ticks;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART for printf */
    UART_Init();

    printf("\n\n");
    printf("+-------------------------------------------------------------------+\n");
    printf("|    Crypto ECC P-256 Signature generation and verification Demo    |\n");
    printf("+-------------------------------------------------------------------+\n");
    printf("\n");

    
    ECC_ENABLE_INT(CRPT);

    i32NBits = ECC_KEY_SIZE;

    /* Initial TRNG */
    XTRNG_RandomInit(&rng, XTRNG_PRNG | XTRNG_LIRC32K);

    do
    {
        /* Generate random number for private key */
        XTRNG_Random(&rng, au8r, (uint32_t)(i32NBits / 8));

        for(i = 0, j = 0; i < i32NBits / 8; i++)
        {
            d[j++] = Byte2Char(au8r[i] & 0xf);
            d[j++] = Byte2Char(au8r[i] >> 4);
        }
        d[j] = 0; // NULL end

        printf("Private key = %s\n", d);

        /* Check if the private key valid */
        if(XECC_IsPrivateKeyValid(XCRPT, ECC_CURVE_TYPE, d))
        {
            //printf("Private key check ok\n");
            break;
        }
        else
        {
            /* Invalid key */
            printf("Current private key is not valid. Need a new one.\n");
        }
    }
    while(1);

    /* Reset SysTick to measure time */
    SysTick->VAL = 0;
    /* Generate public */
    if(XECC_GeneratePublicKey(XCRPT, ECC_CURVE_TYPE, d, Qx, Qy) < 0)
    {
        printf("ECC key generation failed!!\n");
        while(1) {}
    }
    u32Ticks = 0xffffff - SysTick->VAL;

    printf("Public Qx is  %s\n", Qx);
    printf("Public Qy is  %s\n", Qy);
    printf("Elapsed time: %d.%d ms\n\n", u32Ticks / CyclesPerUs / 1000, u32Ticks / CyclesPerUs % 1000);

    /*
        Try to generate signature serveral times with private key and verificate them with the same
        public key.

    */
    for(m = 0; m < 3; m++)
    {
        printf("//-------------------------------------------------------------------------//\n");

        /* Generate random number k */
        XTRNG_Random(&rng, au8r, (uint32_t)(i32NBits / 8));

        for(i = 0, j = 0; i < i32NBits / 8; i++)
        {
            k[j++] = Byte2Char(au8r[i] & 0xf);
            k[j++] = Byte2Char(au8r[i] >> 4);
        }
        k[j] = 0; // NULL end

        printf("  k  = %s\n", k);

        if(XECC_IsPrivateKeyValid(XCRPT, ECC_CURVE_TYPE, k))
        {
            //printf("Private key check ok\n");
        }
        else
        {
            /* Invalid key */
            printf("Current k is not valid\n");
            while(1) {}
        }

        SysTick->VAL = 0;
        /* Generate signature */
        if(XECC_GenerateSignature(XCRPT, ECC_CURVE_TYPE, msg, d, k, R, S) < 0)
        {
            printf("ECC signature generation failed!!\n");
            while(1) {}
        }
        u32Ticks = 0xffffff - SysTick->VAL;

        printf("  R  = %s\n", R);
        printf("  S  = %s\n", S);
        printf("  msg= %s\n", msg);
        printf("Elapsed time: %d.%d ms\n", u32Ticks / CyclesPerUs / 1000, u32Ticks / CyclesPerUs % 1000);

        SysTick->VAL = 0;
        /* Verify signature */
        i32Err = XECC_VerifySignature(XCRPT, ECC_CURVE_TYPE, msg, Qx, Qy, R, S);
        u32Ticks = 0xffffff - SysTick->VAL;
        if(i32Err < 0)
        {
            printf("ECC signature verification failed!!\n");
            while(1) {}
        }
        else
        {
            printf("ECC digital signature verification OK.\n");
        }
        printf("Elapsed time: %d.%d ms\n\n", u32Ticks / CyclesPerUs / 1000, u32Ticks / CyclesPerUs % 1000);
    }

    while(1) {}
}

/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/
