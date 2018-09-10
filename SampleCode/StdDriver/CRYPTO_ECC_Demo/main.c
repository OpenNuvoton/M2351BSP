/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Show whole ECC flow. Including private key/public key/Signature generation and
 *           Signature verification.
 *
 * @note
 * Copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "NuMicro.h"

#define KEY_LENGTH          192  /* Select ECC P-192 curve, 192-bits key length */
#define PRNG_KEY_SIZE       PRNG_KEY_SIZE_256
#define CURVE_P_SIZE        CURVE_P_192

char d[168];                         /* private key */
char Qx[168], Qy[168];               /* temporary buffer used to keep output public keys */
char k[168];                         /* random integer k form [1, n-1]                */
char msg[] = "This is a message. It could be encypted.";
char R[168], S[168];                 /* temporary buffer used to keep digital signature (R,S) pair */




#define ENDIAN(x)   ((((x)>>24)&0xff) | (((x)>>8)&0xff00) | (((x)<<8)&0xff0000) | ((x)<<24))

uint8_t Byte2Char(uint8_t c)
{
    if(c < 10)
        return (c + '0');
    if(c < 16)
        return (c - 10 + 'a');

    return 0;
}


void CRPT_IRQHandler()
{
    ECC_DriverISR(CRPT);
}


void  dump_buff_hex(uint8_t *pucBuff, int nBytes)
{
    int     nIdx, i;

    nIdx = 0;
    while(nBytes > 0)
    {
        printf("0x%04X  ", nIdx);
        for(i = 0; i < 16; i++)
            printf("%02x ", pucBuff[nIdx + i]);
        printf("  ");
        for(i = 0; i < 16; i++)
        {
            if((pucBuff[nIdx + i] >= 0x20) && (pucBuff[nIdx + i] < 127))
                printf("%c", pucBuff[nIdx + i]);
            else
                printf(".");
            nBytes--;
        }
        nIdx += 16;
        printf("\n");
    }
    printf("\n");
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
    int32_t i, j, nbits, m, err;
    uint32_t time;
    BL_RNG_T rng;
    uint8_t au8r[KEY_LENGTH / 8];

    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART for printf */
    DEBUG_PORT_Init();

    printf("+---------------------------------------------+\n");
    printf("|   Crypto ECC Public Key Generation Demo     |\n");
    printf("+---------------------------------------------+\n");

    NVIC_EnableIRQ(CRPT_IRQn);
    ECC_ENABLE_INT(CRPT);

    nbits = KEY_LENGTH;

    /* Initial TRNG */
    BL_RandomInit(&rng, BL_RNG_PRNG | BL_RNG_LIRC32K);

    do
    {

        /* Generate random number for private key */
        BL_Random(&rng, au8r, nbits / 8);

        for(i = 0, j = 0; i < nbits / 8; i++)
        {
            d[j++] = Byte2Char(au8r[i] & 0xf);
            d[j++] = Byte2Char(au8r[i] >> 4);
        }
        d[j] = 0; // NULL end


        printf("Private key = %s\n", d);

        /* Check if the private key valid */
        if(ECC_IsPrivateKeyValid(CRPT, CURVE_P_SIZE, d))
        {
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
    if(ECC_GeneratePublicKey(CRPT, CURVE_P_SIZE, d, Qx, Qy) < 0)
    {
        printf("ECC key generation failed!!\n");
        while(1);
    }
    time = 0xffffff - SysTick->VAL;

    printf("Public Qx is %s\n", Qx);
    printf("Public Qy is %s\n", Qy);
    printf("Elapsed time: %d.%d ms\n", time / CyclesPerUs / 1000, time / CyclesPerUs % 1000);

    /*
        Try to generate signature serveral times with private key and verificate them with the same
        public key.

    */
    for(m = 0; m < 3; m++)
    {
        printf("//-------------------------------------------------------------------------//\n");

        /* Generate random number k */
        BL_Random(&rng, au8r, nbits / 8);

        for(i = 0, j = 0; i < nbits / 8; i++)
        {
            k[j++] = Byte2Char(au8r[i] & 0xf);
            k[j++] = Byte2Char(au8r[i] >> 4);
        }
        k[j] = 0; // NULL End

        printf("  k = %s\n", k);

        if(ECC_IsPrivateKeyValid(CRPT, CURVE_P_SIZE, k))
        {
            /* Private key check ok */
        }
        else
        {
            /* Invalid key */
            printf("Current k is not valid\n");
            while(1);

        }

        SysTick->VAL = 0;
        if(ECC_GenerateSignature(CRPT, CURVE_P_SIZE, msg, d, k, R, S) < 0)
        {
            printf("ECC signature generation failed!!\n");
            while(1);
        }
        time = 0xffffff - SysTick->VAL;

        printf("  R  = %s\n", R);
        printf("  S  = %s\n", S);
        printf("  msg= %s\n", msg);
        printf("Elapsed time: %d.%d ms\n", time / CyclesPerUs / 1000, time / CyclesPerUs % 1000);

        SysTick->VAL = 0;
        err = ECC_VerifySignature(CRPT, CURVE_P_SIZE, msg, Qx, Qy, R, S);
        time = 0xffffff - SysTick->VAL;
        if(err < 0)
        {
            printf("ECC signature verification failed!!\n");
            while(1);
        }
        else
        {
            printf("ECC digital signature verification OK.\n");
        }
        printf("Elapsed time: %d.%d ms\n", time / CyclesPerUs / 1000, time / CyclesPerUs % 1000);
    }

    while(1);
}



