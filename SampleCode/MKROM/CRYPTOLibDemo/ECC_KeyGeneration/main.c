/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Use MKROM Crypto library to show ECC P-256 key generation function.
 *
 * Copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"


#define ECC_CURVE_TYPE      CURVE_P_256
#define ECC_KEY_SIZE        256     /* Select ECC P-256 curve, 256-bits key length */

static char d[]  = "380a67fcfc01ca7073da7c2c54296a61327f77262a7d4674c3d8e29a63e3fa20"; /* private key */
static char Qx[] = "755b3819f05a3e9f32d4d599062834aac5220f75955378414a8f63716a152ce2"; /* expected answer: public key 1 */
static char Qy[] = "91c413f1915ed7b47473fd797647ba3d83e8224377909af5b30c530eaad79fd7"; /* expected answer: public key 2 */
static char key1[68], key2[68];                /* temporary buffer used to keep output public keys */

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
    int32_t i;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART for printf */
    UART_Init();

    printf("\n\n");
    printf("+---------------------------------------------------+\n");
    printf("|    Crypto ECC P-256 Public Key Generation Demo    |\n");
    printf("+---------------------------------------------------+\n");
    printf("\n");

    
    ECC_ENABLE_INT(CRPT);

    if(XECC_GeneratePublicKey(XCRPT, ECC_CURVE_TYPE, d, key1, key2) < 0)
    {
        printf("ECC key generation failed!!\n");
        while(1) {}
    }

    if(memcmp(Qx, key1, ECC_KEY_SIZE / 8))
    {
        printf("Public key 1 [%s] is not matched with expected [%s]!\n", key1, Qx);

        if(memcmp(Qx, key1, ECC_KEY_SIZE / 8) == 0)
            printf("PASS.\n");
        else
            printf("Error !!\n");

        for(i = 0; i < ECC_KEY_SIZE / 8; i++)
        {
            if(Qx[i] != key1[i])
                printf("\n%d - 0x%x 0x%x\n", i, Qx[i], key1[i]);
        }
        while(1) {}
    }

    if(memcmp(Qy, key2, ECC_KEY_SIZE / 8))
    {
        printf("Public key 2 [%s] is not matched with expected [%s]!\n", key2, Qy);
        while(1) {}
    }

    printf("Private key: %s\n", d);
    printf("Public key0: %s\n", key1);
    printf("Public key1: %s\n", key2);
    
    printf("ECC public key compared OK.\n");

    while(1) {}
}

/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/
