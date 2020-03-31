/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Use MKROM Crypto library to show ECC P-256 ECDSA signature verification function.
 *
 * Copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"


#define ECC_CURVE_TYPE      CURVE_P_256

static char sha_msg[] = "7DDD1A452CE194D7F4C1CD2C41A58C9513CCE48111461384EC9C2B6E57B837FB";    /* 256-bits message                              */
static char Qx[] = "755b3819f05a3e9f32d4d599062834aac5220f75955378414a8f63716a152ce2";         /* public key 1                                  */
static char Qy[] = "91c413f1915ed7b47473fd797647ba3d83e8224377909af5b30c530eaad79fd7";         /* public key 2                                  */
static char R[] = "742fda512ad62f9a900f686fa01013c2661ccb42ee6c95b6a998de6a1be6a777";          /* Expected answer: R of (R,S) digital signature */
static char S[] = "88221bc22161e6f45f08bf9dabc3c307da278935eedf9273722d75eea07e2028";          /* Expected answer: S of (R,S) digital signature */

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
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART for printf */
    UART_Init();

    printf("\n\n");
    printf("+----------------------------------------------------+\n");
    printf("|    Crypto ECC P-256 Signature Verification Demo    |\n");
    printf("+----------------------------------------------------+\n");
    printf("\n");
    
    
    ECC_ENABLE_INT(CRPT);

    if(XECC_VerifySignature(XCRPT, ECC_CURVE_TYPE, sha_msg, Qx, Qy, R, S) < 0)
    {
        printf("ECC signature verification failed!!\n");
        while(1) {}
    }

    printf("msg: %s\n", sha_msg);
    printf("Qx:  %s\n", Qx);
    printf("Qy:  %s\n", Qy);
    printf("R:   %s\n", R);
    printf("S:   %s\n", S);    
    printf("ECC digital signature verification OK.\n");
    
    while(1) {}
}

/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/
