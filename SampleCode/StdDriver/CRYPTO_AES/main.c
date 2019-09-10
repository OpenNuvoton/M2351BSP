/**************************************************************************//**
 * @file     main.c
 * @version  V1.10
 * $Revision: 10 $
 * $Date: 15/11/19 10:11a $
 * @brief    Show Crypto IP AES-128 ECB mode encrypt/decrypt function.
 *
 * @note
 * Copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

// Key: 0x000102030405060708090a0b0c0d0e0f000102030405060708090a0b0c0d0e0f
uint32_t au32MyAESKey[8] =
{
    0x00010203, 0x04050607, 0x08090a0b, 0x0c0d0e0f,
    0x00010203, 0x04050607, 0x08090a0b, 0x0c0d0e0f
};

// IV: 0x10000000000000000000000000000001
uint32_t au32MyAESIV[4] =
{
    0x10000000, 0x00000000, 0x00000000, 0x00000001
};

#ifdef __ICCARM__
#pragma data_alignment=4
uint8_t au8InputData[] =
{
#else
__attribute__((aligned(4))) uint8_t au8InputData[] =
{
#endif
    0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88,
    0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff
};

#ifdef __ICCARM__
#pragma data_alignment=4
uint8_t au8OutputData[1024];
#else
__attribute__((aligned(4))) uint8_t au8OutputData[1024];
#endif

static volatile int32_t  g_AES_done;

void CRPT_IRQHandler()
{
    if(AES_GET_INT_FLAG(CRPT))
    {
        g_AES_done = 1;
        AES_CLR_INT_FLAG(CRPT);
    }
}


void DumpBuffHex(uint8_t *pucBuff, int nBytes)
{
    int32_t i32Idx, i;

    i32Idx = 0;
    while(nBytes > 0)
    {
        printf("0x%04X  ", i32Idx);
        for(i = 0; i < 16; i++)
            printf("%02x ", pucBuff[i32Idx + i]);
        printf("  ");
        for(i = 0; i < 16; i++)
        {
            if((pucBuff[i32Idx + i] >= 0x20) && (pucBuff[i32Idx + i] < 127))
                printf("%c", pucBuff[i32Idx + i]);
            else
                printf(".");
            nBytes--;
        }
        i32Idx += 16;
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
    SystemCoreClock = 128000000 / 2;       // HCLK
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
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 for printf */
    DEBUG_PORT_Init();

    printf("+---------------------------------------+\n");
    printf("|     Crypto AES Driver Sample Code     |\n");
    printf("+---------------------------------------+\n");

    NVIC_EnableIRQ(CRPT_IRQn);
    AES_ENABLE_INT(CRPT);

    /*---------------------------------------
     *  AES-128 ECB mode encrypt
     *---------------------------------------*/
    AES_Open(CRPT, 0, 1, AES_MODE_ECB, AES_KEY_SIZE_128, AES_IN_OUT_SWAP);
    AES_SetKey(CRPT, 0, au32MyAESKey, AES_KEY_SIZE_128);
    AES_SetInitVect(CRPT, 0, au32MyAESIV);
    AES_SetDMATransfer(CRPT, 0, (uint32_t)au8InputData, (uint32_t)au8OutputData, sizeof(au8InputData));

    g_AES_done = 0;
    /* Start AES Eecrypt */
    AES_Start(CRPT, 0, CRYPTO_DMA_ONE_SHOT);
    /* Waiting for AES calculation */
    while(!g_AES_done);

    printf("AES encrypt done.\n\n");
    DumpBuffHex(au8OutputData, sizeof(au8InputData));

    /*---------------------------------------
     *  AES-128 ECB mode decrypt
     *---------------------------------------*/
    AES_Open(CRPT, 0, 0, AES_MODE_ECB, AES_KEY_SIZE_128, AES_IN_OUT_SWAP);
    AES_SetKey(CRPT, 0, au32MyAESKey, AES_KEY_SIZE_128);
    AES_SetInitVect(CRPT, 0, au32MyAESIV);
    AES_SetDMATransfer(CRPT, 0, (uint32_t)au8OutputData, (uint32_t)au8InputData, sizeof(au8InputData));

    g_AES_done = 0;
    /* Start AES decrypt */
    AES_Start(CRPT, 0, CRYPTO_DMA_ONE_SHOT);
    /* Waiting for AES calculation */
    while(!g_AES_done);

    printf("AES decrypt done.\n\n");
    DumpBuffHex(au8InputData, sizeof(au8InputData));

    while(1);
}



