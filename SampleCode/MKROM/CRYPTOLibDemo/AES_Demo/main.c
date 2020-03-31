/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Use MKROM Crypto library to show AES-128 ECB mode encrypt/decrypt function.
 *
 * Copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"


static uint32_t s_au32MyAESKey[8] =
{
    0x00010203, 0x04050607, 0x08090a0b, 0x0c0d0e0f,
    0x00010203, 0x04050607, 0x08090a0b, 0x0c0d0e0f
};

static uint32_t s_au32MyAESIV[4] =
{
    0x00000000, 0x00000000, 0x00000000, 0x00000000
};

#ifdef __ICCARM__
#pragma data_alignment=4
uint8_t s_au8InputData[] =
{
#else
static __attribute__((aligned(4))) uint8_t s_au8InputData[] =
{
#endif
    0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88,
    0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff
};

#ifdef __ICCARM__
#pragma data_alignment=4
uint8_t s_au8OutputData[1024];
#else
static __attribute__((aligned(4))) uint8_t s_au8OutputData[1024];
#endif

static volatile uint32_t s_u32IsAES_done = 0;

void CRPT_IRQHandler(void);
void dump_buff_hex(uint8_t *pucBuff, int32_t i32Bytes);
void SYS_Init(void);
void UART_Init(void);

void CRPT_IRQHandler()
{
    if(AES_GET_INT_FLAG(CRPT))
    {
        s_u32IsAES_done = 1;
        AES_CLR_INT_FLAG(CRPT);
    }
}

void dump_buff_hex(uint8_t *pucBuff, int32_t i32Bytes)
{
    int32_t i, i32Idx;

    i32Idx = 0;
    while(i32Bytes > 0)
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
            i32Bytes--;
        }
        i32Idx += 16;
        printf("\n");
    }
    printf("\n");
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
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART for printf */
    UART_Init();

    printf("\n\n");
    printf("+-------------------------------+\n");
    printf("|    Crypto AES-128 ECB Demo    |\n");
    printf("+-------------------------------+\n");
    printf("\n");
   

    NVIC_EnableIRQ(CRPT_IRQn);
    AES_ENABLE_INT(CRPT);

    /*---------------------------------------
     *  AES-128 ECB mode encrypt
     *---------------------------------------*/
    XAES_Open(XCRPT, 0, 1, AES_MODE_ECB, AES_KEY_SIZE_128, AES_IN_OUT_SWAP);
    XAES_SetKey(XCRPT, 0, s_au32MyAESKey, AES_KEY_SIZE_128);
    XAES_SetInitVect(XCRPT, 0, s_au32MyAESIV);
    XAES_SetDMATransfer(XCRPT, 0, (uint32_t)s_au8InputData, (uint32_t)s_au8OutputData, sizeof(s_au8InputData));

    s_u32IsAES_done = 0;
    XAES_Start(XCRPT, 0, CRYPTO_DMA_ONE_SHOT);
    while(s_u32IsAES_done == 0) {}

    printf("AES encrypt done.\n\n");
    dump_buff_hex(s_au8OutputData, sizeof(s_au8InputData));

    /*---------------------------------------
     *  AES-128 ECB mode decrypt
     *---------------------------------------*/
    XAES_Open(XCRPT, 0, 0, AES_MODE_ECB, AES_KEY_SIZE_128, AES_IN_OUT_SWAP);
    XAES_SetKey(XCRPT, 0, s_au32MyAESKey, AES_KEY_SIZE_128);
    XAES_SetInitVect(XCRPT, 0, s_au32MyAESIV);
    XAES_SetDMATransfer(XCRPT, 0, (uint32_t)s_au8OutputData, (uint32_t)s_au8InputData, sizeof(s_au8InputData));

    s_u32IsAES_done = 0;
    XAES_Start(XCRPT, 0, CRYPTO_DMA_ONE_SHOT);
    while(s_u32IsAES_done == 0) {}

    printf("AES decrypt done.\n\n");
    dump_buff_hex(s_au8InputData, sizeof(s_au8InputData));

    while(1) {}
}

/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/
