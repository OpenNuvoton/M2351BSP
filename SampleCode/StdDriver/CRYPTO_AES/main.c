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

void CRPT_IRQHandler(void);
void DumpBuffHex(uint8_t *pucBuff, int nBytes);
void SYS_Init(void);
void DEBUG_PORT_Init(void);

/* 
   NOTE: 
         AES Engine key format is Key0_Reg = First key word, Key1_Reg = Second key word,
         and so on.
*/
// Key: 0x9C866E5FD8E0DBA8F76044C2AAB1E50C115D8153534AC15185419F0A5B3E6A4B
static uint32_t s_au32MyAESKey[8] =
{
    0x9C866E5F,    0xD8E0DBA8,    0xF76044C2,    0xAAB1E50C,
    0x115D8153,    0x534AC151,    0x85419F0A,    0x5B3E6A4B
};

// IV: 0x1000000030000000000000000000000a
static uint32_t s_au32MyAESIV[4] =
{
    0x10000000, 0x30000000, 0x00000000, 0x0000000a
};

static __ALIGNED(4) uint8_t s_au8InputData[] =
{
    0xd7, 0x3e, 0xbb, 0x64, 0x93, 0xdc, 0x47, 0x93, 
    0x44, 0xe7, 0xd9, 0x06, 0x46, 0x72, 0x77, 0x2a, 
    0xee, 0x68, 0xb3, 0x1f, 0x6b, 0x83, 0x8a, 0xfc, 
    0x77, 0xec, 0x17, 0xbe, 0x7e, 0xbf, 0x17, 0xa6, 
    0xa8, 0xd4, 0x80, 0xaa, 0xb2, 0x26, 0xb3, 0x51, 
    0x46, 0xb6, 0x22, 0xbb, 0x4a, 0x4e, 0x35, 0xcf, 
    0x8c, 0xac, 0x4b, 0xb3, 0x4d, 0x39, 0xd2, 0x6c, 
    0x38, 0x92, 0xcf, 0xd0, 0x13, 0xe5, 0xb4, 0x6c,

};

static __ALIGNED(4) uint8_t s_au8OutputData[1024];
static __ALIGNED(4) uint8_t s_au8OutputData2[1024];

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
    uint32_t i;
    uint32_t u32AesMode;
    uint32_t u32KeySizeOpt;
    
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
     *  AES encrypt
     *---------------------------------------*/
    u32AesMode = AES_MODE_CFB;
    u32KeySizeOpt = AES_KEY_SIZE_256;
     
    AES_Open(CRPT, 0, 1, u32AesMode, u32KeySizeOpt, AES_IN_OUT_SWAP);
    AES_SetKey(CRPT, 0, s_au32MyAESKey, u32KeySizeOpt);
    
    /* The IV should be changed at each encrypt/decrypt */
    AES_SetInitVect(CRPT, 0, s_au32MyAESIV);
    
    AES_SetDMATransfer(CRPT, 0, (uint32_t)s_au8InputData, (uint32_t)s_au8OutputData, sizeof(s_au8InputData));

    g_AES_done = 0;
    /* Start AES Eecrypt */
    AES_Start(CRPT, 0, CRYPTO_DMA_ONE_SHOT);
    /* Waiting for AES calculation */
    while(!g_AES_done);

    printf("AES encrypt done.\n\n");
    DumpBuffHex(s_au8OutputData, sizeof(s_au8InputData));

    /*---------------------------------------
     *  AES decrypt
     *---------------------------------------*/
    AES_Open(CRPT, 0, 0, u32AesMode, u32KeySizeOpt, AES_IN_OUT_SWAP);
    AES_SetKey(CRPT, 0, s_au32MyAESKey, u32KeySizeOpt);
    
    /* The IV should be changed at each encrypt/decrypt */
    AES_SetInitVect(CRPT, 0, s_au32MyAESIV);
    
    AES_SetDMATransfer(CRPT, 0, (uint32_t)s_au8OutputData, (uint32_t)s_au8OutputData2, sizeof(s_au8InputData));

    g_AES_done = 0;
    /* Start AES decrypt */
    AES_Start(CRPT, 0, CRYPTO_DMA_ONE_SHOT);
    /* Waiting for AES calculation */
    while(!g_AES_done);

    printf("AES decrypt done.\n\n");
    
    printf("Original data:\n");
    DumpBuffHex(s_au8InputData, sizeof(s_au8InputData));
    
    printf("Decrypt data :\n");
    DumpBuffHex(s_au8OutputData2, sizeof(s_au8InputData));

    /* Compare the decrpt results */
    for(i=0;i<sizeof(s_au8InputData);i++)
    {
        if(s_au8InputData[i] != s_au8OutputData2[i])
        {
            printf("FAIL: Decrypt result is not correct!\n");
            break;
        }
    }
    
    if(i == sizeof(s_au8InputData))
    {
        printf("AES Encrypt/Decrypt OK!\n");
    }


    while(1);
}



