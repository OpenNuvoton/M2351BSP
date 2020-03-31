/**************************************************************************//**
 * @file     main.c
 * @version  V1.10
 * $Revision: 10 $
 * $Date: 15/11/19 10:11a $
 * @brief    Show how to encrypt large data in Flash.
 *
 * @note
 * Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"


#define FLASH_PT_BASE  (0x01000)              /* Plaintext base in flash */
#define FLASH_PT_SIZE  (0x1000)               /* Plaintext data size. Must be DMA_BUF_SIZE alignment. */
#define DMA_BUF_SIZE   (128)                  /* buffer for AES DMA read */

static uint32_t s_au32FlashDataBuf[DMA_BUF_SIZE/4];  /*128 bytes*/


void CRPT_IRQHandler(void);
void DumpBuffHex(uint8_t *pucBuff, int nBytes);
void SYS_Init(void);
void DEBUG_PORT_Init(void);

/* 
   NOTE: 
         AES Engine key format is Key0_Reg = First key word, Key1_Reg = Second key word,
         and so on.
*/
// Key: 0x180ED20B8B95FC7A71D35CFD47A14F940036147DBD6DA72181FB82270283E45D
static uint32_t s_au32MyAESKey[8] =
{
    0x180ED20B, 0x8B95FC7A, 0x71D35CFD, 0x47A14F94, 
    0x0036147D, 0xBD6DA721, 0x81FB8227, 0x0283E45D, 
};

// IV: 0x10000000300000000000000000000001
static uint32_t s_au32MyAESIV[4] =
{
    0x10000000, 0x30000000, 0x00000000, 0x00000001
};


static __ALIGNED(4) uint8_t s_au8OutputData[FLASH_PT_SIZE];
static __ALIGNED(4) uint8_t s_au8OutputData2[FLASH_PT_SIZE];

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
    uint32_t u32FlashAddr;
    uint32_t u32AesMode, u32KeySizeOpt;
    uint8_t *pu8OutBuf;

    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 for printf */
    DEBUG_PORT_Init();

    printf("+---------------------------------------+\n");
    printf("|     Crypto AES Driver Sample Code     |\n");
    printf("+---------------------------------------+\n");

    /* Prepare plaintext data in flash*/
    NVIC_EnableIRQ(CRPT_IRQn);
    AES_ENABLE_INT(CRPT);

    
    u32AesMode = AES_MODE_ECB;
    u32KeySizeOpt = AES_KEY_SIZE_128;
    
    /* Encrypt data in flash block by block */
    pu8OutBuf = s_au8OutputData;
    for(u32FlashAddr = FLASH_PT_BASE; u32FlashAddr < (FLASH_PT_BASE+FLASH_PT_SIZE); u32FlashAddr+=DMA_BUF_SIZE )
    {
        /* Read the data to RAM buffer */
        memcpy(s_au32FlashDataBuf, (void *)u32FlashAddr, DMA_BUF_SIZE);
            
        /* Encrypt the data in RAM buffer */
        AES_Open(CRPT, 0, 1, u32AesMode, u32KeySizeOpt, AES_IN_OUT_SWAP);
        AES_SetKey(CRPT, 0, s_au32MyAESKey, u32KeySizeOpt);
        AES_SetInitVect(CRPT, 0, s_au32MyAESIV);
        
        /* Encrypt data will store to another RAM buffer */
        AES_SetDMATransfer(CRPT, 0, (uint32_t)s_au32FlashDataBuf, (uint32_t)pu8OutBuf, DMA_BUF_SIZE);

        g_AES_done = 0;
        /* Start AES Eecrypt */
        if(u32FlashAddr == FLASH_PT_BASE)
        {
            /* First block */
            printf("first ");
            AES_Start(CRPT, 0, CRYPTO_DMA_FIRST);
        }
        else if(u32FlashAddr == (FLASH_PT_BASE + FLASH_PT_SIZE - DMA_BUF_SIZE))
        {
            /* Last block */
            printf(" last\n");
            AES_Start(CRPT, 0, CRYPTO_DMA_LAST);
        }
        else
        {
            /* Middle blocks */
            printf(".");
            AES_Start(CRPT, 0, CRYPTO_DMA_CONTINUE);
        }
        
        /* Waiting for AES calculation */
        while(!g_AES_done);
        
        pu8OutBuf += DMA_BUF_SIZE;
    }

    /* Decrypt the data in RAM buffer to checking the decrypt result */
    AES_Open(CRPT, 0, 0, u32AesMode, u32KeySizeOpt, AES_IN_OUT_SWAP);
    AES_SetKey(CRPT, 0, s_au32MyAESKey, u32KeySizeOpt);
    AES_SetInitVect(CRPT, 0, s_au32MyAESIV);
    AES_SetDMATransfer(CRPT, 0, (uint32_t)s_au8OutputData, (uint32_t)s_au8OutputData2, sizeof(s_au8OutputData2));

    g_AES_done = 0;
    /* Start AES decrypt */
    AES_Start(CRPT, 0, CRYPTO_DMA_ONE_SHOT);
    /* Waiting for AES calculation */
    while(!g_AES_done);

    printf("AES decrypt done.\n");
    
    printf("Encrypt data:\n");
    DumpBuffHex(s_au8OutputData, FLASH_PT_SIZE);
    
    printf("Decrypt data:\n");
    DumpBuffHex(s_au8OutputData2, FLASH_PT_SIZE);
    
    /* Compare with original data */
    if(memcmp(s_au8OutputData2, (void *)FLASH_PT_BASE, FLASH_PT_SIZE) == 0)
    {
        printf("AES Encrypt/Decrypt OK!\n");
    }
    else
    {
        printf("AES decrypt data compare failed!\n");
    }
        
    
    while(1);
}



