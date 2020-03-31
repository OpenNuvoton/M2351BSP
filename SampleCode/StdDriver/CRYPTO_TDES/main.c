/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Show Crypto IP Triple DES CBC mode encrypt/decrypt function
 *
 * @note
 * Copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

/* TDES Key:  1e4678a17f2c8a33 800e15ac47891a4c a011453291c23340 */
static uint32_t s_au8MyTDESKey[3][2] =
{
    { 0x1e4678a1, 0x7f2c8a33 },
    { 0x800e15ac, 0x47891a4c },
    { 0xa0114532, 0x91c23340 }
};

/* Initial vector: 1234567890abcdef */
static uint32_t s_au32MyTDESIV[2] = {  0x12345678, 0x90abcdef };


/* The input data for TDES test. NOTE: The input data size must be 8 bytes alignment */
#ifdef __ICCARM__
#pragma data_alignment=4
uint8_t s_au8InputData[] =
{
#else
static __attribute__((aligned(4))) uint8_t s_au8InputData[] =
{
#endif
    0x12, 0x34, 0x56, 0x78, 0xAB, 0xCD, 0xEF, 0x12, 0x34, 0x56, 0x78, 0xAB, 0xCD, 0xEF, 0x11, 0x22
};


#ifdef __ICCARM__
#pragma data_alignment=4
uint8_t s_au8OutputData[1024];
#else
static __attribute__((aligned(4))) uint8_t s_au8OutputData[1024];
#endif

static volatile int  g_TDES_done;


void CRPT_IRQHandler(void);
void DumpBuffHex(uint8_t *pucBuff, int nBytes);
void SYS_Init(void);
void DEBUG_PORT_Init(void);


void CRPT_IRQHandler()
{
    if(TDES_GET_INT_FLAG(CRPT))
    {
        g_TDES_done = 1;
        TDES_CLR_INT_FLAG(CRPT);
    }
}


void DumpBuffHex(uint8_t *pucBuff, int nBytes)
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

    printf("+--------------------------------------------+\n");
    printf("|           Crypto TDES Driver Sample Code   |\n");
    printf("+--------------------------------------------+\n");

    /* Check input data size */
    if((sizeof(s_au8InputData) & 0x7) != 0)
    {
        printf("ERR: The input data size is not 8 bytes alignment.\n");
        printf("     You may pad 0x0 to let it be 8 bytes alignment.\n");
        goto lexit;
    }
    
    /* Enable crypto interrupt */
    NVIC_EnableIRQ(CRPT_IRQn);
    TDES_ENABLE_INT(CRPT);

    /*---------------------------------------
     *  TDES CBC mode encrypt
     *---------------------------------------*/
    TDES_Open(CRPT, 0, 1, 1, 1, TDES_MODE_CBC, TDES_IN_OUT_WHL_SWAP);
    TDES_SetKey(CRPT, 0, s_au8MyTDESKey);
    TDES_SetInitVect(CRPT, 0, s_au32MyTDESIV[0], s_au32MyTDESIV[1]);
    TDES_SetDMATransfer(CRPT, 0, (uint32_t)s_au8InputData, (uint32_t)s_au8OutputData, sizeof(s_au8InputData));

    g_TDES_done = 0;
    /* Start TDEC calculation */
    TDES_Start(CRPT, 0, CRYPTO_DMA_ONE_SHOT);
    
    /* Waiting for TDES done */
    while(!g_TDES_done);

    printf("TDES encrypt done.\n\n");
    DumpBuffHex(s_au8OutputData, sizeof(s_au8InputData));

    /*---------------------------------------
     *  TDES CBC mode decrypt
     *---------------------------------------*/
    TDES_Open(CRPT, 0, 0, 1, 1, TDES_MODE_CBC, TDES_IN_OUT_WHL_SWAP);
    TDES_SetKey(CRPT, 0, s_au8MyTDESKey);
    TDES_SetInitVect(CRPT, 0, s_au32MyTDESIV[0], s_au32MyTDESIV[1]);
    TDES_SetDMATransfer(CRPT, 0, (uint32_t)s_au8OutputData, (uint32_t)s_au8InputData, sizeof(s_au8InputData));

    g_TDES_done = 0;
    TDES_Start(CRPT, 0, CRYPTO_DMA_ONE_SHOT);
    while(!g_TDES_done);

    printf("TDES decrypt done.\n\n");
    DumpBuffHex(s_au8InputData, sizeof(s_au8InputData));

lexit:

    while(1);
}



