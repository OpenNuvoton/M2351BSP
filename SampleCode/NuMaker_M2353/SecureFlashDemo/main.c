/******************************************************************************
 * @file     main.c
 * @version  V3.00
 * @brief    Show how to config/erase XOM region.
 * @note
 * Copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "string.h"
#include "NuMicro.h"
#include "SFLib.h"


#define BUF_SIZE        512

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
static int32_t CompareBufferToValue(uint8_t* pBuf1, const uint8_t value, int32_t sizeBytes);
static void DumpBufHex(uint8_t *pucBuff, int nBytes);
void InitBuf(uint8_t *dataBuffer);
void SYS_Init(void);
void UART0_Init(void);

/*---------------------------------------------------------------------------------------------------------*/
/* Define global parameters                                                                                */
/*---------------------------------------------------------------------------------------------------------*/


void SYS_Init(void)
{
    uint32_t u32TimeOutCnt;

    /* Unlock registers */
    SYS_UnlockReg();

    /*-----------------------------------------------------------------------------------------------------*/
    /* Setting clock tree: PCLK0 = CPUCLK = CRYPTOCLK = HCLK = 64MHz. PLL = 64MHz                          */
    /*-----------------------------------------------------------------------------------------------------*/

    /* Enable PLL */
    CLK->PLLCTL = CLK_PLLCTL_64MHz_HIRC;

    /* Waiting for PLL stable */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while((CLK->STATUS & CLK_STATUS_PLLSTB_Msk) == 0)
        if(--u32TimeOutCnt == 0) break;

    /* Set HCLK divider to 1 */
    CLK->CLKDIV0 = (CLK->CLKDIV0 & (~CLK_CLKDIV0_HCLKDIV_Msk)) | CLK_CLKDIV0_HCLK(1);

    /* Set PCLK0 divider to HCLK */
    CLK->PCLKDIV = (CLK->PCLKDIV & (~CLK_PCLKDIV_APB0DIV_Msk)) | CLK_PCLKDIV_APB0DIV_HCLK;

    /* Switch HCLK clock source to PLL */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLKSEL_Msk)) | CLK_CLKSEL0_HCLKSEL_PLL;

    /* Select IP clock source */
    CLK->CLKSEL1 = CLK_CLKSEL1_UART0SEL_HIRC;
    CLK->CLKSEL2 = CLK_CLKSEL2_QSPI0SEL_PLL ;

    /* Enable IP clock */
    CLK->AHBCLK  |= CLK_AHBCLK_CRPTCKEN_Msk;
    CLK->APBCLK0 |= CLK_APBCLK0_UART0CKEN_Msk;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & (~(UART0_RXD_PB12_Msk | UART0_TXD_PB13_Msk))) | UART0_RXD_PB12 | UART0_TXD_PB13;

    /* Setup QSPI0 multi-function pins */
    SYS->GPA_MFPL &= ~(QSPI0_MOSI0_PA0_Msk | QSPI0_MISO0_PA1_Msk | QSPI0_CLK_PA2_Msk | QSPI0_SS_PA3_Msk | QSPI0_MOSI1_PA4_Msk | QSPI0_MISO1_PA5_Msk);
    SYS->GPA_MFPL |= (QSPI0_MOSI0_PA0 | QSPI0_MISO0_PA1 | QSPI0_CLK_PA2 | QSPI0_SS_PA3 | QSPI0_MOSI1_PA4 | QSPI0_MISO1_PA5);


}

void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    UART0->LINE = UART_PARITY_NONE | UART_STOP_BIT_1 | UART_WORD_LEN_8;
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC, 115200);

}

int32_t main(void)
{
    extern void func(void);
    uint8_t u8DataBuffer[BUF_SIZE]   = { 0 };
    int     u32Item = 0;
    uint32_t u32FlashAddr = 0;
    uint32_t u32Ver;

    SYS_Init();

    UART0_Init();


    /*
        This sample code needs to execute on M2353 with SFI library.
        We need to check whether the SFI library existed or not.
    */
    if((FMC->XOMSTS & FMC_XOMSTS_XOMR3ON_Msk) == 0)
    {
        if((FMC->XOMR3STS & FMC_XOMR3STS_BASE_Msk) != 0x0007C000)
        {
            printf("Error: Cannot detect native SFI library on the device.\n");
            printf("       Please check if the device is M2353 or ask for supporting.\n");
            goto lexit;
        }
    }

    /*-------------------------------------------------------------------------------------*/
    /* Secure Flash Library init                                                           */
    /*-------------------------------------------------------------------------------------*/
    SFL_Init();

    printf("\n\n");
    printf("System clock: %d Hz.\n", SystemCoreClock);
    printf("SPI clock:    %d Hz.\n\n", QSPI_GetBusClock(QSPI0));


    //Perfrom the get SFI library version
    u32Ver = SFL_GetVersion();
    printf("SFI library version : 0.%X.%X \n", (u32Ver >> 8) & 0xffu, u32Ver & 0xffu);

    /*-------------------------------------------------------------------------------------*/
    /* Secure Session init                                                                 */
    /*-------------------------------------------------------------------------------------*/

    if(SFL_SessionInit() != 0)
    {
        printf("Secure Flash initial result fail\n");
        goto lexit;
    }
    printf("Secure Flash initial completed\n");
    if(SFL_SessionOpen() != 0)
    {
        printf("Open session fail\n");
        goto lexit;
    }
    printf("Open session completed\n");
    if(SFL_ConfigInterface(SFL_IF_QUAD) != 0)
    {
        printf("Configration interface fail\n");
        goto lexit;
    }
    printf("Configration interface to quad \n");

    /*-----------------------------------------------------------------------------------------------------*/
    /* Get flash size                                                                                      */
    /*-----------------------------------------------------------------------------------------------------*/
    printf("Secure Flash Size: %u Bytes\n", SFL_GetFlashSize());
    printf("Secure Flash can be accessed now\n");


    while(1)
    {
        printf("\n+----------------------------------------------------+\n");
        printf("|             Secure Flash Sample Code              |\n");
        printf("+----------------------------------------------------+\n");
        printf("| [r] Read %d Bytes data from Secure Flash.         |\n", BUF_SIZE);
        printf("| [w] Write & Read %d Bytes data from Secure Flash. |\n", BUF_SIZE);
        printf("| [e] Erase & Read %d Bytes data from Secure Flash. |\n", BUF_SIZE);
        printf("+----------------------------------------------------+\n");
        printf("Please select the option from the list.\n\n");

        u32Item = getchar();

        switch(u32Item)
        {

            /*-----------------------------------------------------------------------------------------------------*/
            /* Read from Flash                                                                                     */
            /*-----------------------------------------------------------------------------------------------------*/
            case 'r':
            case 'R':
            {
                InitBuf(u8DataBuffer);
                if(SFL_Exec(SFL_CMD_READ, u8DataBuffer, u32FlashAddr, BUF_SIZE) != 0)
                {
                    printf("Secure Flash Read result fail!\n");
                }
                else
                {
                    DumpBufHex(u8DataBuffer, BUF_SIZE);
                }
                break;
            }
            /*-----------------------------------------------------------------------------------------------------*/
            /* Write & Read from Flash                                                                             */
            /*-----------------------------------------------------------------------------------------------------*/
            case 'w':
            case 'W':
            {
                InitBuf(u8DataBuffer);
                if(SFL_Exec(SFL_CMD_WRITE, u8DataBuffer, u32FlashAddr, BUF_SIZE) != 0)
                {
                    printf("Secure Flash Write fail!\n");
                    goto lexit;
                }
                if(SFL_Exec(SFL_CMD_READ, u8DataBuffer, u32FlashAddr, BUF_SIZE) != 0)
                {
                    printf("Secure Flash Read result fail!\n");
                    goto lexit;
                }
                DumpBufHex(u8DataBuffer, BUF_SIZE);
                break;
            }
            /*-----------------------------------------------------------------------------------------------------*/
            /* Erase & read from Flash                                                                             */
            /*-----------------------------------------------------------------------------------------------------*/
            case 'e':
            case 'E':
            {
                InitBuf(u8DataBuffer);
                if(SFL_Exec(SFL_CMD_ERASE, u8DataBuffer, u32FlashAddr, BUF_SIZE) != 0)
                {
                    printf("Secure Flash Erase fail!\n");
                    break;
                }
                if(SFL_Exec(SFL_CMD_READ, u8DataBuffer, u32FlashAddr, BUF_SIZE) != 0)
                {
                    printf("Secure Flash Read result fail!\n");
                    break;
                }
                if(CompareBufferToValue(u8DataBuffer, 0xFF, BUF_SIZE) != 0)
                {
                    printf("Secure Flash Erase fail!\n");
                    break;
                }
                DumpBufHex(u8DataBuffer, BUF_SIZE);
                break;
            }
            default:
            {
                printf("Invalide input. Please try again!\n");
            }
        }
    }

lexit:

    for(;;)
    {
        __WFI();
    }

}


/* print buffer bytes in Hex format */
static void DumpBufHex(uint8_t *pucBuff, int nBytes)
{
    int     nIdx, i;

    nIdx = 0;
    while(nBytes >= 16)
    {
        printf("0x%04X  ", nIdx);
        for(i = 0; i < 16; i++)
            printf("%02X ", pucBuff[nIdx + i]);
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

    if(nBytes)
    {
        printf("0x%04X  ", nIdx);
        for(i = 0; i < nBytes; i++)
            printf("%02X ", pucBuff[nIdx + i]);

        for(i = nBytes; i < 16; i++)
            printf("   ");

        printf("  ");
        for(i = 0; i < nBytes; i++)
        {
            if((pucBuff[nIdx + i] >= 0x20) && (pucBuff[nIdx + i] < 127))
                printf("%c", pucBuff[nIdx + i]);
            else
                printf(".");
        }
        for(i = nBytes; i < 16; i++)
            printf(" ");

        printf("\n");
    }


    printf("\n");
}


/* Compare buffer to value */
static int32_t CompareBufferToValue(uint8_t* pBuf1, const uint8_t value, int32_t sizeBytes)
{
    int32_t uiCount;

    for(uiCount = 0; uiCount < sizeBytes; uiCount++)
    {
        if(pBuf1[uiCount] != value)
            return -1;
    }

    return 0;
}

/* Initial data buffer for writing */
void InitBuf(uint8_t *u8DataBuffer)
{
    int32_t i = 0;
    for(i = 0; i < BUF_SIZE; i++)
    {
        u8DataBuffer[i] = (uint8_t)(i % 256);
    }
}

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
