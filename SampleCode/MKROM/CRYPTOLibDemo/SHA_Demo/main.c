/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Use MKROM Crypto library to generate SHA-256 message digest.
 *
 * Copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"


#ifdef __ICCARM__
#pragma data_alignment=4
char s_acInputString[] = "123456789ABCdef";
#else
static __attribute__((aligned(4))) char s_acInputString[] = "123456789ABCdef";
#endif

//Hex result: FEBEBE5E04EBE9E4BFE95FD57B25CBE0270128860BB42DCD9E2D13B21F5C18F9
static uint32_t s_au32Expect[] = 
{
    0x5EBEBEFE, 0xE4E9EB04, 0xD55FE9BF, 0xE0CB257B,
    0x86280127, 0xCD2DB40B, 0xB2132D9E, 0xF9185C1F
};


static volatile uint32_t s_u32IsSHA_done = 0;

void CRPT_IRQHandler(void);
void dump_buff_hex(uint8_t *pucBuff, int32_t i32Bytes);
int32_t do_compare(uint8_t *output, uint8_t *expect, int32_t cmp_len);
void SYS_Init(void);
void UART_Init(void);

void CRPT_IRQHandler()
{
    if(SHA_GET_INT_FLAG(CRPT))
    {
        s_u32IsSHA_done = 1;
        SHA_CLR_INT_FLAG(CRPT);
    }
}

void dump_buff_hex(uint8_t *pucBuff, int32_t i32Bytes)
{
    int32_t i = 0;
    
    while(i32Bytes > 0)
    {
        printf("%02X", pucBuff[i++]);
        i32Bytes--;
    }
    printf("\n");
}

int32_t do_compare(uint8_t *output, uint8_t *expect, int32_t cmp_len)
{
    int32_t i;

    if(memcmp(expect, output, (uint32_t)cmp_len))
    {
        printf("\nMismatch!! - %d\n", cmp_len);
        for(i = 0; i < cmp_len; i++)
            printf("0x%02x    0x%02x\n", expect[i], output[i]);
        return -1;
    }
    
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
    uint32_t au32Output[(256 / 8)];
    
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART for printf */
    UART_Init();

    printf("\n\n");
    printf("+---------------------------+\n");
    printf("|    Crypto SHA-256 Demo    |\n");
    printf("+---------------------------+\n");
    printf("\n");

    
    NVIC_EnableIRQ(CRPT_IRQn);
    SHA_ENABLE_INT(CRPT);

    /*---------------------------------------
     *  SHA-256
     *---------------------------------------*/
    XSHA_Open(XCRPT, SHA_MODE_SHA256, SHA_IN_OUT_SWAP, 0);

    printf("Input string data is %s.\n\n", s_acInputString);    
    XSHA_SetDMATransfer(XCRPT, (uint32_t)&s_acInputString[0],  sizeof(s_acInputString)-1);

    s_u32IsSHA_done = 0;
    XSHA_Start(XCRPT, CRYPTO_DMA_ONE_SHOT);
    while(s_u32IsSHA_done == 0) {}

    XSHA_Read(XCRPT, au32Output);

    printf("SHA-256:\n");    
    dump_buff_hex((uint8_t *)au32Output, (256 / 8));
        
    /*--------------------------------------------*/
    /*  Compare                                   */
    /*--------------------------------------------*/
    if(do_compare((uint8_t *)&au32Output[0], (uint8_t *)&s_au32Expect[0], (256 / 8)) < 0)
    {
        printf("Compare error!\n");
        while(1) {}
    }
    
    while(1) {}
}

/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/
