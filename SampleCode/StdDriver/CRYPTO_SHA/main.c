/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Show Crypto IP SHA function
 *
 *
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "NuMicro.h"
#include "vector_parser.h"

extern void OpenTestVector(void);
extern int  GetNextPattern(void);


/* 160 bit length for SHA1 (20 bytes) */
static int32_t  g_i32DigestLength = 20;

static volatile int g_SHA_done;

void CRPT_IRQHandler(void);
int  do_compare(uint8_t *output, uint8_t *expect, int cmp_len);
int32_t RunSHA(void);
void SYS_Init(void);
void DEBUG_PORT_Init(void);


void CRPT_IRQHandler()
{
    if(SHA_GET_INT_FLAG(CRPT))
    {
        g_SHA_done = 1;
        SHA_CLR_INT_FLAG(CRPT);
    }
}


int  do_compare(uint8_t *output, uint8_t *expect, int cmp_len)
{
    int   i;

    if(memcmp(expect, output, (uint32_t)cmp_len))
    {
        printf("\nMismatch!! - %d\n", cmp_len);
        for(i = 0; i < cmp_len; i++)
            printf("0x%02x    0x%02x\n", expect[i], output[i]);
        return -1;
    }
    return 0;
}


int32_t RunSHA()
{
    uint32_t  au32OutputDigest[8];

    SHA_Open(CRPT, SHA_MODE_SHA1, SHA_IN_SWAP, 0);

    SHA_SetDMATransfer(CRPT, (uint32_t)&g_au8ShaData[0], (uint32_t)(g_i32DataLen / 8));

    printf("Key len= %d bits\n", g_i32DataLen);

    g_SHA_done = 0;
    /* Start SHA calculation */
    SHA_Start(CRPT, CRYPTO_DMA_ONE_SHOT);
    
    /* Waiting for SHA calcuation done */
    while(!g_SHA_done) ;
    
    /* Read SHA calculation result */
    SHA_Read(CRPT, au32OutputDigest);

    /* Compare calculation result with golden pattern */
    if(do_compare((uint8_t *)&au32OutputDigest[0], &g_au8ShaDigest[0], g_i32DigestLength) < 0)
    {
        printf("Compare error!\n");
        while(1);
    }
    return 0;
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


/*-----------------------------------------------------------------------------*/
int main(void)
{

    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 for printf */
    DEBUG_PORT_Init();

    printf("+-----------------------------------+\n");
    printf("|  M2351 Crypto SHA Sample Demo      |\n");
    printf("+-----------------------------------+\n");

    NVIC_EnableIRQ(CRPT_IRQn);
    SHA_ENABLE_INT(CRPT);

    /* Load test vector data base */
    OpenTestVector();

    while(1)
    {   
        /* Get data from test vector to calcualte and 
           compre the result with golden pattern */
        if(GetNextPattern() < 0)
            break;

        RunSHA();
    }

    printf("SHA test done.\n");

    while(1);
}
