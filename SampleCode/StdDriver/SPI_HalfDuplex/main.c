/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief
 *           Demonstrate SPI half-duplex mode.
 *           SPI0 will be configured as Master mode and SPI1 will be configured as Slave mode.
 *           Both SPI0 and SPI1 will be configured as half-duplex mode.
 *
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define TEST_COUNT  4

static uint32_t s_au32DestinationData[TEST_COUNT];
static volatile uint32_t s_u32RxDataCount;

/* Function prototype declaration */
void SYS_Init(void);
void SPI_Init(void);

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and HCLK source divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable HXT clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Wait for HXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Enable PLL */
    CLK->PLLCTL = CLK_PLLCTL_128MHz_HIRC;

    /* Waiting for PLL stable */
    CLK_WaitClockReady(CLK_STATUS_PLLSTB_Msk);

    /* Select HCLK clock source as PLL and HCLK source divider as 2 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_PLL, CLK_CLKDIV0_HCLK(2));

    /* Select UART module clock source as HXT and UART module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HXT, CLK_CLKDIV0_UART0(1));

    /* Select PCLK as the clock source of SPI0 and SPI1 */
    CLK_SetModuleClock(SPI0_MODULE, CLK_CLKSEL2_SPI0SEL_PCLK1, MODULE_NoMsk);
    CLK_SetModuleClock(SPI1_MODULE, CLK_CLKSEL2_SPI1SEL_PCLK0, MODULE_NoMsk);

    /* Enable UART peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable SPI0 peripheral clock */
    CLK_EnableModuleClock(SPI0_MODULE);

    /* Enable SPI1 peripheral clock */
    CLK_EnableModuleClock(SPI1_MODULE);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and CyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & (~(UART0_RXD_PB12_Msk | UART0_TXD_PB13_Msk))) | UART0_RXD_PB12 | UART0_TXD_PB13;

    /* Configure SPI0 related multi-function pins. GPD[3,2,0] : SPI1_SS, SPI1_CLK, SPI1_MOSI. */
    SYS->GPD_MFPL &= ~(SYS_GPD_MFPL_PD3MFP_Msk | SYS_GPD_MFPL_PD2MFP_Msk | SYS_GPD_MFPL_PD0MFP_Msk);
    SYS->GPD_MFPL |= (SYS_GPD_MFPL_PD3MFP_SPI0_SS | SYS_GPD_MFPL_PD2MFP_SPI0_CLK | SYS_GPD_MFPL_PD0MFP_SPI0_MOSI);

    /* Configure SPI1 related multi-function pins. GPH[7:5] : SPI1_SS, SPI1_CLK, SPI1_MOSI. */
    SYS->GPH_MFPL &= ~(SYS_GPH_MFPL_PH7MFP_Msk | SYS_GPH_MFPL_PH6MFP_Msk | SYS_GPH_MFPL_PH5MFP_Msk);
    SYS->GPH_MFPL |= (SYS_GPH_MFPL_PH7MFP_SPI1_SS | SYS_GPH_MFPL_PH6MFP_SPI1_CLK | SYS_GPH_MFPL_PH5MFP_SPI1_MOSI);
}

void SPI_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init SPI                                                                                                */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure as a master, clock idle low, 32-bit transaction, drive output on falling clock edge and latch input on rising edge. */
    /* Set IP clock divider. SPI clock rate = 2MHz */
    SPI_Open(SPI0, SPI_MASTER, SPI_MODE_0, 32, 2000000);

    /* Enable the automatic hardware slave select function. Select the SS pin and configure as low-active. */
    SPI_EnableAutoSS(SPI0, SPI_SS, SPI_SS_ACTIVE_LOW);

    /* Configure SPI1 */
    /* Configure SPI1 as a slave, clock idle low, 32-bit transaction, drive output on falling clock edge and latch input on rising edge. */
    /* Configure SPI1 as a low level active device. SPI peripheral clock rate = f_PCLK0 */
    SPI_Open(SPI1, SPI_SLAVE, SPI_MODE_0, 32, (uint32_t)NULL);
}

int main(void)
{
    uint32_t u32DataCount;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Open(UART0, 115200);

    /* Init SPI */
    SPI_Init();

    printf("\n\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("|                   SPI Half-duplex Mode Sample Code                   |\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("\n");
    printf("Configure SPI0 as a master and SPI1 as a slave.\n");
    printf("Set both SPI0 and SPI1 to half-duplex.\n");
    printf("Bit length of a transaction: 32\n");
    printf("Please connect below I/O connections for SPI0 and SPI1:\n");
    printf("    SPI0_SS(PD3)   <->   SPI1_SS(PH7)\n");
    printf("    SPI0_CLK(PD2)  <->   SPI1_CLK(PH6)\n");
    printf("    SPI0_MOSI(PD0) <->   SPI1_MOSI(PH5)\n\n");
    printf("After the transfer is done, the received data will be printed out.\n");

    /* Set slave SPI1 to half-duplex mode */
    SPI1->CTL |= SPI_CTL_HALFDPX_Msk;
    /* Enable half-duplex will produce TXFBCLR (SPIx_FIFOCTL[9]) and RXFBCLR (SPIx_FIFOCTL[8])*/
    while(SPI1->STATUS & SPI_STATUS_TXRXRST_Msk) {}
    /* Set slave SPI1 data direction to output */
    SPI1->CTL |= SPI_CTL_DATDIR_Msk;

    /* Slave SPI1 prepare data to TX FIFO */
    SPI_WRITE_TX(SPI1, 0x55AA0000);
    SPI_WRITE_TX(SPI1, 0x55AA0001);
    SPI_WRITE_TX(SPI1, 0x55AA0002);
    SPI_WRITE_TX(SPI1, 0x55AA0003);

    /* Set master SPI0 to half-duplex mode */
    SPI0->CTL |= SPI_CTL_HALFDPX_Msk;
    /* Enable half-duplex will produce TXFBCLR (SPIx_FIFOCTL[9]) and RXFBCLR (SPIx_FIFOCTL[8])*/
    while(SPI0->STATUS & SPI_STATUS_TXRXRST_Msk) {}
    /* Set master SPI0 data direction to input */
    SPI0->CTL &= ~SPI_CTL_DATDIR_Msk;

    /* Master SPI0 receive four data from slave SPI1 */
    for(s_u32RxDataCount = 0; s_u32RxDataCount < 4; s_u32RxDataCount++)
    {
        /* Master write TX for generating clock */
        SPI_WRITE_TX(SPI0, 0);
        /* Wait for Rx FIFO not empty */
        while(SPI_GET_RX_FIFO_EMPTY_FLAG(SPI0)) {}
        /* Read data from RX register */
        s_au32DestinationData[s_u32RxDataCount] = SPI_READ_RX(SPI0);
    }

    /* Print the received data */
    printf("SPI0 Received data:\n");
    for(u32DataCount = 0; u32DataCount < TEST_COUNT; u32DataCount++)
    {
        printf("%d:\t0x%X\n", u32DataCount, s_au32DestinationData[u32DataCount]);
    }

    /* Reset slave RX related flags. */
    SPI1->FIFOCTL |= SPI_FIFOCTL_RXRST_Msk;
    /* Set master SPI0 data direction to output */
    SPI0->CTL |= SPI_CTL_DATDIR_Msk;
    /* Set slave SPI1 data direction to input */
    SPI1->CTL &= ~SPI_CTL_DATDIR_Msk;

    /* Master SPI0 prepare data to TX FIFO */
    SPI_WRITE_TX(SPI0, 0xAA550000);
    SPI_WRITE_TX(SPI0, 0xAA550001);
    SPI_WRITE_TX(SPI0, 0xAA550002);
    SPI_WRITE_TX(SPI0, 0xAA550003);

    /* Slave SPI1 receive four data from master SPI0 */
    for(s_u32RxDataCount = 0; s_u32RxDataCount < 4; s_u32RxDataCount++)
    {
        /* Wait for Rx FIFO not empty */
        while(SPI_GET_RX_FIFO_EMPTY_FLAG(SPI1)) {}
        /* Read data from RX register */
        s_au32DestinationData[s_u32RxDataCount] = SPI_READ_RX(SPI1);
    }

    /* Print the received data */
    printf("SPI1 Received data:\n");
    for(u32DataCount = 0; u32DataCount < TEST_COUNT; u32DataCount++)
    {
        printf("%d:\t0x%X\n", u32DataCount, s_au32DestinationData[u32DataCount]);
    }

    printf("The data transfer was done.\n");

    printf("\n\nExit SPI driver sample code.\n");

    /* Reset SPI0 */
    SPI_Close(SPI0);
    /* Reset SPI1 */
    SPI_Close(SPI1);

    while(1);
}

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
