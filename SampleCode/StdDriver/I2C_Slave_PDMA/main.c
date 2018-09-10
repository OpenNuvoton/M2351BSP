
/******************************************************************************
 * @file     main.c
 * @version  V3.00
 * $Revision: 3 $
 * $Date: 16/10/17 2:06p $
 * @brief
 *           Demonstrate how a Slave use PDMA Rx mode receive data from a Master(Loopback).
 * @note
 * Copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define PLL_CLOCK       64000000

#define I2C_PDMA_CH        1
#define I2C_PDMA_RX_LENGTH 100

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint32_t slave_buff_addr;
volatile uint8_t g_au8RxData[3];
volatile uint8_t g_u8DeviceAddr;
volatile uint8_t g_au8TxData[3];
volatile uint8_t g_u8RxData;
volatile uint8_t g_u8DataLen;
volatile uint8_t g_u8EndFlag = 0;
volatile uint8_t g_u8DataLen1;
volatile uint8_t g_u8SlvTestLen;
volatile uint8_t g_u8SlvDataLen;
volatile uint8_t g_au8SlvData[256];
volatile int32_t g_u32IsTestOver;
volatile uint8_t g_au8MstTxData[256];
volatile uint8_t g_u8MstDataLen;
volatile uint8_t g_u8MstEndFlag = 0;

typedef void (*I2C_FUNC)(uint32_t u32Status);
static I2C_FUNC s_I2C0HandlerFn = NULL;
static I2C_FUNC s_I2C1HandlerFn = NULL;

/*---------------------------------------------------------------------------------------------------------*/
/* PDMA IRQ Handler                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
void PDMA0_IRQHandler(void)
{
    uint32_t status = PDMA_GET_INT_STATUS(PDMA0);

    if(status & PDMA_INTSTS_ABTIF_Msk)    /* abort */
    {
        /* Check if channel 1 has abort error */
        if(PDMA_GET_ABORT_STS(PDMA0) & PDMA_ABTSTS_ABTIF1_Msk)
            g_u32IsTestOver = 2;
        /* Clear abort flag of channel 1 */
        PDMA_CLR_ABORT_FLAG(PDMA0, PDMA_ABTSTS_ABTIF1_Msk);
    }
    else if(status & PDMA_INTSTS_TDIF_Msk)      /* done */
    {
        /* Check transmission of channel 1 has been transfer done */
        if(PDMA_GET_TD_STS(PDMA0) & PDMA_TDSTS_TDIF1_Msk)
            g_u32IsTestOver = 1;
        /* Clear transfer done flag of channel 1 */
        PDMA_CLR_TD_FLAG(PDMA0, PDMA_TDSTS_TDIF1_Msk);
    }
    else
        printf("unknown interrupt !!\n");
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C0 IRQ Handler                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void I2C0_IRQHandler(void)
{
    uint32_t u32Status;

    u32Status = I2C_GET_STATUS(I2C0);

    if(I2C_GET_TIMEOUT_FLAG(I2C0))
    {
        /* Clear I2C0 Timeout Flag */
        I2C_ClearTimeoutFlag(I2C0);
    }
    else
    {
        if(s_I2C0HandlerFn != NULL)
            s_I2C0HandlerFn(u32Status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C1 IRQ Handler                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void I2C1_IRQHandler(void)
{
    uint32_t u32Status;

    u32Status = I2C_GET_STATUS(I2C1);

    if(I2C_GET_TIMEOUT_FLAG(I2C1))
    {
        /* Clear I2C1 Timeout Flag */
        I2C_ClearTimeoutFlag(I2C1);
    }
    else
    {
        if(s_I2C1HandlerFn != NULL)
            s_I2C1HandlerFn(u32Status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C0 Master Tx Callback Function                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_MasterTx(uint32_t u32Status)
{
    if(u32Status == 0x08)                           /* START has been transmitted */
    {
        I2C_SET_DATA(I2C0, g_u8DeviceAddr << 1);    /* Write SLA+W to Register I2CDAT */
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        g_u8MstDataLen++;
    }
    else if(u32Status == 0x18)                  /* SLA+W has been transmitted and ACK has been received */
    {
        I2C_SET_DATA(I2C0, g_au8MstTxData[g_u8MstDataLen++]);
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    }
    else if(u32Status == 0x20)                  /* SLA+W has been transmitted and NACK has been received */
    {
        I2C_STOP(I2C0);
        I2C_START(I2C0);
    }
    else if(u32Status == 0x28)                  /* DATA has been transmitted and ACK has been received */
    {
        if(g_u8MstDataLen != I2C_PDMA_RX_LENGTH)
        {
            I2C_SET_DATA(I2C0, g_au8MstTxData[g_u8MstDataLen++]);
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }
        else
        {
            I2C_STOP(I2C0);
            g_u8MstEndFlag = 1;
        }
    }
    else
    {
        /* TO DO */
        printf("Status 0x%x is NOT processed\n", u32Status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C1 Slave TRx Callback Function                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_PDMA_SlaveRx(uint32_t u32Status)
{
    if(u32Status == 0x60)                       /* Own SLA+W has been receive; ACK has been return */
    {
        /*
            Note:
            During PDMA operation, I2C controller will not occur receive Address ACK interrupt
        */
    }
    else if(u32Status == 0x80)                 /* Previously address with own SLA address
                                                  Data has been received; ACK has been returned*/
    {
        /*
            Note:
            During PDMA operation, I2C controller will not occur receive Data ACK interrupt
        */
    }
    else if(u32Status == 0x88)                 /* Previously addressed with own SLA address; NOT ACK has
                                                   been returned */
    {
        g_u8SlvDataLen = 0;
        I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI_AA);
    }
    else if(u32Status == 0xA0)                 /* A STOP or repeated START has been received while still
                                                   addressed as Slave/Receiver*/
    {
        g_u8SlvDataLen = 0;
        I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI_AA);
    }
    else
    {
        /* TO DO */
        printf("Status 0x%x is NOT processed\n", u32Status);
        while(1);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/* I2C Slave PDMA Rx Channel Configuration                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_Slave_PDMA_Rx_Init(void)
{
    /* Reset PDMA module */
    SYS_ResetModule(PDMA0_RST);

    /* Open Channel 1 */
    PDMA_Open(PDMA0, 1 << I2C_PDMA_CH);

    /* Transfer count is I2C_PDMA_RX_LENGTH, transfer width is 8 bits(one byte) */
    PDMA_SetTransferCnt(PDMA0, I2C_PDMA_CH, PDMA_WIDTH_8, I2C_PDMA_RX_LENGTH);

    /* Set source address is (I2C0->DAT), destination address is g_au8MstRxData, Source/Destination increment size is 32 bits(one word) */
    PDMA_SetTransferAddr(PDMA0, I2C_PDMA_CH, (uint32_t)(&(I2C1->DAT)), PDMA_SAR_FIX, (uint32_t)g_au8SlvData, PDMA_DAR_INC);

    /* Single request type */
    PDMA_SetBurstType(PDMA0, I2C_PDMA_CH, PDMA_REQ_SINGLE, 0);

    /* Set request source; set basic mode. */
    PDMA_SetTransferMode(PDMA0, I2C_PDMA_CH, PDMA_I2C1_RX, FALSE, 0);

    /* Set PDMA Basic Mode */
    PDMA0->DSCT[I2C_PDMA_CH].CTL = (PDMA0->DSCT[I2C_PDMA_CH].CTL & ~PDMA_DSCT_CTL_OPMODE_Msk) | PDMA_OP_BASIC;

    /* Enable interrupt */
    PDMA_EnableInt(PDMA0, I2C_PDMA_CH, PDMA_INT_TRANS_DONE);

    /* Enable NVIC for PDMA */
    NVIC_EnableIRQ(PDMA0_IRQn);

    g_u32IsTestOver = 0;
}


void SYS_Init(void)
{
    /* Set PF multi-function pins for XT1_OUT(PF.2) and XT1_IN(PF.3) */
    SYS->GPF_MFPL = (SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF2MFP_Msk)) | SYS_GPF_MFPL_PF2MFP_XT1_OUT;
    SYS->GPF_MFPL = (SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF3MFP_Msk)) | SYS_GPF_MFPL_PF3MFP_XT1_IN;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
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

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART module clock source as HXT `and UART module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HXT, CLK_CLKDIV0_UART0(1));

    /* Enable I2C0 peripheral clock */
    CLK_EnableModuleClock(I2C0_MODULE);

    /* Enable I2C1 peripheral clock */
    CLK_EnableModuleClock(I2C1_MODULE);

    /* Enable PDMA0 peripheral clock */
    CLK_EnableModuleClock(PDMA0_MODULE);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and CyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & (~(UART0_RXD_PB12_Msk | UART0_TXD_PB13_Msk))) | UART0_RXD_PB12 | UART0_TXD_PB13;

    /* Set PA multi-function pins for I2C0 SDA and SCL */
    SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA4MFP_Msk | SYS_GPA_MFPL_PA5MFP_Msk);
    SYS->GPA_MFPL |= (SYS_GPA_MFPL_PA4MFP_I2C0_SDA | SYS_GPA_MFPL_PA5MFP_I2C0_SCL);

    /* Set PA multi-function pins for I2C1 SDA and SCL */
    SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA2MFP_Msk | SYS_GPA_MFPL_PA3MFP_Msk);
    SYS->GPA_MFPL |= (SYS_GPA_MFPL_PA2MFP_I2C1_SDA | SYS_GPA_MFPL_PA3MFP_I2C1_SCL);
}

void I2C0_Init(void)
{
    /* Open I2C module and set bus clock */
    I2C_Open(I2C0, 100000);

    /* Get I2C0 Bus Clock */
    printf("I2C0 clock %d Hz\n", I2C_GetBusClockFreq(I2C0));

    /* Set I2C 4 Slave Addresses */
    I2C_SetSlaveAddr(I2C0, 0, 0x15, 0);   /* Slave Address : 0x15 */
    I2C_SetSlaveAddr(I2C0, 1, 0x35, 0);   /* Slave Address : 0x35 */
    I2C_SetSlaveAddr(I2C0, 2, 0x55, 0);   /* Slave Address : 0x55 */
    I2C_SetSlaveAddr(I2C0, 3, 0x75, 0);   /* Slave Address : 0x75 */

    /* Set I2C0 4 Slave Addresses Mask */
    I2C_SetSlaveAddrMask(I2C0, 0, 0x01);
    I2C_SetSlaveAddrMask(I2C0, 1, 0x04);
    I2C_SetSlaveAddrMask(I2C0, 2, 0x01);
    I2C_SetSlaveAddrMask(I2C0, 3, 0x04);

    /* Enable I2C interrupt */
    I2C_EnableInt(I2C0);
    NVIC_EnableIRQ(I2C0_IRQn);
}

void I2C1_Init(void)
{
    /* Open I2C1 module and set bus clock */
    I2C_Open(I2C1, 100000);

    /* Get I2C1 Bus Clock */
    printf("I2C1 clock %d Hz\n", I2C_GetBusClockFreq(I2C1));

    /* Set I2C1 4 Slave Addresses */
    I2C_SetSlaveAddr(I2C1, 0, 0x16, 0);   /* Slave Address : 0x16 */
    I2C_SetSlaveAddr(I2C1, 1, 0x36, 0);   /* Slave Address : 0x36 */
    I2C_SetSlaveAddr(I2C1, 2, 0x56, 0);   /* Slave Address : 0x56 */
    I2C_SetSlaveAddr(I2C1, 3, 0x76, 0);   /* Slave Address : 0x76 */

    /* Set I2C1 4 Slave Addresses Mask */
    I2C_SetSlaveAddrMask(I2C1, 0, 0x04);
    I2C_SetSlaveAddrMask(I2C1, 1, 0x02);
    I2C_SetSlaveAddrMask(I2C1, 2, 0x04);
    I2C_SetSlaveAddrMask(I2C1, 3, 0x02);

    /* Enable I2C1 interrupt */
    I2C_EnableInt(I2C1);
    NVIC_EnableIRQ(I2C1_IRQn);
}

void I2C0_Close(void)
{
    /* Disable I2C0 interrupt and clear corresponding NVIC bit */
    I2C_DisableInt(I2C0);
    NVIC_DisableIRQ(I2C0_IRQn);

    /* Disable I2C0 and close I2C0 clock */
    I2C_Close(I2C0);
    CLK_DisableModuleClock(I2C0_MODULE);

}

void I2C1_Close(void)
{
    /* Disable I2C1 interrupt and clear corresponding NVIC bit */
    I2C_DisableInt(I2C1);
    NVIC_DisableIRQ(I2C1_IRQn);

    /* Disable I2C1 and close I2C1 clock */
    I2C_Close(I2C1);
    CLK_DisableModuleClock(I2C1_MODULE);
}

void I2C_Write_to_Slave_PDMA_RX(uint8_t u8SlvAddr)
{
    uint32_t i;


    g_au8MstTxData[0] = ((u8SlvAddr << 1) | 0x00);
    g_u8DeviceAddr = u8SlvAddr;

    /* I2C0 prepare transmit data bytes */
    for(i = 1; i < 200; i++)
    {
        g_au8MstTxData[i] = i;
    }

    g_u8MstDataLen = 0;
    g_u8MstEndFlag = 0;

    /* I2C0 function to write data to slave */
    s_I2C0HandlerFn = (I2C_FUNC)I2C_MasterTx;

    /* I2C0 as master sends START signal */
    I2C_START(I2C0);

    /* Wait I2C0 Tx Finish */
    while(g_u8MstEndFlag == 0);
    g_u8MstEndFlag = 0;

    /* Waiting for I2C0 bus become free */
    while((I2C0->STATUS1 & I2C_STATUS1_ONBUSY_Msk) ==  I2C_STATUS1_ONBUSY_Msk);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    uint32_t u32i;
    uint8_t u8Err;

    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();
    /* Lock protected registers */
    SYS_LockReg();

    /* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Open(UART0, 115200);

    /*
        This sample code sets I2C bus clock to 100kHz. Then, Master accesses Slave with Byte Write
        and Byte Read operations, and check if the read data is equal to the programmed data.
    */

    printf("+----------------------------------------------------------+\n");
    printf("| I2C Driver Sample Code. Show how a Master access [Slave] |\n");
    printf("| use [PDMA Rx] mode                                       |\n");
    printf("| I2C Master (I2C0) <---> I2C Slave(I2C1)                  |\n");
    printf("+----------------------------------------------------------+\n");

    printf("\n");
    printf("Configure I2C0 as Master, and I2C1 as a Slave.\n");
    printf("The I/O connection I2C0 to I2C1:\n");
    printf("I2C0_SDA(PA.4), I2C0_SCL(PA.5)\n");
    printf("I2C1_SDA(PA.2), I2C1_SCL(PA.3)\n\n");

    /* Init I2C0 */
    I2C0_Init();

    /* Init I2C1 */
    I2C1_Init();

    /* I2C1 enter no address SLV mode */
    I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI_AA);

    /* Clear Slave receive data buffer */
    for(u32i = 0; u32i < 256; u32i++)
    {
        g_au8SlvData[u32i] = 0;
    }

    /* Init Slave PDMA RX */
    I2C_Slave_PDMA_Rx_Init();

    /* Enable I2C1 PDMA RX mode */
    I2C1->CTL1 |= I2C_CTL1_RXPDMAEN_Msk;

    /* I2C1 function to Slave receive/transmit data */
    s_I2C1HandlerFn = I2C_PDMA_SlaveRx;
    printf("\nI2C1 Slave Mode is Running.\n\n");

    /* I2C0 access I2C1*/
    I2C_Write_to_Slave_PDMA_RX(0x16);

    /* Waiting for PDMA channel 1 transfer done*/
    while(g_u32IsTestOver == 0);
    g_u32IsTestOver = 0;

    u8Err = 0;
    for(u32i = 0; u32i < I2C_PDMA_RX_LENGTH; u32i++)
    {
        /* Compare Master Tx data with Slave Rx data */
        if(g_au8MstTxData[u32i] != g_au8SlvData[u32i])
        {
            u8Err = 1;
            printf("[%03d]: Tx[0x%X] != Rx[0x%X]\n", u32i, g_au8MstTxData[u32i], g_au8SlvData[u32i]);
        }
    }

    if(u8Err)
        printf("Master write data to Slave(PDMA RX) fail...\n");
    else
        printf("Master write data to Slave(PDMA RX) pass...\n");


    /* Disable PDMA channel */
    PDMA_Close(PDMA0);

    /* Disable PDMA Interrupt */
    PDMA_DisableInt(PDMA0, I2C_PDMA_CH, PDMA_INT_TRANS_DONE);
    NVIC_DisableIRQ(PDMA0_IRQn);

    s_I2C0HandlerFn = NULL;
    s_I2C1HandlerFn = NULL;

    /* Close I2C0,I2C1 */
    I2C0_Close();
    I2C1_Close();

    while(1);

}
/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/


