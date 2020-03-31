
/******************************************************************************
 * @file     main.c
 * @version  V3.00
 * $Revision: 3 $
 * $Date: 19/07/24 2:06p $
 * @brief
 *           Demonstrate I2C PDMA mode and need to connect I2C0 (master) and I2C1 (slave).
 * @note
 * Copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define PLL_CLOCK       64000000

#define I2C0_PDMA_TX_CH      0
#define I2C1_PDMA_RX_CH      1
#define I2C0_PDMA_RX_CH      2
#define I2C1_PDMA_TX_CH      3
#define PDMA_TEST_LENGTH    16

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
static uint8_t s_au8MasterTxBuffer[PDMA_TEST_LENGTH];
static uint8_t s_au8MasterRxBuffer[PDMA_TEST_LENGTH];
static uint8_t s_au8SlaveTxBuffer[PDMA_TEST_LENGTH];
static uint8_t s_au8SlaveRxBuffer[PDMA_TEST_LENGTH];
static volatile uint32_t s_u32PDMADoneFlag = 0;

static volatile uint8_t s_u8DeviceAddr = 0x16;
static volatile uint8_t s_u8MasterDataLen = 0;
static volatile uint8_t s_u8SlaveDataLen = 0;
static volatile uint16_t s_u16SlaveBufferAddr = 0;

typedef void (*I2C_FUNC)(uint32_t u32Status);

static I2C_FUNC s_I2C0HandlerFn = NULL;
static I2C_FUNC s_I2C1HandlerFn = NULL;


void PDMA0_IRQHandler(void);
void I2C0_IRQHandler(void);
void I2C1_IRQHandler(void);
void I2C_PDMA_SlaveRx(uint32_t u32Status);
void I2C_PDMA_SlaveTx(uint32_t u32Status);
void I2C_PDMA_MasterTx(uint32_t u32Status);
void I2C_PDMA_MasterRx(uint32_t u32Status);
void SYS_Init(void);
void UART0_Init(void);
void I2C0_Init(void);
void I2C1_Init(void);
void PDMA_Init(void);
void I2C_PDMA(void);

/*---------------------------------------------------------------------------------------------------------*/
/*  PDMA IRQ Handler                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void PDMA0_IRQHandler(void)
{
    uint32_t u32Status = PDMA0->TDSTS;

    /* Master TX */
    if (u32Status & (0x1 << I2C0_PDMA_TX_CH))
    {
        printf("\n I2C0 Tx done  ");
        PDMA0->TDSTS = 0x1 << I2C0_PDMA_TX_CH;
    }

    /* Master RX */
    if (u32Status & (0x1 << I2C0_PDMA_RX_CH))
    {
        printf("\n I2C0 Rx done  ");
        PDMA0->TDSTS = 0x1 << I2C0_PDMA_RX_CH;
        s_u32PDMADoneFlag = 1;
    }

    /* Slave RX */
    if (u32Status & (0x1 << I2C1_PDMA_RX_CH))
    {
        printf("\n I2C1 Rx done  ");
        PDMA0->TDSTS = 0x1 << I2C1_PDMA_RX_CH;
        s_u32PDMADoneFlag = 1;
    }

    /* Slave TX */
    if (u32Status & (0x1 << I2C1_PDMA_TX_CH))
    {
        printf("\n I2C1 Tx done  ");
        PDMA0->TDSTS = 0x1 << I2C1_PDMA_TX_CH;
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C0 IRQ Handler                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void I2C0_IRQHandler(void)
{
    uint32_t u32Status;

    u32Status = I2C_GET_STATUS(I2C0);

    if (I2C_GET_TIMEOUT_FLAG(I2C0))
    {
        /* Clear I2C0 Timeout Flag */
        I2C_ClearTimeoutFlag(I2C0);
    }
    else
    {
        if (s_I2C0HandlerFn != NULL)
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

    if (I2C_GET_TIMEOUT_FLAG(I2C1))
    {
        /* Clear I2C1 Timeout Flag */
        I2C_ClearTimeoutFlag(I2C1);
    }
    else
    {
        if (s_I2C1HandlerFn != NULL)
            s_I2C1HandlerFn(u32Status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C1 PDMA Slave Rx Callback Function                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_PDMA_SlaveRx(uint32_t u32Status)
{
    if (u32Status == 0x60)                      /* Own SLA+W has been receive; ACK has been return */
    {
        /*
            Note:
            During PDMA operation, I2C controller will not occur receive Address ACK interrupt
        */
    }
    else if (u32Status == 0x80)                 /* Previously address with own SLA address
                                                  Data has been received; ACK has been returned*/
    {
        /*
            Note:
            During PDMA operation, I2C controller will not occur receive Data ACK interrupt
        */
    }
    else if (u32Status == 0x88)                 /* Previously addressed with own SLA address; NOT ACK has
                                                   been returned */
    {
        I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI_AA);
    }
    else if (u32Status == 0xA0)                 /* A STOP or repeated START has been received while still
                                                   addressed as Slave/Receiver*/
    {
        I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI_AA);
    }
    else
    {
        /* TO DO */
        printf("Status 0x%x is NOT processed\n", u32Status);
        while (1);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C1 PDMA Slave Tx Callback Function                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_PDMA_SlaveTx(uint32_t u32Status)
{
    uint8_t u8data;

    if (u32Status == 0x60)                       /* Own SLA+W has been receive; ACK has been return */
    {
        s_u8SlaveDataLen = 0;
        I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI_AA);
    }
    else if (u32Status == 0x80)                 /* Previously address with own SLA address
                                                   Data has been received; ACK has been returned*/
    {
        u8data = (unsigned char) I2C_GET_DATA(I2C1);
        s_au8SlaveRxBuffer[s_u8SlaveDataLen++] = u8data;
        s_u16SlaveBufferAddr = (uint16_t)((s_au8SlaveRxBuffer[0] << 8) + s_au8SlaveRxBuffer[1]);

        if(s_u8SlaveDataLen == 2)
        {

            I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI);
        }
        else
        {
            I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI_AA);
        }

    }
    else if (u32Status == 0x88)                 /* Previously addressed with own SLA address; NOT ACK has
                                                   been returned */
    {
        I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI_AA);
    }
    else if (u32Status == 0xA0)                 /* A STOP or repeated START has been received while still
                                                   addressed as Slave/Receiver*/
    {
        I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI_AA);

        /* Enable I2C1 Slave TX */
        I2C1->CTL1 = I2C_CTL1_TXPDMAEN_Msk;
    }
    else if (u32Status == 0xA8)                  /* Own SLA+R has been receive; ACK has been return */
    {
        /*
           Note:
           During PDMA operation, I2C controller will not occur START interrupt
        */
    }
    else if (u32Status == 0xB8)                  /* Data byte in I2CDAT has been transmitted ACK has been received */
    {
        /*
           Note:
           During PDMA operation, I2C controller will not occur START interrupt
        */
    }
    else if (u32Status == 0xC0)                 /* Data byte or last data in I2CDAT has been transmitted
                                                   Not ACK has been received */
    {
        I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI_AA);
    }
    else if (u32Status == 0x88)                 /* Previously addressed with own SLA address; NOT ACK has
                                                   been returned */
    {
        I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI_AA);
    }
    else
    {
        /* TO DO */
        printf("Status 0x%x is NOT processed\n", u32Status);

        while (1);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C0 PDMA Master Tx Callback Function                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_PDMA_MasterTx(uint32_t u32Status)
{
    if (u32Status == 0x08)                      /* START has been transmitted */
    {
        /*
           Note:
           During PDMA operation, I2C controller will not occur START interrupt
        */
    }
    else if (u32Status == 0x10)                 /* Repeat START has been transmitted */
    {

    }
    else if (u32Status == 0x18)                 /* SLA+W has been transmitted and ACK has been received */
    {
        /*
           Note:
           During PDMA operation, I2C controller will not occur address ACK interrupt
        */
    }
    else if (u32Status == 0x20)                 /* SLA+W has been transmitted and NACK has been received */
    {
        I2C_STOP(I2C0);
        I2C_START(I2C0);
    }
    else if (u32Status == 0x28)                 /* DATA has been transmitted and ACK has been received */
    {
        /*
           Note:
           During PDMA operation, I2C controller will not occur data ACK interrupt
        */
    }
    else
    {
        /* TO DO */
        printf("Status 0x%x is NOT processed\n", u32Status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C1 PDMA Master Rx Callback Function                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_PDMA_MasterRx(uint32_t u32Status)
{
    if(u32Status == 0x08)                          /* START has been transmitted and prepare SLA+W */
    {
        I2C_SET_DATA(I2C0, (uint32_t)((s_u8DeviceAddr << 1) | 0x00));     /* Write SLA+W to Register I2CDAT */
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    }
    else if(u32Status == 0x18)                     /* SLA+W has been transmitted and ACK has been received */
    {
        s_u8MasterDataLen = 1;
        I2C_SET_DATA(I2C0, s_au8MasterTxBuffer[s_u8MasterDataLen++]);
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    }
    else if(u32Status == 0x20)                     /* SLA+W has been transmitted and NACK has been received */
    {
        I2C_STOP(I2C0);
        I2C_START(I2C0);
    }
    else if(u32Status == 0x28)                     /* DATA has been transmitted and ACK has been received */
    {
        if(s_u8MasterDataLen <= 2)
        {
            I2C_SET_DATA(I2C0, s_au8MasterTxBuffer[s_u8MasterDataLen++]);
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }
        else
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STA_SI);
        }
    }
    else if(u32Status == 0x10)                    /* Repeat START has been transmitted and prepare SLA+R */
    {
        I2C_SET_DATA(I2C0, (uint32_t)((s_u8DeviceAddr << 1) | 0x01));   /* Write SLA+R to Register I2CDAT */
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    }
    else if(u32Status == 0x40)                    /* SLA+R has been transmitted and ACK has been received */
    {
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);

        /* Enable I2C0 Master RX */
        I2C0->CTL1 = I2C_CTL1_RXPDMAEN_Msk;
    }
    else if(u32Status == 0x50)                    /* DATA has been received and ACK has been returned */
    {
        /*
           Note:
           During PDMA operation, I2C controller will not occur receive data ACK interrupt
        */
    }
    else if(u32Status == 0x58)                    /* DATA has been received and NACK has been returned */
    {
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    }
    else
    {
        /* TO DO */
        printf("Status 0x%x is NOT processed\n", u32Status);
    }
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

void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset IP */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);
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

    /* Enable I2C1 interrupt */
    I2C_EnableInt(I2C1);
    NVIC_EnableIRQ(I2C1_IRQn);
}

void PDMA_Init(void)
{
    /* Reset PDMA module */
    SYS_ResetModule(PDMA0_RST);

    /* Open PDMA Channel */
    PDMA_Open(PDMA0, 1 << I2C0_PDMA_TX_CH); /* Channel 0 for I2C0 TX */
    PDMA_Open(PDMA0, 1 << I2C1_PDMA_RX_CH); /* Channel 1 for I2C1 RX */
    PDMA_Open(PDMA0, 1 << I2C0_PDMA_RX_CH); /* Channel 0 for I2C0 RX */
    PDMA_Open(PDMA0, 1 << I2C1_PDMA_TX_CH); /* Channel 1 for I2C1 TX */
    /* Select basic mode */
    PDMA_SetTransferMode(PDMA0, I2C0_PDMA_TX_CH, PDMA_I2C0_TX, 0, 0);
    PDMA_SetTransferMode(PDMA0, I2C1_PDMA_RX_CH, PDMA_I2C1_RX, 0, 0);
    PDMA_SetTransferMode(PDMA0, I2C0_PDMA_RX_CH, PDMA_I2C0_RX, 0, 0);
    PDMA_SetTransferMode(PDMA0, I2C1_PDMA_TX_CH, PDMA_I2C1_TX, 0, 0);
    /* Set data width and transfer count */
    PDMA_SetTransferCnt(PDMA0, I2C0_PDMA_TX_CH, PDMA_WIDTH_8, PDMA_TEST_LENGTH);
    PDMA_SetTransferCnt(PDMA0, I2C1_PDMA_RX_CH, PDMA_WIDTH_8, PDMA_TEST_LENGTH);
    PDMA_SetTransferCnt(PDMA0, I2C0_PDMA_RX_CH, PDMA_WIDTH_8, PDMA_TEST_LENGTH - 3); /* except Slave Address and two bytes Data Address */
    PDMA_SetTransferCnt(PDMA0, I2C1_PDMA_TX_CH, PDMA_WIDTH_8, PDMA_TEST_LENGTH - 3); /* except Slave Address and two bytes Data Address */
    /* Set PDMA Transfer Address */
    PDMA_SetTransferAddr(PDMA0, I2C0_PDMA_TX_CH, ((uint32_t)(&s_au8MasterTxBuffer[0])), PDMA_SAR_INC, (uint32_t)(&(I2C0->DAT)), PDMA_DAR_FIX);
    PDMA_SetTransferAddr(PDMA0, I2C1_PDMA_RX_CH, (uint32_t)(&(I2C1->DAT)), PDMA_SAR_FIX, ((uint32_t)(&s_au8SlaveRxBuffer[0])), PDMA_DAR_INC);
    PDMA_SetTransferAddr(PDMA0, I2C0_PDMA_RX_CH, (uint32_t)(&(I2C0->DAT)), PDMA_SAR_FIX, ((uint32_t)(&s_au8MasterRxBuffer[0])), PDMA_DAR_INC);
    PDMA_SetTransferAddr(PDMA0, I2C1_PDMA_TX_CH, ((uint32_t)(&s_au8SlaveTxBuffer[0])), PDMA_SAR_INC, (uint32_t)(&(I2C1->DAT)), PDMA_DAR_FIX);
    /* Select Single Request */
    PDMA_SetBurstType(PDMA0, I2C0_PDMA_TX_CH, PDMA_REQ_SINGLE, 0);
    PDMA_SetBurstType(PDMA0, I2C1_PDMA_RX_CH, PDMA_REQ_SINGLE, 0);
    PDMA_SetBurstType(PDMA0, I2C0_PDMA_RX_CH, PDMA_REQ_SINGLE, 0);
    PDMA_SetBurstType(PDMA0, I2C1_PDMA_TX_CH, PDMA_REQ_SINGLE, 0);
    /* Set PDMA Basic Mode */
    PDMA0->DSCT[I2C0_PDMA_TX_CH].CTL = (PDMA0->DSCT[I2C0_PDMA_TX_CH].CTL & ~PDMA_DSCT_CTL_OPMODE_Msk) | PDMA_OP_BASIC;
    PDMA0->DSCT[I2C1_PDMA_RX_CH].CTL = (PDMA0->DSCT[I2C1_PDMA_RX_CH].CTL & ~PDMA_DSCT_CTL_OPMODE_Msk) | PDMA_OP_BASIC;
    PDMA0->DSCT[I2C0_PDMA_RX_CH].CTL = (PDMA0->DSCT[I2C0_PDMA_RX_CH].CTL & ~PDMA_DSCT_CTL_OPMODE_Msk) | PDMA_OP_BASIC;
    PDMA0->DSCT[I2C1_PDMA_TX_CH].CTL = (PDMA0->DSCT[I2C1_PDMA_TX_CH].CTL & ~PDMA_DSCT_CTL_OPMODE_Msk) | PDMA_OP_BASIC;

    PDMA_EnableInt(PDMA0, I2C0_PDMA_TX_CH, PDMA_INT_TRANS_DONE);
    PDMA_EnableInt(PDMA0, I2C1_PDMA_RX_CH, PDMA_INT_TRANS_DONE);
    PDMA_EnableInt(PDMA0, I2C0_PDMA_RX_CH, PDMA_INT_TRANS_DONE);
    PDMA_EnableInt(PDMA0, I2C1_PDMA_TX_CH, PDMA_INT_TRANS_DONE);
    NVIC_EnableIRQ(PDMA0_IRQn);

}

void I2C_PDMA(void)
{
    uint8_t i;

    for (i = 0; i < PDMA_TEST_LENGTH; i++)
    {
        s_au8MasterTxBuffer[i] = i;
        s_au8SlaveRxBuffer[i] = 0xff;
    }

    s_au8MasterTxBuffer[0] = (uint8_t)((s_u8DeviceAddr << 1) | 0x00);   /* 1 byte SLV + W */
    s_au8MasterTxBuffer[1] = 0x00;                             /* 2 bytes Data address */
    s_au8MasterTxBuffer[2] = 0x00;

    PDMA_Init();

    /* I2C enter no address SLV mode */
    I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI | I2C_CTL_AA);
    /* Enable I2C1 PDMA RX mode */
    I2C1->CTL1 = I2C_CTL1_RXPDMAEN_Msk;

    /* I2C1 function to Slave receive data */
    s_I2C1HandlerFn = I2C_PDMA_SlaveRx;

    s_u32PDMADoneFlag = 0;

    /* Enable I2C TX */
    I2C0->CTL1 = I2C_CTL1_TXPDMAEN_Msk;

    s_I2C0HandlerFn = (I2C_FUNC)I2C_PDMA_MasterTx;

    /* Send START condition, start the PDMA data transmit */
    I2C_START(I2C0);

    while (!s_u32PDMADoneFlag);

    /* Disable I2C0 PDMA TX mode */
    I2C0->CTL1 &= ~I2C_CTL1_TXPDMAEN_Msk;
    /* Disable I2C1 PDMA RX mode */
    I2C1->CTL1 &= ~I2C_CTL1_RXPDMAEN_Msk;

    for (i = 0; i < PDMA_TEST_LENGTH; i++)
    {
        if (s_au8SlaveRxBuffer[i] != s_au8MasterTxBuffer[i])
        {
            printf("\n Slave Receive Data Compare Error !!");
            while (1);
        }
        else
        {
            if(i > 2)
                s_au8SlaveTxBuffer[i-3] = s_au8MasterTxBuffer[i];
        }
    }

    /* Test Master RX and Slave TX with PDMA function */

    /* I2C0 function to Master receive data */
    s_I2C0HandlerFn = (I2C_FUNC)I2C_PDMA_MasterRx;
    /* I2C1 function to Slave transmit data */
    s_I2C1HandlerFn = I2C_PDMA_SlaveTx;

    s_u32PDMADoneFlag = 0;

    /* Send START condition */
    I2C_START(I2C0);

    while (!s_u32PDMADoneFlag);
    I2C_STOP(I2C0);
    /* Disable I2C0 PDMA RX mode */
    I2C0->CTL1 &= ~I2C_CTL1_RXPDMAEN_Msk;
    /* Disable I2C1 PDMA TX mode */
    I2C1->CTL1 &= ~I2C_CTL1_TXPDMAEN_Msk;

    for (i = 0; i < PDMA_TEST_LENGTH - 3; i++)
    {
        if (s_au8MasterRxBuffer[i] != s_au8MasterTxBuffer[i+3])
        {
            printf("\n Slave Receive Data Compare Error !!");

            while (1);
        }
    }

    printf("\nI2C PDMA test Pass.\n");
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 for printf */
    UART0_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /*
        This sample code sets I2C bus clock to 100kHz. Then, Master accesses Slave with Byte Write
        and Byte Read operations, and check if the read data is equal to the programmed data.
    */

    printf("+-------------------------------------------------------+\n");
    printf("| I2C Driver Sample Code for PDMA                       |\n");
    printf("|                                                       |\n");
    printf("| I2C Master (I2C0) <---> I2C Slave(I2C1)               |\n");
    printf("+-------------------------------------------------------+\n");

    printf("\n");
    printf("Configure I2C0 as Master, and I2C1 as a Slave.\n");
    printf("The I/O connection I2C0 to I2C1:\n");
    printf("I2C0_SDA(PA.4), I2C0_SCL(PA.5)\n");
    printf("I2C1_SDA(PA.2), I2C1_SCL(PA.3)\n\n");

    /* Init I2C0 */
    I2C0_Init();
    /* Init I2C1 */
    I2C1_Init();

    I2C_PDMA();

    while (1);
}
