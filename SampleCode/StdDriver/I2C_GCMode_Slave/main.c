
/******************************************************************************
 * @file     main.c
 * @version  V3.00
 * $Revision: 3 $
 * $Date: 16/10/17 2:06p $
 * @brief
 *           Show a Slave how to receive data from Master in GC (General Call) mode.
 *           This sample code needs to work with I2C_GCMode_Master.
 * @note
 * Copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define PLLCTL_SETTING  CLK_PLLCTL_48Hz_HXT
#define PLL_CLOCK       4800000

/* The size of Slave receive buffer, should adjust it if MasterTx transferring size exceeds this value */
#define SLV_DATA_BUF_SIZE    256

static volatile uint32_t s_u32SlaveBuffAddr;
static volatile uint8_t s_au8SlvData[SLV_DATA_BUF_SIZE];
static volatile uint8_t s_au8SlvRxData[3];
/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
static volatile uint8_t s_au8SlvTxData[3];
static volatile uint8_t s_u8SlvDataLen;
static volatile uint8_t s_u8SlvEndFlag = 0;

typedef void (*I2C_FUNC)(uint32_t u32Status);
volatile static I2C_FUNC s_I2C0HandlerFn = NULL;
static volatile uint8_t s_u8SlvTRxAbortFlag = 0;
static volatile uint8_t s_u8SlvWarningMsgFlag = 0;


void I2C0_IRQHandler(void);
void I2C_GCSlaveRx(uint32_t u32Status);
void SYS_Init(void);
void I2C0_Init(void);
void I2C0_Close(void);

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
/*  I2C GC mode Rx Callback Function                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_GCSlaveRx(uint32_t u32Status)
{
    if(u32Status == 0x70)                      /* Reception of the general call address and one more data byte;
                                                                        ACK has been return */
    {
        s_u8SlvDataLen = 0;
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
    }
    else if(u32Status == 0x90)                 /* Previously addressed with General Call; Data has been received
                                                   ACK has been returned */
    {
        s_au8SlvRxData[s_u8SlvDataLen] = (unsigned char) I2C_GET_DATA(I2C0);
        s_u8SlvDataLen++;

        if(s_u8SlvDataLen == 2)
        {
            s_u32SlaveBuffAddr = (uint32_t)((s_au8SlvRxData[0] << 8) + s_au8SlvRxData[1]);

            /* Exceed s_au8SlvData buffer size, use it as ring buffer  */
            while(s_u32SlaveBuffAddr >= SLV_DATA_BUF_SIZE)
            {

                if(s_u32SlaveBuffAddr == SLV_DATA_BUF_SIZE)
                {
                    /* Set flag to show warning */
                    s_u8SlvWarningMsgFlag = 1;
                }
                s_u32SlaveBuffAddr -= SLV_DATA_BUF_SIZE;
            }
        }
        if(s_u8SlvDataLen == 3)
        {
            s_au8SlvData[s_u32SlaveBuffAddr] = s_au8SlvRxData[2];
            s_u8SlvDataLen = 0;
        }
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
    }
    else if(u32Status == 0x98)                 /* Previously addressed with General Call; Data byte has been
                                                   received; NOT ACK has been returned */
    {
        s_u8SlvDataLen = 0;
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
    }
    else if(u32Status == 0xA0)                 /* A STOP or repeated START has been received while still addressed
                                                   as SLV receiver */
    {
        s_u8SlvDataLen = 0;
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
        if(s_u32SlaveBuffAddr == 0xFF)
        {
            s_u8SlvEndFlag = 1;
        }
    }
    else
    {
        printf("[SlaveTRx] Status [0x%x] Unexpected abort!!\n", u32Status);
        if(u32Status == 0x68)               /* Slave receive arbitration lost, clear SI */
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
        }
        else if(u32Status == 0xB0)          /* Address transmit arbitration lost, clear SI  */
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
        }
        else                                /* Slave bus error, stop I2C and clear SI */
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI);
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }
        s_u8SlvTRxAbortFlag = 1;
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
}

void I2C0_Init(void)
{
    /* Open I2C module and set bus clock */
    I2C_Open(I2C0, 100000);

    /* Get I2C0 Bus Clock */
    printf("I2C clock %d Hz\n", I2C_GetBusClockFreq(I2C0));

    /* Set I2C 4 Slave Addresses */
    I2C_SetSlaveAddr(I2C0, 0, 0x15, I2C_GCMODE_ENABLE);   /* Slave Address : 0x15 */
    I2C_SetSlaveAddr(I2C0, 1, 0x35, I2C_GCMODE_ENABLE);   /* Slave Address : 0x35 */
    I2C_SetSlaveAddr(I2C0, 2, 0x55, I2C_GCMODE_ENABLE);   /* Slave Address : 0x55 */
    I2C_SetSlaveAddr(I2C0, 3, 0x75, I2C_GCMODE_ENABLE);   /* Slave Address : 0x75 */

    /* Enable I2C interrupt */
    I2C_EnableInt(I2C0);
    NVIC_EnableIRQ(I2C0_IRQn);
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

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    uint32_t u32i;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Open(UART0, 115200);

    /*
        This sample code sets I2C bus clock to 100kHz. Then, accesses Slave (GC Mode) with Byte Write
        and Byte Read operations, and check if the read data is equal to the programmed data.
    */

    printf("\n");
    printf("+-----------------------------------------------------------------+\n");
    printf("| I2C Driver Sample Code (Slave) for access Slave (GC Mode)       |\n");
    printf("|                                                                 |\n");
    printf("| I2C Master (I2C0) <---> I2C Slave(I2C0)(Address: 0x00)          |\n");
    printf("+-----------------------------------------------------------------+\n");

    printf("Configure I2C0 as a slave.\n");
    printf("The I/O connection for I2C0:\n");
    printf("I2C0_SDA(PA.4), I2C0_SCL(PA.5)\n");

    /* Init I2C0 */
    I2C0_Init();

    /* I2C enter no address SLV mode */
    I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);

    /* Clear receive buffer */
    for(u32i = 0; u32i < SLV_DATA_BUF_SIZE; u32i++)
    {
        s_au8SlvData[u32i] = 0;
    }

    s_u8SlvEndFlag = 0;

    /* I2C function to Slave receive data */
    s_I2C0HandlerFn = I2C_GCSlaveRx;

    while(1)
    {
        printf("\n");
        printf("Slave(GC Mode) waiting for receiving data.\n");
        while(s_u8SlvEndFlag == 0 && s_u8SlvTRxAbortFlag == 0);

        /* When I2C abort, clear SI to enter non-addressed SLV mode*/
        if(s_u8SlvTRxAbortFlag)
        {
            s_u8SlvTRxAbortFlag = 0;

            while(I2C0->CTL0 & I2C_CTL0_SI_Msk);
            printf("I2C Slave re-start. status[0x%x]\n", I2C0->STATUS0);
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
        }
        /* Show warning message when Master transferring size exceeds slave buffer size*/
        if(s_u8SlvWarningMsgFlag)
        {
            printf("Warning: MasterTx size exceeds slaveRx buffer size!    \n");
            printf("         Please adjust the value of SLV_DATA_BUF_SIZE! \n");
            s_u8SlvWarningMsgFlag = 0;
        }

        /* Check receive data correct or not */
        for(u32i = 0; u32i < SLV_DATA_BUF_SIZE; u32i++)
        {
            s_au8SlvTxData[0] = (uint8_t)((u32i & 0xFF00) >> 8);
            s_au8SlvTxData[1] = (uint8_t)(u32i & 0x00FF);
            s_au8SlvTxData[2] = (uint8_t)(s_au8SlvTxData[1] + 3);
            if(s_au8SlvData[u32i] != s_au8SlvTxData[2])
            {
                printf("GC Mode Receive data fail.\n");
                while(1);
            }
        }

        printf("GC Mode receive data OK. Any key to continue.\n");
        getchar();
    }
}
/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/


