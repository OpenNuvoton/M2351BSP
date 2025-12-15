/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief
 *           Show how to control SMBus interface and use SMBus protocol between Host and Slave.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define SMBUS_ALERT_RESPONSE_ADDRESS    0x0C
#define SMBUS_DEFAULT_ADDRESS           0x61
#define ARP_COMMAND                     0x01

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
const uint8_t g_u8MasterAddr[4] = {0x15, 0x35, 0x55, 0x75};
const uint8_t g_u8SlaveAddr[4] = {0x18, 0x38, 0x58, 0x78};

volatile uint8_t g_au8RxData[4];
volatile uint8_t g_u8DeviceAddr;
volatile uint8_t g_au8TxData[4];
volatile uint8_t g_u8DataLen0;
volatile uint8_t g_u8EndFlag = 0;
volatile uint8_t g_u8SendPEC = 0;
volatile uint8_t g_u8AlertInt0 = 0;
volatile uint8_t g_u8AlertAddrAck0 = 0;

typedef void (*I2C_FUNC)(uint32_t u32Status);

static volatile I2C_FUNC s_I2C0HandlerFn = NULL;

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C0 IRQ Handler                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void I2C0_IRQHandler(void)
{
    uint32_t u32Status;

    u32Status = I2C_GET_STATUS(I2C0);
    /* Check Transmit byte done interrupt flag */
    if((I2C_SMBusGetStatus(I2C0) & I2C_BUSSTS_BCDONE_Msk) == I2C_BUSSTS_BCDONE_Msk)
    {
        I2C_SMBusClearInterruptFlag(I2C0, I2C_BUSSTS_BCDONE_Msk);
        return;
    }

    /* Occur receive PEC packet error */
    if((I2C_SMBusGetStatus(I2C0) & I2C_BUSSTS_PECERR_Msk) == I2C_BUSSTS_PECERR_Msk)
    {
        I2C_SMBusClearInterruptFlag(I2C0, I2C_BUSSTS_PECERR_Msk);
        return;
    }

    /* Check Alert Interrupt when I2C0 is Host */
    if(((I2C_SMBusGetStatus(I2C0) & I2C_BUSSTS_ALERT_Msk) == I2C_BUSSTS_ALERT_Msk) &
            ((I2C0->BUSCTL & I2C_BUSCTL_BMHEN_Msk) == I2C_BUSCTL_BMHEN_Msk))
    {
        I2C_SMBusClearInterruptFlag(I2C0, I2C_BUSSTS_ALERT_Msk);
        g_u8AlertInt0 = 1;
        return ;
    }

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
/*  I2C Master Tx Callback Function                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_MasterTx(uint32_t u32Status)
{
    if(u32Status == 0x08)                       /* START has been transmitted */
    {
        I2C_SET_DATA(I2C0, g_u8DeviceAddr << 1);     /* Write SLA+W to Register I2CDAT */
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    }
    else if(u32Status == 0x18)                  /* SLA+W has been transmitted and ACK has been received */
    {
        I2C_SET_DATA(I2C0, g_au8TxData[g_u8DataLen0++]);
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    }
    else if(u32Status == 0x20)                  /* SLA+W has been transmitted and NACK has been received */
    {
        I2C_STOP(I2C0);
        I2C_START(I2C0);
    }
    else if(u32Status == 0x28)                  /* DATA has been transmitted and ACK has been received */
    {
        if(g_u8DataLen0 != 3)
        {
            I2C_SET_DATA(I2C0, g_au8TxData[g_u8DataLen0++]);
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }
        else
        {
            if(g_u8SendPEC == 0)
            {
                g_u8SendPEC = 1;
                I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
            }
            else
            {
                I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI);
                g_u8EndFlag = 1;
            }
        }
    }
    else
    {
        /* TO DO */
        printf("Status 0x%x is NOT processed\n", u32Status);
        if(u32Status == 0x38)                   /* Master arbitration lost, stop I2C and clear SI */
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI);
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }
        else if(u32Status == 0x00)              /* Master bus error, stop I2C and clear SI */
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI);
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }
        else if(u32Status == 0x30)              /* Master transmit data NACK, stop I2C and clear SI */
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI);
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }
        else if(u32Status == 0x48)              /* Master receive address NACK, stop I2C and clear SI */
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI);
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }
        else
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI);
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }
    }
    I2C_WAIT_SI_CLEAR(I2C0);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C Master Alert Callback Function                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_MasterAlert(uint32_t u32Status)
{
    if(u32Status == 0x08)                       /* START has been transmitted */
    {
        I2C_SET_DATA(I2C0, (g_u8DeviceAddr << 1) + 1);             /* Write SLA+R to Register I2CDAT */
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    }
    else if(u32Status == 0x18)                  /* SLA+W has been transmitted and ACK has been received */
    {
        I2C_SET_DATA(I2C0, g_au8TxData[g_u8DataLen0++]);
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    }
    else if(u32Status == 0x20)                  /* SLA+W has been transmitted and NACK has been received */
    {
        I2C_STOP(I2C0);
        I2C_START(I2C0);
    }
    else if(u32Status == 0x40)
    {
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
    }
    else if(u32Status == 0x50)
    {
        if(g_u8DataLen0 == 0)
        {
            g_au8RxData[g_u8DataLen0] = (unsigned char)I2C_GET_DATA(I2C0);
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
            g_u8DataLen0++;
        }
        else
        {
            g_au8RxData[g_u8DataLen0] = (unsigned char) I2C_GET_DATA(I2C0);
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI_AA);
            g_u8AlertAddrAck0 = 1;
        }
    }
    else if(u32Status == 0x28)                  /* DATA has been transmitted and ACK has been received */
    {
        if(g_u8DataLen0 != 3)
        {
            I2C_SET_DATA(I2C0, g_au8TxData[g_u8DataLen0++]);
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }
        else
        {
            if(g_u8SendPEC == 0)
            {
                g_u8SendPEC = 1;
                I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
            }
            else
            {
                I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI);
                g_u8EndFlag = 1;
            }
        }
    }
    else
    {
        /* TO DO */
        printf("Status 0x%x is NOT processed\n", u32Status);
        if(u32Status == 0x38)                   /* Master arbitration lost, stop I2C and clear SI */
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI);
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }
        else if(u32Status == 0x00)              /* Master bus error, stop I2C and clear SI */
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI);
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }
        else if(u32Status == 0x30)              /* Master transmit data NACK, stop I2C and clear SI */
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI);
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }
        else if(u32Status == 0x48)              /* Master receive address NACK, stop I2C and clear SI */
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI);
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }
        else
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI);
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }
    }
    I2C_WAIT_SI_CLEAR(I2C0);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C Master Default Address and Acknowledge by Manual Callback Function                                 */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_MasterDefaultAddrACKM(uint32_t u32Status)
{
    if(u32Status == 0x08)                            /* START has been transmitted */
    {
        I2C_SET_DATA(I2C0, g_u8DeviceAddr << 1);     /* Write SLA+W to Register I2CDAT */
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    }
    else if(u32Status == 0x18)                       /* SLA+W has been transmitted and ACK has been received */
    {
        I2C_SET_DATA(I2C0, g_au8TxData[g_u8DataLen0++]);
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    }
    else if(u32Status == 0x20)                       /* SLA+W has been transmitted and NACK has been received */
    {
        I2C_STOP(I2C0);
        I2C_START(I2C0);
    }
    else if(u32Status == 0x28)                       /* DATA has been transmitted and ACK has been received */
    {
        if(g_u8SendPEC == 0)
        {
            g_u8SendPEC = 1;
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }
        else
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI);
            g_u8EndFlag = 1;
        }
    }
    else
    {
        /* TO DO */
        printf("Status 0x%x is NOT processed\n", u32Status);
        if(u32Status == 0x38)                   /* Master arbitration lost, stop I2C and clear SI */
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI);
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }
        else if(u32Status == 0x00)              /* Master bus error, stop I2C and clear SI */
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI);
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }
        else if(u32Status == 0x30)              /* Master transmit data NACK, stop I2C and clear SI */
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI);
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }
        else if(u32Status == 0x48)              /* Master receive address NACK, stop I2C and clear SI */
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI);
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }
        else
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI);
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }
    }
    I2C_WAIT_SI_CLEAR(I2C0);
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

    /* Select UART module clock source as HXT and UART module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HXT, CLK_CLKDIV0_UART0(1));

    /* Enable I2C0 clock */
    CLK_EnableModuleClock(I2C0_MODULE);
    
    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and CyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART0 RXD and TXD */
    SET_UART0_RXD_PB12();
    SET_UART0_TXD_PB13();

    /* Set I2C0 multi-function pins */
    SET_I2C0_SDA_PA4();
    SET_I2C0_SCL_PA5();
    SET_I2C0_SMBSUS_PC2();
    SET_I2C0_SMBAL_PC3();
    
    /* I2C pin enable schmitt trigger */
    PA->SMTEN |= (GPIO_SMTEN_SMTEN4_Msk | GPIO_SMTEN_SMTEN5_Msk);

    PC0 = 1;
    GPIO_SetMode(PC, BIT0, GPIO_MODE_OUTPUT);
}

void UART0_Init(void)
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
    /* Reset I2C0 */
    SYS_ResetModule(I2C0_RST);

    /* Open I2C module and set bus clock */
    I2C_Open(I2C0, 100000);

    /* Get I2C0 Bus Clock */
    printf("I2C0 clock %d Hz\n", I2C_GetBusClockFreq(I2C0));

    /* Set I2C0 4 Slave addresses */
    I2C_SetSlaveAddr(I2C0, 0, g_u8MasterAddr[0], 0);   /* Slave Address : 0x15 */
    I2C_SetSlaveAddr(I2C0, 1, g_u8MasterAddr[1], 0);   /* Slave Address : 0x35 */
    I2C_SetSlaveAddr(I2C0, 2, g_u8MasterAddr[2], 0);   /* Slave Address : 0x55 */
    I2C_SetSlaveAddr(I2C0, 3, g_u8MasterAddr[3], 0);   /* Slave Address : 0x75 */

    /* Set I2C0 4 Slave addresses mask bits*/
    I2C_SetSlaveAddrMask(I2C0, 0, 0x01);
    I2C_SetSlaveAddrMask(I2C0, 1, 0x04);
    I2C_SetSlaveAddrMask(I2C0, 2, 0x01);
    I2C_SetSlaveAddrMask(I2C0, 3, 0x04);

    /* Enable I2C0 interrupt */
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

int32_t SMBusSendByteTest(uint8_t slvaddr)
{
    uint32_t i, u32TimeOutCnt;

    g_u8DeviceAddr = slvaddr;

    for(i = 0; i < 0x100; i++)
    {
        /* Init transmission bytes */
        g_au8TxData[0] = (uint8_t)((i & 0xFF00) >> 8);
        g_au8TxData[1] = (uint8_t)(i & 0x00FF);
        g_au8TxData[2] = (uint8_t)(g_au8TxData[1] + 3);

        g_u8DataLen0 = 0;
        g_u8EndFlag = 0;
        g_u8SendPEC = 0;

        /* I2C0 function to write data to slave */
        s_I2C0HandlerFn = (I2C_FUNC)I2C_MasterTx;

        /* I2C0 as master sends START signal */
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STA);

        /* Wait I2C0 transmit finish */
        u32TimeOutCnt = I2C_TIMEOUT;
        while(g_u8EndFlag == 0)
        {
            if(--u32TimeOutCnt == 0)
            {
                printf("Wait for I2C transmit finish time-out!\n");
                return -1;
            }
        }
        g_u8EndFlag = 0;

    }
    return 0;
}

int32_t SMBusAlertTest(uint8_t slvaddr)
{
    uint32_t u32TimeOutCnt;

    g_u8DeviceAddr = slvaddr;

    /* I2C function to Send Alert Response Address to bus */
    s_I2C0HandlerFn = (I2C_FUNC)I2C_MasterAlert;

    /* I2C0 Send Start condition */
    I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STA);

    /* Init receive data index */
    g_u8DataLen0 = 0;

    /* Waiting for Get Alert Address */
    u32TimeOutCnt = I2C_TIMEOUT;
    while(g_u8AlertAddrAck0 == 0)
    {
        if(--u32TimeOutCnt == 0)
        {
            printf("Wait for I2C get alert address time-out!\n");
            return -1;
        }
    }
    g_u8AlertAddrAck0 = 0;

    return 0;
}

int32_t SMBusDefaultAddressTest(uint8_t slvaddr)
{
    uint32_t u32TimeOutCnt;

    g_u8DeviceAddr = slvaddr;

    /* Set Transmission ARP command */
    g_au8TxData[0] = ARP_COMMAND;

    g_u8DataLen0 = 0;
    g_u8EndFlag = 0;
    g_u8SendPEC = 0;

    /* I2C0 function to write data to slave */
    s_I2C0HandlerFn = (I2C_FUNC)I2C_MasterDefaultAddrACKM;

    /* I2C0 as master sends START signal */
    I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STA);

    /* Wait I2C0 transmit finish */
    u32TimeOutCnt = I2C_TIMEOUT;
    while(g_u8EndFlag == 0)
    {
        if(--u32TimeOutCnt == 0)
        {
            printf("Wait for I2C transmit finish time-out!\n");
            return -1;
        }
    }
    g_u8EndFlag = 0;

    printf("\n");
    printf("Master Sends ARP Command(0x01) to Slave (0x%X) Test OK\n", slvaddr);
    return 0;
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    uint32_t i, ch = 0;
    uint32_t u32TimeOutCnt;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 for printf */
    UART0_Init();

    while(ch != 0x30)
    {
        /*
            This sample code sets I2C bus clock to 100kHz. Then, Master accesses Slave with Byte Write
            and Byte Read operations, and check if the read data is equal to the programmed data.
        */
        printf("\n");
        printf("+-------------------------------------------------------+\n");
        printf("|  M2351 I2C SMBUS Master Driver Sample Code            |\n");
        printf("|                                                       |\n");
        printf("| I2C Master (I2C0) <---> I2C Slave(I2C0)               |\n");
        printf("+-------------------------------------------------------+\n");

        printf("Configure I2C0 as master.\n");
        printf("The I/O connection for I2C0:\n");
        printf("I2C0_SDA(PA.4),    I2C0_SCL(PA.5)\n");
        printf("I2C0_SMBSUS(PC.2), I2C0_SMBAL(PC.3)\n");
        printf("PC.0 is used to synchronize with Slave.\n");
        printf("\n");
        printf("Select I2C Slave(I2C0) test item first!\n");
        printf("Press any key to continue.\n");
        getchar();

        /* Init I2C0 */
        I2C0_Init();

        printf("\n");
        printf("[1] SMBus Send Bytes Protocol with PEC Test\n");
        printf("[2] SMBus Alert Function Test\n");
        printf("[3] Simple ARP and ACK Control by Manual Test\n");
        printf("[0] Exit\n");
        ch = getchar();
        printf("Select: %c\n", ch);
        if(ch == '1')
        {
            /* I2C0 Bus Management enable */
            I2C_SMBusOpen(I2C0, I2C_SMBH_ENABLE);

            /* I2C0 Bus PEC Check and transmit enable */
            I2C_SMBusPECTxEnable(I2C0, I2C_PECTX_ENABLE);

            /* Set I2C0 Payload bytes */
            I2C_SMBusSetPacketByteCount(I2C0, 4);            // I2C0: 1byte address + 3byte data

            /* Access Slave with no address mask */
            printf("\n");
            printf(" == SMBus Send Bytes Protocol test ==\n");

            /* SMBus send byte protocol test*/
            if(SMBusSendByteTest(g_u8SlaveAddr[0]) < 0) goto lexit;

            printf("\n");
            printf("SMBus transmit data done.\n");
        }
        else if(ch == '2')
        {
            /* I2C0 Bus Management Enable */
            I2C_SMBusOpen(I2C0, I2C_SMBH_ENABLE);

            /* I2C0 Bus PEC Check and transmit Enable */
            I2C_SMBusPECTxEnable(I2C0, I2C_PECTX_DISABLE);

            /* Set I2C0 Payload bytes */
            I2C_SMBusSetPacketByteCount(I2C0, 2);            // I2C0: 1byte address + 1byte data

            /* Alert pin support if BMHEN(I2C0->BUSCTL[4]) = 0 */
            I2C_SMBUS_ENABLE_ALERT(I2C0);

            /* Enable Host SUSCON pin function and output Hi */
            I2C_SMBUS_SET_SUSCON_OUT(I2C0);
            I2C_SMBUS_SET_SUSCON_HIGH(I2C0);

            /* Access Slave with no address mask */
            printf("\n");
            printf(" == SMBus Alert Function Test ==\n");

            /* Pull PC0 low to notify Slave */
            PC0 = 0;

            /* Wait I2C0 get Alert interrupt */
            u32TimeOutCnt = I2C_TIMEOUT;
            while(g_u8AlertInt0 == 0)
            {
                if(--u32TimeOutCnt == 0)
                {
                    printf("Wait for I2C alert interrupt time-out!\n");
                    goto lexit;
                }
            }

            /* I2C0 Get Alert Request */
            g_u8AlertInt0 = 0;
            printf("\n");
            printf("Master I2C0 Get Alert Interrupt Request\n");

            /* I2C0 Send Alert Response Address(ARA) to I2C bus */
            if(SMBusAlertTest(SMBUS_ALERT_RESPONSE_ADDRESS) < 0) goto lexit;

            /* Printf the Alert Slave address */
            printf("\n");
            printf("Get Alert Address 0x%X test OK.\n", g_au8RxData[0]);

            /* Output PC0 low to notify Slave that Master has received Alert request */
            PC0 = 1;
            CLK_SysTickDelay(50);

            /* Output I2C0 SUSCON pin Low */
            I2C_SMBUS_SET_SUSCON_LOW(I2C0);
            printf("\n");
            printf("Master I2C0 SUSCON Pin output Lo, check Slave SUSCON Pin status\n");

            printf("\n");
            printf("SMBus Alert Test Done\n");
            PC0 = 1;
        }
        else if(ch == '3')
        {
            /* I2C0 Bus management enable */
            I2C_SMBusOpen(I2C0, I2C_SMBH_ENABLE);

            /* I2C0 Bus PEC check and transmit enable */
            I2C_SMBusPECTxEnable(I2C0, I2C_PECTX_ENABLE);

            /* Set I2C0 Payload bytes */
            I2C_SMBusSetPacketByteCount(I2C0, 2);            // I2C0: 1byte address + 1byte data

            printf("\n");
            printf("== Simple ARP and Acknowledge by Manual Test ==\n");

            /* I2C0 sends Default Address and ARP Command (0x01) to Slave */
            if(SMBusDefaultAddressTest(SMBUS_DEFAULT_ADDRESS) < 0) goto lexit;
        }
    }

lexit:

    s_I2C0HandlerFn = NULL;

    printf("SMBus Test Exit\n");

    /* Close I2C0 */
    I2C0_Close();

    while(1);
}
