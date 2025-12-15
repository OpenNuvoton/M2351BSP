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
const uint8_t g_u8SlaveAddr[4] = {0x18, 0x38, 0x58, 0x78};

volatile uint32_t slave_buff_addr;
volatile uint8_t g_u8SlvData[256];
volatile uint8_t g_au8RxData[4];
volatile uint8_t g_u8DataLen1;
volatile uint8_t g_u8EndFlag = 0;
volatile uint8_t g_u8AlertInt1 = 0;
volatile uint8_t g_u8PECErr = 0;

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
        g_u8AlertInt1 = 1;
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
/*  I2C Slave TRx Callback Function                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_SlaveTRx(uint32_t u32Status)
{
    if(u32Status == 0x60)                       /* Own SLA+W has been receive; ACK has been return */
    {
        g_u8DataLen1 = 0;
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
    }
    else if(u32Status == 0x80)                 /* Previously address with own SLA address
                                                   Data has been received; ACK has been returned*/
    {
        g_au8RxData[g_u8DataLen1] = (unsigned char)I2C_GET_DATA(I2C0);
        g_u8DataLen1++;
       
        if(g_u8DataLen1 == 2)
        {
            slave_buff_addr = (g_au8RxData[0] << 8) + g_au8RxData[1];
        }
        if(g_u8DataLen1 == 3)
        {
            g_u8SlvData[slave_buff_addr] = g_au8RxData[2];
        }
        if(g_u8DataLen1 == 4)
        {
            if(g_au8RxData[3] != I2C_SMBusGetPECValue(I2C0))
            {
                g_u8PECErr = 1;
            }
            g_u8DataLen1 = 0;
        }
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
    }
    else if(u32Status == 0xA8)                  /* Own SLA+R has been receive; ACK has been return */
    {
        I2C_SET_DATA(I2C0, g_u8SlvData[slave_buff_addr]);
        slave_buff_addr++;
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
    }
    else if(u32Status == 0xC0)                 /* Data byte or last data in I2CDAT has been transmitted
                                                   Not ACK has been received */
    {
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
    }
    else if(u32Status == 0x88)                 /* Previously addressed with own SLA address; NOT ACK has
                                                   been returned */
    {
        g_u8DataLen1 = 0;
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
    }
    else if(u32Status == 0xA0)                 /* A STOP or repeated START has been received while still
                                                   addressed as Slave/Receiver*/
    {
        g_u8DataLen1 = 0;
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
    }
    else
    {
        /* TO DO */
        printf("Status 0x%x is NOT processed\n", u32Status);
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
    }
    I2C_WAIT_SI_CLEAR(I2C0);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C Slave Alert Callback Function                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_SlaveAlert(uint32_t u32Status)
{
    if(u32Status == 0x60)                       /* Own SLA+W has been receive; ACK has been return */
    {
        g_u8DataLen1 = 0;
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
    }
    else if(u32Status == 0x80)                 /* Previously address with own SLA address
                                                   Data has been received; ACK has been returned*/
    {
        g_au8RxData[g_u8DataLen1] = (unsigned char) I2C_GET_DATA(I2C0);
        g_u8DataLen1++;

        if(g_u8DataLen1 == 2)
        {
            slave_buff_addr = (g_au8RxData[0] << 8) + g_au8RxData[1];
        }
        if(g_u8DataLen1 == 3)
        {
            g_u8SlvData[slave_buff_addr] = g_au8RxData[2];

        }
        if(g_u8DataLen1 == 4)
        {
            if(g_au8RxData[3] != I2C_SMBusGetPECValue(I2C0))
            {
                g_u8PECErr = 1;
            }
            g_u8DataLen1 = 0;
        }
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
    }
    else if(u32Status == 0xA8)                  /* Own SLA+R has been receive; ACK has been return */
    {
        I2C_SET_DATA(I2C0, g_u8SlaveAddr[0]);
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
    }
    else if(u32Status == 0xC0)                 /* Data byte or last data in I2CDAT has been transmitted
                                                   Not ACK has been received */
    {
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
    }
    else if(u32Status == 0x88)                 /* Previously addressed with own SLA address; NOT ACK has
                                                   been returned */
    {
        g_u8DataLen1 = 0;
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
    }
    else if(u32Status == 0xA0)                 /* A STOP or repeated START has been received while still
                                                   addressed as Slave/Receiver*/
    {
        g_u8DataLen1 = 0;
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
    }
    else if(u32Status == 0xB8)
    {
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
    }
    else
    {
        /* TO DO */
        printf("Status 0x%x is NOT processed\n", u32Status);
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
    }
    I2C_WAIT_SI_CLEAR(I2C0);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C Slave Default Address and Acknowledge by Manual Callback Function                                  */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_SlaveDefaultAddrACKM(uint32_t u32Status)
{
    if(u32Status == 0x60)                       /* Own SLA+W has been receive; ACK has been return */
    {
        g_u8DataLen1 = 0;
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    }
    else if(u32Status == 0xF0)
    {
        /* Enable ACKMEN & SLV in Receive Mode */
        g_au8RxData[g_u8DataLen1] = (unsigned char) I2C_GET_DATA(I2C0);
        g_u8DataLen1++;

        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);           //Acknowledge by Manual
    }
    else if(u32Status == 0x80)                 /* Previously address with own SLA address
                                                   Data has been received; ACK has been returned*/
    {
        g_au8RxData[g_u8DataLen1] = (unsigned char) I2C_GET_DATA(I2C0);
        g_u8DataLen1++;

        if(g_u8DataLen1 == 2)
        {
            g_u8EndFlag = 1;
            if(g_au8RxData[1] != I2C_SMBusGetPECValue(I2C0))
            {
                g_u8PECErr = 1;
            }
            g_u8DataLen1 = 0;
        }
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
    }
    else if(u32Status == 0xA8)                  /* Own SLA+R has been receive; ACK has been return */
    {
        I2C_SET_DATA(I2C0, g_u8SlvData[slave_buff_addr]);
        slave_buff_addr++;
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
    }
    else if(u32Status == 0xC0)                 /* Data byte or last data in I2CDAT has been transmitted
                                                   Not ACK has been received */
    {
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
    }
    else if(u32Status == 0x88)                 /* Previously addressed with own SLA address; NOT ACK has
                                                   been returned */
    {
        g_u8DataLen1 = 0;
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
    }
    else if(u32Status == 0xA0)                 /* A STOP or repeated START has been received while still
                                                   addressed as Slave/Receiver*/
    {
        g_u8DataLen1 = 0;
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
    }
    else
    {
        /* TO DO */
        printf("Status 0x%x is NOT processed\n", u32Status);
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
    }
    I2C_WAIT_SI_CLEAR(I2C0);
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
    PA->SMTEN |= GPIO_SMTEN_SMTEN4_Msk | GPIO_SMTEN_SMTEN5_Msk;

    PC0 = 0;
    GPIO_SetMode(PC, BIT0, GPIO_MODE_INPUT);
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
    /* Slave address : 0x18 */
    I2C_SetSlaveAddr(I2C0, 0, g_u8SlaveAddr[0], 0);   /* Slave Address : 0x18 */
    /* Slave address : 0x38 */
    I2C_SetSlaveAddr(I2C0, 1, g_u8SlaveAddr[1], 0);   /* Slave Address : 0x38 */
    /* Slave address : 0x58 */
    I2C_SetSlaveAddr(I2C0, 2, g_u8SlaveAddr[2], 0);   /* Slave Address : 0x58 */
    /* Slave address : 0x78 */
    I2C_SetSlaveAddr(I2C0, 3, g_u8SlaveAddr[3], 0);   /* Slave Address : 0x78 */

    /* Set I2C0 4 Slave addresses mask bits*/
    /* Slave address mask Bits: 0x04 */
    I2C_SetSlaveAddrMask(I2C0, 0, 0x04);
    /* Slave address mask Bits: 0x02 */
    I2C_SetSlaveAddrMask(I2C0, 1, 0x02);
    /* Slave address mask Bits: 0x04 */
    I2C_SetSlaveAddrMask(I2C0, 2, 0x04);
    /* Slave address mask Bits: 0x02 */
    I2C_SetSlaveAddrMask(I2C0, 3, 0x02);

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
        printf("|  M2351 I2C SMBUS Slave Driver Sample Code             |\n");
        printf("|                                                       |\n");
        printf("| I2C Master (I2C0) <---> I2C Slave(I2C0)               |\n");
        printf("+-------------------------------------------------------+\n");

        printf("Configure I2C0 as slave.\n");
        printf("The I/O connection for I2C0:\n");
        printf("I2C0_SDA(PA.4),    I2C0_SCL(PA.5)\n");
        printf("I2C0_SMBSUS(PC.2), I2C0_SMBAL(PC.3)\n");
        printf("PC.0 is used to synchronize with Master.\n");
        printf("\n");

        /* Init I2C0 */
        I2C0_Init();

        /* I2C0 enter no address SLV mode */
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);

        printf("\n");
        printf("I2C Slave Mode is Running.\n");
        for(i = 0; i < 0x100; i++)
        {
            g_u8SlvData[i] = 0;
        }

        printf("\n");
        printf("[1] SMBus Send Bytes Protocol with PEC Test\n");
        printf("[2] SMBus Alert Function Test\n");
        printf("[3] Simple ARP and ACK Control by Manual Test\n");
        printf("[0] Exit\n");
        ch = getchar();
        printf("Select: %c\n", ch);
        if(ch == '1')
        {
            /* I2C0 Bus Management init */
            I2C_SMBusOpen(I2C0, I2C_SMBD_ENABLE);

            /* I2C0 Bus PEC check enable */
            I2C_SMBusPECTxEnable(I2C0, I2C_PECTX_DISABLE);

            /* Set I2C0 Payload bytes */
            I2C_SMBusSetPacketByteCount(I2C0, 4);            // I2C0: 1byte address + 3byte data

            g_u8PECErr = 0;
            slave_buff_addr = 0;
            g_u8DataLen1 = 0xFF;

            /* I2C function to Slave receive/transmit data */
            s_I2C0HandlerFn = I2C_SlaveTRx;

            printf("\n");
            printf("SMBus is waiting for receiving data...\n");
            while ((slave_buff_addr < 255))
            {
                if ((g_u8DataLen1 == 0) && (slave_buff_addr == 0))
                {
                    printf("\nSMBus is receiving data...\n");
                }

                if (g_u8PECErr)
                {
                    printf("PEC Check Error !\n");
                    goto lexit;
                }
            }

            printf("\nSMBus received data complete and checked PEC done.\n");
        }
        else if(ch == '2')
        {
            printf("\n");
            printf(" == SMBus Alert Function Test ==\n");

            /* I2C0 Bus Management init */
            I2C_SMBusOpen(I2C0, I2C_SMBD_ENABLE);

            /* I2C0 Bus PEC Check enable */
            I2C_SMBusPECTxEnable(I2C0, I2C_PECTX_ENABLE);

            /* Set I2C0 Payload bytes */
            I2C_SMBusSetPacketByteCount(I2C0, 2);            // I2C0: 1byte address + 1byte data

            /* Wait Master ready to pull PC0 low */
            while (PC0 != 0);

            /* Release Slave Alert pin to Hi */
            I2C_SMBUS_DISABLE_ALERT(I2C0);

            /* Set Slave SUSCON pin is Input */
            I2C_SMBUS_SET_SUSCON_IN(I2C0);

            g_u8PECErr = 0;

            /* I2C function to Slave for receive/transmit data */
            s_I2C0HandlerFn = I2C_SlaveAlert;

            /* Access Slave with no address mask */

            /* I2C0 Slave has a Alert request, pull Alert Pin to Lo */
            I2C_SMBUS_ENABLE_ALERT(I2C0);
            
            printf("\n");
            printf("Slave I2C0 has Alert Request and Alert Pin Pull Lo. \n");

            /* Wait Master to notify it has received Alert request */
            while (PC0 != 1);

            /* Show I2C0 SUSCON pin state before I2C0 pull Lo */
            printf("\n");
            printf("I2C0 SUSCON Pin state is %d\n", (int)((I2C0->BUSSTS & I2C_BUSSTS_SCTLDIN_Msk) >> I2C_BUSSTS_SCTLDIN_Pos));

            /* Wait Master to pull SUSCON low */
            CLK_SysTickDelay(100);

            /* Show I2C0 SUSCON pin state after Master has pulled SUSCON low */
            printf("\n");
            printf("I2C0 SUSCON Pin state change to %d\n", (int)((I2C0->BUSSTS & I2C_BUSSTS_SCTLDIN_Msk) >> I2C_BUSSTS_SCTLDIN_Pos));

            printf("\n");
            printf("SMBus Alert Test Done\n");
        }
        else if(ch == '3')
        {
            /* I2C0 Bus Management init */
            I2C_SMBusOpen(I2C0, I2C_SMBD_ENABLE);

            /* I2C0 Bus PEC check enable */
            I2C_SMBusPECTxEnable(I2C0, I2C_PECTX_DISABLE);

            /* I2C0 Acknowledge by Manual enable */
            I2C_SMBUS_ACK_MANUAL(I2C0);

            /* Set I2C0 Payload bytes */
            I2C_SMBusSetPacketByteCount(I2C0, 2);            // I2C0: 1byte address + 1byte data

            g_u8PECErr = 0;
            g_u8DataLen1 = 0;
            g_u8EndFlag = 0;

            /* I2C function to Slave receive/transmit data */
            s_I2C0HandlerFn = I2C_SlaveDefaultAddrACKM;

            printf("\n");
            printf("== Simple ARP and Acknowledge by Manual Test ==\n");

            while (g_u8EndFlag == 0)
                ;

            /* Show I2C0 get ARP command from  I2C0 */
            printf("\n");
            printf("Slave Get ARP Command is 0x%X\n", g_au8RxData[0]);

            /* Check Slave get command */
            printf("\n");
            if(g_au8RxData[0] != ARP_COMMAND)
            {
                printf("Get Wrong ARP Command, Please Check again !\n");
            }
            else
            {
                printf("Default Address and Acknowledge by Manual test OK.\n");
            }
        }
    }

lexit:

    s_I2C0HandlerFn = NULL;

    printf("SMBus Test Exit\n");

    /* Close I2C0 */
    I2C0_Close();

    while(1);
}
