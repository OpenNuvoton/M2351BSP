
/******************************************************************************
 * @file     main.c
 * @version  V3.00
 * $Revision: 3 $
 * $Date: 16/10/17 2:06p $
 * @brief    Show a Master how to access 10-bit address Slave (loopback)
 * @note
 * Copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define PLLCTL_SETTING  CLK_PLLCTL_48Hz_HXT
#define PLL_CLOCK       4800000

#define SLV_10BIT_ADDR (0x1E<<2)

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
static volatile uint8_t s_u8DeviceHAddr;
static volatile uint8_t s_u8DeviceLAddr;
static volatile uint8_t s_u8SlvData[256];
static volatile uint8_t s_au8MstTxData[3];
static volatile uint8_t s_u8MstRxData;
static volatile uint8_t s_u8MstEndFlag = 0;
static volatile uint32_t s_u32SlaveBuffAddr;
static volatile uint8_t s_au8SlvRxData[4];
static volatile uint16_t s_u16SlvRcvAddr;
static volatile uint8_t s_u8MstDataLen;
static volatile uint8_t s_u8SlvDataLen;

static volatile enum UI2C_MASTER_EVENT s_eMasterEvent;
static volatile enum UI2C_SLAVE_EVENT s_eSlaveEvent;

typedef void (*UI2C_FUNC)(uint32_t u32Status);

volatile static UI2C_FUNC s_UI2C0HandlerFn = NULL;
volatile static UI2C_FUNC s_UI2C1HandlerFn = NULL;

void USCI0_IRQHandler(void);
void USCI1_IRQHandler(void);
void UI2C_MasterRx(uint32_t u32Status);
void UI2C_MasterTx(uint32_t u32Status);
void UI2C_LB_SlaveTRx(uint32_t u32Status);
void SYS_Init(void);
void UI2C0_Init(uint32_t u32ClkSpeed);
void UI2C1_Init(uint32_t u32ClkSpeed);
int32_t Read_Write_SLAVE(uint16_t u16SlvAddr);
/*---------------------------------------------------------------------------------------------------------*/
/*  USCI0_I2C IRQ Handler                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void USCI0_IRQHandler(void)
{
    uint32_t u32Status;

    u32Status = (UI2C0->PROTSTS);
    if(s_UI2C0HandlerFn != NULL)
        s_UI2C0HandlerFn(u32Status);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  USCI0_I2C IRQ Handler                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void USCI1_IRQHandler(void)
{
    uint32_t u32Status;

    u32Status = (UI2C1->PROTSTS);
    if(s_UI2C1HandlerFn != NULL)
        s_UI2C1HandlerFn(u32Status);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  USCI_I2C0 Rx Callback Function                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
void UI2C_MasterRx(uint32_t u32Status)
{
    if((UI2C0->PROTSTS & UI2C_PROTSTS_TOIF_Msk) == UI2C_PROTSTS_TOIF_Msk)
    {
        /* Clear USCI_I2C0 Timeout Flag */
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_TOIF_Msk);
    }
    else if((u32Status & UI2C_PROTSTS_STARIF_Msk) == UI2C_PROTSTS_STARIF_Msk)
    {
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_STARIF_Msk);  /* Clear START INT Flag */

        if(s_eMasterEvent == MASTER_SEND_START)
        {
            UI2C_SET_DATA(UI2C0, (uint16_t)(s_u8DeviceHAddr << 1) | 0x00); /* Write SLA+W to Register TXDAT */
            s_eMasterEvent = MASTER_SEND_H_WR_ADDRESS;
        }
        else if(s_eMasterEvent == MASTER_SEND_REPEAT_START)
        {
            UI2C_SET_DATA(UI2C0, (uint16_t)(s_u8DeviceHAddr << 1) | 0x01); /* Write SLA+R to Register TXDAT */
            s_eMasterEvent = MASTER_SEND_H_RD_ADDRESS;
        }

        UI2C_SET_CONTROL_REG(UI2C0, UI2C_CTL_PTRG);
    }
    else if((u32Status & UI2C_PROTSTS_ACKIF_Msk) == UI2C_PROTSTS_ACKIF_Msk)
    {
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_ACKIF_Msk);  /* Clear ACK INT Flag */
        if(s_eMasterEvent == MASTER_SEND_H_WR_ADDRESS)
        {
            UI2C_SET_DATA(UI2C0, s_u8DeviceLAddr);
            s_eMasterEvent = MASTER_SEND_L_ADDRESS;
            UI2C_SET_CONTROL_REG(UI2C0, UI2C_CTL_PTRG);
        }
        else if(s_eMasterEvent == MASTER_SEND_L_ADDRESS)
        {
            UI2C_SET_DATA(UI2C0, s_au8MstTxData[s_u8MstDataLen++]);  /* SLA+W has been transmitted and write ADDRESS to Register TXDAT */
            s_eMasterEvent = MASTER_SEND_DATA;
            UI2C_SET_CONTROL_REG(UI2C0, UI2C_CTL_PTRG);
        }
        else if(s_eMasterEvent == MASTER_SEND_DATA)
        {
            if(s_u8MstDataLen != 2)
            {
                UI2C_SET_DATA(UI2C0, s_au8MstTxData[s_u8MstDataLen++]);  /* ADDRESS has been transmitted and write DATA to Register TXDAT */
                UI2C_SET_CONTROL_REG(UI2C0, UI2C_CTL_PTRG);
            }
            else
            {
                s_eMasterEvent = MASTER_SEND_REPEAT_START;
                UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_STA));    /* Send repeat START signal */
            }
        }
        else if(s_eMasterEvent == MASTER_SEND_H_RD_ADDRESS)
        {
            s_eMasterEvent = MASTER_READ_DATA;
            UI2C_SET_CONTROL_REG(UI2C0, UI2C_CTL_PTRG);
        }
    }
    else if((u32Status & UI2C_PROTSTS_NACKIF_Msk) == UI2C_PROTSTS_NACKIF_Msk)
    {
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_NACKIF_Msk);      /* Clear NACK INT Flag */

        if(s_eMasterEvent == MASTER_SEND_H_WR_ADDRESS)
        {
            s_eMasterEvent = MASTER_SEND_START;
            UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_STA));    /* Send START signal */
        }
        else if(s_eMasterEvent == MASTER_SEND_L_ADDRESS)
        {
            s_eMasterEvent = MASTER_STOP;
            UI2C_SET_CONTROL_REG(UI2C0, UI2C_CTL_STO);
        }
        else if(s_eMasterEvent == MASTER_READ_DATA)
        {
            s_u8MstRxData = (uint8_t) UI2C_GET_DATA(UI2C0);

            s_eMasterEvent = MASTER_STOP;
            UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_STO));    /* DATA has been received and send STOP signal */
        }
        else
            /* TO DO */
            printf("Status 0x%x is NOT processed\n", u32Status);
    }
    else if((u32Status & UI2C_PROTSTS_STORIF_Msk) == UI2C_PROTSTS_STORIF_Msk)
    {
        s_u8MstEndFlag = 1;

        /* Clear STOP INT Flag */
        UI2C0->PROTSTS =  UI2C_PROTSTS_STORIF_Msk;

        /* Trigger USCI I2C */
        UI2C0->PROTCTL |= UI2C_PROTCTL_PTRG_Msk;
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  USCI_I2C0 Tx Callback Function                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
void UI2C_MasterTx(uint32_t u32Status)
{
    if((UI2C0->PROTSTS & UI2C_PROTSTS_TOIF_Msk) == UI2C_PROTSTS_TOIF_Msk)
    {
        /* Clear USCI_I2C0 Timeout Flag */
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_TOIF_Msk);
    }
    else if((u32Status & UI2C_PROTSTS_STARIF_Msk) == UI2C_PROTSTS_STARIF_Msk)
    {
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_STARIF_Msk);               /* Clear START INT Flag */

        UI2C_SET_DATA(UI2C0, (uint16_t)(s_u8DeviceHAddr << 1) | 0x00);     /* Write SLA+W to Register TXDAT */
        s_eMasterEvent = MASTER_SEND_H_WR_ADDRESS;

        UI2C_SET_CONTROL_REG(UI2C0, UI2C_CTL_PTRG);
    }
    else if((u32Status & UI2C_PROTSTS_ACKIF_Msk) == UI2C_PROTSTS_ACKIF_Msk)
    {
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_ACKIF_Msk);   /* Clear ACK INT Flag */

        /* Event process */
        if(s_eMasterEvent == MASTER_SEND_H_WR_ADDRESS)
        {
            UI2C_SET_DATA(UI2C0, s_u8DeviceLAddr);
            s_eMasterEvent = MASTER_SEND_L_ADDRESS;
            UI2C_SET_CONTROL_REG(UI2C0, UI2C_CTL_PTRG);
        }
        else if(s_eMasterEvent == MASTER_SEND_L_ADDRESS)
        {
            UI2C_SET_DATA(UI2C0, s_au8MstTxData[s_u8MstDataLen++]);  /* SLA+W has been transmitted and write ADDRESS to Register TXDAT */
            s_eMasterEvent = MASTER_SEND_DATA;
            UI2C_SET_CONTROL_REG(UI2C0, UI2C_CTL_PTRG);
        }
        else if(s_eMasterEvent == MASTER_SEND_DATA)
        {
            if(s_u8MstDataLen != 3)
            {
                UI2C_SET_DATA(UI2C0, s_au8MstTxData[s_u8MstDataLen++]);  /* ADDRESS has been transmitted and write DATA to Register TXDAT */
                UI2C_SET_CONTROL_REG(UI2C0, UI2C_CTL_PTRG);
            }
            else
            {
                s_eMasterEvent = MASTER_STOP;
                UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_STO));        /* Send STOP signal */
            }
        }
    }
    else if((u32Status & UI2C_PROTSTS_NACKIF_Msk) == UI2C_PROTSTS_NACKIF_Msk)
    {
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_NACKIF_Msk);  /* Clear NACK INT Flag */

        s_u8MstEndFlag = 0;
        if(s_eMasterEvent == MASTER_SEND_H_WR_ADDRESS)
        {
            /* SLA+W has been transmitted and NACK has been received */
            s_eMasterEvent = MASTER_SEND_START;
            UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_STA));            /* Send START signal */
        }
        else if(s_eMasterEvent == MASTER_SEND_L_ADDRESS)
        {
            /* ADDRESS has been transmitted and NACK has been received */
            s_eMasterEvent = MASTER_STOP;
            UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_STO));            /* Send STOP signal */
        }
        else if(s_eMasterEvent == MASTER_SEND_DATA)
        {
            /* ADDRESS has been transmitted and NACK has been received */
            s_eMasterEvent = MASTER_STOP;
            UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_STO));            /* Send STOP signal */
        }
        else
            printf("Get Wrong NACK Event\n");
    }
    else if((u32Status & UI2C_PROTSTS_STORIF_Msk) == UI2C_PROTSTS_STORIF_Msk)
    {
        s_u8MstEndFlag = 1;

        /* Clear STOP INT Flag */
        UI2C0->PROTSTS =  UI2C_PROTSTS_STORIF_Msk;

        /* Trigger USCI I2C */
        UI2C0->PROTCTL |= UI2C_PROTCTL_PTRG_Msk;
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  USCI_I2C1 TRx Callback Function                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
void UI2C_LB_SlaveTRx(uint32_t u32Status)
{
    if((u32Status & UI2C_PROTSTS_STARIF_Msk) == UI2C_PROTSTS_STARIF_Msk)
    {
        /* Clear START INT Flag */
        UI2C_CLR_PROT_INT_FLAG(UI2C1, UI2C_PROTSTS_STARIF_Msk);

        /* Event process */
        s_u8SlvDataLen = 0;
        s_eSlaveEvent = SLAVE_H_RD_ADDRESS_ACK;

        UI2C_SET_CONTROL_REG(UI2C1, (UI2C_CTL_PTRG | UI2C_CTL_AA));
    }
    else if((u32Status & UI2C_PROTSTS_ACKIF_Msk) == UI2C_PROTSTS_ACKIF_Msk)
    {
        /* Clear ACK INT Flag */
        UI2C_CLR_PROT_INT_FLAG(UI2C1, UI2C_PROTSTS_ACKIF_Msk);

        /* Event process */
        if(s_eSlaveEvent == SLAVE_H_WR_ADDRESS_ACK)
        {
            s_u8SlvDataLen = 0;

            s_eSlaveEvent = SLAVE_L_WR_ADDRESS_ACK;
            s_u16SlvRcvAddr = (uint8_t)UI2C_GET_DATA(UI2C1);
        }
        else if(s_eSlaveEvent == SLAVE_H_RD_ADDRESS_ACK)
        {
            s_u8SlvDataLen = 0;

            UI2C_SET_DATA(UI2C1, s_u8SlvData[s_u32SlaveBuffAddr]);
            s_u32SlaveBuffAddr++;
            s_u16SlvRcvAddr = (uint8_t)UI2C_GET_DATA(UI2C1);
        }
        else if(s_eSlaveEvent == SLAVE_L_WR_ADDRESS_ACK)
        {
            if((UI2C1->PROTSTS & UI2C_PROTSTS_SLAREAD_Msk) == UI2C_PROTSTS_SLAREAD_Msk)
            {
                UI2C_SET_DATA(UI2C1, s_u8SlvData[s_u32SlaveBuffAddr]);
                s_u32SlaveBuffAddr++;
            }
            else
            {
                s_eSlaveEvent = SLAVE_GET_DATA;
            }
            s_u16SlvRcvAddr = (uint8_t)UI2C_GET_DATA(UI2C1);
        }
        else if(s_eSlaveEvent == SLAVE_L_RD_ADDRESS_ACK)
        {
            UI2C_SET_DATA(UI2C1, s_u8SlvData[s_u32SlaveBuffAddr]);
            s_u32SlaveBuffAddr++;
        }
        else if(s_eSlaveEvent == SLAVE_GET_DATA)
        {
            s_au8SlvRxData[s_u8SlvDataLen] = (uint8_t)UI2C_GET_DATA(UI2C1);
            s_u8SlvDataLen++;

            if(s_u8SlvDataLen == 2)
            {
                s_u32SlaveBuffAddr = (uint16_t)(s_au8SlvRxData[0] << 8) + s_au8SlvRxData[1];
            }
            if(s_u8SlvDataLen == 3)
            {
                s_u8SlvData[s_u32SlaveBuffAddr] = s_au8SlvRxData[2];
                s_u8SlvDataLen = 0;
            }
        }

        UI2C_SET_CONTROL_REG(UI2C1, (UI2C_CTL_PTRG | UI2C_CTL_AA));
    }
    else if((u32Status & UI2C_PROTSTS_NACKIF_Msk) == UI2C_PROTSTS_NACKIF_Msk)
    {
        /* Clear NACK INT Flag */
        UI2C_CLR_PROT_INT_FLAG(UI2C1, UI2C_PROTSTS_NACKIF_Msk);

        /* Event process */
        s_u8SlvDataLen = 0;
        s_eSlaveEvent = SLAVE_H_WR_ADDRESS_ACK;

        UI2C_SET_CONTROL_REG(UI2C1, (UI2C_CTL_PTRG | UI2C_CTL_AA));
    }
    else if((u32Status & UI2C_PROTSTS_STORIF_Msk) == UI2C_PROTSTS_STORIF_Msk)
    {
        /* Clear STOP INT Flag */
        UI2C_CLR_PROT_INT_FLAG(UI2C1, UI2C_PROTSTS_STORIF_Msk);

        /* Event process */
        s_u8SlvDataLen = 0;
        s_eSlaveEvent = SLAVE_H_WR_ADDRESS_ACK;

        UI2C_SET_CONTROL_REG(UI2C1, (UI2C_CTL_PTRG | UI2C_CTL_AA));
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

    /* Enable UI2C0 peripheral clock */
    CLK_EnableModuleClock(USCI0_MODULE);

    /* Enable UI2C1 peripheral clock */
    CLK_EnableModuleClock(USCI1_MODULE);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and CyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & (~(UART0_RXD_PB12_Msk | UART0_TXD_PB13_Msk))) | UART0_RXD_PB12 | UART0_TXD_PB13;

    /* Set PA multi-function pins for UI2C0_SDA(PA.10) and UI2C0_SCL(PA.11) */
    SYS->GPA_MFPH &= ~(SYS_GPA_MFPH_PA10MFP_Msk | SYS_GPA_MFPH_PA11MFP_Msk);
    SYS->GPA_MFPH |= (SYS_GPA_MFPH_PA10MFP_USCI0_DAT0 | SYS_GPA_MFPH_PA11MFP_USCI0_CLK);

    /* Set PB multi-function pins for UI2C1_SDA(PB.2) and UI2C1_SCL(PB.1) */
    SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB2MFP_Msk | SYS_GPB_MFPL_PB1MFP_Msk);
    SYS->GPB_MFPL |= (SYS_GPB_MFPL_PB2MFP_USCI1_DAT0 | SYS_GPB_MFPL_PB1MFP_USCI1_CLK);
}

void UI2C0_Init(uint32_t u32ClkSpeed)
{
    /* Open USCI_I2C0 and set clock to 100k */
    UI2C_Open(UI2C0, u32ClkSpeed);

    /* Get USCI_I2C0 Bus Clock */
    printf("UI2C0 clock %d Hz\n", UI2C_GetBusClockFreq(UI2C0));

    /* Set USCI_I2C0 Slave Addresses */
    UI2C_SetSlaveAddr(UI2C0, 0, 0x115, UI2C_GCMODE_DISABLE);   /* Slave Address : 0x15 */
    UI2C_SetSlaveAddr(UI2C0, 1, 0x135, UI2C_GCMODE_DISABLE);   /* Slave Address : 0x35 */

    /* Set USCI_I2C0 Slave Addresses Mask */
    UI2C_SetSlaveAddrMask(UI2C0, 0, 0x01);                     /* Slave Address Mask : 0x1 */
    UI2C_SetSlaveAddrMask(UI2C0, 1, 0x04);                     /* Slave Address Mask : 0x4 */

    UI2C_ENABLE_PROT_INT(UI2C0, (UI2C_PROTIEN_ACKIEN_Msk | UI2C_PROTIEN_NACKIEN_Msk | UI2C_PROTIEN_STORIEN_Msk | UI2C_PROTIEN_STARIEN_Msk));
    NVIC_EnableIRQ(USCI0_IRQn);
}

void UI2C1_Init(uint32_t u32ClkSpeed)
{
    /* Open UI2C1 and set clock to 100k */
    UI2C_Open(UI2C1, u32ClkSpeed);

    /* Enable UI2C1 10-bit address mode */
    UI2C_ENABLE_10BIT_ADDR_MODE(UI2C1);

    /* Get UI2C1 Bus Clock */
    printf("UI2C1 clock %d Hz\n", UI2C_GetBusClockFreq(UI2C1));

    /* Set UI2C1 Slave Addresses */
    UI2C_SetSlaveAddr(UI2C1, 0, 0x116, UI2C_GCMODE_DISABLE);   /* Slave Address : 0x16 */
    UI2C_SetSlaveAddr(UI2C1, 1, 0x136, UI2C_GCMODE_DISABLE);   /* Slave Address : 0x36 */

    /* Set UI2C1 Slave Addresses Mask */
    UI2C_SetSlaveAddrMask(UI2C1, 0, 0x04);                     /* Slave Address Mask : 0x4 */
    UI2C_SetSlaveAddrMask(UI2C1, 1, 0x02);                     /* Slave Address Mask : 0x2 */

    /* Enable UI2C1 protocol interrupt */
    UI2C_ENABLE_PROT_INT(UI2C1, (UI2C_PROTIEN_ACKIEN_Msk | UI2C_PROTIEN_NACKIEN_Msk | UI2C_PROTIEN_STORIEN_Msk | UI2C_PROTIEN_STARIEN_Msk));
    NVIC_EnableIRQ(USCI1_IRQn);
}

int32_t Read_Write_SLAVE(uint16_t u16SlvAddr)
{
    uint32_t i;

    /* Init Send 10-bit Addr */
    s_u8DeviceHAddr = (u16SlvAddr >> 8) | SLV_10BIT_ADDR;
    s_u8DeviceLAddr = u16SlvAddr & 0xFF;

    for(i = 0; i < 0x100; i++)
    {
        s_au8MstTxData[0] = (uint8_t)((i & 0xFF00) >> 8);
        s_au8MstTxData[1] = (uint8_t)(i & 0x00FF);
        s_au8MstTxData[2] = (uint8_t)(s_au8MstTxData[1] + 3);

        s_u8MstDataLen = 0;
        s_u8MstEndFlag = 0;

        /* USCI_I2C function to write data to slave */
        s_UI2C0HandlerFn = (UI2C_FUNC)UI2C_MasterTx;

        /* USCI_I2C as master sends START signal */
        s_eMasterEvent = MASTER_SEND_START;
        UI2C_SET_CONTROL_REG(UI2C0, UI2C_CTL_STA);

        /* Wait USCI_I2C Tx Finish */
        while(s_u8MstEndFlag == 0);
        s_u8MstEndFlag = 0;

        /* USCI_I2C function to read data from slave */
        s_UI2C0HandlerFn = (UI2C_FUNC)UI2C_MasterRx;

        s_u8MstDataLen = 0;

        s_eMasterEvent = MASTER_SEND_START;
        UI2C_SET_CONTROL_REG(UI2C0, UI2C_CTL_STA);

        /* Wait USCI_I2C Rx Finish */
        while(s_u8MstEndFlag == 0);
        s_u8MstEndFlag = 0;

        /* Compare data */
        if(s_u8MstRxData != s_au8MstTxData[2])
        {
            printf("USCI_I2C Byte Write/Read Failed, Data 0x%x\n", s_u8MstRxData);
            return -1;
        }
    }
    printf("Master Access Slave (0x%X) Test OK\n", u16SlvAddr);
    return 0;
}

/*---------------------------------------------------------------------------------------------------------*/
/*                        Main function                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
int main()
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
        This sample code sets USCI_I2C bus clock to 100kHz. Then, Master accesses Slave with Byte Write
        and Byte Read operations, and check if the read data is equal to the programmed data.
    */

    printf("+-------------------------------------------------------+\n");
    printf("|  USCI_I2C Driver Sample Code for Master access        |\n");
    printf("|  10-bit address Slave (Loopback)                      |\n");
    printf("|  UI2C0(Master)  <----> UI2C1(Slave)                   |\n");
    printf("+-------------------------------------------------------+\n");

    printf("\n");
    printf("Configure UI2C0 as a Master, UI2C1 as a Slave\n");
    printf("The I/O connection for UI2C0 & UI2C1:\n");
    printf("UI2C0_SDA(PA.10), UI2C0_SCL(PA.11)\n");
    printf("UI2C1_SDA(PB.2),  UI2C1_SCL(PB.1)\n");

    /* Init USCI_I2C0 and USCI_I2C1 */
    UI2C0_Init(100000);
    UI2C1_Init(100000);

    s_eSlaveEvent = SLAVE_H_WR_ADDRESS_ACK;
    UI2C_SET_CONTROL_REG(UI2C1, (UI2C_CTL_PTRG | UI2C_CTL_AA));

    for(u32i = 0; u32i < 0x100; u32i++)
        s_u8SlvData[u32i] = 0;

    /* I2C1 function to Slave receive/transmit data */
    s_UI2C1HandlerFn = UI2C_LB_SlaveTRx;

    /* Master Access Slave with no address mask */
    printf("\n");
    printf(" == No Mask Address ==\n");
    Read_Write_SLAVE(0x116);
    Read_Write_SLAVE(0x136);
    printf("SLAVE Address test OK.\n");

    /* Master Access Slave with address mask */
    printf("\n");
    printf(" == Mask Address ==\n");
    Read_Write_SLAVE(0x116 & ~0x04);
    Read_Write_SLAVE(0x136 & ~0x02);
    printf("SLAVE Address Mask test OK.\n");

    while(1);
}
/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/


