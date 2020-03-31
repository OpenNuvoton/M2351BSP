
/******************************************************************************
 * @file     main.c
 * @version  V3.00
 * $Revision: 3 $
 * $Date: 16/10/17 2:06p $
 * @brief    Show how to use USCI_I2C interface to access EEPROM.
 * @note
 * Copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define PLL_CLOCK       64000000
/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
static uint8_t s_u8DeviceAddr;
static uint8_t s_au8TxData[3];
static volatile uint8_t s_u8RxData;
static volatile uint8_t s_u8DataLenM;
static volatile uint8_t s_u8EndFlagM = 0;
static volatile enum UI2C_MASTER_EVENT s_eMasterEvent;

typedef void (*UI2C_FUNC)(uint32_t u32Status);
volatile static UI2C_FUNC s_UI2C0HandlerFn = NULL;

void USCI0_IRQHandler(void);
void USCI_I2C_EEPROM_MasterTx(uint32_t u32Status);
void USCI_I2C_EEPROM_MasterRx(uint32_t u32Status);
void SYS_Init(void);
void UI2C0_Init(void);
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
/*  USCI_I2C Tx Callback Function                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
void USCI_I2C_EEPROM_MasterTx(uint32_t u32Status)
{
    if((u32Status & UI2C_PROTSTS_STARIF_Msk) == UI2C_PROTSTS_STARIF_Msk)
    {
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_STARIF_Msk);                    /* Clear START INT Flag */
        UI2C_SET_DATA(UI2C0, (uint16_t)(s_u8DeviceAddr << 1) | 0x00);                        /* Write SLA+W to Register TXDAT */
        s_eMasterEvent = MASTER_SEND_ADDRESS;
        UI2C_SET_CONTROL_REG(UI2C0, UI2C_CTL_PTRG);
    }
    else if((u32Status & UI2C_PROTSTS_ACKIF_Msk) == UI2C_PROTSTS_ACKIF_Msk)
    {
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_ACKIF_Msk);                     /* Clear ACK INT Flag */
        if(s_eMasterEvent == MASTER_SEND_ADDRESS)
        {
            UI2C_SET_DATA(UI2C0, s_au8TxData[s_u8DataLenM++]);                     /* SLA+W has been transmitted and write ADDRESS to Register TXDAT */
            s_eMasterEvent = MASTER_SEND_DATA;
            UI2C_SET_CONTROL_REG(UI2C0, UI2C_CTL_PTRG);
        }
        else if(s_eMasterEvent == MASTER_SEND_DATA)
        {
            if(s_u8DataLenM != 3)
            {
                UI2C_SET_DATA(UI2C0, s_au8TxData[s_u8DataLenM++]);                 /* ADDRESS has been transmitted and write DATA to Register TXDAT */
                UI2C_SET_CONTROL_REG(UI2C0, UI2C_CTL_PTRG);
            }
            else
            {
                s_eMasterEvent = MASTER_STOP;
                UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_STO));       /* Send STOP signal */
            }
        }
    }
    else if((u32Status & UI2C_PROTSTS_NACKIF_Msk) == UI2C_PROTSTS_NACKIF_Msk)
    {
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_NACKIF_Msk);                    /* Clear NACK INT Flag */
        s_u8EndFlagM = 0;

        if(s_eMasterEvent == MASTER_SEND_ADDRESS)
        {
            /* SLA+W has been transmitted and NACK has been received */
            s_eMasterEvent = MASTER_SEND_START;
            UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_STA));           /* Send START signal */
        }
        else if(s_eMasterEvent == MASTER_SEND_DATA)
        {
            /* ADDRESS has been transmitted and NACK has been received */
            s_eMasterEvent = MASTER_STOP;
            UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_STO));           /* Send STOP signal */
        }
        else
            printf("Get Wrong NACK Event\n");
    }
    else if((u32Status & UI2C_PROTSTS_STORIF_Msk) == UI2C_PROTSTS_STORIF_Msk)
    {
        s_u8EndFlagM = 1;

        /* Clear STOP INT Flag */
        UI2C0->PROTSTS =  UI2C_PROTSTS_STORIF_Msk;

        /* Trigger USCI I2C */
        UI2C0->PROTCTL |= UI2C_PROTCTL_PTRG_Msk;
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  USCI_I2C Rx Callback Function                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
void USCI_I2C_EEPROM_MasterRx(uint32_t u32Status)
{
    if((u32Status & UI2C_PROTSTS_STARIF_Msk) == UI2C_PROTSTS_STARIF_Msk)
    {
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_STARIF_Msk);    /* Clear START INT Flag */

        if(s_eMasterEvent == MASTER_SEND_START)
        {
            UI2C_SET_DATA(UI2C0, (uint16_t)(s_u8DeviceAddr << 1) | 0x00);    /* Write SLA+W to Register TXDAT */
            s_eMasterEvent = MASTER_SEND_ADDRESS;
        }
        else if(s_eMasterEvent == MASTER_SEND_REPEAT_START)
        {
            UI2C_SET_DATA(UI2C0, (uint16_t)(s_u8DeviceAddr << 1) | 0x01);    /* Write SLA+R to Register TXDAT */
            s_eMasterEvent = MASTER_SEND_H_RD_ADDRESS;
        }

        UI2C_SET_CONTROL_REG(UI2C0, UI2C_CTL_PTRG);
    }
    else if((u32Status & UI2C_PROTSTS_ACKIF_Msk) == UI2C_PROTSTS_ACKIF_Msk)
    {
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_ACKIF_Msk);    /* Clear ACK INT Flag */

        if(s_eMasterEvent == MASTER_SEND_ADDRESS)
        {
            UI2C_SET_DATA(UI2C0, s_au8TxData[s_u8DataLenM++]);    /* SLA+W has been transmitted and write ADDRESS to Register TXDAT */
            s_eMasterEvent = MASTER_SEND_DATA;
            UI2C_SET_CONTROL_REG(UI2C0, UI2C_CTL_PTRG);
        }
        else if(s_eMasterEvent == MASTER_SEND_DATA)
        {
            if(s_u8DataLenM != 2)
            {
                UI2C_SET_DATA(UI2C0, s_au8TxData[s_u8DataLenM++]);    /* ADDRESS has been transmitted and write DATA to Register TXDAT */
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
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_NACKIF_Msk);    /* Clear NACK INT Flag */

        if(s_eMasterEvent == MASTER_SEND_ADDRESS)
        {
            s_eMasterEvent = MASTER_SEND_START;
            UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_STA));    /* Send START signal */
        }
        else if(s_eMasterEvent == MASTER_READ_DATA)
        {
            s_u8RxData = (unsigned char) UI2C_GET_DATA(UI2C0) & 0xFF;
            s_u8EndFlagM = 1;
            s_eMasterEvent = MASTER_STOP;
            UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_STO));    /* DATA has been received and send STOP signal */
        }
        else
            printf("Get Wrong NACK Event\n");
    }
    else if((u32Status & UI2C_PROTSTS_STORIF_Msk) == UI2C_PROTSTS_STORIF_Msk)
    {
        s_u8EndFlagM = 1;

        /* Clear STOP INT Flag */
        UI2C0->PROTSTS =  UI2C_PROTSTS_STORIF_Msk;

        /* Trigger USCI I2C */
        UI2C0->PROTCTL |= UI2C_PROTCTL_PTRG_Msk;
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
}

void UI2C0_Init(void)
{
    /* Open USCI_I2C0 and set clock to 100k */
    UI2C_Open(UI2C0, 100000);

    /* Get USCI_I2C0 Bus Clock */
    printf("USCI I2C clock %d Hz\n", UI2C_GetBusClockFreq(UI2C0));

    /* Set USCI_I2C0 Slave Addresses */
    UI2C_SetSlaveAddr(UI2C0, 0, 0x15, UI2C_GCMODE_DISABLE);   /* Slave Address : 0x15 */

    UI2C_ENABLE_PROT_INT(UI2C0, (UI2C_PROTIEN_ACKIEN_Msk | UI2C_PROTIEN_NACKIEN_Msk | UI2C_PROTIEN_STORIEN_Msk | UI2C_PROTIEN_STARIEN_Msk));

    NVIC_EnableIRQ(USCI0_IRQn);
}

int main(void)
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
        This sample code sets I2C bus clock to 100kHz. Then, accesses EEPROM 24LC64 with Byte Write
        and Byte Read operations, and check if the read data is equal to the programmed data.
    */

    printf("+-------------------------------------------------------+\n");
    printf("|     USCI_I2C Driver Sample Code with EEPROM 24LC64    |\n");
    printf("+-------------------------------------------------------+\n");

    printf("\n");
    printf("Configure UI2C0 as a Master\n");
    printf("The I/O connection for UI2C0:\n");
    printf("UI2C0_SDA(PA.10), UI2C0_SCL(PA.11)\n");

    /* Init USCI_I2C0 to access EEPROM */
    UI2C0_Init();

    s_u8DeviceAddr = 0x50;

    for(u32i = 0; u32i < 0x100; u32i++)
    {
        s_au8TxData[0] = (uint8_t)((u32i & 0xFF00) >> 8);
        s_au8TxData[1] = (uint8_t)(u32i & 0x00FF);
        s_au8TxData[2] = (uint8_t)(s_au8TxData[1] + 3);

        s_u8DataLenM = 0;
        s_u8EndFlagM = 0;

        /* USCI_I2C function to write data to slave */
        s_UI2C0HandlerFn = (UI2C_FUNC)USCI_I2C_EEPROM_MasterTx;

        /* USCI_I2C as master sends START signal */
        s_eMasterEvent = MASTER_SEND_START;
        UI2C_SET_CONTROL_REG(UI2C0, UI2C_CTL_STA);

        /* Wait USCI_I2C Tx Finish */
        while(s_u8EndFlagM == 0);
        s_u8EndFlagM = 0;

        /* USCI_I2C function to read data from slave */
        s_UI2C0HandlerFn = (UI2C_FUNC)USCI_I2C_EEPROM_MasterRx;

        s_u8DataLenM = 0;
        s_u8DeviceAddr = 0x50;

        s_eMasterEvent = MASTER_SEND_START;
        UI2C_SET_CONTROL_REG(UI2C0, UI2C_CTL_STA);

        /* Wait USCI_I2C Rx Finish */
        while(s_u8EndFlagM == 0);
        s_u8EndFlagM = 0;

        /* Compare data */
        if(s_u8RxData != s_au8TxData[2])
        {
            printf("USCI_I2C Byte Write/Read Failed, Data 0x%x\n", s_u8RxData);
            return -1;
        }
    }

    printf("\nUSCI_I2C Access EEPROM Test OK\n");

    while(1);
}
/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/


