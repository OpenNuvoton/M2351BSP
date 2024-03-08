/***************************************************************************//**
 * @file     main.c
 * @brief    ISP tool main function
 * @version  0x32
 * @date     14, June, 2017
 *
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2017-2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

#define V6M_AIRCR_VECTKEY_DATA            0x05FA0000UL
#define V6M_AIRCR_SYSRESETREQ             0x00000004UL

#define PLLCTL_SETTING                    CLK_PLLCTL_64MHz_HIRC
#define PLL_CLOCK                         64000000
#define HCLK_DIV                          1

#define GPIO_SETMODE(port, pin, u32Mode)  (port->MODE = (port->MODE & ~(0x3ul << (pin << 1))) | (u32Mode << (pin << 1)))

#define CAN_BAUD_RATE                     500000
#define Master_ISP_ID                     0x487
#define Device0_ISP_ID                    0x784
#define CAN_ISP_DtatLength                0x08
#define CAN_RETRY_COUNTS                  0x1fffffff

#define CMD_READ_CONFIG                   0xA2000000
#define CMD_RUN_APROM                     0xAB000000
#define CMD_GET_DEVICEID                  0xB1000000

/*---------------------------------------------------------------------------*/
/*  Function Declare                                                         */
/*---------------------------------------------------------------------------*/
extern char GetChar(void);

/* Declare a CAN message structure */
typedef struct
{
    uint32_t  Address;
    uint32_t  Data;
} STR_CANMSG_ISP;

static STR_CANMSG_T s_rrMsg;
static volatile uint8_t s_u8CANPackageFlag = 0, s_u8CANAckFlag = 0;

void CAN_MsgInterrupt(CAN_T *tCAN, uint32_t u32IIDR);
void CAN0_IRQHandler(void);
int32_t SYS_Init(void);
void CAN_Package_ACK(CAN_T *tCAN);
void CAN_Init(void);
void ProcessHardFault(void);
void SH_Return(void);

/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle CAN interrupt event                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void CAN_MsgInterrupt(CAN_T *tCAN, uint32_t u32IIDR)
{
    if(u32IIDR == 1)
    {
        CAN_Receive(tCAN, 0, &s_rrMsg);
        s_u8CANPackageFlag = 1;
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/* CAN interrupt handler                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void CAN0_IRQHandler(void)
{
    uint32_t u8IIDRstatus;
    u8IIDRstatus = CAN0->IIDR;

    if(u8IIDRstatus == 0x00008000)        /* Check Status Interrupt Flag (Error status Int and Status change Int) */
    {
        /**************************/
        /* Status Change interrupt*/
        /**************************/
        if(CAN0->STATUS & CAN_STATUS_RXOK_Msk)
        {
            CAN0->STATUS &= ~CAN_STATUS_RXOK_Msk;   /* Clear RxOK status*/
        }

        if(CAN0->STATUS & CAN_STATUS_TXOK_Msk)
        {
            CAN0->STATUS &= ~CAN_STATUS_TXOK_Msk;    /* Clear TxOK status*/
            s_u8CANAckFlag = 0;
        }
    }
    else if(u8IIDRstatus != 0)
    {
        CAN_MsgInterrupt(CAN0, u8IIDRstatus);
        CAN_CLR_INT_PENDING_BIT(CAN0, (uint8_t)(u8IIDRstatus - 1)); /* Clear Interrupt Pending */
    }
}

int32_t SYS_Init(void)
{
    uint32_t u32TimeOutCnt;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC clock */
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;

    /* Waiting for clock ready */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while(!(CLK->STATUS & CLK_STATUS_HIRCSTB_Msk))
        if(--u32TimeOutCnt == 0) return -1;

    /* Set core clock as PLL_CLOCK from PLL */
    CLK->PLLCTL = PLLCTL_SETTING;

    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while(!(CLK->STATUS & CLK_STATUS_PLLSTB_Msk))
        if(--u32TimeOutCnt == 0) return -1;

    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLKSEL_Msk)) | CLK_CLKSEL0_HCLKSEL_PLL;
    CLK->CLKDIV0 &= ~CLK_CLKDIV0_HCLKDIV_Msk;
    CLK->CLKDIV0 |= CLK_CLKDIV0_HCLK(HCLK_DIV);
    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    //SystemCoreClockUpdate();
    PllClock        = PLL_CLOCK;                        // PLL
    SystemCoreClock = PLL_CLOCK / HCLK_DIV;             // HCLK
    CyclesPerUs     = SystemCoreClock / 1000000;  // For CLK_SysTickDelay()

    return 0;
}

/*----------------------------------------------------------------------------*/
/*  Tx Msg by Normal Mode Function (With Message RAM)                         */
/*----------------------------------------------------------------------------*/
void CAN_Package_ACK(CAN_T *tCAN)
{
    STR_CANMSG_T tMsg;
    s_u8CANAckFlag = 1;
    /* Send a 11-bit Standard Identifier message */
    tMsg.FrameType = CAN_DATA_FRAME;
    tMsg.IdType    = CAN_STD_ID;
    tMsg.Id        = Device0_ISP_ID;
    tMsg.DLC       = CAN_ISP_DtatLength;
    memcpy(&tMsg.Data, &s_rrMsg.Data, 8);

    if(CAN_Transmit(tCAN, MSG(5), &tMsg) == FALSE)    // Configure Msg RAM and send the Msg in the RAM
    {
        return;
    }

    while(s_u8CANAckFlag);
}

void CAN_Init(void)
{
    /* Enable CAN module clock */
    CLK->APBCLK0 |= CLK_APBCLK0_CAN0CKEN_Msk;
    /* Set PC multi-function pins for CAN RXD(PC.9) and TXD(PC.10) */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & (~(CAN0_RXD_PB10_Msk | CAN0_TXD_PB11_Msk))) | CAN0_RXD_PB10 | CAN0_TXD_PB11;

    /* Set CAN transceiver to high speed mode */
    GPIO_SETMODE(PC, 11, GPIO_MODE_OUTPUT);
    PC11 = 0;
    CAN_Open(CAN0, CAN_BAUD_RATE, CAN_NORMAL_MODE);
    CAN_EnableInt(CAN0, (CAN_CON_IE_Msk | CAN_CON_SIE_Msk | CAN_CON_EIE_Msk));
    NVIC_SetPriority(CAN0_IRQn, (1 << __NVIC_PRIO_BITS) - 2);
    NVIC_EnableIRQ(CAN0_IRQn);
    /* Set CAN reveive message */
    CAN_SetRxMsg(CAN0, MSG(0), CAN_STD_ID, Master_ISP_ID);
}

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    uint32_t  Address, Data;
    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Init System, IP clock and multi-function I/O */
    if( SYS_Init() < 0 ) goto _APROM;
    /* Enable FMC ISP function */
    FMC_Open();
    FMC_ENABLE_AP_UPDATE();
    FMC_ENABLE_CFG_UPDATE();
    /* Init CAN port */
    CAN_Init();
    SysTick->LOAD = 300000 * CyclesPerUs;
    SysTick->VAL   = (0x00);
    SysTick->CTRL = SysTick->CTRL | SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

    while(1)
    {
        if(s_u8CANPackageFlag == 1)
        {
            break;
        }

        if(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)
        {
            goto _APROM;
        }
    }

    /* stat update program */
    while(1)
    {
        if(s_u8CANPackageFlag)
        {
            s_u8CANPackageFlag = 0;
            Address = (uint32_t)((s_rrMsg.Data[3]<<24)|(s_rrMsg.Data[2]<<16)|(s_rrMsg.Data[1]<<8)|s_rrMsg.Data[0]);
            Data = (uint32_t)((s_rrMsg.Data[7]<<24)|(s_rrMsg.Data[6]<<16)|(s_rrMsg.Data[5]<<8)|s_rrMsg.Data[4]);

            if(Address == CMD_GET_DEVICEID)
            {
                s_rrMsg.Data[4] = _GET_BYTE0(SYS->PDID);
                s_rrMsg.Data[5] = _GET_BYTE1(SYS->PDID);
                s_rrMsg.Data[6] = _GET_BYTE2(SYS->PDID);
                s_rrMsg.Data[7] = _GET_BYTE3(SYS->PDID);
            }
            else if(Address == CMD_READ_CONFIG)
            {
                uint32_t u32ReadData;

                u32ReadData = FMC_Read(Data);
                s_rrMsg.Data[4] = _GET_BYTE0(u32ReadData);
                s_rrMsg.Data[5] = _GET_BYTE1(u32ReadData);
                s_rrMsg.Data[6] = _GET_BYTE2(u32ReadData);
                s_rrMsg.Data[7] = _GET_BYTE3(u32ReadData);
            }
            else if(Address == CMD_RUN_APROM)
            {
                break;
            }
            else
            {
                if((Address % FMC_FLASH_PAGE_SIZE) == 0)
                {
                    FMC_Erase(Address);
                }

                FMC_Write(Address, Data);         //program ROM
                Data = FMC_Read(Address);
                memcpy(&s_rrMsg.Data[4], &Data, 4); //update data
            }

            CAN_Package_ACK(CAN0);            //send CAN ISP Package (ACK)
        }
    }

_APROM:

    /* Reset system and boot from APROM */
    SYS->RSTSTS = (SYS_RSTSTS_PORF_Msk | SYS_RSTSTS_PINRF_Msk);
    FMC->ISPCTL &= ~(FMC_ISPCTL_ISPEN_Msk | FMC_ISPCTL_BS_Msk);
    SCB->AIRCR = (V6M_AIRCR_VECTKEY_DATA | V6M_AIRCR_SYSRESETREQ);

    /* Trap the CPU */
    while(1);
}

void ProcessHardFault()
{}

void SH_Return()
{}
