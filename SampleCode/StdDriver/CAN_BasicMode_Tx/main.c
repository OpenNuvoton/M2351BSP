/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    Demonstrate CAN bus transmit a message with basic mode
 *
 *
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
static STR_CANMSG_T rrMsg;

void CAN_ShowMsg(STR_CANMSG_T* Msg);
void CAN_MsgInterrupt(CAN_T *tCAN, uint32_t u32IIDR);
void CAN0_IRQHandler(void);
void SYS_Init(void);
void DEBUG_PORT_Init(void);
void CAN_Init(CAN_T  *tCAN);
void CAN_STOP(CAN_T  *tCAN);
void Note_Configure(void);
void SelectCANSpeed(CAN_T  *tCAN);
void Test_BasicMode_Tx(CAN_T  *tCAN);

/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle CAN interrupt event                                                            */
/*---------------------------------------------------------------------------------------------------------*/
void CAN_MsgInterrupt(CAN_T *tCAN, uint32_t u32IIDR)
{
    if(u32IIDR == 1)
    {
        printf("Msg-0 INT and Callback\n");
        CAN_Receive(tCAN, 0, &rrMsg);
        CAN_ShowMsg(&rrMsg);
    }
    if(u32IIDR == 5 + 1)
    {
        printf("Msg-5 INT and Callback \n");
        CAN_Receive(tCAN, 5, &rrMsg);
        CAN_ShowMsg(&rrMsg);
    }
    if(u32IIDR == 31 + 1)
    {
        printf("Msg-31 INT and Callback \n");
        CAN_Receive(tCAN, 31, &rrMsg);
        CAN_ShowMsg(&rrMsg);
    }
}


/**
  * @brief  CAN0_IRQ Handler.
  * @param  None.
  * @return None.
  */
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
            CAN0->STATUS &= ~CAN_STATUS_RXOK_Msk;   /* Clear Rx Ok status*/

            printf("RX OK INT\n") ;
        }

        if(CAN0->STATUS & CAN_STATUS_TXOK_Msk)
        {
            CAN0->STATUS &= ~CAN_STATUS_TXOK_Msk;    /* Clear Tx Ok status*/

            printf("TX OK INT\n") ;
        }

        /**************************/
        /* Error Status interrupt */
        /**************************/
        if(CAN0->STATUS & CAN_STATUS_EWARN_Msk)
        {
            printf("EWARN INT\n") ;
        }

        if(CAN0->STATUS & CAN_STATUS_BOFF_Msk)
        {
            printf("BOFF INT\n") ;
        }
    }
    else if(u8IIDRstatus != 0)
    {
        printf("=> Interrupt Pointer = %d\n", CAN0->IIDR - 1);

        CAN_MsgInterrupt(CAN0, u8IIDRstatus);

        CAN_CLR_INT_PENDING_BIT(CAN0, (uint8_t)((CAN0->IIDR) - 1)); /* Clear Interrupt Pending */
    }
    else if(CAN0->WU_STATUS == 1)
    {
        printf("Wake up\n");

        CAN0->WU_STATUS = 0;                       /* Write '0' to clear */
    }

}


void SYS_Init(void)
{
    uint32_t u32TimeOutCnt;

    /* Enable PLL */
    CLK->PLLCTL = CLK_PLLCTL_128MHz_HIRC;

    /* Waiting for PLL stable */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while((CLK->STATUS & CLK_STATUS_PLLSTB_Msk) == 0)
        if(--u32TimeOutCnt == 0) break;

    /* Set HCLK divider to 2 */
    CLK->CLKDIV0 = (CLK->CLKDIV0 & (~CLK_CLKDIV0_HCLKDIV_Msk)) | 1;

    /* Switch HCLK clock source to PLL */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLKSEL_Msk)) | CLK_CLKSEL0_HCLKSEL_PLL;

    /* Select IP clock source */
    CLK->CLKSEL1 = CLK_CLKSEL1_UART0SEL_HIRC;
    CLK->CLKSEL3 = CLK_CLKSEL3_UART5SEL_HIRC;

    /* Enable IP clock */
    CLK->APBCLK0 |= CLK_APBCLK0_UART0CKEN_Msk | CLK_APBCLK0_TMR0CKEN_Msk | CLK_APBCLK0_UART5CKEN_Msk;


    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    //SystemCoreClockUpdate();
    PllClock        = 128000000;           // PLL
    SystemCoreClock = 128000000 / 2;       // HCLK
    CyclesPerUs     = 64000000 / 1000000;  // For CLK_SysTickDelay()

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & (~(UART0_RXD_PB12_Msk | UART0_TXD_PB13_Msk))) | UART0_RXD_PB12 | UART0_TXD_PB13;

    /* Set multi-function pins for CAN0 RXD and TXD */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & (~(CAN0_RXD_PA4_Msk | CAN0_TXD_PA5_Msk))) | CAN0_RXD_PA4 | CAN0_TXD_PA5;


}

void DEBUG_PORT_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    DEBUG_PORT->LINE = UART_PARITY_NONE | UART_STOP_BIT_1 | UART_WORD_LEN_8;
    DEBUG_PORT->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC, 115200);

}


/**
  * @brief      Init CAN driver
  */

void CAN_Init(CAN_T  *tCAN)
{
    if(tCAN == CAN0)
    {
        // Enable IP clock
        CLK->APBCLK0 |= CLK_APBCLK0_CAN0CKEN_Msk;

        // Reset CAN0
        SYS->IPRST1 |= SYS_IPRST1_CAN0RST_Msk;
        SYS->IPRST1 &= ~SYS_IPRST1_CAN0RST_Msk;
    }
}

/**
  * @brief      Disable CAN
  * @details    Reset and clear all CAN control and disable CAN IP
  */
void CAN_STOP(CAN_T  *tCAN)
{
    if(tCAN == CAN0)
    {
        /* Disable CAN0 Clock and Reset it */
        SYS->IPRST1 |= SYS_IPRST1_CAN0RST_Msk;
        SYS->IPRST1 &= ~SYS_IPRST1_CAN0RST_Msk;
        CLK->APBCLK0 &= ~CLK_APBCLK0_CAN0CKEN_Msk;
    }
}

/*----------------------------------------------------------------------------*/
/*  Some description about how to create test environment                     */
/*----------------------------------------------------------------------------*/
void Note_Configure()
{
    printf("\n\n");
    printf("+------------------------------------------------------------------------+\n");
    printf("|  About CAN sample code configure                                       |\n");
    printf("+------------------------------------------------------------------------+\n");
    printf("|   The sample code provide a simple sample code for you study CAN       |\n");
    printf("|   Before execute it, please check description as below                 |\n");
    printf("|                                                                        |\n");
    printf("|   1.CAN_TX and CAN_RX should be connected to your CAN transceiver      |\n");
    printf("|   2.Using two boards and connect to the same CAN BUS                   |\n");
    printf("|   3.Check the terminal resistor of bus is connected                    |\n");
    printf("|   4.Using UART0 as print message port                                  |\n");
    printf("|                                                                        |\n");
    printf("|  |--------|       |-----------| CANBUS  |-----------|       |--------| |\n");
    printf("|  |        |------>|           |<------->|           |<------|        | |\n");
    printf("|  |        |CAN_TX |   CAN     |  CAN_H  |   CAN     |CAN_TX |        | |\n");
    printf("|  |  MCU0  |       |Transceiver|         |Transceiver|       |  MCU1  | |\n");
    printf("|  |        |<------|           |<------->|           |------>|        | |\n");
    printf("|  |        |CAN_RX |           |  CAN_L  |           |CAN_RX |        | |\n");
    printf("|  |--------|       |-----------|         |-----------|       |--------| |\n");
    printf("|   |                                                           |        |\n");
    printf("|   |                                                           |        |\n");
    printf("|   V                                                           V        |\n");
    printf("| UART0                                                         UART0    |\n");
    printf("|(print message)                                          (print message)|\n");
    printf("+------------------------------------------------------------------------+\n");
}

void SelectCANSpeed(CAN_T  *tCAN)
{
    int32_t i32Item;
    uint32_t u32BaudRate = 0;

    printf("Please select CAN speed you desired\n");
    printf("[0] 1000Kbps\n");
    printf("[1]  500Kbps\n");
    printf("[2]  250Kbps\n");
    printf("[3]  125Kbps\n");
    printf("[4]  100Kbps\n");
    printf("[5]   50Kbps\n");

    i32Item = getchar();
    printf("%c\n", i32Item) ;
    if(i32Item == '1')
        u32BaudRate = CAN_Open(tCAN,  500000, CAN_BASIC_MODE);
    else if(i32Item == '2')
        u32BaudRate = CAN_Open(tCAN,  250000, CAN_BASIC_MODE);
    else if(i32Item == '3')
        u32BaudRate = CAN_Open(tCAN,  125000, CAN_BASIC_MODE);
    else if(i32Item == '4')
        u32BaudRate = CAN_Open(tCAN,  100000, CAN_BASIC_MODE);
    else if(i32Item == '5')
        u32BaudRate = CAN_Open(tCAN,   50000, CAN_BASIC_MODE);
    else
        u32BaudRate = CAN_Open(tCAN, 1000000, CAN_BASIC_MODE);

    if(u32BaudRate > 1000000)
        printf("Set CAN bit rate is fail\n");
}


/*----------------------------------------------------------------------------*/
/*  Test Function                                                             */
/*----------------------------------------------------------------------------*/
void CAN_ShowMsg(STR_CANMSG_T* Msg)
{
    uint8_t i;
    printf("Read ID=%8X, Type=%s, DLC=%d,Data=", Msg->Id, Msg->IdType ? "EXT" : "STD", Msg->DLC);
    for(i = 0; i < Msg->DLC; i++)
        printf("%02X,", Msg->Data[i]);
    printf("\n\n");
}
/*----------------------------------------------------------------------------*/
/*  Send Tx Msg by Basic Mode Function (Without Message RAM)                  */
/*----------------------------------------------------------------------------*/
void Test_BasicMode_Tx(CAN_T  *tCAN)
{
    //int32_t delaycount;
    STR_CANMSG_T msg1;
    //delaycount=1000;
    //CAN_EnterTestMode(tCAN, CAN_TEST_BASIC);
    CAN_EnableInt(tCAN, CAN_CON_IE_Msk | CAN_CON_SIE_Msk | CAN_CON_EIE_Msk);
    NVIC_SetPriority(CAN0_IRQn, (1 << __NVIC_PRIO_BITS) - 2);
    NVIC_EnableIRQ(CAN0_IRQn);

    /* Send Message No.1 */
    msg1.FrameType = CAN_DATA_FRAME;
    msg1.IdType   = CAN_STD_ID;
    msg1.Id       = 0x001;
    msg1.DLC      = 2;
    msg1.Data[0]  = 0x00;
    msg1.Data[1]  = 0x2;
    //CAN_BasicSendMsg(tCAN, &msg1);
    CAN_Transmit(tCAN, 0, &msg1);
    printf("Send STD_ID:0x1,Data[0,2]\n");
    //SYS_SysTickDelay(delaycount);

    /* Send Message No.2 */
    msg1.FrameType = CAN_DATA_FRAME;
    msg1.IdType   = CAN_STD_ID;
    msg1.Id       = 0x1AC;
    msg1.DLC      = 8;
    msg1.Data[0]  = 0x11;
    msg1.Data[1]  = 0x12;
    msg1.Data[2]  = 0x13;
    msg1.Data[3]  = 0x14;
    msg1.Data[4]  = 0x15;
    msg1.Data[5]  = 0x16;
    msg1.Data[6]  = 0x17;
    msg1.Data[7]  = 0x18;

    //CAN_BasicSendMsg(tCAN, &msg1);
    CAN_Transmit(tCAN, 0, &msg1);
    printf("Send STD_ID:0x1AC,Data[11,12,13,14,15,16,17,18]\n");
    //SYS_SysTickDelay(delaycount);

    /* Send Message No.3 */
    msg1.FrameType = CAN_DATA_FRAME;
    msg1.IdType   = CAN_STD_ID;
    msg1.Id       = 0x310;
    msg1.DLC      = 8;
    msg1.Data[0]  = 0x21;
    msg1.Data[1]  = 0x22;
    msg1.Data[2]  = 0x23;
    msg1.Data[3]  = 0x24;
    msg1.Data[4]  = 0x25;
    msg1.Data[5]  = 0x26;
    msg1.Data[6]  = 0x27;
    msg1.Data[7]  = 0x28;
    //CAN_BasicSendMsg(tCAN, &msg1);
    CAN_Transmit(tCAN, 0, &msg1);
    printf("Send STD_ID:0x310,Data[21,22,23,24,25,26,27,28]\n");
    //SYS_SysTickDelay(delaycount);

    /* Send Message No.4 */
    msg1.FrameType = CAN_DATA_FRAME;
    msg1.IdType   = CAN_EXT_ID;
    msg1.Id       = 0x3377;
    msg1.DLC      = 8;
    msg1.Data[0]  = 0x31;
    msg1.Data[1]  = 0x32;
    msg1.Data[2]  = 0x33;
    msg1.Data[3]  = 0x34;
    msg1.Data[4]  = 0x35;
    msg1.Data[5]  = 0x36;
    msg1.Data[6]  = 0x37;
    msg1.Data[7]  = 0x38;
    //CAN_BasicSendMsg(tCAN, &msg1);
    CAN_Transmit(tCAN, 0, &msg1);
    printf("Send EXT_ID:0x3377,Data[31,32,33,34,35,36,37,38]\n");
    //SYS_SysTickDelay(delaycount);

    /* Send Message No.5 */
    msg1.FrameType = CAN_DATA_FRAME;
    msg1.IdType   = CAN_EXT_ID;
    msg1.Id       = 0x7755;
    msg1.DLC      = 8;
    msg1.Data[0]  = 0x41;
    msg1.Data[1]  = 0x42;
    msg1.Data[2]  = 0x43;
    msg1.Data[3]  = 0x44;
    msg1.Data[4]  = 0x45;
    msg1.Data[5]  = 0x46;
    msg1.Data[6]  = 0x47;
    msg1.Data[7]  = 0x48;

    CAN_Transmit(tCAN, 0, &msg1);
    printf("Send EXT_ID:0x7755,Data[41,42,43,44,45,46,47,48]\n");

}



int main()
{
    CAN_T *tCAN;

    tCAN = (CAN_T *) CAN0;

    SYS_UnlockReg();

    SYS_Init();
    DEBUG_PORT_Init();

    /* Select CAN Multi-Function */
    CAN_Init(tCAN);
    Note_Configure();
    SelectCANSpeed(tCAN);
    //SelectCANSpeed(CAN1);

    printf("\n");
    printf("+------------------------------------------------------------------ +\n");
    printf("|  Nuvoton CAN BUS DRIVER DEMO                                      |\n");
    printf("+-------------------------------------------------------------------+\n");
    printf("|  Transmit a message by basic mode                                 |\n");
    printf("+-------------------------------------------------------------------+\n");

    printf("Press any key to continue ...\n\n");
    getchar();
    Test_BasicMode_Tx(tCAN);

    while(1) ;
}

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
