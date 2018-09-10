/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    Demonstrate CAN bus receive a message with basic mode
 *
 *
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
STR_CANMSG_T rrMsg;

void CAN_ShowMsg(STR_CANMSG_T* Msg);

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

        CAN_CLR_INT_PENDING_BIT(CAN0, ((CAN0->IIDR) - 1));     /* Clear Interrupt Pending */

    }
    else if(CAN0->WU_STATUS == 1)
    {
        printf("Wake up\n");

        CAN0->WU_STATUS = 0;                       /* Write '0' to clear */
    }

}



void SYS_Init(void)
{


    /* Enable PLL */
    CLK->PLLCTL = CLK_PLLCTL_128MHz_HIRC;

    /* Waiting for PLL stable */
    while((CLK->STATUS & CLK_STATUS_PLLSTB_Msk) == 0);

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
    SystemCoreClock = 128000000 / 1;       // HCLK
    CyclesPerUs     = 64000000 / 1000000;  // For SYS_SysTickDelay()

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
    printf("|   2.Using two MCU boards and connect to the same CAN BUS               |\n");
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
    uint32_t unItem;
    int32_t i32Err = 0;

    printf("Please select CAN speed you desired\n");
    printf("[0] 1000Kbps\n");
    printf("[1]  500Kbps\n");
    printf("[2]  250Kbps\n");
    printf("[3]  125Kbps\n");
    printf("[4]  100Kbps\n");
    printf("[5]   50Kbps\n");

    unItem = getchar();
    printf("%c\n", unItem) ;
    if(unItem == '1')
        i32Err = CAN_Open(tCAN,  500000, CAN_BASIC_MODE);
    else if(unItem == '2')
        i32Err = CAN_Open(tCAN,  250000, CAN_BASIC_MODE);
    else if(unItem == '3')
        i32Err = CAN_Open(tCAN,  125000, CAN_BASIC_MODE);
    else if(unItem == '4')
        i32Err = CAN_Open(tCAN,  100000, CAN_BASIC_MODE);
    else if(unItem == '5')
        i32Err = CAN_Open(tCAN,   50000, CAN_BASIC_MODE);
    else
        i32Err = CAN_Open(tCAN, 1000000, CAN_BASIC_MODE);

    if(i32Err < 0)
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


void CAN_ResetIF(CAN_T *tCAN, uint8_t u8IF_Num)
{
    if(u8IF_Num > 1)
        return;
    tCAN->IF[u8IF_Num].CREQ     = 0x0;          // set bit15 for sending
    tCAN->IF[u8IF_Num].CMASK    = 0x0;
    tCAN->IF[u8IF_Num].MASK1    = 0x0;          // useless in basic mode
    tCAN->IF[u8IF_Num].MASK2    = 0x0;          // useless in basic mode
    tCAN->IF[u8IF_Num].ARB1     = 0x0;          // ID15~0
    tCAN->IF[u8IF_Num].ARB2     = 0x0;          // MsgVal, eXt, xmt, ID28~16
    tCAN->IF[u8IF_Num].MCON     = 0x0;              // DLC
    tCAN->IF[u8IF_Num].DAT_A1   = 0x0;          // data0,1
    tCAN->IF[u8IF_Num].DAT_A2   = 0x0;          // data2,3
    tCAN->IF[u8IF_Num].DAT_B1   = 0x0;          // data4,5
    tCAN->IF[u8IF_Num].DAT_B2   = 0x0;          // data6,7
}

/*----------------------------------------------------------------------------*/
/*  Read Rx Msg by Basic Mode Function (Without Message RAM)                  */
/*----------------------------------------------------------------------------*/
void Test_BasicMode_Rx(CAN_T *tCAN)
{

    STR_CANMSG_T rMsg[5];
    int32_t i;

    /*  Wait status flag changed and with read IF2 */
    printf("\b Total 40 bytes data(using 5 frames)will be receive by CAN0 from CAN BUS\n");

    for(i = 0; i < 5; i++)
    {
        CAN_ResetIF(tCAN, 1);
        tCAN->IF[1].MCON = 0;
        while(CAN_Receive(tCAN, 0, &rMsg[i]) == FALSE);
    }
    for(i = 0; i < 5; i++)
        CAN_ShowMsg(&rMsg[i]);
    printf("CAN TEST END\n");
    return;
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

    printf("\n");
    printf("+------------------------------------------------------------------ +\n");
    printf("|  Nuvoton CAN BUS DRIVER DEMO                                      |\n");
    printf("+-------------------------------------------------------------------+\n");
    printf("|  Receive a message by basic mode                                 |\n");
    printf("+-------------------------------------------------------------------+\n");

    printf("Press any key to continue ...\n\n");
    getchar();
    Test_BasicMode_Rx(tCAN);

    while(1) ;

}

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
