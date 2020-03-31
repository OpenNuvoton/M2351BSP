/****************************************************************************
 * @file     main.c
 * @version  V3.00
 * @brief    Transmit and receive UART data with PDMA.
 *
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"



#define UART_RX_DMA_CH 0
#define UART_TX_DMA_CH 1


/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
static int32_t s_i32UartTestLength = 64;
static uint8_t s_au8SrcArray[64];
static uint8_t s_au8DestArray[64];
static volatile int32_t s_i32IntCnt;
static volatile int32_t s_i32IsTestOver;
static volatile uint32_t s_u32TwoChannelPdmaTest = 0;


/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void);
void ClearBuf(uint32_t u32Addr, uint32_t u32Length, uint8_t u8Pattern);
void BuildSrcPattern(uint32_t u32Addr, uint32_t u32Length);
void PDMA_UART_TxTest(void);
void PDMA_UART_RxTest(void);
void PDMA_Callback_0(void);
void PDMA_Callback_1(void);
void PDMA0_IRQHandler(void);
void UART0_IRQHandler(void);
void UART1_IRQHandler(void);
void PDMA_UART(int32_t i32option);
void SYS_Init(void);
void UART0_Init(void);
void UART1_Init(void);
/*---------------------------------------------------------------------------------------------------------*/
/* Clear buffer function                                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
void ClearBuf(uint32_t u32Addr, uint32_t u32Length, uint8_t u8Pattern)
{
    uint8_t* pu8Ptr;
    uint32_t u32Idx;

    pu8Ptr = (uint8_t *)u32Addr;

    for(u32Idx = 0; u32Idx < u32Length; u32Idx++)
    {
        *pu8Ptr++ = u8Pattern;
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/* Build Src Pattern function                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void BuildSrcPattern(uint32_t u32Addr, uint32_t u32Length)
{
    uint32_t u32Idxi = 0, u32Idxj, u32Loop;
    uint8_t* pu8Addr;

    pu8Addr = (uint8_t *)u32Addr;

    do
    {
        if(u32Length > 256)
            u32Loop = 256;
        else
            u32Loop = u32Length;

        u32Length = u32Length - u32Loop;

        for(u32Idxj = 0; u32Idxj < u32Loop; u32Idxj++)
            *pu8Addr++ = (uint8_t)(u32Idxj + u32Idxi);

        u32Idxi++;
    }
    while((u32Loop != 0) || (u32Length != 0));
}

/*---------------------------------------------------------------------------------------------------------*/
/* UART Tx PDMA Channel Configuration                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
void PDMA_UART_TxTest(void)
{
    /* UART Tx PDMA channel configuration */
    /* Set transfer width (8 bits) and transfer count */
    PDMA_SetTransferCnt(PDMA0, UART_TX_DMA_CH, PDMA_WIDTH_8, (uint32_t)s_i32UartTestLength);

    /* Set source/destination address and attributes */
    PDMA_SetTransferAddr(PDMA0, UART_TX_DMA_CH, (uint32_t)s_au8SrcArray, PDMA_SAR_INC, (uint32_t)&UART1->DAT, PDMA_DAR_FIX);

    /* Set request source; set basic mode. */
    PDMA_SetTransferMode(PDMA0, UART_TX_DMA_CH, PDMA_UART1_TX, FALSE, 0);

    /* Single request type */
    PDMA_SetBurstType(PDMA0, UART_TX_DMA_CH, PDMA_REQ_SINGLE, 0);

    /* Disable table interrupt */
    PDMA0->DSCT[UART_TX_DMA_CH].CTL |= PDMA_DSCT_CTL_TBINTDIS_Msk;
}

/*---------------------------------------------------------------------------------------------------------*/
/* UART Rx PDMA Channel Configuration                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
void PDMA_UART_RxTest(void)
{
    /* UART Rx PDMA channel configuration */
    /* Set transfer width (8 bits) and transfer count */
    PDMA_SetTransferCnt(PDMA0, UART_RX_DMA_CH, PDMA_WIDTH_8, (uint32_t)s_i32UartTestLength);

    /* Set source/destination address and attributes */
    PDMA_SetTransferAddr(PDMA0, UART_RX_DMA_CH, (uint32_t)&UART1->DAT, PDMA_SAR_FIX, (uint32_t)s_au8DestArray, PDMA_DAR_INC);

    /* Set request source; set basic mode. */
    PDMA_SetTransferMode(PDMA0, UART_RX_DMA_CH, PDMA_UART1_RX, FALSE, 0);

    /* Single request type */
    PDMA_SetBurstType(PDMA0, UART_RX_DMA_CH, PDMA_REQ_SINGLE, 0);

    /* Disable table interrupt */
    PDMA0->DSCT[UART_RX_DMA_CH].CTL |= PDMA_DSCT_CTL_TBINTDIS_Msk;
}

/*---------------------------------------------------------------------------------------------------------*/
/* PDMA Callback function                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void PDMA_Callback_0(void)
{
    printf("\tTransfer Done %d!\r", ++s_i32IntCnt);

    /* Use PDMA to do UART loopback test 10 times */
    if(s_i32IntCnt < 10)
    {
        /* UART Tx and Rx PDMA configuration */
        PDMA_UART_TxTest();
        PDMA_UART_RxTest();

        /* Enable UART Tx and Rx PDMA function */
        UART1->INTEN |= (UART_INTEN_RXPDMAEN_Msk | UART_INTEN_TXPDMAEN_Msk);
    }
    else
    {
        /* Test is over */
        s_i32IsTestOver = TRUE;
    }
}

void PDMA_Callback_1(void)
{
    int32_t i32Idx;
    uint8_t u8InChar = 0xFF;

    printf("\tTransfer Done %d!\t", ++s_i32IntCnt);

    /* Show UART Rx data */
    for(i32Idx = 0; i32Idx < s_i32UartTestLength; i32Idx++)
    {
        u8InChar = inpb(((uint32_t)s_au8DestArray + (uint32_t)i32Idx));
        printf(" 0x%x(%c),", u8InChar, u8InChar);
    }
    printf("\n");

    /* Use PDMA to do UART Rx test 10 times */
    if(s_i32IntCnt < 10)
    {
        /* UART Rx PDMA configuration */
        PDMA_UART_RxTest();

        /* Enable UART Rx PDMA function */
        UART1->INTEN |= UART_INTEN_RXPDMAEN_Msk;
    }
    else
    {
        /* Test is over */
        s_i32IsTestOver = TRUE;
    }
}

void PDMA0_IRQHandler(void)
{
    /* Get PDMA interrupt status */
    uint32_t u32Status = PDMA_GET_INT_STATUS(PDMA0);

    if(u32Status & PDMA_INTSTS_ABTIF_Msk)  /* Target Abort */
    {
        s_i32IsTestOver = 2;
        PDMA0->ABTSTS = PDMA0->ABTSTS;
    }
    else if(u32Status & PDMA_INTSTS_TDIF_Msk) /* Transfer Done */
    {
        /* UART Tx PDMA transfer done interrupt flag */
        if(PDMA_GET_TD_STS(PDMA0) & (1 << UART_TX_DMA_CH))
        {
            /* Clear PDMA transfer done interrupt flag */
            PDMA_CLR_TD_FLAG(PDMA0, (1 << UART_TX_DMA_CH));

            /* Disable UART Tx PDMA function */
            UART1->INTEN &= ~UART_INTEN_TXPDMAEN_Msk;
        }

        /* UART Rx PDMA transfer done interrupt flag */
        if(PDMA_GET_TD_STS(PDMA0) & (1 << UART_RX_DMA_CH))
        {
            /* Clear PDMA transfer done interrupt flag */
            PDMA_CLR_TD_FLAG(PDMA0, (1 << UART_RX_DMA_CH));

            /* Disable UART Rx PDMA function */
            UART1->INTEN &= ~UART_INTEN_RXPDMAEN_Msk;

            /* Handle PDMA transfer done interrupt event */
            if(s_u32TwoChannelPdmaTest == 1)
            {
                PDMA_Callback_0();
            }
            else if(s_u32TwoChannelPdmaTest == 0)
            {
                PDMA_Callback_1();
            }
        }
    }
    else
    {
        printf("unknown interrupt, status=0x%x !!\n", u32Status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle UART Channel 0 interrupt event                                                            */
/*---------------------------------------------------------------------------------------------------------*/
void UART0_IRQHandler(void)
{
    /* Get UART0 Rx data and send the data to UART1 Tx */
    if(UART_GET_INT_FLAG(UART0, UART_INTSTS_RDAIF_Msk))
        UART1->DAT = UART0->DAT;
}

/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle UART Channel 1 interrupt event                                                            */
/*---------------------------------------------------------------------------------------------------------*/
void UART1_IRQHandler(void)
{
    uint32_t u32Data;
    uint32_t u32IntSts = UART1->INTSTS;

    if(u32IntSts & UART_INTSTS_HWRLSIF_Msk)
    {
        if(UART1->FIFOSTS & UART_FIFOSTS_BIF_Msk)
            printf("\n BIF \n");
        if(UART1->FIFOSTS & UART_FIFOSTS_FEF_Msk)
            printf("\n FEF \n");
        if(UART1->FIFOSTS & UART_FIFOSTS_PEF_Msk)
            printf("\n PEF \n");

        u32Data = UART1->DAT; /* read out data */
        printf("\n Error Data is '0x%x' \n", u32Data);
        UART1->FIFOSTS = (UART_FIFOSTS_BIF_Msk | UART_FIFOSTS_FEF_Msk | UART_FIFOSTS_PEF_Msk);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/* PDMA Sample Code:                                                                                       */
/*         i32option : ['1'] UART1 TX/RX PDMA Loopback                                                     */
/*                     [Others] UART1 RX PDMA test                                                         */
/*---------------------------------------------------------------------------------------------------------*/
void PDMA_UART(int32_t i32option)
{
    /* Source data initiation */
    BuildSrcPattern((uint32_t)s_au8SrcArray, (uint32_t)s_i32UartTestLength);
    ClearBuf((uint32_t)s_au8DestArray, (uint32_t)s_i32UartTestLength, 0xFF);

    /* Reset PDMA module */
    SYS_ResetModule(PDMA0_RST);

    if(i32option == '1')
    {
        printf("  [Using TWO PDMA channel].\n");
        printf("  This sample code will use PDMA to do UART1 loopback test 10 times.\n");
        printf("  Please connect UART1_RXD(PB.6) <--> UART1_TXD(PB.7) before testing.\n");
        printf("  After connecting PB.6 <--> PB.7, press any key to start transfer.\n");
        s_u32TwoChannelPdmaTest = 1;
        getchar();
    }
    else
    {
        s_i32UartTestLength = 2;      /* Test Length */
        printf("  [Using ONE PDMA channel].\n");
        printf("  This sample code will use PDMA to do UART1 Rx test 10 times.\n");
        printf("  Please connect UART1_RXD(PB.6) <--> UART1_TXD(PB.7) before testing.\n");
        printf("  After connecting PB6.9 <--> PB,7, press any key to start transfer.\n");
        s_u32TwoChannelPdmaTest = 0;
        getchar();
        printf("  Please input %d bytes to trigger PDMA one time.(Ex: Press 'a''b')\n", s_i32UartTestLength);
    }

    if(s_u32TwoChannelPdmaTest == 1)
    {
        /* Enable PDMA channel */
        PDMA_Open(PDMA0, (1 << UART_RX_DMA_CH) | (1 << UART_TX_DMA_CH));

        /* UART Tx and Rx PDMA configuration */
        PDMA_UART_TxTest();
        PDMA_UART_RxTest();

        /* Enable PDMA Transfer Done Interrupt */
        PDMA_EnableInt(PDMA0, UART_RX_DMA_CH, PDMA_INT_TRANS_DONE);
        PDMA_EnableInt(PDMA0, UART_TX_DMA_CH, PDMA_INT_TRANS_DONE);
    }
    else
    {
        /* Enable PDMA channel */
        PDMA_Open(PDMA0, (1 << UART_RX_DMA_CH));

        /* UART Rx PDMA configuration */
        PDMA_UART_RxTest();

        /* Enable PDMA Transfer Done Interrupt */
        PDMA_EnableInt(PDMA0, UART_RX_DMA_CH, PDMA_INT_TRANS_DONE);
    }

    /* Enable PDMA Transfer Done Interrupt */
    s_i32IntCnt = 0;
    s_i32IsTestOver = FALSE;
    NVIC_EnableIRQ(PDMA0_IRQn);

    /* Enable UART0 RDA interrupt */
    if(s_u32TwoChannelPdmaTest == 0)
    {
        UART_EnableInt(UART0, UART_INTEN_RDAIEN_Msk);
    }

    /* Enable Receive Line interrupt */
    UART1->INTEN |= UART_INTEN_RLSIEN_Msk;
    NVIC_EnableIRQ(UART1_IRQn);

    /* Enable UART Tx and Rx PDMA function */
    if(s_u32TwoChannelPdmaTest == 1)
        UART1->INTEN |= UART_INTEN_TXPDMAEN_Msk;
    else
        UART1->INTEN &= ~UART_INTEN_TXPDMAEN_Msk;

    UART1->INTEN |= UART_INTEN_RXPDMAEN_Msk;

    /* Wait for PDMA operation finish */
    while(s_i32IsTestOver == FALSE);

    /* Check PDMA status */
    if(s_i32IsTestOver == 2)
        printf("target abort...\n");

    /* Disable UART Tx and Rx PDMA function */
    UART1->INTEN &= ~(UART_INTEN_TXPDMAEN_Msk | UART_INTEN_RXPDMAEN_Msk);

    /* Disable PDMA channel */
    PDMA_Close(PDMA0);

    /* Disable PDMA Interrupt */
    PDMA_DisableInt(PDMA0, UART_RX_DMA_CH, PDMA_INT_TRANS_DONE);
    PDMA_DisableInt(PDMA0, UART_TX_DMA_CH, PDMA_INT_TRANS_DONE);
    NVIC_DisableIRQ(PDMA0_IRQn);

    /* Disable UART0 RDA interrupt */
    UART_DisableInt(UART0, UART_INTEN_RDAIEN_Msk);
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

    /* Select HCLK clock source as HIRC and HCLK clock divider as 1 */
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

    /* Enable UART and PDMA module clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(UART1_MODULE);
    CLK_EnableModuleClock(PDMA0_MODULE);

    /* Select UART module clock source as HXT and UART module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HXT, CLK_CLKDIV0_UART0(1));
    CLK_SetModuleClock(UART1_MODULE, CLK_CLKSEL1_UART1SEL_HXT, CLK_CLKDIV0_UART1(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & (~(UART0_RXD_PB12_Msk | UART0_TXD_PB13_Msk))) | UART0_RXD_PB12 | UART0_TXD_PB13;

    /* Set PB multi-function pins for UART1 RXD(PB.6) and TXD(PB.7) */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB6MFP_Msk)) | SYS_GPB_MFPL_PB6MFP_UART1_RXD;
    SYS->GPB_MFPL = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB7MFP_Msk)) | SYS_GPB_MFPL_PB7MFP_UART1_TXD;

}

void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART0 */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}

void UART1_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART1 */
    SYS_ResetModule(UART1_RST);

    /* Configure UART1 and set UART1 Baudrate */
    UART_Open(UART1, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{

    int32_t i32Item;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    /* Init UART1 */
    UART1_Init();

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);

    printf("\nUART PDMA Sample Program");

    /* UART PDMA sample function */
    do
    {
        printf("\n\n");
        printf("+------------------------------------------------------------------------+\n");
        printf("|                      UART PDMA Driver Sample Code                      |\n");
        printf("+------------------------------------------------------------------------+\n");
        printf("| [1] Using TWO PDMA channel to test. < TX1(CH1)-->RX1(CH0) >            |\n");
        printf("| [2] Using ONE PDMA channel to test. < TX1-->RX1(CH0) >                 |\n");
        printf("+------------------------------------------------------------------------+\n");
        i32Item = getchar();

        s_i32IsTestOver = FALSE;
        if((i32Item == '1') || (i32Item == '2'))
        {
            PDMA_UART(i32Item);
            printf("\n\n  UART PDMA sample code is complete.\n");
        }

    }
    while(i32Item != 27);

    printf("\nUART PDMA Sample Program End\n");

    while(1);

}

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/

