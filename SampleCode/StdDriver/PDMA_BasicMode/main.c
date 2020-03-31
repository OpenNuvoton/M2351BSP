/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Use PDMA0 channel 2 to transfer data from memory to memory.
 * @note
 * Copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Macro, type and constant definitions                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
#define PDMA_TEST_LENGTH 64

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
static __attribute__((aligned)) uint8_t s_au8SrcArray[256];
static __attribute__((aligned)) uint8_t s_au8DestArray[256];
static uint32_t volatile s_u32IsTestOver = 0;

void PDMA0_IRQHandler(void);
void SYS_Init(void);
void UART0_Init(void);

/**
 * @brief       DMA0 IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The DMA default IRQ, declared in startup_M2351.s.
 */
void PDMA0_IRQHandler(void)
{
    uint32_t u32Status = PDMA_GET_INT_STATUS(PDMA0);

    if(u32Status & PDMA_INTSTS_ABTIF_Msk)    /* abort */
    {
        /* Check if channel 2 has abort error */
        if(PDMA_GET_ABORT_STS(PDMA0) & PDMA_ABTSTS_ABTIF2_Msk)
            s_u32IsTestOver = 2;
        /* Clear abort flag of channel 2 */
        PDMA_CLR_ABORT_FLAG(PDMA0, PDMA_ABTSTS_ABTIF2_Msk);
    }
    else if(u32Status & PDMA_INTSTS_TDIF_Msk)      /* done */
    {
        /* Check transmission of channel 2 has been transfer done */
        if(PDMA_GET_TD_STS(PDMA0) & PDMA_TDSTS_TDIF2_Msk)
            s_u32IsTestOver = 1;
        /* Clear transfer done flag of channel 2 */
        PDMA_CLR_TD_FLAG(PDMA0, PDMA_TDSTS_TDIF2_Msk);
    }
    else
        printf("unknown interrupt !!\n");
}

void SYS_Init(void)
{
    /* Set PF multi-function pins for XT1_OUT(PF.2) and XT1_IN(PF.3) */
    SYS->GPF_MFPL = (SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF2MFP_Msk)) | XT1_OUT_PF2;
    SYS->GPF_MFPL = (SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF3MFP_Msk)) | XT1_IN_PF3;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC clock (Internal RC 22.1184 MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and and HCLK clock divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable HXT clock (external XTAL 12MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Waiting for HXT clock ready */
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

    /* Enable PDMA clock source */
    CLK_EnableModuleClock(PDMA0_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & (~(UART0_RXD_PB12_Msk | UART0_TXD_PB13_Msk))) | UART0_RXD_PB12 | UART0_TXD_PB13;
}

void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}


/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    /* If user want to write protected register, please issue SYS_UnlockReg() to unlock protected register. */
    SYS_LockReg();

    /* Init UART for printf */
    UART0_Init();

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);
    printf("+------------------------------------------------------+ \n");
    printf("|    PDMA Memory to Memory Driver Sample Code          | \n");
    printf("+------------------------------------------------------+ \n");

    /* Reset PDMA module */
    SYS_ResetModule(PDMA0_RST);

    /*------------------------------------------------------------------------------------------------------

                         s_au8SrcArray                         s_au8DestArray
                         ---------------------------   -->   ---------------------------
                       /| [0]  | [1]  |  [2] |  [3] |       | [0]  | [1]  |  [2] |  [3] |\
                        |      |      |      |      |       |      |      |      |      |
       PDMA_TEST_LENGTH |            ...            |       |            ...            | PDMA_TEST_LENGTH
                        |      |      |      |      |       |      |      |      |      |
                       \| [60] | [61] | [62] | [63] |       | [60] | [61] | [62] | [63] |/
                         ---------------------------         ---------------------------
                         \                         /         \                         /
                               32bits(one word)                     32bits(one word)

      PDMA transfer configuration:

        Channel = 2
        Operation mode = basic mode
        Request source = PDMA_MEM(memory to memory)
        transfer done and table empty interrupt = enable

        Transfer count = PDMA_TEST_LENGTH
        Transfer width = 32 bits(one word)
        Source address = s_au8SrcArray
        Source address increment size = 32 bits(one word)
        Destination address = s_au8DestArray
        Destination address increment size = 32 bits(one word)
        Transfer type = burst transfer

        Total transfer length = PDMA_TEST_LENGTH * 32 bits
    ------------------------------------------------------------------------------------------------------*/

    /* Open Channel 2 */
    PDMA_Open(PDMA0, 1 << 2);
    /* Transfer count is PDMA_TEST_LENGTH, transfer width is 32 bits(one word) */
    PDMA_SetTransferCnt(PDMA0, 2, PDMA_WIDTH_32, PDMA_TEST_LENGTH);
    /* Set source address is s_au8SrcArray, destination address is s_au8DestArray, Source/Destination increment size is 32 bits(one word) */
    PDMA_SetTransferAddr(PDMA0, 2, (uint32_t)s_au8SrcArray, PDMA_SAR_INC, (uint32_t)s_au8DestArray, PDMA_DAR_INC);
    /* Request source is memory to memory */
    PDMA_SetTransferMode(PDMA0, 2, PDMA_MEM, FALSE, 0);
    /* Transfer type is burst transfer and burst size is 4 */
    PDMA_SetBurstType(PDMA0, 2, PDMA_REQ_BURST, PDMA_BURST_4);

    /* Enable interrupt */
    PDMA_EnableInt(PDMA0, 2, PDMA_INT_TRANS_DONE);

    /* Enable NVIC for PDMA */
    NVIC_EnableIRQ(PDMA0_IRQn);
    s_u32IsTestOver = 0;

    /* Generate a software request to trigger transfer with PDMA channel 2  */
    PDMA_Trigger(PDMA0, 2);

    /* Waiting for transfer done */
    while(s_u32IsTestOver == 0);

    /* Check transfer result */
    if(s_u32IsTestOver == 1)
        printf("test done...\n");
    else if(s_u32IsTestOver == 2)
        printf("target abort...\n");

    /* Close channel 2 */
    PDMA_Close(PDMA0);

    while(1);
}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
