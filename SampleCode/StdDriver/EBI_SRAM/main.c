/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Configure EBI interface to access BS616LV4017 (SRAM) on EBI interface.
 *
 * @copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


/*---------------------------------------------------------------------------------------------------------*/
/* Global Interface Variables Declarations                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
extern void SRAM_BS616LV4017(uint32_t u32MaxSize);
void AccessEBIWithPDMA(void);
void Configure_EBI_16BIT_Pins(void);
void SYS_Init(void);
void UART_Init(void);
void PDMA0_IRQHandler(void);

void Configure_EBI_16BIT_Pins(void)
{
    /* AD0 ~ AD7*/
    SYS->GPC_MFPL &= ~(EBI_AD0_PC0_Msk | EBI_AD1_PC1_Msk | EBI_AD2_PC2_Msk | EBI_AD3_PC3_Msk |
                       EBI_AD4_PC4_Msk | EBI_AD5_PC5_Msk);
    SYS->GPD_MFPH &= ~(EBI_AD6_PD8_Msk | EBI_AD7_PD9_Msk);
    SYS->GPC_MFPL |= (EBI_AD0_PC0 | EBI_AD1_PC1 | EBI_AD2_PC2 | EBI_AD3_PC3 | EBI_AD4_PC4 | EBI_AD5_PC5);
    SYS->GPD_MFPH |= (EBI_AD6_PD8 | EBI_AD7_PD9);
    /* AD8 ~ AD15 */
    SYS->GPE_MFPH &= (EBI_AD8_PE14_Msk | EBI_AD9_PE15_Msk);
    SYS->GPE_MFPL &= (EBI_AD10_PE1_Msk | EBI_AD11_PE0_Msk);
    SYS->GPH_MFPH &= (EBI_AD12_PH8_Msk | EBI_AD13_PH9_Msk | EBI_AD14_PH10_Msk | EBI_AD15_PH11_Msk);
    SYS->GPE_MFPH |= (EBI_AD8_PE14 | EBI_AD9_PE15);
    SYS->GPE_MFPL |= (EBI_AD10_PE1 | EBI_AD11_PE0);
    SYS->GPH_MFPH |= (EBI_AD12_PH8 | EBI_AD13_PH9 | EBI_AD14_PH10 | EBI_AD15_PH11);
    /* ADDR16 ~ ADDR19; */
    SYS->GPF_MFPH &= ~(EBI_ADR16_PF9_Msk | EBI_ADR17_PF8_Msk);
    SYS->GPF_MFPL &= ~(EBI_ADR18_PF7_Msk | EBI_ADR19_PF6_Msk);
    SYS->GPF_MFPH |= (EBI_ADR16_PF9 | EBI_ADR17_PF8);
    SYS->GPF_MFPL |= (EBI_ADR18_PF7 | EBI_ADR19_PF6);

    /* EBI nWR and nRD pins on PA.10 and PA.11 */
    SYS->GPA_MFPH &= ~(EBI_nWR_PA10_Msk | EBI_nRD_PA11_Msk);
    SYS->GPA_MFPH |= (EBI_nWR_PA10 | EBI_nRD_PA11);

    /* EBI nWRH and nWRL pins on PB.6 and PB.7 */
    SYS->GPB_MFPL &= ~(EBI_nWRH_PB6_Msk | EBI_nWRL_PB7_Msk);
    SYS->GPB_MFPL |= (EBI_nWRH_PB6 | EBI_nWRL_PB7);

    /* EBI nCS0 pin on PD.12 */
    SYS->GPD_MFPH &= ~EBI_nCS0_PD12_Msk;
    SYS->GPD_MFPH |= EBI_nCS0_PD12;

    /* EBI ALE pin on PA.8 */
    SYS->GPA_MFPH &= ~EBI_ALE_PA8_Msk;
    SYS->GPA_MFPH |= EBI_ALE_PA8;
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Switch HCLK clock source to HIRC */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable PLL */
    CLK->PLLCTL = CLK_PLLCTL_128MHz_HIRC;

    /* Waiting for PLL stable */
    CLK_WaitClockReady(CLK_STATUS_PLLSTB_Msk);

    /* Select HCLK clock source as PLL and HCLK source divider as 2 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_PLL, CLK_CLKDIV0_HCLK(2));

    /* Set SysTick source to HCLK/2 */
    CLK_SetSysTickClockSrc(CLK_CLKSEL0_STCLKSEL_HCLK_DIV2);

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Enable EBI module clock */
    CLK_EnableModuleClock(EBI_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & (~(UART0_RXD_PB12_Msk | UART0_TXD_PB13_Msk))) | UART0_RXD_PB12 | UART0_TXD_PB13;
}

void UART_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART module */
    SYS_ResetModule(UART0_RST);

    /* Configure UART and set UART Baudrate */
    UART_Open(DEBUG_PORT, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART for printf */
    UART_Init();

    /* Lock protected registers */
    SYS_LockReg();

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+--------------------------------------------------------+\n");
    printf("|    EBI SRAM Sample Code on Bank0 with PDMA transfer    |\n");
    printf("+--------------------------------------------------------+\n\n");

    printf("*********************************************************************\n");
    printf("* Please connect BS62LV1600EC SRAM to EBI bank0 before accessing !! *\n");
    printf("* EBI pins settings:                                                *\n");
    printf("*   - AD0 ~ AD5 on PC.0~5                                           *\n");
    printf("*   - AD6 ~ AD7 on PD.8 and PD.9                                    *\n");
    printf("*   - AD8 ~ AD9 on PE.14 and PE.15                                  *\n");
    printf("*   - AD10 ~ AD11 on PE.1 and PE.0                                  *\n");
    printf("*   - AD12 ~ AD15 on PH.8~11                                        *\n");
    printf("*   - ADR16 ~ ADR19 on PF.9~6                                       *\n");
    printf("*   - nWR on PA.10                                                  *\n");
    printf("*   - nRD on PA.11                                                  *\n");
    printf("*   - nWRL on PB.7                                                  *\n");
    printf("*   - nWRH on PB.6                                                  *\n");
    printf("*   - nCS0 on PD.12                                                 *\n");
    printf("*   - ALE on PA.8                                                   *\n");
    printf("*********************************************************************\n\n");

    /* Configure multi-function pins for EBI 16-bit application */
    Configure_EBI_16BIT_Pins();

    /* Initialize EBI bank0 to access external SRAM */
    EBI_Open(EBI_BANK0, EBI_BUSWIDTH_16BIT, EBI_TIMING_NORMAL, 0, EBI_CS_ACTIVE_LOW);

    /* Start to test EBI SRAM */
    SRAM_BS616LV4017(512 * 1024);

    /* EBI SRAM with PDMA test */
    AccessEBIWithPDMA();

    /* Disable EBI function */
    EBI_Close(EBI_BANK0);

    /* Disable EBI clock */
    CLK_DisableModuleClock(EBI_MODULE);

    printf("*** SRAM Test OK ***\n");

    while(1) {}
}

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables for PDMA                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
static uint32_t s_u32TransLen = 64;
static volatile uint32_t s_au32SrcArray[64];
static volatile uint32_t s_u32IsTestOver = 0;

/**
 * @brief       DMA IRQ
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

    if(u32Status & PDMA_INTSTS_ABTIF_Msk)        /* abort */
    {
        if(PDMA_GET_ABORT_STS(PDMA0) & PDMA_ABTSTS_ABTIF2_Msk)
            s_u32IsTestOver = 2;
        PDMA_CLR_ABORT_FLAG(PDMA0, PDMA_ABTSTS_ABTIF2_Msk);
    }
    else if(u32Status & PDMA_INTSTS_TDIF_Msk)   /* done */
    {
        if(PDMA_GET_TD_STS(PDMA0) & PDMA_TDSTS_TDIF2_Msk)
            s_u32IsTestOver = 1;
        PDMA_CLR_TD_FLAG(PDMA0, PDMA_TDSTS_TDIF2_Msk);
    }
    else
        printf("unknown interrupt !!\n");
}

void AccessEBIWithPDMA(void)
{
    uint32_t i;
    uint32_t u32Result0 = 0x5A5A, u32Result1 = 0x5A5A;

    printf("[[ Access EBI with PDMA ]]\n");

    /* Enable PDMA clock source */
    CLK_EnableModuleClock(PDMA0_MODULE);

    /* Reset PDMA module */
    SYS_ResetModule(PDMA0_RST);

    for(i = 0; i < 64; i++)
    {
        s_au32SrcArray[i] = 0x76570000 + i;
        u32Result0 += s_au32SrcArray[i];
    }

    /* Open Channel 2 */
    PDMA_Open(PDMA0, (1 << 2));

    //burst size is 4
    PDMA_SetBurstType(PDMA0, 2, PDMA_REQ_BURST, PDMA_BURST_4);

    /* transfer width is one word(32 bit) */
    PDMA_SetTransferCnt(PDMA0, 2, PDMA_WIDTH_32, s_u32TransLen);
    PDMA_SetTransferAddr(PDMA0, 2, (uint32_t)s_au32SrcArray, PDMA_SAR_INC, EBI_BANK0_BASE_ADDR, PDMA_DAR_INC);
    PDMA_SetTransferMode(PDMA0, 2, PDMA_MEM, FALSE, 0);

    PDMA_EnableInt(PDMA0, 2, PDMA_INT_TRANS_DONE);
    NVIC_EnableIRQ(PDMA0_IRQn);

    s_u32IsTestOver = 0;
    PDMA_Trigger(PDMA0, 2);
    while(s_u32IsTestOver == 0) {}
    /* Transfer internal SRAM to EBI SRAM done */

    /* Clear internal SRAM data */
    for(i = 0; i < 64; i++)
    {
        s_au32SrcArray[i] = 0x0;
    }

    /* transfer width is one word(32 bit) */
    PDMA_SetTransferCnt(PDMA0, 2, PDMA_WIDTH_32, s_u32TransLen);
    PDMA_SetTransferAddr(PDMA0, 2, EBI_BANK0_BASE_ADDR, PDMA_SAR_INC, (uint32_t)s_au32SrcArray, PDMA_DAR_INC);
    PDMA_SetTransferMode(PDMA0, 2, PDMA_MEM, FALSE, 0);

    s_u32IsTestOver = 0;
    PDMA_Trigger(PDMA0, 2);
    while(s_u32IsTestOver == 0) {}
    /* Transfer EBI SRAM to internal SRAM done */
    for(i = 0; i < 64; i++)
    {
        u32Result1 += s_au32SrcArray[i];
    }

    if(s_u32IsTestOver == 1)
    {
        if((u32Result0 == u32Result1) && (u32Result0 != 0x5A5A))
        {
            printf("        PASS (0x%X)\n\n", u32Result0);
        }
        else
        {
            printf("        FAIL - data matched (0x%X)\n\n", u32Result0);
            while(1) {}
        }
    }
    else
    {
        printf("        PDMA fail\n\n");
        while(1) {}
    }

    PDMA_Close(PDMA0);
}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
