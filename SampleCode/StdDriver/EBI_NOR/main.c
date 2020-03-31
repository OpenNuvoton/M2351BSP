/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Configure EBI interface to access MX29LV320T (NOR Flash) on EBI interface.
 *
 * @copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


/*---------------------------------------------------------------------------------------------------------*/
/* Global Interface Variables Declarations                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
extern void NOR_MX29LV320T_RESET(uint32_t u32Bank);
extern int32_t NOR_MX29LV320T_CheckStatus(uint32_t u32DstAddr, uint16_t u16Data, uint32_t u32TimeoutMs);
extern uint16_t NOR_MX29LV320T_READ(uint32_t u32Bank, uint32_t u32DstAddr);
extern int32_t NOR_MX29LV320T_WRITE(uint32_t u32Bank, uint32_t u32DstAddr, uint16_t u16Data);
extern void NOR_MX29LV320T_GET_ID(uint32_t u32Bank, uint16_t *pu16IDTable);
extern int32_t NOR_MX29LV320T_EraseChip(uint32_t u32Bank, uint32_t u32IsCheckBlank);

void Configure_EBI_16BIT_Pins(void);
void SYS_Init(void);
void UART_Init(void);


void Configure_EBI_16BIT_Pins(void)
{
    /* AD0 ~ AD7*/
    SYS->GPC_MFPL &= ~(EBI_AD0_PC0_Msk | EBI_AD1_PC1_Msk | EBI_AD2_PC2_Msk | EBI_AD3_PC3_Msk |
                       EBI_AD4_PC4_Msk | EBI_AD5_PC5_Msk);
    SYS->GPD_MFPH &= ~(EBI_AD6_PD8_Msk | EBI_AD7_PD9_Msk);
    SYS->GPC_MFPL |= (EBI_AD0_PC0 | EBI_AD1_PC1 | EBI_AD2_PC2 | EBI_AD3_PC3 | EBI_AD4_PC4 | EBI_AD5_PC5);
    SYS->GPD_MFPH |= (EBI_AD6_PD8 | EBI_AD7_PD9);
    /* AD8 ~ AD15 */
    SYS->GPE_MFPH &= ~(EBI_AD8_PE14_Msk | EBI_AD9_PE15_Msk);
    SYS->GPE_MFPL &= ~(EBI_AD10_PE1_Msk | EBI_AD11_PE0_Msk);
    SYS->GPH_MFPH &= ~(EBI_AD12_PH8_Msk | EBI_AD13_PH9_Msk | EBI_AD14_PH10_Msk | EBI_AD15_PH11_Msk);
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

    /* EBI nCS1 pin on PD.11 */
    SYS->GPD_MFPH &= ~EBI_nCS1_PD11_Msk;
    SYS->GPD_MFPH |= EBI_nCS1_PD11;

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
    uint32_t u32Addr, u32MaxEBISize;
    uint16_t u16WData, u16RData;
    uint16_t u16IDTable[2];

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART for printf */
    UART_Init();

    /* Lock protected registers */
    SYS_LockReg();

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+-----------------------------------------+\n");
    printf("|    EBI Nor Flash Sample Code on Bank1   |\n");
    printf("+-----------------------------------------+\n\n");

    printf("************************************************************************\n");
    printf("* Please connect MX29LV320T nor flash to EBI bank1 before accessing !! *\n");
    printf("* EBI pins settings:                                                   *\n");
    printf("*   - AD0 ~ AD5 on PC.0~5                                              *\n");
    printf("*   - AD6 ~ AD7 on PD.8 and PD.9                                       *\n");
    printf("*   - AD8 ~ AD9 on PE.14 and PE.15                                     *\n");
    printf("*   - AD10 ~ AD11 on PE.1 and PE.0                                     *\n");
    printf("*   - AD12 ~ AD15 on PH.8~11                                           *\n");
    printf("*   - ADR16 ~ ADR19 on PF.9~6                                          *\n");
    printf("*   - nWR on PA.10                                                     *\n");
    printf("*   - nRD on PA.11                                                     *\n");
    printf("*   - nWRL on PB.7                                                     *\n");
    printf("*   - nWRH on PB.6                                                     *\n");
    printf("*   - nCS1 on PD.11                                                    *\n");
    printf("*   - ALE on PA.8                                                      *\n");
    printf("************************************************************************\n\n");

    /* Configure multi-function pins for EBI 16-bit application */
    Configure_EBI_16BIT_Pins();

    /* Initialize EBI bank1 to access external nor */
    EBI_Open(EBI_BANK1, EBI_BUSWIDTH_16BIT, EBI_TIMING_NORMAL, 0, EBI_CS_ACTIVE_LOW);


    /* Step 1, check ID */
    NOR_MX29LV320T_GET_ID(EBI_BANK1, (uint16_t *)u16IDTable);
    printf(">> Manufacture ID: 0x%X, Device ID: 0x%X .... ", u16IDTable[0], u16IDTable[1]);
    if((u16IDTable[0] != 0xC2) || (u16IDTable[1] != 0x22A8))
    {
        printf("FAIL !!!\n\n");
        while(1) {}
    }
    else
    {
        printf("PASS !!!\n\n");
    }


    /* Step 2, erase chip */
    if(NOR_MX29LV320T_EraseChip(EBI_BANK1, TRUE) < 0)
        while(1) {}


    /* Step 3, program flash and compare data */
    printf(">> Run program flash test ......\n");
    u32MaxEBISize = EBI_MAX_SIZE;
    for(u32Addr = 0; u32Addr < u32MaxEBISize; u32Addr += 2)
    {
        u16WData = (0x7657 + u32Addr / 2) & 0xFFFF;
        if(NOR_MX29LV320T_WRITE(EBI_BANK1, u32Addr, u16WData) < 0)
        {
            printf("Program [0x%08X]: [0x%08X] FAIL !!!\n\n", (uint32_t)(EBI_BANK0_BASE_ADDR + (0x100000 * EBI_BANK1)) + u32Addr, u16WData);
            while(1) {}
        }
        else
        {
            /* Show UART message ...... */
            if((u32Addr % 256) == 0)
                printf("Program [0x%08X]:[0x%08X] !!!       \r", (uint32_t)(EBI_BANK0_BASE_ADDR + (0x100000 * EBI_BANK1)) + u32Addr, u16WData);
        }
    }

    for(u32Addr = 0; u32Addr < u32MaxEBISize; u32Addr += 2)
    {
        u16WData = (0x7657 + (u32Addr / 2)) & 0xFFFF;
        u16RData = NOR_MX29LV320T_READ(EBI_BANK1, u32Addr);
        if(u16WData != u16RData)
        {
            printf("Compare [0x%08X] FAIL !!! (W:0x%08X, R:0x%08X)\n\n", (uint32_t)(EBI_BANK0_BASE_ADDR + (0x100000 * EBI_BANK1)) + u32Addr, u16WData, u16RData);
            while(1) {}
        }
        else
        {
            /* Show UART message ...... */
            if((u32Addr % 256) == 0)
                printf("Read [0x%08X]: [0x%08X] !!!         \r", (uint32_t)(EBI_BANK0_BASE_ADDR + (0x100000 * EBI_BANK1) + u32Addr), u16RData);
        }
    }
    printf(">> Program flash OK !!!                             \n\n");

    /* Disable EBI function */
    EBI_Close(EBI_BANK1);

    /* Disable EBI clock */
    CLK->AHBCLK &= ~CLK_AHBCLK_EBICKEN_Msk;

    while(1) {}
}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
