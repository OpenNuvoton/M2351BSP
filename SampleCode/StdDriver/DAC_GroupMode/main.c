/****************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    Demonstrate DAC0 and DAC1 work in group mode
 *
 * @copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

static const uint16_t s_au16Sine[] = {2047, 2251, 2453, 2651, 2844, 3028, 3202, 3365, 3515, 3650, 3769, 3871, 3954,
                         4019, 4064, 4088, 4095, 4076, 4040, 3984, 3908, 3813, 3701, 3573, 3429, 3272,
                         3102, 2921, 2732, 2536, 2335, 2132, 1927, 1724, 1523, 1328, 1141,  962,  794,
                         639,  497,  371,  262,  171,   99,   45,   12,    0,    7,   35,   84,  151,
                         238,  343,  465,  602,  754,  919, 1095, 1281, 1475, 1674, 1876
                        };

static const uint32_t s_u32ArraySize = sizeof(s_au16Sine) / sizeof(uint16_t);
static uint32_t s_u32Index = 0;
static uint32_t s_u32Dac0Done = 0, s_u32Dac1Done = 0;

void DAC_IRQHandler(void);
void SYS_Init(void);


void DAC_IRQHandler(void)
{
    if(DAC_GET_INT_FLAG(DAC0, 0))
    {
        /* Clear the DAC conversion complete finish flag */
        DAC_CLR_INT_FLAG(DAC0, 0);
        DAC_WRITE_DATA(DAC0, 0, s_au16Sine[s_u32Index]);
        s_u32Dac0Done = 1;

    }
    if(DAC_GET_INT_FLAG(DAC1, 0))
    {

        /* Clear the DAC conversion complete finish flag */
        DAC_CLR_INT_FLAG(DAC1, 0);
        DAC_WRITE_DATA(DAC1, 0, s_au16Sine[s_u32Index >= s_u32ArraySize / 2 ? s_u32Index - s_u32ArraySize / 2 : s_u32Index + s_u32ArraySize / 2]);
        s_u32Dac1Done = 1;

        if(++s_u32Index == s_u32ArraySize)
            s_u32Index = 0;
    }

    if(s_u32Dac0Done == 1 && s_u32Dac1Done == 1)
    {
        DAC_START_CONV(DAC0);
        s_u32Dac0Done = s_u32Dac1Done = 0;
    }

    return;
}


void SYS_Init(void)
{

    /* Set PF multi-function pins for XT1_OUT(PF.2) and XT1_IN(PF.3) */
    SYS->GPF_MFPL = (SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF2MFP_Msk)) | XT1_OUT_PF2;
    SYS->GPF_MFPL = (SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF3MFP_Msk)) | XT1_IN_PF3;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable HIRC clock (Internal RC 22.1184 MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Enable HXT clock (external XTAL 12MHz) */
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

    /* Enable DAC module clock */
    CLK_EnableModuleClock(DAC_MODULE);

    /*
      Note:
            Because VCOM pins(UART0_RXD_PB12 and UART0_TXD_PB13) of NuMaker-PFM-M2351 board are conflict to DAC0_OUT_PB12 and DAC1_OUT_PB13 pins.
            So, this DAC sample code takes UART0_RXD_PA0 and UART0_TXD_PA1 as debug port.
    */
    /* Set multi-function pins for UART0 RXD and TXD */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & (~(UART0_RXD_PA0_Msk | UART0_TXD_PA1_Msk))) | UART0_RXD_PA0 | UART0_TXD_PA1;

    /* Set PB multi-function pins for DAC voltage output */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~SYS_GPB_MFPH_PB12MFP_Msk) | DAC0_OUT_PB12;
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~SYS_GPB_MFPH_PB13MFP_Msk) | DAC1_OUT_PB13;

    /* Disable digital input path of analog pin DAC0_OUT and DAC1_OUT to prevent leakage */
    GPIO_DISABLE_DIGITAL_PATH(PB, (1ul << 12) | (1ul << 13));

    /* Lock protected registers */
    SYS_LockReg();


}

int32_t main(void)
{

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);

    printf("\n");
    printf("+------------------------------------------------------------------------+\n");
    printf("|                          DAC Driver Sample Code                        |\n");
    printf("+------------------------------------------------------------------------+\n");
    printf("DAC0 and DAC1 is configured in group mode and update simultaneously\n");
    /* Single Mode test */
    /* Set the software trigger, enable DAC even trigger mode and enable D/A converter */
    DAC_Open(DAC0, 0, DAC_SOFTWARE_TRIGGER);
    DAC_Open(DAC1, 0, DAC_SOFTWARE_TRIGGER);

    /* Enable DAC to work in group mode, once group mode enabled, DAC1 is configured by DAC0 registers */
    DAC_ENABLE_GROUP_MODE(DAC0);

    /* The DAC conversion settling time is 1us */
    DAC_SetDelayTime(DAC0, 1);

    /* Set DAC 12-bit holding data */
    DAC_WRITE_DATA(DAC0, 0, s_au16Sine[0]);
    DAC_WRITE_DATA(DAC1, 0, s_au16Sine[s_u32ArraySize / 2]);

    /* Clear the DAC conversion complete finish flag for safe */
    DAC_CLR_INT_FLAG(DAC0, 0);

    /* Enable the DAC interrupt */
    DAC_ENABLE_INT(DAC0, 0);
    DAC_ENABLE_INT(DAC1, 0);
    NVIC_EnableIRQ(DAC_IRQn);

    /* Start A/D conversion */
    DAC_START_CONV(DAC0);

    while(1);

}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
