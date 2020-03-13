/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    Demonstrate how ACMP compare VBG output with ACMP1_P1 value
 *
 * @copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

void ACMP01_IRQHandler(void)
{
    static uint32_t u32Cnt = 0;

    /* Clear ACMP 1 interrupt flag */
    ACMP_CLR_INT_FLAG(ACMP01, 1);
    /* Check Comparator 0 Output Status */
    if(ACMP_GET_OUTPUT(ACMP01, 1))
        printf("ACMP1_P voltage > Band-gap voltage (%d)\n", u32Cnt);
    else
        printf("ACMP1_P voltage <= Band-gap voltage (%d)\n", u32Cnt);

    u32Cnt++;
}


void SYS_Init(void)
{
    /* Enable external 12MHz XTAL */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Select HXT as the clock source of UART */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HXT, CLK_CLKDIV0_UART0(1));

    /* Enable UART peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);
    /* Enable ACMP01 peripheral clock */
    CLK_EnableModuleClock(ACMP01_MODULE);

    /* Enable ACMP01 peripheral clock */
    CLK_EnableModuleClock(DAC_MODULE);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and CyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /* Set PB4 multi-function pin for ACMP1 positive input pin and PB6 multi-function pin for ACMP1 output pin*/
    SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB4MFP_Msk | SYS_GPB_MFPL_PB6MFP_Msk);
    SYS->GPB_MFPL |= SYS_GPB_MFPL_PB4MFP_ACMP1_P1 | SYS_GPB_MFPL_PB6MFP_ACMP1_O;

    /* Set multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & (~(UART0_RXD_PB12_Msk | UART0_TXD_PB13_Msk))) | UART0_RXD_PB12 | UART0_TXD_PB13;

    /* Disable digital input path of analog pin ACMP1_P1 to prevent leakage */
    GPIO_DISABLE_DIGITAL_PATH(PB, (1ul << 4));

}

/*
 * When the voltage of the positive input is greater than the voltage of the
 * negative input, the analog comparator outputs logical one; otherwise, it outputs
 * logical zero. This sample code will show the expression of the comparator's
 * number when detecting a transition of analog comparator's output.
 */

int32_t main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();
    /* Lock protected registers */
    SYS_LockReg();

    /* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Open(UART0, 115200);

    printf("\nThis sample code demonstrates ACMP1 function. Using ACMP1_P1 (PB4) as ACMP1\n");
    printf("positive input and using internal band-gap voltage as the negative input.\n");
    printf("The compare result reflects on ACMP1_O (PB6).\n");

    printf("Press any key to start ...");
    getchar();
    printf("\n");

    /* Configure ACMP1. Enable ACMP1 and select DAC0 output as the source of ACMP negative input. */
    ACMP_Open(ACMP01, 1, ACMP_CTL_NEGSEL_DAC, ACMP_CTL_HYSTERESIS_DISABLE);
    /* Select P1 as ACMP positive input channel */
    ACMP_SELECT_P(ACMP01, 1, ACMP_CTL_POSSEL_P1);
    /* Enable interrupt */
    ACMP_ENABLE_INT(ACMP01, 1);

    /* Enable ACMP01 interrupt */
    NVIC_EnableIRQ(ACMP01_IRQn);

    while(1);

}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/


