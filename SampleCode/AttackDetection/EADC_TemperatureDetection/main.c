/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Show how to measure temperature by EADC.
 *
 * @copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


#define VREF_VOLTAGE (3.3)

/*---------------------------------------------------------------------------------------------------------*/
/* Define global variables and constants                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
static volatile uint32_t s_u32AdcIntFlag;

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void);
uint32_t GetTemperatureCodeFromADC(void);
double GetTemperature(void);
void SYS_Init(void);
void UART0_Init(void);
void EADC0_IRQHandler(void);

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and HCLK source divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable PLL */
    CLK->PLLCTL = CLK_PLLCTL_128MHz_HIRC;

    /* Waiting for PLL stable */
    CLK_WaitClockReady(CLK_STATUS_PLLSTB_Msk);

    /* Select HCLK clock source as PLL and HCLK source divider as 2 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_PLL, CLK_CLKDIV0_HCLK(2));

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART module clock source as HIRC and UART module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Enable EADC module clock */
    CLK_EnableModuleClock(EADC_MODULE);

    /* EADC clock source is 64MHz, set divider to 8, EADC clock is 64/8 MHz */
    CLK_SetModuleClock(EADC_MODULE, 0, CLK_CLKDIV0_EADC(8));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & (~(UART0_RXD_PB12_Msk | UART0_TXD_PB13_Msk))) | UART0_RXD_PB12 | UART0_TXD_PB13;

    /* Enable temperature sensor */
    SYS->IVSCTL |= SYS_IVSCTL_VTEMPEN_Msk;
}

void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART0 module */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/* Get temperature                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
double GetTemperature(void)
{
    double dmVoffset = 708.58154;

    double dAvgTemperatureCode = 0;
    double dmVT = 0;
    double dT;

    dAvgTemperatureCode = 0;
    dmVT = 0;

    /* Get ADC code of temperature sensor */
    dAvgTemperatureCode = GetTemperatureCodeFromADC();

    /* ADC code to voltage conversion formula: */
    dmVT = (dAvgTemperatureCode) * (VREF_VOLTAGE) * 1000 / 4096;

    /* Get temperature with temperature sensor */
    /* Temperature sensor formula: Tx = (VT - Voffset)/(-1.8118) */
    dT = (dmVT - dmVoffset) / (-1.8118);

    return (dT);
}

/*---------------------------------------------------------------------------------------------------------*/
/* EADC function test                                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
uint32_t GetTemperatureCodeFromADC(void)
{
    /* Set input mode as single-end and enable the A/D converter */
    EADC_Open(EADC, EADC_CTL_DIFFEN_SINGLE_END);

    /* Set sample module 17 external sampling time to 0xFF */
    EADC_SetExtendSampleTime(EADC, 17, 0xFF);

    /* Clear the A/D ADINT0 interrupt flag for safe */
    EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF0_Msk);

    /* Enable the sample module 17 interrupt.  */
    EADC_ENABLE_INT(EADC, BIT0);//Enable sample module A/D ADINT0 interrupt.
    EADC_ENABLE_SAMPLE_MODULE_INT(EADC, 0, BIT17);//Enable sample module 17 interrupt.
    NVIC_EnableIRQ(EADC0_IRQn);

    /* Reset the ADC interrupt indicator and trigger sample module 17 to start A/D conversion */
    s_u32AdcIntFlag = 0;
    EADC_START_CONV(EADC, BIT17);

    /* Wait EADC conversion done */
    while(s_u32AdcIntFlag == 0);

    /* Disable the ADINT0 interrupt */
    EADC_DISABLE_INT(EADC, BIT0);

    /* Return the conversion result of the sample module 17 */
    return EADC_GET_CONV_DATA(EADC, 17);
}


/*---------------------------------------------------------------------------------------------------------*/
/* EADC IRQ Handler                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
void EADC0_IRQHandler(void)
{
    s_u32AdcIntFlag = 1;
    /* Clear the A/D ADINT0 interrupt flag */
    EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF0_Msk);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    double dTemperature;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);
    printf("+--------------------------------------------+\n");
    printf("|   EADC Tamperature Detection Sample Code   |\n");
    printf("+--------------------------------------------+\n\n");

    /* Measure temperature */
    dTemperature = GetTemperature();
    if(dTemperature > 50)
        printf("Abnormal temperature: %dC\n", (int32_t)dTemperature);
    else
        printf("Chip temperature: %dC\n", (int32_t)dTemperature);

    /* Reset EADC module */
    SYS_ResetModule(EADC_RST);

    /* Disable EADC IP clock */
    CLK_DisableModuleClock(EADC_MODULE);

    /* Disable External Interrupt */
    NVIC_DisableIRQ(EADC0_IRQn);

    while(1);

}

/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/
