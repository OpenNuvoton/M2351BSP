/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Use ADINT interrupt to do the EADC continuous scan conversion.
 *
 * @copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


/*---------------------------------------------------------------------------------------------------------*/
/* Define global variables and constants                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
static volatile uint32_t s_u32AdcIntFlag, s_u32COVNUMFlag = 0;

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void);
void EADC_FunctionTest(void);
void EADC0_IRQHandler(void);
void SYS_Init(void);
void UART0_Init(void);

/*---------------------------------------------------------------------------------------------------------*/
/* EADC interrupt handler                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void EADC0_IRQHandler(void)
{
    s_u32AdcIntFlag = 1;
    EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF0_Msk);      /* Clear the A/D ADINT0 interrupt flag */
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

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and and HCLK source divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

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

    /* Enable EADC module clock */
    CLK_EnableModuleClock(EADC_MODULE);

    /* EADC clock source is 64MHz, set divider to 8, EADC clock is 64/8 MHz */
    CLK_SetModuleClock(EADC_MODULE, 0, CLK_CLKDIV0_EADC(8));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & (~(UART0_RXD_PB12_Msk | UART0_TXD_PB13_Msk))) | UART0_RXD_PB12 | UART0_TXD_PB13;

    /* Configure the GPB10 - GPB13 ADC analog input pins (fast channel).  */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB10MFP_Msk)) | EADC0_CH10_PB10;
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB11MFP_Msk)) | EADC0_CH11_PB11;
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB14MFP_Msk)) | EADC0_CH14_PB14;
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB15MFP_Msk)) | EADC0_CH15_PB15;

    /* Disable the GPB10,11,14,15 digital input path to avoid the leakage current. */
    GPIO_DISABLE_DIGITAL_PATH(PB, BIT15 | BIT14 | BIT11 | BIT10);

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
/* EADC function test                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void EADC_FunctionTest()
{
    uint8_t  u8SampleCnt = 0;
    int32_t  ai32ConversionData[8] = {0};
    int32_t  i32Option;

    printf("\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("|                      ADINT trigger mode test                         |\n");
    printf("+----------------------------------------------------------------------+\n");

    printf("\nIn this test, software will get conversion result from fast channels.\n");

    while(1)
    {
        printf("\n\nSelect input mode:\n");
        printf("  [1] Single end input (channel 10, 11, 14 and 15)\n");
        printf("  [2] Differential input (input channel pair 5 and 7)\n");
        printf("  Other keys: exit continuous scan mode test\n");
        i32Option = getchar();
        if(i32Option == '1')
        {
            /* Set input mode as single-end and enable the A/D converter */
            EADC_Open(EADC, EADC_CTL_DIFFEN_SINGLE_END);

            /* Configure the sample 4 module for analog input channel 10 and enable ADINT0 trigger source */
            EADC_ConfigSampleModule(EADC, 4, EADC_ADINT0_TRIGGER, 10);
            /* Configure the sample 5 module for analog input channel 11 and enable ADINT0 trigger source */
            EADC_ConfigSampleModule(EADC, 5, EADC_ADINT0_TRIGGER, 11);
            /* Configure the sample 6 module for analog input channel 14 and enable ADINT0 trigger source */
            EADC_ConfigSampleModule(EADC, 6, EADC_ADINT0_TRIGGER, 14);
            /* Configure the sample 7 module for analog input channel 15 and enable ADINT0 trigger source */
            EADC_ConfigSampleModule(EADC, 7, EADC_ADINT0_TRIGGER, 15);

            /* Clear the A/D ADINT0 interrupt flag for safe */
            EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF0_Msk);

            /* Enable the sample module 7 interrupt */
            EADC_ENABLE_INT(EADC, BIT0);//Enable sample module  A/D ADINT0 interrupt.
            EADC_ENABLE_SAMPLE_MODULE_INT(EADC, 0, BIT7);//Enable sample module 7 interrupt.
            NVIC_EnableIRQ(EADC0_IRQn);

            /* Reset the ADC indicator and trigger sample module 7 to start A/D conversion */
            s_u32AdcIntFlag = 0;
            s_u32COVNUMFlag = 0;
            EADC_START_CONV(EADC, BIT7);

            __WFI();

            /* Disable the sample module 7 interrupt */
            EADC_DISABLE_SAMPLE_MODULE_INT(EADC, 0, BIT7);

            /* Wait conversion done */
            while(EADC_GET_DATA_VALID_FLAG(EADC, (BIT7 | BIT6 | BIT5 | BIT4)) != (BIT7 | BIT6 | BIT5 | BIT4));

            /* Get the conversion result of the sample module */
            for(u8SampleCnt = 0; u8SampleCnt < 4; u8SampleCnt++)
                ai32ConversionData[u8SampleCnt] = EADC_GET_CONV_DATA(EADC, u8SampleCnt + 4);

            printf("Conversion result of channel %d: 0x%X (%d)\n", 10, ai32ConversionData[0], ai32ConversionData[0]);
            printf("Conversion result of channel %d: 0x%X (%d)\n", 11, ai32ConversionData[1], ai32ConversionData[1]);
            printf("Conversion result of channel %d: 0x%X (%d)\n", 14, ai32ConversionData[2], ai32ConversionData[2]);
            printf("Conversion result of channel %d: 0x%X (%d)\n", 15, ai32ConversionData[3], ai32ConversionData[3]);

        }
        else if(i32Option == '2')
        {
            /* In differential analog input mode, only the even number of the two corresponding channels needs to be enabled in CHSEL (EADC_SCTLn[3:0]).
               The conversion result will be placed to the corresponding data register of the enabled channel. */

            /* Set input mode as differential and enable the A/D converter */
            EADC_Open(EADC, EADC_CTL_DIFFEN_DIFFERENTIAL);

            /* Configure the sample module 4 for analog input channel 10 and enable ADINT0 trigger source */
            EADC_ConfigSampleModule(EADC, 4, EADC_ADINT0_TRIGGER, 10);
            /* Configure the sample module 6 for analog input channel 14 and enable ADINT0 trigger source */
            EADC_ConfigSampleModule(EADC, 6, EADC_ADINT0_TRIGGER, 14);

            /* Clear the A/D ADINT0 interrupt flag for safe */
            EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF0_Msk);

            /* Enable the sample module 6 interrupt */
            EADC_ENABLE_INT(EADC, BIT0);//Enable sample module A/D ADINT0 interrupt.
            EADC_ENABLE_SAMPLE_MODULE_INT(EADC, 0, BIT6);//Enable sample module 6 interrupt.
            NVIC_EnableIRQ(EADC0_IRQn);

            /* Reset the ADC indicator and trigger sample module 6 to start A/D conversion */
            s_u32AdcIntFlag = 0;
            s_u32COVNUMFlag = 0;
            EADC_START_CONV(EADC, BIT6);

            __WFI();

            /* Disable the sample module 6 interrupt */
            EADC_DISABLE_SAMPLE_MODULE_INT(EADC, 0, BIT6);

            /* Wait conversion done */
            while(EADC_GET_DATA_VALID_FLAG(EADC, (BIT6 | BIT4)) != (BIT6 | BIT4));

            /* Get the conversion result of the sample module */
            for(u8SampleCnt = 0; u8SampleCnt < 4; u8SampleCnt++)
                ai32ConversionData[u8SampleCnt] = EADC_GET_CONV_DATA(EADC, u8SampleCnt + 4);

            printf("Conversion result of channel pair %d: 0x%X (%d)\n", 5, ai32ConversionData[0], ai32ConversionData[0]);
            printf("Conversion result of channel pair %d: 0x%X (%d)\n", 7, ai32ConversionData[2], ai32ConversionData[2]);

        }
        else
            return ;

        /* Reset the sample module 4, 5, 6, 7 for analog input channel and disable ADINT0 trigger source */
        EADC_ConfigSampleModule(EADC, 4, EADC_SOFTWARE_TRIGGER, 0);
        EADC_ConfigSampleModule(EADC, 5, EADC_SOFTWARE_TRIGGER, 0);
        EADC_ConfigSampleModule(EADC, 6, EADC_SOFTWARE_TRIGGER, 0);
        EADC_ConfigSampleModule(EADC, 7, EADC_SOFTWARE_TRIGGER, 0);

        /* Clear the conversion result of the sample module */
        for(u8SampleCnt = 0; u8SampleCnt < 4; u8SampleCnt++)
            ai32ConversionData[u8SampleCnt] = 0;
    }
}


/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\nSystem clock rate: %d Hz", SystemCoreClock);

    /* EADC function test */
    EADC_FunctionTest();

    /* Reset EADC module */
    SYS_ResetModule(EADC_RST);

    /* Disable EADC IP clock */
    CLK_DisableModuleClock(EADC_MODULE);

    /* Disable External Interrupt */
    NVIC_DisableIRQ(EADC0_IRQn);

    printf("Exit EADC sample code\n");

    while(1);

}


/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
