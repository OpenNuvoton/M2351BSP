/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate how to trigger multiple sample modules and got conversion results in order of priority.
 *
 * @copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Define global variables and constants                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
static volatile uint32_t s_u32EadcInt0Flag, s_u32EadcInt1Flag, s_u32EadcInt2Flag, s_u32EadcInt3Flag;

static uint32_t s_au32IntModule[4];    /* save the sample module number for ADINT0~3 */
static volatile uint32_t s_au32IntSequence[4];  /* save the interrupt sequence for ADINT0~3 */
static volatile uint32_t s_u32IntSequenceIndex;

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void);
void EADC_FunctionTest(void);
void SYS_Init(void);
void UART0_Init(void);
void EADC0_IRQHandler(void);
void EADC1_IRQHandler(void);
void EADC2_IRQHandler(void);
void EADC3_IRQHandler(void);

void SYS_Init(void)
{
    /* Set PF multi-function pins for XT1_OUT(PF.2) and XT1_IN(PF.3) */
    SYS->GPF_MFPL = (SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF2MFP_Msk)) | XT1_OUT_PF2;
    SYS->GPF_MFPL = (SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF3MFP_Msk)) | XT1_IN_PF3;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC clock (Internal RC 22.1184MHz) */
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

    /* Configure the EADC analog input pins.  */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~SYS_GPB_MFPL_PB0MFP_Msk) | SYS_GPB_MFPL_PB0MFP_EADC0_CH0;
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~SYS_GPB_MFPL_PB6MFP_Msk) | SYS_GPB_MFPL_PB6MFP_EADC0_CH6;
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~SYS_GPB_MFPL_PB7MFP_Msk) | SYS_GPB_MFPL_PB7MFP_EADC0_CH7;
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~SYS_GPB_MFPH_PB8MFP_Msk) | SYS_GPB_MFPH_PB8MFP_EADC0_CH8;
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~SYS_GPB_MFPH_PB9MFP_Msk) | SYS_GPB_MFPH_PB9MFP_EADC0_CH9;

    /* Disable the GPB0, GPB6~9 digital input path to avoid the leakage current. */
    GPIO_DISABLE_DIGITAL_PATH(PB, BIT9 | BIT8 | BIT7 | BIT6 | BIT0);
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
    int32_t  i32Option;
    int32_t  i32ConversionData, i;

    printf("\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("|                      EADC Pending Priority sample code               |\n");
    printf("+----------------------------------------------------------------------+\n");

    /* Set the EADC and enable the A/D converter */
    EADC_Open(EADC, EADC_CTL_DIFFEN_SINGLE_END);

    while(1)
    {
        printf("Select test items:\n");
        printf("  [1] Assign interrupt ADINT0~3 to Sample Module 0~3\n");
        printf("  [2] Assign interrupt ADINT3~0 to Sample Module 0~3\n");
        printf("  Other keys: exit EADC test\n");
        i32Option = getchar();

        if(i32Option == '1')
        {
            s_au32IntModule[0] = 0;  /* Assign ADINT0 to Sample module 0 */
            s_au32IntModule[1] = 1;  /* Assign ADINT1 to Sample module 1 */
            s_au32IntModule[2] = 2;  /* Assign ADINT2 to Sample module 2 */
            s_au32IntModule[3] = 3;  /* Assign ADINT3 to Sample module 3 */
        }
        else if(i32Option == '2')
        {
            s_au32IntModule[0] = 3;  /* Assign ADINT0 to Sample module 3 */
            s_au32IntModule[1] = 2;  /* Assign ADINT1 to Sample module 2 */
            s_au32IntModule[2] = 1;  /* Assign ADINT2 to Sample module 1 */
            s_au32IntModule[3] = 0;  /* Assign ADINT3 to Sample module 0 */
        }
        else
            break;  /* exit while loop */

        /* Configure the sample module for analog input channel and software trigger source. */
        EADC_ConfigSampleModule(EADC, 0, EADC_SOFTWARE_TRIGGER, 6);
        EADC_ConfigSampleModule(EADC, 1, EADC_SOFTWARE_TRIGGER, 7);
        EADC_ConfigSampleModule(EADC, 2, EADC_SOFTWARE_TRIGGER, 8);
        EADC_ConfigSampleModule(EADC, 3, EADC_SOFTWARE_TRIGGER, 9);

        /* Clear the A/D ADINTx interrupt flag for safe */
        EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF0_Msk | EADC_STATUS2_ADIF1_Msk | EADC_STATUS2_ADIF2_Msk | EADC_STATUS2_ADIF3_Msk);

        /* Enable the sample module interrupt.  */
        EADC_ENABLE_INT(EADC, BIT0 | BIT1 | BIT2 | BIT3);

        EADC_ENABLE_SAMPLE_MODULE_INT(EADC, 0, BIT0 << s_au32IntModule[0]);
        EADC_ENABLE_SAMPLE_MODULE_INT(EADC, 1, BIT0 << s_au32IntModule[1]);
        EADC_ENABLE_SAMPLE_MODULE_INT(EADC, 2, BIT0 << s_au32IntModule[2]);
        EADC_ENABLE_SAMPLE_MODULE_INT(EADC, 3, BIT0 << s_au32IntModule[3]);

        NVIC_EnableIRQ(EADC0_IRQn);
        NVIC_EnableIRQ(EADC1_IRQn);
        NVIC_EnableIRQ(EADC2_IRQn);
        NVIC_EnableIRQ(EADC3_IRQn);

        /* Reset the EADC interrupt indicator and trigger sample module to start A/D conversion */
        s_u32IntSequenceIndex = 0;
        s_u32EadcInt0Flag = 0;
        s_u32EadcInt1Flag = 0;
        s_u32EadcInt2Flag = 0;
        s_u32EadcInt3Flag = 0;

        /* Start EADC conversion for sample module 0 ~ 3 at the same time */
        EADC_START_CONV(EADC, BIT0 | BIT1 | BIT2 | BIT3);

        /* Wait all EADC interrupt (g_u32EadcIntxFlag will be set at EADC_INTx_IRQHandler() function) */
        while((s_u32EadcInt0Flag == 0) || (s_u32EadcInt1Flag == 0) ||
                (s_u32EadcInt2Flag == 0) || (s_u32EadcInt3Flag == 0));

        /* Get the conversion result of the sample module */
        printf("The ADINTx interrupt sequence is:\n");

        for(i = 0; i < 4; i++)
        {
            i32ConversionData = EADC_GET_CONV_DATA(EADC, s_au32IntModule[i]);
            printf("ADINT%d: #%d, Module %d, Conversion result: 0x%X (%d)\n", i, s_au32IntSequence[i], s_au32IntModule[i], i32ConversionData, i32ConversionData);
        }

        printf("\n");

        /* Disable the ADINTx interrupt */
        EADC_DISABLE_INT(EADC, BIT0 | BIT1 | BIT2 | BIT3);

        EADC_DISABLE_SAMPLE_MODULE_INT(EADC, 0, BIT0 << s_au32IntModule[0]);
        EADC_DISABLE_SAMPLE_MODULE_INT(EADC, 1, BIT0 << s_au32IntModule[1]);
        EADC_DISABLE_SAMPLE_MODULE_INT(EADC, 2, BIT0 << s_au32IntModule[2]);
        EADC_DISABLE_SAMPLE_MODULE_INT(EADC, 3, BIT0 << s_au32IntModule[3]);

        NVIC_DisableIRQ(EADC0_IRQn);
        NVIC_DisableIRQ(EADC1_IRQn);
        NVIC_DisableIRQ(EADC2_IRQn);
        NVIC_DisableIRQ(EADC3_IRQn);
    }   /* End of while(1) */

    /* Disable the A/D converter */
    EADC_Close(EADC);
}



/*---------------------------------------------------------------------------------------------------------*/
/* EADC interrupt handler                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void EADC0_IRQHandler(void)
{
    s_u32EadcInt0Flag = 1;
    /* Clear the A/D ADINT0 interrupt flag */
    EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF0_Msk);

    /* Save the interrupt sequence about ADINT0 */
    s_au32IntSequence[0] = s_u32IntSequenceIndex++;
}

void EADC1_IRQHandler(void)
{
    s_u32EadcInt1Flag = 1;
    /* Clear the A/D ADINT1 interrupt flag */
    EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF1_Msk);

    /* Save the interrupt sequence about ADINT1 */
    s_au32IntSequence[1] = s_u32IntSequenceIndex++;
}

void EADC2_IRQHandler(void)
{
    s_u32EadcInt2Flag = 1;
    /* Clear the A/D ADINT2 interrupt flag */
    EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF2_Msk);

    /* Save the interrupt sequence about ADINT2 */
    s_au32IntSequence[2] = s_u32IntSequenceIndex++;
}

void EADC3_IRQHandler(void)
{
    s_u32EadcInt3Flag = 1;
    /* Clear the A/D ADINT3 interrupt flag */
    EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF3_Msk);

    /* Save the interrupt sequence about ADINT3 */
    s_au32IntSequence[3] = s_u32IntSequenceIndex++;
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

    /* Disable EADC IP clock */
    CLK_DisableModuleClock(EADC_MODULE);

    /* Reset EADC module */
    SYS_ResetModule(EADC_RST);

    /* Disable External Interrupt */
    NVIC_DisableIRQ(EADC0_IRQn);
    NVIC_DisableIRQ(EADC1_IRQn);
    NVIC_DisableIRQ(EADC2_IRQn);
    NVIC_DisableIRQ(EADC3_IRQn);

    printf("Exit EADC sample code\n");

    while(1);

}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
