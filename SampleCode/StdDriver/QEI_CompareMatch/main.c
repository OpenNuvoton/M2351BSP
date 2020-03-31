/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Show the usage of QEI compare function.
 *
 * @copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define PLL_CLOCK       64000000


#define QEI0A   PC0
#define QEI0B   PC1


/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void QEI0_IRQHandler(void);
void SYS_Init(void);
int32_t main(void);
void UART0_Init(void);

/*---------------------------------------------------------------------------------------------------------*/
/*  QEI0 IRQ Handler                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void QEI0_IRQHandler(void)
{

    if(QEI_GET_INT_FLAG(QEI0, QEI_STATUS_CMPF_Msk))     /* Compare-match flag */
    {
        printf("Compare-match INT!\n\n");
        QEI_CLR_INT_FLAG(QEI0, QEI_STATUS_CMPF_Msk);
    }

    if(QEI_GET_INT_FLAG(QEI0, QEI_STATUS_OVUNF_Msk))    /* Counter Overflow or underflow flag */
    {
        printf("Overflow INT!\n\n");
        QEI_CLR_INT_FLAG(QEI0, QEI_STATUS_OVUNF_Msk);
    }
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PF multi-function pins for XT1_OUT(PF.2) and XT1_IN(PF.3) */
    SYS->GPF_MFPL &= (~(SYS_GPF_MFPL_PF2MFP_Msk | SYS_GPF_MFPL_PF3MFP_Msk));
    SYS->GPF_MFPL |= (SYS_GPF_MFPL_PF2MFP_XT1_OUT | SYS_GPF_MFPL_PF3MFP_XT1_IN);

    /* Enable Internal RC 22.1184MHz clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable external XTAL 12MHz clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Waiting for external XTAL clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Enable PLL */
    CLK->PLLCTL = CLK_PLLCTL_128MHz_HIRC;

    /* Waiting for PLL stable */
    CLK_WaitClockReady(CLK_STATUS_PLLSTB_Msk);

    /* Select HCLK clock source as PLL and HCLK source divider as 2 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_PLL, CLK_CLKDIV0_HCLK(2));

    /* Set PCLK0/PCLK1 to HCLK/2 */
    CLK->CLKSEL0 |= 0xc0;

    /* Enable UART0 and QEI0 module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    CLK_EnableModuleClock(QEI0_MODULE);

    /* Enable TIMER0 module clock */
    CLK_EnableModuleClock(TMR0_MODULE);

    /* Select UART module clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HXT, CLK_CLKDIV0_UART0(1));

    /* Select TIMER0 module clock source */
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HXT, 0);
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & (~(UART0_RXD_PB12_Msk | UART0_TXD_PB13_Msk))) | UART0_RXD_PB12 | UART0_TXD_PB13;

    /* Set PH multi-function pins for QEI0_A, QEI0_B, QEI0_Z */
    SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA4MFP_Msk | SYS_GPA_MFPL_PA3MFP_Msk | SYS_GPA_MFPL_PA5MFP_Msk);
    SYS->GPA_MFPL |= (SYS_GPA_MFPL_PA4MFP_QEI0_A | SYS_GPA_MFPL_PA3MFP_QEI0_B | SYS_GPA_MFPL_PA5MFP_QEI0_INDEX);

}

void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);

    printf("+--------------------------------------+\n");
    printf("|     M2351 QEI Driver Sample Code      |\n");
    printf("+--------------------------------------+\n\n");
    printf("  >> Please connect PC.0 and PA.4 << \n");
    printf("  >> Please connect PC.1 and PA.3 << \n");
    printf("     Press any key to start test\n\n");
    getchar();

    /* Configure PA.0 and PA.1 as output mode */
    GPIO_SetMode(PC, BIT0, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PC, BIT1, GPIO_MODE_OUTPUT);

    QEI0A = 0;
    QEI0B = 0;

    /* Set QEI counting mode as X4 Compare-counting mode,
       set maximum counter value and enable IDX, QEA and QEB input */
    QEI_Open(QEI0, QEI_CTL_X4_COMPARE_COUNTING_MODE, 0x20000);

    /* Set counter compare value */
    QEI_SET_CNT_CMP(QEI0, 0x10000);

    /* Enable compare function */
    QEI_ENABLE_CNT_CMP(QEI0);

    /* Enable QEI interrupt */
    QEI_EnableInt(QEI0, QEI_CTL_CMPIEN_Msk | QEI_CTL_OVUNIEN_Msk);

    /* Start QEI function */
    QEI_Start(QEI0);

    /* Wait compare-match and overflow interrupt happened */
    while(1)
    {
        QEI0A = 1;
        CLK_SysTickDelay(16);
        QEI0B = 1;
        CLK_SysTickDelay(16);
        QEI0A = 0;
        CLK_SysTickDelay(16);
        QEI0B = 0;
        CLK_SysTickDelay(16);
    }

}
