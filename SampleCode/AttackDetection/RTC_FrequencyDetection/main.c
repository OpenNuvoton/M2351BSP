/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Show the usage of LXT clock frequency monitor function.
 *
 * @copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

void TAMPER_IRQHandler(void);
void SYS_Init(void);
void RTC_Init(void);
void UART0_Init(void);
void TIMER0_Init(void);
void GetActiveLXTandLIRC32Freq(uint32_t *u32LXTFreq, uint32_t *LIRC32Freq);

/*---------------------------------------------------------------------------------------------------------*/
/*  TAMPER IRQ Handler                                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
void TAMPER_IRQHandler(void)
{
    uint32_t u32Reg;

    u32Reg = RTC->INTSTS;

    if(u32Reg & RTC_INTSTS_CLKFIF_Msk)
    {
        printf("LXT clock frequency monitor fail interrupt is happened!\n");
        printf("LXT frequency is abnormal! RTC clock is switched to LIRC32.\n\n");

        /* Disable LXT clock frequency monitor fail interrupt */
        RTC->INTEN &= ~RTC_INTEN_CLKFIEN_Msk;

        /* Write 1 to clear LXT Clock frequency monitor fail interrupt */
        RTC->INTSTS = RTC_INTSTS_CLKFIF_Msk;
    }

    if(u32Reg & RTC_INTSTS_CLKSPIF_Msk)
    {
        printf("LXT clock frequency monitor stop interrupt is happened!\n");
        printf("LXT frequency is abnormal! RTC clock is switched to LIRC32.\n\n");

        /* Disable LXT clock frequency monitor stop interrupt */
        RTC->INTEN &= ~RTC_INTEN_CLKSPIEN_Msk;

        /* Write 1 to clear LXT Clock frequency monitor stop interrupt */
        RTC->INTSTS = RTC_INTSTS_CLKSPIF_Msk;
    }
}

void SYS_Init(void)
{
    /* Set PF multi-function pins for XT1_OUT(PF.2) and XT1_IN(PF.3) */
    SYS->GPF_MFPL = (SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF2MFP_Msk)) | SYS_GPF_MFPL_PF2MFP_XT1_OUT;
    SYS->GPF_MFPL = (SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF3MFP_Msk)) | SYS_GPF_MFPL_PF3MFP_XT1_IN;

    /* Set PF multi-function pins for X32_OUT(PF.4) and X32_IN(PF.5) */
    SYS->GPF_MFPL = (SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF4MFP_Msk)) | SYS_GPF_MFPL_PF4MFP_X32_OUT;
    SYS->GPF_MFPL = (SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF5MFP_Msk)) | SYS_GPF_MFPL_PF5MFP_X32_IN;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC, HXT and LXT clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);
    CLK_EnableXtalRC(CLK_PWRCTL_LXTEN_Msk);

    /* Wait for HIRC, HXT and LXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);
    CLK_WaitClockReady(CLK_STATUS_LXTSTB_Msk);

    /* Select HCLK clock source as HIRC and HCLK clock divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable PLL */
    CLK->PLLCTL = CLK_PLLCTL_128MHz_HIRC;

    /* Waiting for PLL stable */
    CLK_WaitClockReady(CLK_STATUS_PLLSTB_Msk);

    /* Select HCLK clock source as PLL and HCLK source divider as 2 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_PLL, CLK_CLKDIV0_HCLK(2));

    /* Select RTC peripheral clock source as LXT */
    CLK_SetModuleClock(RTC_MODULE, CLK_CLKSEL3_RTCSEL_LXT, MODULE_NoMsk);

    /* Enable RTC peripheral clock */
    CLK_EnableModuleClock(RTC_MODULE);

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART module clock source as HIRC and UART module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & (~(UART0_RXD_PB12_Msk | UART0_TXD_PB13_Msk))) | UART0_RXD_PB12 | UART0_TXD_PB13;

    /* Set PC multi-function pins for CLKO(PC.13) */
    SYS->GPC_MFPH = (SYS->GPC_MFPH & ~SYS_GPC_MFPH_PC13MFP_Msk) | SYS_GPC_MFPH_PC13MFP_CLKO;
}

void RTC_Init(void)
{
    /* Back to RTC initial setting */
    RTC->CLKDCTL &= ~(RTC_CLKDCTL_LXTFDEN_Msk | RTC_CLKDCTL_LXTFSW_Msk | RTC_CLKDCTL_LXTSPSW_Msk);
    RTC->LXTCTL &= ~(RTC_LXTCTL_LIRC32KEN_Msk | RTC_LXTCTL_C32KS_Msk);
    RTC->INTSTS = RTC_INTSTS_CLKFIF_Msk | RTC_INTSTS_CLKSPIF_Msk;
}

void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART0 module */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);
}

void TIMER0_Init(void)
{
    CLK->CLKSEL1 = (CLK->CLKSEL1 & 0xFFFFF8FF) | CLK_CLKSEL1_TMR0SEL_HXT;
    CLK->APBCLK0 |= CLK_APBCLK0_TMR0CKEN_Msk;

    TIMER0->CMP = 0xFFFFFF;
    TIMER0->CTL = TIMER_PERIODIC_MODE | TIMER_CTL_CNTEN_Msk | (12 - 1);
}

void GetActiveLXTandLIRC32Freq(uint32_t *u32LXTFreq, uint32_t *LIRC32Freq)
{
    uint32_t u32GetCNT;

    CLK->APBCLK0 |= (CLK_APBCLK0_TMR0CKEN_Msk | CLK_APBCLK0_TMR1CKEN_Msk);
    CLK->CLKSEL1 = (CLK->CLKSEL1 & ~(CLK_CLKSEL1_TMR0SEL_Msk | CLK_CLKSEL1_TMR1SEL_Msk)) | (CLK_CLKSEL1_TMR0SEL_HXT | CLK_CLKSEL1_TMR1SEL_LXT);

    TIMER0->CMP = __HXT;
    TIMER0->CTL = 0;

    TIMER1->CMP = 0xFFFFFF;
    TIMER1->CTL = TIMER_PERIODIC_MODE | TIMER_CTL_CNTEN_Msk;

    /* LXT source from external LXT */
    RTC->LXTCTL &= ~(RTC_LXTCTL_LIRC32KEN_Msk | RTC_LXTCTL_C32KS_Msk);
    CLK_SysTickDelay(10000);
    TIMER0->INTSTS = 0x3;
    TIMER1->CNT = 0x555; //reset counter value
    while(1)
    {
        if(TIMER1->CNT == 10)
        {
            TIMER0->CTL = TIMER_CTL_CNTEN_Msk;
            break;
        }
    }
    while(TIMER0->INTSTS == 0);
    u32GetCNT = TIMER1->CNT;
    *u32LXTFreq = u32GetCNT - 10;

    /* LXT source from LIRC32 */
    RTC->LXTCTL |= (RTC_LXTCTL_LIRC32KEN_Msk | RTC_LXTCTL_C32KS_Msk);
    CLK_SysTickDelay(10000);
    TIMER0->INTSTS = 0x3;
    TIMER1->CNT = 0x555; //reset counter value
    while(1)
    {
        if(TIMER1->CNT == 10)
        {
            TIMER0->CTL = TIMER_CTL_CNTEN_Msk;
            break;
        }
    }
    while(TIMER0->INTSTS == 0);
    u32GetCNT = TIMER1->CNT;
    *LIRC32Freq = u32GetCNT - 10;

    /* LXT source from external LXT */
    RTC->LXTCTL &= ~(RTC_LXTCTL_LIRC32KEN_Msk | RTC_LXTCTL_C32KS_Msk);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    uint32_t u32LXTFreq, u32LIRC32Freq, u32STDCount;
    int32_t i32Option;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init RTC */
    RTC_Init();

    /* Init UART0 for printf */
    UART0_Init();

    /* Init TIMER0 */
    TIMER0_Init();

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);
    printf("+-------------------------------------------------------------+\n");
    printf("|             RTC Frequency Detection Sample Code             |\n");
    printf("+-------------------------------------------------------------+\n");
    printf("| LXT clock frequency monitor fail/stop interrupt will happen |\n");
    printf("| if LXT clock frequency isn't oscillating in normal range.   |\n");
    printf("| User can switch RTC clock source from LXT to LIRC32.        |\n");
    printf("+-------------------------------------------------------------+\n");

    GetActiveLXTandLIRC32Freq(&u32LXTFreq, &u32LIRC32Freq);

    if(u32LXTFreq > u32LIRC32Freq)
    {
        u32STDCount = ((1000000000 / u32LXTFreq) * 255) / (1000000000 / u32LIRC32Freq);
        printf("LXT is faster than LIRC32. [%dHz > %dHz]\n", u32LXTFreq, u32LIRC32Freq);
    }
    else
    {
        u32STDCount = ((1000000000 / u32LIRC32Freq) * 255) / (1000000000 / u32LXTFreq);
        printf("LIRC32 is faster than LXT. [%dHz > %dHz]\n", u32LIRC32Freq, u32LXTFreq);
    }

    /* Enable clock output, select CLKO clock source as LXT and set clock output frequency is LXT.
       RTC clock source will be switched to LIRC32 if LXT frequency isn't oscillating in normal range.
       You can check if RTC clock source is switched to LIRC32 by clock output pin output frequency.
    */

    /* Output selected clock to CKO, CKO Clock = LXT / 1 */
    CLK_EnableCKO(CLK_CLKSEL1_CLKOSEL_LXT, 0, 1);

    /* Set the LXT clock frequency monitor fail/stop boundary value.
       The fail/stop boundary value should be less than u32STDCount.
    */
    RTC->CDBR = ((u32STDCount - 5) << RTC_CDBR_FAILBD_Pos) | ((u32STDCount - 5) << RTC_CDBR_STOPBD_Pos);

    /* Set clock fail/stop detector function enabled */
    RTC->CLKDCTL = RTC_CLKDCTL_LXTFDEN_Msk;

    /* Set clock fail/stop detector switch LIRC32K enabled */
    RTC->CLKDCTL |= RTC_CLKDCTL_LXTFSW_Msk | RTC_CLKDCTL_LXTSPSW_Msk;

    /* Enable LIRC32K source */
    RTC->LXTCTL |= RTC_LXTCTL_LIRC32KEN_Msk;

    /* Clock frequency monitor fail/stop interrupt enabled */
    RTC->INTEN = RTC_INTEN_CLKFIEN_Msk | RTC_INTEN_CLKSPIEN_Msk;

    /* Enable tamper detection interrupt */
    NVIC_EnableIRQ(TAMPER_IRQn);

    printf("\nSelect:\n");
    printf("    [0] Smaller frequency deviation monitoring\n");
    printf("    [1] Larger frequency deviation monitoring\n");

    i32Option = getchar();
    printf("\n");
    printf("Select item [%c]\n", i32Option);

    if(i32Option == '0')
    {
        printf("Modify LXT fail boundary to detect FAIL flag.\n\n");

        /* Modify the LXT clock frequency monitor fail boundary value to detect FAIL flag.
           The fail boundary value should be more than u32STDCount.
        */
        RTC->CDBR = ((u32STDCount + 1) << RTC_CDBR_FAILBD_Pos);
    }
    else if(i32Option == '1')
    {
        printf("Stop LXT to detect FAIL and STOP flag.\n\n");
    }

    /* Wait for clock frequency monitor fail/stop detector interrupt happened */
    while(1);
}

/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/
