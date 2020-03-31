/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief
 *           Show how to enter to different Power-down mode and wake-up by RTC.
 *
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"



#define PDMD_FLAG_ADDR      0x20000FFC



extern int IsDebugFifoEmpty(void);
static uint32_t s_u32PowerDownMode;
static volatile uint32_t s_u32RTCTickINT;


void RTC_IRQHandler(void);
void RTC_Init(void);
void PowerDownFunction(void);
void CheckPowerSource(void);
void SYS_Init(void);
void UART0_Init(void);

/**
 * @brief       IRQ Handler for RTC Interrupt
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The RTC_IRQHandler is default IRQ of RTC, declared in startup_M2351.s.
 */
void RTC_IRQHandler(void)
{
    /* To check if RTC tick interrupt occurred */
    if(RTC_GET_TICK_INT_FLAG(RTC) == 1)
    {
        /* Clear RTC tick interrupt flag */
        RTC_CLEAR_TICK_INT_FLAG(RTC);
    }

    s_u32RTCTickINT++;
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Function for RTC wake-up source setting                                                                */
/*---------------------------------------------------------------------------------------------------------*/
void RTC_Init(void)
{
    S_RTC_TIME_DATA_T sWriteRTC;

    /* Init RTC in the start of sample code */
    if(RTC->INIT != RTC_INIT_ACTIVE_Msk)
    {
        printf("\n\nCPU @ %dHz\n", SystemCoreClock);
        printf("+------------------------------------------+\n");
        printf("|    Power-down and Wake-up Sample Code    |\n");
        printf("+------------------------------------------+\n\n");

        /* Open RTC */
        sWriteRTC.u32Year       = 2017;
        sWriteRTC.u32Month      = 3;
        sWriteRTC.u32Day        = 16;
        sWriteRTC.u32DayOfWeek  = RTC_MONDAY;
        sWriteRTC.u32Hour       = 0;
        sWriteRTC.u32Minute     = 0;
        sWriteRTC.u32Second     = 0;
        sWriteRTC.u32TimeScale  = RTC_CLOCK_24;
        RTC_Open(&sWriteRTC);
        printf("# Set RTC current date/time: 2017/03/16 00:00:00.\n");

        /* It is the start of sample code */
        M32(PDMD_FLAG_ADDR) = 0;
    }

    /* Clear RTC tick interrupt flag */
    RTC_CLEAR_TICK_INT_FLAG(RTC);

    /* Enable RTC NVIC */
    NVIC_EnableIRQ(RTC_IRQn);

    /* Enable RTC tick interrupt and wake-up function will be enabled also */
    RTC_EnableInt(RTC_INTEN_TICKIEN_Msk);
    RTC_SetTickPeriod(RTC_TICK_1_SEC);

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable RTC wake-up from SPD and DPD */
    CLK_ENABLE_RTCWK();

    /* Lock protected registers */
    SYS_LockReg();

}

/*---------------------------------------------------------------------------------------------------------*/
/*  Function for System Entry to Power Down Mode                                                           */
/*---------------------------------------------------------------------------------------------------------*/
void PowerDownFunction(void)
{
    /* Select Power-down mode */
    CLK_SetPowerDownMode(s_u32PowerDownMode);

    /* To check if all the debug messages are finished */
    while(IsDebugFifoEmpty() == 0);

    /* Enter to Power-down mode */
    CLK_PowerDown();
}

/*-----------------------------------------------------------------------------------------------------------*/
/*  Function for Check Power Manager Status                                                                  */
/*-----------------------------------------------------------------------------------------------------------*/
void CheckPowerSource(void)
{
    S_RTC_TIME_DATA_T sReadRTC;

    /* Get Power Manager Status */
    uint32_t u32Status = CLK_GetPMUWKSrc();

    /* Clear wake-up status flag */
    CLK->PMUSTS = CLK_PMUSTS_CLRWK_Msk;

    /* Check Power Manager Status is wake-up by RTC */
    if(u32Status & CLK_PMUSTS_RTCWK_Msk)
    {
        s_u32PowerDownMode = M32(PDMD_FLAG_ADDR);
        switch(s_u32PowerDownMode)
        {

            case CLK_PMUCTL_PDMSEL_PD:

                /* It is the start of sample code by pressing reset button */
                printf("\n\nCPU @ %dHz\n", SystemCoreClock);
                printf("+------------------------------------------+\n");
                printf("|    Power-down and Wake-up Sample Code    |\n");
                printf("+------------------------------------------+\n");
                break;

            case CLK_PMUCTL_PDMSEL_SPD:

                /* Wake-up from Standby Power-down Mode */
                printf("Wake-up!\n");

                /* Read current RTC date/time */
                RTC_GetDateAndTime(&sReadRTC);
                printf("# Get RTC current date/time: %d/%02d/%02d %02d:%02d:%02d.\n",
                       sReadRTC.u32Year, sReadRTC.u32Month, sReadRTC.u32Day, sReadRTC.u32Hour, sReadRTC.u32Minute, sReadRTC.u32Second);

                /* Next Power-down Mode is Deep Power-down Mode */
                M32(PDMD_FLAG_ADDR) = CLK_PMUCTL_PDMSEL_DPD;
                break;

            default:

                /* Wake-up from Deep Power-down Mode */
                printf("Wake-up!\n");

                /* Read current RTC date/time */
                RTC_GetDateAndTime(&sReadRTC);
                printf("# Get RTC current date/time: %d/%02d/%02d %02d:%02d:%02d.\n",
                       sReadRTC.u32Year, sReadRTC.u32Month, sReadRTC.u32Day, sReadRTC.u32Hour, sReadRTC.u32Minute, sReadRTC.u32Second);

                /* End of sample code and clear Power-down Mode flag */
                printf("\nSample code end. Press Reset Button and continue.\n");
                M32(PDMD_FLAG_ADDR) = 0;
                while(1);

        }

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
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk | CLK_PWRCTL_HXTEN_Msk | CLK_PWRCTL_LXTEN_Msk);

    /* Wait for HIRC, HXT and LXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk | CLK_STATUS_LXTSTB_Msk | CLK_STATUS_HXTSTB_Msk);

    /* Enable PLL */
    CLK->PLLCTL = CLK_PLLCTL_128MHz_HIRC;

    /* Wait for PLL stable */
    CLK_WaitClockReady(CLK_STATUS_PLLSTB_Msk);

    /* Select HCLK clock source as PLL and HCLK source divider as 2 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_PLL, CLK_CLKDIV0_HCLK(2));

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART module clock source as HXT and UART module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HXT, CLK_CLKDIV0_UART0(1));

    /* Enable peripheral clock */
    CLK_EnableModuleClock(RTC_MODULE);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & (~(UART0_RXD_PB12_Msk | UART0_TXD_PB13_Msk))) | UART0_RXD_PB12 | UART0_TXD_PB13;

    /* Set PC multi-function pins for CLKO(PC.13) */
    SYS->GPC_MFPH = (SYS->GPC_MFPH & (~SYS_GPC_MFPH_PC13MFP_Msk)) | SYS_GPC_MFPH_PC13MFP_CLKO;

}

void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART0 */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    S_RTC_TIME_DATA_T sReadRTC;
    uint8_t u8NeedRst = 0;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();
    
    /* Release I/O hold status */
    CLK->IOPDCTL = 1;    

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    /* Enable Clock Output function, output clock is stopped in Power-down mode */
    CLK_EnableCKO(CLK_CLKSEL1_CLKOSEL_HCLK, 3, 0);

    /* Get power manager wake up source */
    CheckPowerSource();

    /* RTC wake-up source setting */
    RTC_Init();

    /*
        This sample code will enter to different Power-down mode and wake-up by RTC:
        1. Normal Power-down mode (PD).
        2. Low Leakage Power-down mode (LLPD).
        3. Fast Wake-up Power-down mode (FWPD).
        4. Ultra Low Leakage Power-down mode (ULLPD).
        5. Standby Power-down mode (SPD).
        6. Deep Power-down mode (DPD).
    */
    while(1)
    {

        /* Select Power-down mode */
        s_u32PowerDownMode = M32(PDMD_FLAG_ADDR);
        switch(s_u32PowerDownMode)
        {
            case CLK_PMUCTL_PDMSEL_PD:
                printf("\nSystem enter to PD power-down mode ... ");
                break;
            case CLK_PMUCTL_PDMSEL_LLPD:
                printf("\nSystem enter to LLPD power-down mode ... ");
                break;
            case CLK_PMUCTL_PDMSEL_FWPD:
                printf("\nSystem enter to FWPD power-down mode ... ");
                break;
            case CLK_PMUCTL_PDMSEL_ULLPD:
                printf("\nSystem enter to ULLPD power-down mode ... ");
                break;
            case CLK_PMUCTL_PDMSEL_SPD:
                printf("\nSystem enter to SPD power-down mode ... ");
                break;
            case CLK_PMUCTL_PDMSEL_DPD:
                printf("\nSystem enter to DPD power-down mode ... ");
                break;
            default:
                printf("\nInit sample code. Press Reset Button and continue.\n");
                M32(PDMD_FLAG_ADDR) = 0;
                u8NeedRst = 1;
                break;
        }
        
        if (u8NeedRst)
            while(1){}

        /* Unlock protected registers before setting Power-down mode */
        SYS_UnlockReg();

        /* Enter to Power-down mode */
        PowerDownFunction();
        while(s_u32RTCTickINT == 0);
        printf("Wake-up!\n");

        /* Read current RTC date/time after wake-up */
        RTC_GetDateAndTime(&sReadRTC);
        printf("# Get RTC current date/time: %d/%02d/%02d %02d:%02d:%02d.\n",
               sReadRTC.u32Year, sReadRTC.u32Month, sReadRTC.u32Day, sReadRTC.u32Hour, sReadRTC.u32Minute, sReadRTC.u32Second);

        /* Select next Power-down mode */
        switch(s_u32PowerDownMode)
        {
            case CLK_PMUCTL_PDMSEL_PD:
                M32(PDMD_FLAG_ADDR) = CLK_PMUCTL_PDMSEL_LLPD;
                break;
            case CLK_PMUCTL_PDMSEL_LLPD:
                M32(PDMD_FLAG_ADDR) = CLK_PMUCTL_PDMSEL_FWPD;
                break;
            case CLK_PMUCTL_PDMSEL_FWPD:
                M32(PDMD_FLAG_ADDR) = CLK_PMUCTL_PDMSEL_ULLPD;
                break;
            case CLK_PMUCTL_PDMSEL_ULLPD:
                M32(PDMD_FLAG_ADDR) = CLK_PMUCTL_PDMSEL_SPD;
                break;
            default:
                printf("\nPress Reset Button and continue.\n");
                M32(PDMD_FLAG_ADDR) = 0;
                u8NeedRst = 1;
                break;
        }

        if (u8NeedRst)
            while(1){}
    }


}


/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
