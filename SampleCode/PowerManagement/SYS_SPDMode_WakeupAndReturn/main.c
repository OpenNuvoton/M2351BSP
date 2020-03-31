/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief
 *           Show how to continue executing code after wake-up form SPD Power-down mode by SRAM data retention function.
 *
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


void SYS_Init(void);
void RTC_Init(void);
void UART0_Init(void);
extern uint32_t __Enter_SPD(void);


static volatile uint32_t s_u32RTCTickINT=0;

void PowerDownFunction(void);
void RTC_IRQHandler(void);

void PowerDownFunction(void)
{
 
    /* Select SPD Power-down mode */
    CLK_SetPowerDownMode(CLK_PMUCTL_PDMSEL_SPD);        

    /* Set the processor uses deep sleep as its low power mode */
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

    /* Set system Power-down enabled */
    CLK->PWRCTL |= CLK_PWRCTL_PDEN_Msk;         
    
    /* Reserve R0-R7, LR and enter to Power-down mode */        
    __set_PRIMASK(1);
    __ASM volatile("push {r0-r7} \n");   
    __ASM volatile("push {lr} \n");       
    __Enter_SPD();  
    
    /* Restore R0-R7 and LR */
    __ASM volatile("pop {r0} \n");     
    __ASM volatile("mov lr, r0 \n");      
    __ASM volatile("pop {r0-r7} \n");  
    __set_PRIMASK(0);      
    
    /* Initialization after wake-up from SPD */
    if(CLK->PMUSTS&CLK_PMUSTS_RTCWK_Msk)
    {      
        SYS_UnlockReg();                            /* Unlock protected registers */
        SYS_Init();                                 /* Init System, peripheral clock and multi-function I/O */
        UART0_Init();                               /* Init UART0 for printf */
        PA10 = PA10;                                /* LED toggle in RTC interrupt */
        GPIO_SetMode(PA, BIT10, GPIO_MODE_OUTPUT);            
        CLK->IOPDCTL = 1;                           /* Release I/O hold status */            
        RTC_Init();                                 /* Init RTC */           
    }                

}


/**
 * @brief       IRQ Handler for RTC Interrupt
 * @param       None
 * @return      None
 * @details     The RTC_IRQHandler is default IRQ of RTC, declared in mystartup_M2351.s.
 */
void RTC_IRQHandler(void)
{      
  
    /* To check if RTC tick interrupt occurred */
    if(RTC_GET_TICK_INT_FLAG(RTC) == 1)
    {
        /* Clear RTC tick interrupt flag */
        RTC_CLEAR_TICK_INT_FLAG(RTC);
        while((RTC_GET_TICK_INT_FLAG(RTC) == 1));
    }

    s_u32RTCTickINT = 1;
    GPIO_TOGGLE(PA10);
    
}

void RTC_Init(void)
{
    S_RTC_TIME_DATA_T sWriteRTC;             
       
    /* Enable LXT clock if it is not enabled before */
    if( (CLK->PWRCTL & CLK_PWRCTL_LXTEN_Msk) == 0 )
    {
        CLK->PWRCTL |= CLK_PWRCTL_LXTEN_Msk;
        while((CLK->STATUS&CLK_STATUS_LXTSTB_Msk) == 0);        
    }     
    
    /* Enable RTC colck */
    CLK_EnableModuleClock(RTC_MODULE);        

    /* RTC clock source select LXT */
    CLK_SetModuleClock(RTC_MODULE, CLK_CLKSEL3_RTCSEL_LXT, 0);    
    
    /* Initial RTC if it is not initialed before */    
    if(RTC->INIT != RTC_INIT_ACTIVE_Msk)
    {                              
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
        printf("# Set RTC current date/time: 2017/03/16 00:00:00.\n\n");             
        
        /* Enable RTC tick interrupt and wake-up function will be enabled also */
        RTC_EnableInt(RTC_INTEN_TICKIEN_Msk);
        RTC_SetTickPeriod(RTC_TICK_1_SEC);           
    }

    /* Enable RTC wake-up from SPD and DPD */
    CLK_ENABLE_RTCWK();    
    
    /* Enable RTC NVIC */
    NVIC_EnableIRQ(RTC_IRQn);
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

    /* Enable HIRC and HXT clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk|CLK_PWRCTL_HXTEN_Msk);

    /* Wait for HIRC and HXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk|CLK_STATUS_HXTSTB_Msk);

    /* Enable PLL */
    CLK->PLLCTL = CLK_PLLCTL_128MHz_HIRC;

    /* Wait for PLL stable */
    CLK_WaitClockReady(CLK_STATUS_PLLSTB_Msk);

    /* Select HCLK clock source as PLL and HCLK source divider as 2 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_PLL, CLK_CLKDIV0_HCLK(2));

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART module clock source as HXT and UART module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & (~(UART0_RXD_PB12_Msk | UART0_TXD_PB13_Msk))) | UART0_RXD_PB12 | UART0_TXD_PB13;

}

void UART0_Init(void)
{      
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART0 */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);    
}

int main( void )
{   
     S_RTC_TIME_DATA_T sReadRTC; 
     
    /* Unlock protected registers */
    SYS_UnlockReg();
    
    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();
    
    /* LED toggle in RTC interrupt */
    GPIO_SetMode(PA, BIT10, GPIO_MODE_OUTPUT);

    /* Unlock protected registers before setting Power-down mode */
    SYS_UnlockReg();

    printf("\n");
    printf("+--------------------------------------------------------+\n");
    printf("|   SPD Power-down Mode Wake-up and Return Sample Code   |\n");    
    printf("+--------------------------------------------------------+\n");
    
    /* Init RTC */ 
    RTC_Init();    
    
    while(1)
    {            
      
        /* Read current RTC date/time */
        RTC_GetDateAndTime(&sReadRTC);
        printf("# Get RTC current date/time: %d/%02d/%02d %02d:%02d:%02d.\n",
               sReadRTC.u32Year, sReadRTC.u32Month, sReadRTC.u32Day, sReadRTC.u32Hour, sReadRTC.u32Minute, sReadRTC.u32Second);         
      
        s_u32RTCTickINT = 0;        
        
        /* Enter to SPD Power-down mode */
        printf("Enter to SPD ... ");   
        UART_WAIT_TX_EMPTY(DEBUG_PORT);
        PowerDownFunction();
       
        /* Wait RTC interrupt */
        while(s_u32RTCTickINT==0);       
        
        /* Check wake-up from SPD by RTC flag */
        if(CLK->PMUSTS&CLK_PMUSTS_RTCWK_Msk)
        {
            printf("RTC ");
            
            /* Clear wake-up flag */
            CLK->PMUSTS = CLK_PMUSTS_CLRWK_Msk;   
        }       
        
        printf("wake-up!\n\n");                                    
        
    }    
        
}

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
