/**************************************************************************//**
 * @file     exeinsram.c
 * @version  V3.00
 * @brief    Implement a code and execute in SRAM.
 *
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#include <stdio.h>
#include "NuMicro.h"


void GPIO_SingleCycleIO_Test(void);


#if (defined(__GNUC__) && !defined(__ARMCC_VERSION))
__attribute__ ((used, long_call, section(".fastcode"))) void GPIO_SingleCycleIO_Test(void)
#else
void GPIO_SingleCycleIO_Test(void)
#endif
{
    uint32_t u32CounterTMR0 = 0, u32CounterTMR2 = 0;
    
    /* Configure TIMER2 to count PA.0 toggle event (falling edge) */      
    TIMER2->CTL = TIMER_ONESHOT_MODE;  
    TIMER_EnableEventCounter(TIMER2, TIMER_COUNTER_EVENT_FALLING);      
    TIMER2->CTL |= TIMER_CTL_CNTEN_Msk;     
 
    /* Configure TIMER0 to measure the elapsed time */ 
    TIMER0->CTL = TIMER_ONESHOT_MODE;   
    TIMER0->CTL |= TIMER_CTL_CNTEN_Msk;      
    
    /* Toggle PA.0 state 50 times */
    PA0=1;
    PA0=0;
    PA0=1;
    PA0=0;
    PA0=1;
    PA0=0;
    PA0=1;
    PA0=0;
    PA0=1;
    PA0=0;
    
    PA0=1;
    PA0=0;
    PA0=1;
    PA0=0;
    PA0=1;
    PA0=0;
    PA0=1;
    PA0=0;
    PA0=1;
    PA0=0; 
    
    PA0=1;
    PA0=0;
    PA0=1;
    PA0=0;
    PA0=1;
    PA0=0;
    PA0=1;
    PA0=0;
    PA0=1;
    PA0=0;     
    
    PA0=1;
    PA0=0;
    PA0=1;
    PA0=0;
    PA0=1;
    PA0=0;
    PA0=1;
    PA0=0;
    PA0=1;
    PA0=0;    

    PA0=1;
    PA0=0;
    PA0=1;
    PA0=0;
    PA0=1;
    PA0=0;
    PA0=1;
    PA0=0;
    PA0=1;
    PA0=0;  
    
    /* Get TIMER0 and TIMER2 counter */
    u32CounterTMR0 = TIMER0->CNT;   
    u32CounterTMR2 = TIMER2->CNT;  
     
    /* Stop TIMER0 and TIMER2 */
    TIMER_Stop(TIMER0);  
    TIMER_Stop(TIMER2);     

    /* Print result */   
    printf("Toggle speed measurement result\n");         
    printf("=================================================\n"); 
    printf("GPIO Falling Edge Counts (A)        : %d\n",        u32CounterTMR2);  
    printf("Total Elapsed Time       (B)        : %.2f(us)\n",  (double)u32CounterTMR0/CyclesPerUs);      
    printf("Average Toggle Speed     (C)=(A)/(B): %.2f(MHz)\n", (double)(u32CounterTMR2*CyclesPerUs)/u32CounterTMR0);     


}

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
