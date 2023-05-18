/*
 * FreeRTOS Kernel V10.4.6
 * Copyright (C) 2021 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 * Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * https://www.FreeRTOS.org
 * https://github.com/FreeRTOS
 *
 */

/*-----------------------------------------------------------
 * Implementation of functions defined in portable.h for the ARM CM0 port.
 *----------------------------------------------------------*/

/* Scheduler includes. */
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "NuMicro.h"

/* Constants required to manipulate the NVIC. */
#define portNVIC_SYSTICK_CTRL_REG             ( *( ( volatile uint32_t * ) 0xe000e010 ) )
#define portNVIC_SYSTICK_LOAD_REG             ( *( ( volatile uint32_t * ) 0xe000e014 ) )
#define portNVIC_SYSTICK_CURRENT_VALUE_REG    ( *( ( volatile uint32_t * ) 0xe000e018 ) )
#define portNVIC_INT_CTRL_REG                 ( *( ( volatile uint32_t * ) 0xe000ed04 ) )
#define portNVIC_SHPR3_REG                    ( *( ( volatile uint32_t * ) 0xe000ed20 ) )
#define portNVIC_SYSTICK_CLK_BIT              ( 1UL << 2UL )
#define portNVIC_SYSTICK_INT_BIT              ( 1UL << 1UL )
#define portNVIC_SYSTICK_ENABLE_BIT           ( 1UL << 0UL )
#define portNVIC_SYSTICK_COUNT_FLAG_BIT       ( 1UL << 16UL )
#define portNVIC_PENDSVSET_BIT                ( 1UL << 28UL )
#define portMIN_INTERRUPT_PRIORITY            ( 255UL )
#define portNVIC_PENDSV_PRI                   ( portMIN_INTERRUPT_PRIORITY << 16UL )
#define portNVIC_SYSTICK_PRI                  ( portMIN_INTERRUPT_PRIORITY << 24UL )

/* Constants required to set up the initial stack. */
#define portINITIAL_XPSR                      ( 0x01000000 )

/* The systick is a 24-bit counter. */
#define portMAX_24_BIT_NUMBER                 ( 0xffffffUL )

/* A fiddle factor to estimate the number of SysTick counts that would have
 * occurred while the SysTick counter is stopped during tickless idle
 * calculations. */
#ifndef portMISSED_COUNTS_FACTOR
    #define portMISSED_COUNTS_FACTOR          ( 45UL )
#endif

/* Let the user override the pre-loading of the initial LR with the address of
 * prvTaskExitError() in case it messes up unwinding of the stack in the
 * debugger. */
#ifdef configTASK_RETURN_ADDRESS
    #define portTASK_RETURN_ADDRESS    configTASK_RETURN_ADDRESS
#else
    #define portTASK_RETURN_ADDRESS    prvTaskExitError
#endif

/*
 * Setup the timer to generate the tick interrupts. The implementation in this
 * file is weak to allow application writers to change the timer used to
 * generate the tick interrupt.
 */
void vPortSetupTimerInterrupt( void );

/*
 * Exception handlers.
 */
void xPortPendSVHandler( void ) __attribute__( ( naked ) );
void xPortSysTickHandler( void );
void vPortSVCHandler( void );

/*
 * Start first task is a separate function so it can be tested in isolation.
 */
static void vPortStartFirstTask( void ) __attribute__( ( naked ) );

/*
 * Used to catch tasks that attempt to return from their implementing function.
 */
static void prvTaskExitError( void );

/*-----------------------------------------------------------*/

/* Each task maintains its own interrupt status in the critical nesting
 * variable. */
static UBaseType_t uxCriticalNesting = 0xaaaaaaaa;

/*-----------------------------------------------------------*/

/*
 * The number of SysTick increments that make up one tick period.
 */
#if ( configUSE_TICKLESS_IDLE == 1 )
    static uint32_t ulTimerCountsForOneTick = 0;
#endif /* configUSE_TICKLESS_IDLE */

/*
 * The maximum number of tick periods that can be suppressed is limited by the
 * 24 bit resolution of the SysTick timer.
 */
#if ( configUSE_TICKLESS_IDLE == 1 )
    static uint32_t xMaximumPossibleSuppressedTicks = 0;
#endif /* configUSE_TICKLESS_IDLE */

/*
 * Compensate for the CPU cycles that pass while the SysTick is stopped (low
 * power functionality only.
 */
#if ( configUSE_TICKLESS_IDLE == 1 )
    static uint32_t ulStoppedTimerCompensation = 0;
#endif /* configUSE_TICKLESS_IDLE */

/*-----------------------------------------------------------*/

/*
 * See header file for description.
 */
StackType_t * pxPortInitialiseStack( StackType_t * pxTopOfStack,
                                     TaskFunction_t pxCode,
                                     void * pvParameters )
{
    /* Simulate the stack frame as it would be created by a context switch
     * interrupt. */
    pxTopOfStack--;                                          /* Offset added to account for the way the MCU uses the stack on entry/exit of interrupts. */
    *pxTopOfStack = portINITIAL_XPSR;                        /* xPSR */
    pxTopOfStack--;
    *pxTopOfStack = ( StackType_t ) pxCode;                  /* PC */
    pxTopOfStack--;
    *pxTopOfStack = ( StackType_t ) portTASK_RETURN_ADDRESS; /* LR */
    pxTopOfStack -= 5;                                       /* R12, R3, R2 and R1. */
    *pxTopOfStack = ( StackType_t ) pvParameters;            /* R0 */
    pxTopOfStack -= 8;                                       /* R11..R4. */

    return pxTopOfStack;
}
/*-----------------------------------------------------------*/

static void prvTaskExitError( void )
{
    volatile uint32_t ulDummy = 0UL;

    /* A function that implements a task must not exit or attempt to return to
     * its caller as there is nothing to return to. If a task wants to exit it
     * should instead call vTaskDelete( NULL ).
     *
     * Artificially force an assert() to be triggered if configASSERT() is
     * defined, then stop here so application writers can catch the error. */
    configASSERT( uxCriticalNesting == ~0UL );
    portDISABLE_INTERRUPTS();

    while( ulDummy == 0 )
    {
        /* This file calls prvTaskExitError() after the scheduler has been
         * started to remove a compiler warning about the function being defined
         * but never called. ulDummy is used purely to quieten other warnings
         * about code appearing after this function is called - making ulDummy
         * volatile makes the compiler think the function could return and
         * therefore not output an 'unreachable code' warning for code that appears
         * after it. */
    }
}
/*-----------------------------------------------------------*/

void vPortSVCHandler( void )
{
    /* This function is no longer used, but retained for backward
     * compatibility. */
}
/*-----------------------------------------------------------*/

void vPortStartFirstTask( void )
{
    /* The MSP stack is not reset as, unlike on M3/4 parts, there is no vector
     * table offset register that can be used to locate the initial stack value.
     * Not all M0 parts have the application vector table at address 0. */
    __asm volatile (
        "	.syntax unified				\n"
        "	ldr  r2, pxCurrentTCBConst2	\n"/* Obtain location of pxCurrentTCB. */
        "	ldr  r3, [r2]				\n"
        "	ldr  r0, [r3]				\n"/* The first item in pxCurrentTCB is the task top of stack. */
        "	adds r0, #32					\n"/* Discard everything up to r0. */
        "	msr  psp, r0					\n"/* This is now the new top of stack to use in the task. */
        "	movs r0, #2					\n"/* Switch to the psp stack. */
        "	msr  CONTROL, r0				\n"
        "	isb							\n"
        "	pop  {r0-r5}					\n"/* Pop the registers that are saved automatically. */
        "	mov  lr, r5					\n"/* lr is now in r5. */
        "	pop  {r3}					\n"/* Return address is now in r3. */
        "	pop  {r2}					\n"/* Pop and discard XPSR. */
        "	cpsie i						\n"/* The first task has its context and interrupts can be enabled. */
        "	bx   r3						\n"/* Finally, jump to the user defined task code. */
        "								\n"
        "	.align 4					\n"
        "pxCurrentTCBConst2: .word pxCurrentTCB	  "
        );
}
/*-----------------------------------------------------------*/

/*
 * See header file for description.
 */
BaseType_t xPortStartScheduler( void )
{
    /* Make PendSV, CallSV and SysTick the same priority as the kernel. */
    portNVIC_SHPR3_REG |= portNVIC_PENDSV_PRI;
    portNVIC_SHPR3_REG |= portNVIC_SYSTICK_PRI;

    /* Start the timer that generates the tick ISR. Interrupts are disabled
     * here already. */
    vPortSetupTimerInterrupt();

    /* Initialise the critical nesting count ready for the first task. */
    uxCriticalNesting = 0;

    /* Start the first task. */
    vPortStartFirstTask();

    /* Should never get here as the tasks will now be executing!  Call the task
     * exit error function to prevent compiler warnings about a static function
     * not being called in the case that the application writer overrides this
     * functionality by defining configTASK_RETURN_ADDRESS. Call
     * vTaskSwitchContext() so link time optimisation does not remove the
     * symbol. */
    vTaskSwitchContext();
    prvTaskExitError();

    /* Should not get here! */
    return 0;
}
/*-----------------------------------------------------------*/

void vPortEndScheduler( void )
{
    /* Not implemented in ports where there is nothing to return to.
     * Artificially force an assert. */
    configASSERT( uxCriticalNesting == 1000UL );
}
/*-----------------------------------------------------------*/

void vPortYield( void )
{
    /* Set a PendSV to request a context switch. */
    portNVIC_INT_CTRL_REG = portNVIC_PENDSVSET_BIT;

    /* Barriers are normally not required but do ensure the code is completely
     * within the specified behaviour for the architecture. */
    __asm volatile ( "dsb" ::: "memory" );
    __asm volatile ( "isb" );
}
/*-----------------------------------------------------------*/

void vPortEnterCritical( void )
{
    portDISABLE_INTERRUPTS();
    uxCriticalNesting++;
    __asm volatile ( "dsb" ::: "memory" );
    __asm volatile ( "isb" );
}
/*-----------------------------------------------------------*/

void vPortExitCritical( void )
{
    configASSERT( uxCriticalNesting );
    uxCriticalNesting--;

    if( uxCriticalNesting == 0 )
    {
        portENABLE_INTERRUPTS();
    }
}
/*-----------------------------------------------------------*/

uint32_t ulSetInterruptMaskFromISR( void )
{
    __asm volatile (
        " mrs r0, PRIMASK	\n"
        " cpsid i			\n"
        " bx lr				  "
        ::: "memory"
        );
}
/*-----------------------------------------------------------*/

void vClearInterruptMaskFromISR( __attribute__( ( unused ) ) uint32_t ulMask )
{
    __asm volatile (
        " msr PRIMASK, r0	\n"
        " bx lr				  "
        ::: "memory"
        );
}
/*-----------------------------------------------------------*/

void xPortPendSVHandler( void )
{
    /* This is a naked function. */

    __asm volatile
    (
        "	.syntax unified						\n"
        "	mrs r0, psp							\n"
        "										\n"
        "	ldr	r3, pxCurrentTCBConst			\n"/* Get the location of the current TCB. */
        "	ldr	r2, [r3]						\n"
        "										\n"
        "	subs r0, r0, #32					\n"/* Make space for the remaining low registers. */
        "	str r0, [r2]						\n"/* Save the new top of stack. */
        "	stmia r0!, {r4-r7}					\n"/* Store the low registers that are not saved automatically. */
        " 	mov r4, r8							\n"/* Store the high registers. */
        " 	mov r5, r9							\n"
        " 	mov r6, r10							\n"
        " 	mov r7, r11							\n"
        " 	stmia r0!, {r4-r7}					\n"
        "										\n"
        "	push {r3, r14}						\n"
        "	cpsid i								\n"
        "	bl vTaskSwitchContext				\n"
        "	cpsie i								\n"
        "	pop {r2, r3}						\n"/* lr goes in r3. r2 now holds tcb pointer. */
        "										\n"
        "	ldr r1, [r2]						\n"
        "	ldr r0, [r1]						\n"/* The first item in pxCurrentTCB is the task top of stack. */
        "	adds r0, r0, #16					\n"/* Move to the high registers. */
        "	ldmia r0!, {r4-r7}					\n"/* Pop the high registers. */
        " 	mov r8, r4							\n"
        " 	mov r9, r5							\n"
        " 	mov r10, r6							\n"
        " 	mov r11, r7							\n"
        "										\n"
        "	msr psp, r0							\n"/* Remember the new top of stack for the task. */
        "										\n"
        "	subs r0, r0, #32					\n"/* Go back for the low registers that are not automatically restored. */
        " 	ldmia r0!, {r4-r7}					\n"/* Pop low registers.  */
        "										\n"
        "	bx r3								\n"
        "										\n"
        "	.align 4							\n"
        "pxCurrentTCBConst: .word pxCurrentTCB	  "
    );
}
/*-----------------------------------------------------------*/

void xPortSysTickHandler( void )
{
    uint32_t ulPreviousMask;

    ulPreviousMask = portSET_INTERRUPT_MASK_FROM_ISR();
    {
        /* Increment the RTOS tick. */
        if( xTaskIncrementTick() != pdFALSE )
        {
            /* Pend a context switch. */
            portNVIC_INT_CTRL_REG = portNVIC_PENDSVSET_BIT;
        }
    }
    portCLEAR_INTERRUPT_MASK_FROM_ISR( ulPreviousMask );
}
/*-----------------------------------------------------------*/

/*
 * Setup the systick timer to generate the tick interrupts at the required
 * frequency.
 */
__attribute__( ( weak ) ) void vPortSetupTimerInterrupt( void )
{
    /* Calculate the constants required to configure the tick interrupt. */
#if ( configUSE_TICKLESS_IDLE == 1 )
    {
        ulTimerCountsForOneTick = ( configCPU_CLOCK_HZ / configTICK_RATE_HZ );
        xMaximumPossibleSuppressedTicks = portMAX_24_BIT_NUMBER / ulTimerCountsForOneTick;
        ulStoppedTimerCompensation = portMISSED_COUNTS_FACTOR;
    }
#endif /* configUSE_TICKLESS_IDLE */

    /* Stop and reset the SysTick. */
    portNVIC_SYSTICK_CTRL_REG = 0UL;
    portNVIC_SYSTICK_CURRENT_VALUE_REG = 0UL;

    /* Configure SysTick to interrupt at the requested rate. */
    portNVIC_SYSTICK_LOAD_REG = ( configCPU_CLOCK_HZ / configTICK_RATE_HZ ) - 1UL;
    portNVIC_SYSTICK_CTRL_REG = portNVIC_SYSTICK_CLK_BIT | portNVIC_SYSTICK_INT_BIT | portNVIC_SYSTICK_ENABLE_BIT;

    /* Enable RTC NVIC */
    NVIC_EnableIRQ(RTC_IRQn);
}
/*-----------------------------------------------------------*/

#if ( configUSE_TICKLESS_IDLE == 1 )

extern S_RTC_TIME_DATA_T sWriteRTC, sReadRTC;
extern volatile uint8_t g_u8IsRTCAlarmINT, g_u8IsGPIOINT;
extern void PowerDownFunction(void);

extern volatile uint32_t g_u32WriteRTC_Ticks, g_u32ReadRTC_Ticks;

void TimeAdd(S_RTC_TIME_DATA_T* t, volatile uint32_t *pu32RtcTicks, uint32_t u32rtcTicks)
{
    uint32_t h, m, s, tick;
    int32_t i32Next;

    m = u32rtcTicks / 128 / 60;
    h = m / 60;
    m = m % 60;
    s = u32rtcTicks / 128 % 60;
    tick = u32rtcTicks & 0x7f;

    *pu32RtcTicks += tick;
    t->u32Second += s;
    t->u32Minute += m;
    t->u32Hour += h;

    do
    {
        i32Next = 0;

        if(*pu32RtcTicks >= 128)
        {
            *pu32RtcTicks -= 128;
            t->u32Second += 1;
            i32Next = 1;
        }

        if(t->u32Second >= 60)
        {
            t->u32Second -= 60;
            t->u32Minute += 1;
            i32Next = 1;
        }

        if(t->u32Minute >= 60)
        {
            t->u32Minute -= 60;
            t->u32Hour += 1;
            i32Next = 1;
        }

        if(t->u32Hour >= 24)
        {
            t->u32Hour -= 24;
            t->u32Minute += 1;
            i32Next = 1;
        }
    } while(i32Next);
}

TickType_t SleepProcess(TickType_t x)
{
    if(x == 0)
        return x;

    /* Set RTC alarm accord to x */

    /* Get start time from RTC */
    RTC_GetDateAndTime(&sReadRTC);
    g_u32ReadRTC_Ticks = ((RTC->TIME & RTC_TIME_HZCNT_Msk) >> RTC_TIME_HZCNT_Pos);

    /* Backup RTC start time for GPIO wake up */
    memcpy(&sWriteRTC, &sReadRTC, sizeof(S_RTC_TIME_DATA_T));
    g_u32WriteRTC_Ticks = g_u32ReadRTC_Ticks;

    /* Calculate idle end time for RTC alarm wake up */
    TimeAdd(&sReadRTC, &g_u32ReadRTC_Ticks, x * 128 / 1000 - 1);

    /* Set wake up alarm */
    RTC_SetAlarmDateAndTime(&sReadRTC);
    RTC->TALM |= (RTC->TALM & (~RTC_TALM_HZCNT_Msk)) | (g_u32ReadRTC_Ticks << RTC_TALM_HZCNT_Pos);

    /* Enable RTC alarm interrupt and wake-up function will be enabled also */
    RTC_EnableInt(RTC_INTEN_ALMIEN_Msk);
    GPIO_CLR_INT_FLAG(PB, BIT1 | BIT2 | BIT3 | BIT4 | BIT5);
    NVIC_EnableIRQ(GPB_IRQn);
    g_u8IsGPIOINT = 0;
    PowerDownFunction();

    /* Wake up now. Enable interrupt for process the pending interrupt handler */
    __enable_irq();
    __ISB();
    __DSB();

    /* Just make sure RTC Alarm event to continue */
    while(g_u8IsRTCAlarmINT == 0);
    g_u8IsRTCAlarmINT = 0;

    __disable_irq();
    RTC_DisableInt(RTC_INTEN_ALMIEN_Msk);
    NVIC_DisableIRQ(GPB_IRQn);

    portNVIC_SYSTICK_CTRL_REG &= ~portNVIC_SYSTICK_ENABLE_BIT;

    /* Set the new reload value. */
    portNVIC_SYSTICK_LOAD_REG = 0;

    /* Clear the SysTick count flag and set the count value back to
     * zero. */
    portNVIC_SYSTICK_CURRENT_VALUE_REG = 0UL;

    /* Restart SysTick. */
    portNVIC_SYSTICK_CTRL_REG |= portNVIC_SYSTICK_ENABLE_BIT;

    return x;
}

__attribute__( ( weak ) ) void vPortSuppressTicksAndSleep( TickType_t xExpectedIdleTime )
{
    uint32_t ulReloadValue, ulCompleteTickPeriods;

    /* Stop the SysTick momentarily. The time the SysTick is stopped for
     * is accounted for as best it can be, but using the tickless mode will
     * inevitably result in some tiny drift of the time maintained by the
     * kernel with respect to calendar time. */
    portNVIC_SYSTICK_CTRL_REG &= ~portNVIC_SYSTICK_ENABLE_BIT;

    /* Calculate the reload value required to wait xExpectedIdleTime
     * tick periods. -1 is used because this code will execute part way
     * through one of the tick periods. */
    ulReloadValue = portNVIC_SYSTICK_CURRENT_VALUE_REG + ( ulTimerCountsForOneTick * ( xExpectedIdleTime - 1UL ) );

    //printf("ReloadValue = %x, cnt for 1 tick =%d, idle time=%d\n", ulReloadValue, ulTimerCountsForOneTick, xExpectedIdleTime);

    if( ulReloadValue > ulStoppedTimerCompensation )
    {
        ulReloadValue -= ulStoppedTimerCompensation;
    }

    /* Enter a critical section but don't use the taskENTER_CRITICAL()
     * method as that will mask interrupts that should exit sleep mode. */
    __disable_irq();
    __DSB();
    __ISB();

    /* If a context switch is pending or a task is waiting for the scheduler
     * to be unsuspended then abandon the low power entry. */
    if( eTaskConfirmSleepModeStatus() == eAbortSleep )
    {
        /* Restart from whatever is left in the count register to complete
         * this tick period. */
        portNVIC_SYSTICK_LOAD_REG = portNVIC_SYSTICK_CURRENT_VALUE_REG;

        /* Restart SysTick. */
        portNVIC_SYSTICK_CTRL_REG |= portNVIC_SYSTICK_ENABLE_BIT;

        /* Reset the reload register to the value required for normal tick
         * periods. */
        portNVIC_SYSTICK_LOAD_REG = ulTimerCountsForOneTick - 1UL;

        /* Re-enable interrupts - see comments above the cpsid instruction()
         * above. */
        __enable_irq();
    }
    else
    {
        /* Set the new reload value. */
        portNVIC_SYSTICK_LOAD_REG = ulReloadValue;

        /* Clear the SysTick count flag and set the count value back to
         * zero. */
        portNVIC_SYSTICK_CURRENT_VALUE_REG = 0UL;

        /* Restart SysTick. */
        portNVIC_SYSTICK_CTRL_REG |= portNVIC_SYSTICK_ENABLE_BIT;

        /* Sleep until something happens. */
        xExpectedIdleTime = SleepProcess(xExpectedIdleTime);

        /* Re-enable interrupts to allow the interrupt that brought the MCU
         * out of sleep mode to execute immediately. See comments above
         * __disable_irq() call above. */
        __enable_irq();
        __DSB();
        __ISB();

        /* Disable interrupts again because the clock is about to be stopped
         * and interrupts that execute while the clock is stopped will increase
         * any slippage between the time maintained by the RTOS and calendar
         * time. */
        __disable_irq();
        __DSB();
        __ISB();

        /* Disable the SysTick clock without reading the
         * portNVIC_SYSTICK_CTRL_REG register to ensure the
         * portNVIC_SYSTICK_COUNT_FLAG_BIT is not cleared if it is set. Again,
         * the time the SysTick is stopped for is accounted for as best it can
         * be, but using the tickless mode will inevitably result in some tiny
         * drift of the time maintained by the kernel with respect to calendar
         * time */
        portNVIC_SYSTICK_CTRL_REG = ( portNVIC_SYSTICK_CLK_BIT | portNVIC_SYSTICK_INT_BIT );

        /* Determine if the SysTick clock has already counted to zero and
         * been set back to the current reload value (the reload back being
         * correct for the entire expected idle time) or if the SysTick is yet
         * to count to zero (in which case an interrupt other than the SysTick
         * must have brought the system out of sleep mode). */
        if( ( portNVIC_SYSTICK_CTRL_REG & portNVIC_SYSTICK_COUNT_FLAG_BIT ) != 0 )
        {
            uint32_t ulCalculatedLoadValue;

            /* The tick interrupt is already pending, and the SysTick count
             * reloaded with ulReloadValue. Reset the
             * portNVIC_SYSTICK_LOAD_REG with whatever remains of this tick
             * period. */
            ulCalculatedLoadValue = ( ulTimerCountsForOneTick - 1UL ) - ( ulReloadValue - portNVIC_SYSTICK_CURRENT_VALUE_REG );

            /* Don't allow a tiny value, or values that have somehow
             * underflowed because the post sleep hook did something
             * that took too long. */
            if( ( ulCalculatedLoadValue < ulStoppedTimerCompensation ) || ( ulCalculatedLoadValue > ulTimerCountsForOneTick ) )
            {
                ulCalculatedLoadValue = ( ulTimerCountsForOneTick - 1UL );
            }

            portNVIC_SYSTICK_LOAD_REG = ulCalculatedLoadValue;

            /* As the pending tick will be processed as soon as this
             * function exits, the tick value maintained by the tick is stepped
             * forward by one less than the time spent waiting. */
            ulCompleteTickPeriods = xExpectedIdleTime - 1UL;
        }

        /* Restart SysTick so it runs from portNVIC_SYSTICK_LOAD_REG
         * again, then set portNVIC_SYSTICK_LOAD_REG back to its standard
         * value. */
        portNVIC_SYSTICK_CURRENT_VALUE_REG = 0UL;
        portNVIC_SYSTICK_CTRL_REG |= portNVIC_SYSTICK_ENABLE_BIT;
        vTaskStepTick( ulCompleteTickPeriods );
        portNVIC_SYSTICK_LOAD_REG = ulTimerCountsForOneTick - 1UL;

        /* Exit with interrpts enabled. */
        __enable_irq();
    }
}

#endif /* configUSE_TICKLESS_IDLE */
