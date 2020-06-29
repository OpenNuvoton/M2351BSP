/*
    FreeRTOS V9.0.0 - Copyright (C) 2016 Real Time Engineers Ltd.
    All rights reserved

    VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation >>>> AND MODIFIED BY <<<< the FreeRTOS exception.

    ***************************************************************************
    >>!   NOTE: The modification to the GPL is included to allow you to     !<<
    >>!   distribute a combined work that includes FreeRTOS without being   !<<
    >>!   obliged to provide the source code for proprietary components     !<<
    >>!   outside of the FreeRTOS kernel.                                   !<<
    ***************************************************************************

    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE.  Full license text is available on the following
    link: http://www.freertos.org/a00114.html

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS provides completely free yet professionally developed,    *
     *    robust, strictly quality controlled, supported, and cross          *
     *    platform software that is more than just the market leader, it     *
     *    is the industry's de facto standard.                               *
     *                                                                       *
     *    Help yourself get started quickly while simultaneously helping     *
     *    to support the FreeRTOS project by purchasing a FreeRTOS           *
     *    tutorial book, reference manual, or both:                          *
     *    http://www.FreeRTOS.org/Documentation                              *
     *                                                                       *
    ***************************************************************************

    http://www.FreeRTOS.org/FAQHelp.html - Having a problem?  Start by reading
    the FAQ page "My application does not run, what could be wrong?".  Have you
    defined configASSERT()?

    http://www.FreeRTOS.org/support - In return for receiving this top quality
    embedded software for free we request you assist our global community by
    participating in the support forum.

    http://www.FreeRTOS.org/training - Investing in training allows your team to
    be as productive as possible as early as possible.  Now you can receive
    FreeRTOS training directly from Richard Barry, CEO of Real Time Engineers
    Ltd, and the world's leading authority on the world's leading RTOS.

    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool, a DOS
    compatible FAT file system, and our tiny thread aware UDP/IP stack.

    http://www.FreeRTOS.org/labs - Where new FreeRTOS products go to incubate.
    Come and try FreeRTOS+TCP, our new open source TCP/IP stack for FreeRTOS.

    http://www.OpenRTOS.com - Real Time Engineers ltd. license FreeRTOS to High
    Integrity Systems ltd. to sell under the OpenRTOS brand.  Low cost OpenRTOS
    licenses offer ticketed support, indemnification and commercial middleware.

    http://www.SafeRTOS.com - High Integrity Systems also provide a safety
    engineered and independently SIL3 certified version for use in safety and
    mission critical applications that require provable dependability.

    1 tab == 4 spaces!
*/

/*-----------------------------------------------------------
 * Simple GPIO (parallel port) IO routines.
 *-----------------------------------------------------------*/

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"

/* Hardware includes. */
#include "M2351.h"

/* Standard demo include. */
#include "partest.h"


/* Shift the LED bit into the correct position within the POW register to
perform the desired operation. */
#define partstON_SHIFT	( 16UL )
#define partstOFF_SHIFT	( 0UL )

/*-----------------------------------------------------------*/

void vParTestInitialise( void )
{
    
    SYS_UnlockReg();
    
    /* Enable PLL */
    CLK->PLLCTL = CLK_PLLCTL_128MHz_HIRC;
    /* Waiting for PLL stable */
    while((CLK->STATUS & CLK_STATUS_PLLSTB_Msk) == 0);
    
    /* Set HCLK divider to 2 */
    CLK->CLKDIV0 = (CLK->CLKDIV0&(~CLK_CLKDIV0_HCLKDIV_Msk)) | 1;
    
    /* Switch HCLK clock source to PLL */
    CLK->CLKSEL0 = CLK_CLKSEL0_HCLKSEL_PLL;
    
    
    CLK->APBCLK0 |= CLK_APBCLK0_UART0CKEN_Msk;
    CLK->CLKSEL1 |= (CLK->CLKSEL1 & (~CLK_CLKSEL1_UART0SEL_Msk)) | CLK_CLKSEL1_UART0SEL_HIRC;
    
    
    PllClock        = 64000000;            // PLL
    SystemCoreClock = 64000000 / 1;        // HCLK
    CyclesPerUs     = 64000000 / 1000000;  // For CLK_SysTickDelay()

    /* Set multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & (~(UART0_RXD_PB12_Msk | UART0_TXD_PB13_Msk))) | UART0_RXD_PB12 | UART0_TXD_PB13;
    
    /* Configure Debug port */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC, 115200);
    UART0->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;

    /* LED IO PA11*/
    PA->MODE = (PA->MODE & (~(3ul << 11*2))) | GPIO_MODE_OUTPUT << 11*2;

#if 1 // ETM MFP
    SYS->GPE_MFPH &= ~(TRACE_CLK_PE12_Msk|TRACE_DATA0_PE11_Msk|TRACE_DATA1_PE10_Msk|TRACE_DATA2_PE9_Msk|TRACE_DATA3_PE8_Msk);
    SYS->GPE_MFPH |= TRACE_CLK_PE12|TRACE_DATA0_PE11|TRACE_DATA1_PE10|TRACE_DATA2_PE9|TRACE_DATA3_PE8;
#endif

#if 0 //Enable CLK output
    /* CKO = clock source / 2^(u32ClkDiv + 1) */
    CLK->CLKOCTL = CLK_CLKOCTL_CLKOEN_Msk | 3 ;

    /* Enable CKO clock source */
    CLK->APBCLK0 |= CLK_APBCLK0_CLKOCKEN_Msk;

    /* Select CKO clock source */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & (~CLK_CLKSEL1_CLKOSEL_Msk)) | CLK_CLKSEL1_CLKOSEL_HCLK;
		
    SYS->GPE_MFPH = SYS_GPE_MFPH_PE8MFP_CLKO;
#endif

    
    printf("\b\b\b\b\b\b\n\nFreeRTOS ...\n");
    
}
/*-----------------------------------------------------------*/

void vParTestSetLED( unsigned long ulLED, signed portBASE_TYPE xValue )
{
    (void)ulLED;
    if( xValue == pdTRUE )
    {
        /* Turn the LED on. */			
        PA11 = 0;
    }
    else
    {
        /* Turn the LED off. */			
        PA11 = 1;
    }
}
/*-----------------------------------------------------------*/

void vParTestToggleLED( unsigned long ulLED )
{
    (void)ulLED;
    PA11 ^= 1;
}
/*-----------------------------------------------------------*/

