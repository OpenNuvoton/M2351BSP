/**************************************************************************//**
 * @file     main_ns.c
 * @version  V1.00
 * @brief    Non-secure sample code for Collaborative Secure Software Development
 *
 * @note
 * Copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#include <arm_cmse.h>
#include "NuMicro.h"                    /* Device header */
#include "cssd_lib.h"                   /* Collaborative Secure Software Development Library header */

void LED_On(uint32_t us);
void LED_Off(uint32_t us);
void SysTick_Handler(void);
/*----------------------------------------------------------------------------
  NonSecure Functions from NonSecure Region
 *----------------------------------------------------------------------------*/
void LED_On(uint32_t us)
{
    (void)us;
    printf("NS LED On call by NS\n");
    PC0_NS = 0;
}

void LED_Off(uint32_t us)
{
    (void)us;
    printf("NS LED Off call by NS\n");
    PC0_NS = 1;
}

/*----------------------------------------------------------------------------
  SysTick IRQ Handler
 *----------------------------------------------------------------------------*/
void SysTick_Handler(void)
{
    static uint32_t u32Ticks;

    switch(u32Ticks++)
    {
        case   0:
            // second developer handle
            LED_On(7u);
            Secure_PA11_LED_On(0u);
            break;
        case 100:
            // second developer handle
            Secure_PA11_LED_Off(0u);
            Secure_PA12_LED_On(0u);
            break;
        case 200:
            // second developer handle
            Secure_PA12_LED_Off(0u);
            break;
        case 300:
            // second developer handle
            LED_Off(7u);
            break;
        case 400:
            // second developer handle
            Secure_PA11_LED_On(0u);
            Secure_PA12_LED_On(0u);
            break;
        case 500:
            // second developer handle
            Secure_PA11_LED_Off(0u);
            Secure_PA12_LED_Off(0u);
            break;
        case 600:
            u32Ticks = 0;
            break;

        default:
            if(u32Ticks > 600)
            {
                u32Ticks = 0;
            }
    }
}

/*----------------------------------------------------------------------------
  Main function
 *----------------------------------------------------------------------------*/
int main(void)
{
    printf("\nNonsecure code is running ...\n");

    /* Init GPIO Port C for non-secure LED control */
    GPIO_SetMode(PC_NS, BIT0, GPIO_MODE_OUTPUT);

    /* Call secure API to get system core clock */
    SystemCoreClock = GetSystemCoreClock();

    /* Generate Systick interrupt each 10 ms */
    SysTick_Config(SystemCoreClock / 100);

    /* Waiting for secure/non-secure SysTick interrupt */
    while(1);
}
