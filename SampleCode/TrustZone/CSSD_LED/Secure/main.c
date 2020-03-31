/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Secure sample code for Collaborative Secure Software Development
 *
 * @note
 * Copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#include <arm_cmse.h>
#include <stdio.h>
#include "NuMicro.h"                      /* Device header */
#include "partition_M2351.h"

#define NEXT_BOOT_BASE  0x10040000
#define JUMP_HERE       0xe7fee7ff      /* Instruction Code of "B ." */

/* typedef for NonSecure callback functions */
typedef __NONSECURE_CALL int32_t (*NonSecure_funcptr)(uint32_t);


__NONSECURE_ENTRY
int32_t Secure_PA11_LED_On(uint32_t num);
__NONSECURE_ENTRY
int32_t Secure_PA11_LED_Off(uint32_t num);
__NONSECURE_ENTRY
int32_t Secure_PA12_LED_On(uint32_t num);
__NONSECURE_ENTRY
int32_t Secure_PA12_LED_Off(uint32_t num);
__NONSECURE_ENTRY
int32_t Secure_PA13_LED_On(uint32_t num);
__NONSECURE_ENTRY
int32_t Secure_PA13_LED_Off(uint32_t num);
__NONSECURE_ENTRY
uint32_t GetSystemCoreClock(void);
int32_t LED_On(void);
int32_t LED_Off(void);
void SysTick_Handler(void);
/*----------------------------------------------------------------------------
  Secure functions exported to NonSecure application
  Must place in Non-secure Callable
 *----------------------------------------------------------------------------*/
__NONSECURE_ENTRY
int32_t Secure_PA11_LED_On(uint32_t num)
{
    (void)num;
    printf("Secure PA11 LED On call by secure\n");
    PA11 = 0;
    return 0;
}

__NONSECURE_ENTRY
int32_t Secure_PA11_LED_Off(uint32_t num)
{
    (void)num;
    printf("Secure PA11 LED Off call by secure\n");
    PA11 = 1;
    return 1;
}

__NONSECURE_ENTRY
int32_t Secure_PA12_LED_On(uint32_t num)
{
    (void)num;
    printf("Secure PA12 LED On call by secure\n");
    PA12 = 0;
    return 0;
}

__NONSECURE_ENTRY
int32_t Secure_PA12_LED_Off(uint32_t num)
{
    (void)num;
    printf("Secure PA12 LED Off call by secure\n");
    PA12 = 1;
    return 1;
}

__NONSECURE_ENTRY
int32_t Secure_PA13_LED_On(uint32_t num)
{
    (void)num;
    printf("Secure PA13 LED On call by secure\n");
    PA13 = 0;
    return 0;
}

__NONSECURE_ENTRY
int32_t Secure_PA13_LED_Off(uint32_t num)
{
    (void)num;
    printf("Secure PA13 LED Off call by secure\n");
    PA13 = 1;
    return 1;
}

__NONSECURE_ENTRY
uint32_t GetSystemCoreClock(void)
{
    printf("System core clock = %d.\n", SystemCoreClock);
    return SystemCoreClock;
}

int32_t LED_On(void)
{
    printf("Secure/Non-secure LED On call by Secure\n");
    PA10 = 0;
    PC1_NS = 0;
    return 0;
}

int32_t LED_Off(void)
{
    printf("Secure/Non-secure LED Off call by Secure\n");
    PA10 = 1;
    PC1_NS = 1;
    return 1;
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
            LED_On();
            break;
        case  50:
            LED_Off();
            break;
        case 100:
            LED_On();
            break;
        case 150:
            LED_Off();
            break;
        case 200:
            LED_On();
            break;
        case 250:
            LED_Off();
            break;
        case 300:
            LED_On();
            break;
        case 350:
            LED_Off();
            break;
        case 400:
            LED_On();
            break;
        case 450:
            LED_Off();
            break;
        case 500:
            LED_On();
            break;
        case 550:
            LED_Off();
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

void SYS_Init(void);
void DEBUG_PORT_Init(void);
void Nonsecure_Init(void);

/*----------------------------------------------------------------------------
  Main function
 *----------------------------------------------------------------------------*/
int main(void)
{
    SYS_UnlockReg();

    SYS_Init();

    /* UART is configured as non-secure for debug in both secure and non-secure region */
    DEBUG_PORT_Init();

    printf("Secure code is running ...\n");

    /* Init GPIO Port A for secure LED control */
    GPIO_SetMode(PA, BIT13 | BIT12 | BIT11 | BIT10, GPIO_MODE_OUTPUT);

    /* Init GPIO Port C for non-secure LED control */
    GPIO_SetMode(PC_NS, BIT1, GPIO_MODE_OUTPUT);

    /* Generate Systick interrupt each 10 ms */
    SysTick_Config(SystemCoreClock / 100);

    Nonsecure_Init();

    do
    {
        __WFI();
    }
    while(1);
}

void Nonsecure_Init(void)
{
    NonSecure_funcptr fp;

    /* SCB_NS.VTOR points to the Non-secure vector table base address. */
    SCB_NS->VTOR = NEXT_BOOT_BASE;

    /* 1st Entry in the vector table is the Non-secure Main Stack Pointer. */
    __TZ_set_MSP_NS(*((uint32_t *)SCB_NS->VTOR)); /* Set up MSP in Non-secure code */

    /* 2nd entry contains the address of the Reset_Handler (CMSIS-CORE) function */
    fp = ((NonSecure_funcptr)(*(((uint32_t *)SCB_NS->VTOR) + 1)));

    /* Clear the LSB of the function address to indicate the function-call
       will cause a state switch from Secure to Non-secure */
    fp = cmse_nsfptr_create(fp);

    /* Check if the Reset_Handler address is in Non-secure space */
    if(cmse_is_nsfptr(fp) && (((uint32_t)fp & 0xf0000000) == 0x10000000))
    {
        printf("Execute non-secure code ...\n");
        fp(0); /* Non-secure function call */
    }
    else
    {
        /* Something went wrong */
        printf("No code in non-secure region!\n");
        printf("CPU will halted at non-secure state\n");

        /* Set nonsecure MSP in nonsecure region */
        __TZ_set_MSP_NS(NON_SECURE_SRAM_BASE + 512);

        /* Try to halted in non-secure state (SRAM) */
        M32(NON_SECURE_SRAM_BASE) = JUMP_HERE;
        fp = (NonSecure_funcptr)(NON_SECURE_SRAM_BASE + 1);
        fp(0);

        while(1);
    }
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable PLL */
    CLK->PLLCTL = CLK_PLLCTL_128MHz_HIRC;

    /* Waiting for PLL stable */
    while((CLK->STATUS & CLK_STATUS_PLLSTB_Msk) == 0);

    /* Set HCLK divider to 2 */
    CLK->CLKDIV0 = (CLK->CLKDIV0 & (~CLK_CLKDIV0_HCLKDIV_Msk)) | 1;

    /* Switch HCLK clock source to PLL */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLKSEL_Msk)) | CLK_CLKSEL0_HCLKSEL_PLL;

    /* Enable UART clock */
    CLK->APBCLK0 |= CLK_APBCLK0_UART0CKEN_Msk;

    /* Select UART clock source */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & (~CLK_CLKSEL1_UART0SEL_Msk)) | CLK_CLKSEL1_UART0SEL_HIRC;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    //SystemCoreClockUpdate();
    PllClock        = 128000000;           // PLL
    SystemCoreClock = 64000000 / 1;        // HCLK
    CyclesPerUs     = 64000000 / 1000000;  // For SYS_SysTickDelay()

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & (~(UART0_RXD_PB12_Msk | UART0_TXD_PB13_Msk))) | UART0_RXD_PB12 | UART0_TXD_PB13;
}

void DEBUG_PORT_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Configure UART and set UART Baudrate */
    DEBUG_PORT->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC, 115200);
    DEBUG_PORT->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}
