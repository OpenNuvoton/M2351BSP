/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Secure sample code for TrustZone
 *
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2017-2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#include <arm_cmse.h>
#include <stdio.h>
#include "NuMicro.h"                      /* Device header */


#define dbg     printf
//#define dbg(...)

#define NEXT_BOOT_BASE  0x10040000
#define JUMP_HERE       0xe7fee7ff      /* Instruction Code of "B ." */


/* typedef for NonSecure callback functions */
typedef __NONSECURE_CALL int32_t (*NonSecure_funcptr)(uint32_t);


void SYS_Init(void);
void UART0_Init(void);
void Boot_Init(uint32_t u32BootBase);
char GetChar(void);
__NONSECURE_ENTRY
uint32_t GetSystemCoreClock(void);
void HardFault_Handler(void)__attribute__((noreturn));

/*----------------------------------------------------------------------------
  Secure functions exported to NonSecure application
  Must place in Non-secure Callable
 *----------------------------------------------------------------------------*/
__NONSECURE_ENTRY
uint32_t GetSystemCoreClock(void)
{
    SystemCoreClockUpdate();
    return SystemCoreClock;
}

/*----------------------------------------------------------------------------
  Main function
 *----------------------------------------------------------------------------*/
int main(void)
{
    uint32_t u32MSP_ns, u32PSP_ns;
    char ch;

    SYS_UnlockReg();
    SYS_Init();

    /* UART0 is configured as Nonsecure for debug in both secure and nonsecure region */
    UART0_Init();

    /* Set PA to non-secure */
    SCU_SET_IONSSET(SCU_IONSSET_PA_Msk);

    /* Init PA for non-secure LED control */
    PA_NS->MODE = (GPIO_MODE_OUTPUT << 10 * 2) | (GPIO_MODE_OUTPUT << 11 * 2);
    PA10_NS = 0;
    PA11_NS = 1;

    /*
        NOTE: It is necessary to prepare non-secure vector table before setting "Hard Fault" exception to be
              non-secure.
    */

    /* SCB_NS.VTOR points to the Non-secure vector table base address. */
    SCB_NS->VTOR = FMC_NON_SECURE_BASE;

    /* 1st Entry in the vector table is the Non-secure Main Stack Pointer. */
    __TZ_set_MSP_NS(*((uint32_t *)SCB_NS->VTOR));      /* Set up MSP in Non-secure code */

    /* Get the stack of Non-secure region */
    u32MSP_ns = __TZ_get_MSP_NS();
    u32PSP_ns = __TZ_get_PSP_NS();

    dbg("Non-secure Region Code Stack: MSP(0x%08x) PSP(0x%08x)\n", u32MSP_ns, u32PSP_ns);


    // Set Hard Fault to non-secure
    //SCB->AIRCR = (0x05FA << 16) | ((SCB->AIRCR&0xfffful) | (1 << SCB_AIRCR_BFHFNMINS_Pos));

    // Set Hard Fault to secure
    //SCB->AIRCR = (0x05FA << 16) | ((SCB->AIRCR&0xfffful) & (~(1 << SCB_AIRCR_BFHFNMINS_Pos)));

    /*
        Warning: It is not recommended to set hard fault handler to be non-secure.
        By default, the hard fault handler should be secure.
    */

    do
    {
        if(SCB->AIRCR & SCB_AIRCR_BFHFNMINS_Msk)
        {
            dbg("Non-secure Hard Fault handled by Non-secure code.\n");
        }
        else
        {
            dbg("Non-secure Hard Fault handled by secure code.\n");
        }
        dbg("+-----------------------------------------------------------+\n");
        dbg("| [0] Go non-secure code                                    |\n");
        dbg("| [1] Write 0 to address 0x0 to generate hard fault         |\n");
        dbg("| [2] Write 0 to address 0x10040000 to generate hard fault  |\n");
        dbg("| [3] Write 0 to address 0x20040000 to generate hard fault  |\n");
        dbg("+-----------------------------------------------------------+\n");
        ch = GetChar();

        switch(ch)
        {
            case '0':
                PA10_NS = 1;
                Boot_Init(NEXT_BOOT_BASE);
                break;
            case '1':
                M32(0) = 0;
                break;
            case '2':
                M32(0x10040000) = 0;
                break;
            case '3':
                M32(0x20040000) = 0;
                break;
            default:
                break;
        }
    }
    while(1);

}



/*----------------------------------------------------------------------------
    Boot_Init function is used to jump to next boot code.
 *----------------------------------------------------------------------------*/
void Boot_Init(uint32_t u32BootBase)
{
    NonSecure_funcptr fp;

    /* SCB_NS.VTOR points to the Non-secure vector table base address. */
    SCB_NS->VTOR = u32BootBase;

    /* 1st Entry in the vector table is the Non-secure Main Stack Pointer. */
    __TZ_set_MSP_NS(*((uint32_t *)SCB_NS->VTOR));      /* Set up MSP in Non-secure code */

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
    uint32_t u32TimeOutCnt;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable PLL */
    CLK->PLLCTL = CLK_PLLCTL_128MHz_HIRC;
    /* Waiting for PLL stable */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while((CLK->STATUS & CLK_STATUS_PLLSTB_Msk) == 0)
        if(--u32TimeOutCnt == 0) break;

    /* Set HCLK divider to 2 */
    CLK->CLKDIV0 = (CLK->CLKDIV0 & (~CLK_CLKDIV0_HCLKDIV_Msk)) | 1;

    /* Switch HCLK clock source to PLL */
    CLK->CLKSEL0 = CLK_CLKSEL0_HCLKSEL_PLL;

    /* Enable IP clock */
    CLK->APBCLK0 |= CLK_APBCLK0_UART0CKEN_Msk;

    /* Select IP clock source */
    CLK->CLKSEL1 = CLK_CLKSEL1_UART0SEL_HIRC;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    //SystemCoreClockUpdate();
    PllClock        = 128000000;            // PLL
    SystemCoreClock = 64000000;             // HCLK
    CyclesPerUs     = 64000000 / 1000000;   // For CLK_SysTickDelay()

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
    /* Reset IP */

    /* Configure UART0 and set UART0 Baudrate */
    UART0_NS->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC, 115200);
    UART0_NS->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}


void HardFault_Handler(void)
{
    PB0 = 0;
    dbg("Secure Hard Fault Handler\n");
    while(1);
}
