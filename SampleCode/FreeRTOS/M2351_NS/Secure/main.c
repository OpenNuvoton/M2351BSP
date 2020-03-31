/*----------------------------------------------------------------------------
 * Name:    main_s.c
 * Purpose: Main function secure mode
 *----------------------------------------------------------------------------*/

#include <arm_cmse.h>
#include <stdio.h>
#include "M2351.h"                      /* Device header */


/* typedef for NonSecure callback functions */
typedef __attribute__((cmse_nonsecure_call)) int32_t (*NonSecure_funcptr)(uint32_t);

#define FMC_SECURE_BOUNDARY     0x40000UL
#define NON_SECURE_BASE         (0x10000000ul+FMC_SECURE_BOUNDARY) /* Base Address of Non-secure Image */
#define SRAM_SECURE_BOUNDARY    0x10000UL



void SYS_Init(void);
void DEBUG_PORT_Init(void);
void Nonsecure_Init(void);

/*----------------------------------------------------------------------------
  Main function
 *----------------------------------------------------------------------------
  
Secure Region Config of the Template:
  [0x00000000:0x00003FFF] Secure. APROM for Secure. (16KB)
  [0x00004000:0x0003FFFF] Secure Nonsecure Callable. APROM for secure & API (240KB)
  [0x00800000:0x00807BFF] Secure. MKROM (31KB)
  [0x00807C00:0x00807FFF] Secure Nonsecure Callable. API table of MKROM (1KB)
  [0x10040000:0x1007FFFF] Nonsecure. APROM for Nonsecure (256KB)
  [0x20000000:0x2000FBFF] Secure. SRAM (63KB)
  [0x2000FC00:0x2000FFFF] Secure Nonesecure Callable. SRAM for NSC API (1KB)
  [0x30010000:0x3001FFFF] Nonsecure. SRAM for Nonsecure.
  [0x40000000:0x00000000] Secure. For Secure IP.
  [0x50000000:0x00000000] Nonsecure. For Nonsecure IP.

FMC.NSCBOND must be 0x40000 (256KB). The code will set it to 0x40000.
(NOTE: It don't do erase, thus the setting may fail if NSCBOND!=0xffffffff)

SRAM secure setting in SCU must be 64KB secure/64KB nonsecure

All secure settings is located in partition_TC8234.h, except FMC.NSCBOND.



SecureAttrib() could be used to show all region settings of SAU and IDAU.

The secure code will call nonsecure code by callback function.
The secure code will keep secure systick timer interrupt, even the nonsecure code is running.
The Secure Nonsecure Callable API will be located at SRAM 0x2000FC00 by scatter loading file.
The secure code will execute nonsecure code.
The nonsecure code will do a while loop and show messages.
The nonsecure code will call secure funcitons by NSC at SRAM.

UART0 is configured as Nonsecure for debug message in both secure and nonsecure.

The flash download algorithm of keil for secure is "TC8234 8M Flash"
The flash download algorithm of keil for nonsecure is "TC8234_NS 8M Flash"

Build & Debug flow:
 1. Build secure project
 2. Build nonsecure project
 3. Download nonsecure code
 4. Do debug with ICE in secure project. Both secure & nonsecure code can be debug. 



*----------------------------------------------------------------------------*/

__attribute__((cmse_nonsecure_entry))
uint32_t GetSystemCoreClock(void);
__attribute__((cmse_nonsecure_entry))
void SecureWrite(uint32_t u32Addr, uint32_t u32Data);
__attribute__((cmse_nonsecure_entry))
uint32_t SecureRead(uint32_t u32Addr);
void SysTick_Handler(void);
void HardFault_Handlerx(void)__attribute__((noreturn));
/*----------------------------------------------------------------------------
  Secure functions exported to NonSecure application
  Must place in Non-secure Callable
 *----------------------------------------------------------------------------*/
__attribute__((cmse_nonsecure_entry))
uint32_t GetSystemCoreClock(void)
{
    return SystemCoreClock;
}

    
__attribute__((cmse_nonsecure_entry))
void SecureWrite(uint32_t u32Addr, uint32_t u32Data)
{
    M32(u32Addr) = u32Data;
}

__attribute__((cmse_nonsecure_entry))
uint32_t SecureRead(uint32_t u32Addr)
{
    return M32(u32Addr);
}



/*----------------------------------------------------------------------------
  SysTick IRQ Handler
 *----------------------------------------------------------------------------*/
void SysTick_Handler(void)
{
    static uint32_t ticks;

    switch(ticks++)
    {
        case   0:
            PB0 = 0;
            PB1 = 1;
            break;
        case 10:
            PB0 = 1;
            PB1 = 0;
            break;
        case 30:
            PB0 = 0;
            PB1 = 0;
            break;

        default:
            if(ticks > 50)
            {
                ticks = 0;
                PB0 = 1;
                PB1 = 1;
            }
    }
}


int main(void)
{
    uint32_t u32MSP_s, u32PSP_s;
    uint32_t u32MSP_ns, u32PSP_ns;
//    volatile uint32_t NonSecure_ResetHandler;
//    NonSecure_funcptr fp;
//    char ch;

    SYS_UnlockReg();
    SYS_Init();

    /* UART0 is configured as Nonsecure for debug in both secure and nonsecure region */
    DEBUG_PORT_Init();
    
    
    /* The setting of non-secure flash base address must be the same as SAU setting. If it is different to SAU,
       We need to set FMC NSCBOND configuration */
    printf("Non-secure flash base = 0x%08x\n", SCU->FNSADDR);


    /* exercise some core register from Secure Mode */
    u32MSP_s = __get_MSP();
    u32PSP_s = __get_PSP();

    printf("Secure Region Code Stack: MSP(0x%08x) PSP(0x%08x)\n", u32MSP_s, u32PSP_s);

    /* Get the stack of Non-secure region */
    __TZ_set_MSP_NS(M32(NON_SECURE_SRAM_BASE));
    u32MSP_ns = __TZ_get_MSP_NS();
    __TZ_set_PSP_NS(M32(NON_SECURE_SRAM_BASE));
    u32PSP_ns = __TZ_get_PSP_NS();

    printf("Non-secure Region Code Stack: MSP(0x%08x) PSP(0x%08x)\n", u32MSP_ns, u32PSP_ns);
    
    
    Nonsecure_Init();
    
    while(1);

}


void Nonsecure_Init(void)
{
    NonSecure_funcptr fp;

    /* SCB_NS.VTOR points to the Non-secure vector table base address. */
    SCB_NS->VTOR = NON_SECURE_BASE;

    /* 1st Entry in the vector table is the Non-secure Main Stack Pointer. */
    __TZ_set_MSP_NS(*((uint32_t *)SCB_NS->VTOR));      /* Set up MSP in Non-secure code */

    /* 2nd entry contains the address of the Reset_Handler (CMSIS-CORE) function */
    fp = ((NonSecure_funcptr)(*(((uint32_t *)SCB_NS->VTOR) + 1)));

    /* Clear the LSB of the function address to indicate the function-call
       will cause a state switch from Secure to Non-secure */
    fp = cmse_nsfptr_create(fp);

    /* Check if the Reset_Handler address is in Non-secure space */
    /* Check if the Reset_Handler address is in Non-secure space */
    if(cmse_is_nsfptr(fp) && (((uint32_t)fp&0xf0000000) == 0x10000000))
    {
        printf("Execute non-secure code ...\n");
        fp(0);                             /* Non-secure function call */
    }
    else
    {
        /* Something went wrong */
        printf("No code in non-secure region!\n");
        printf("CPU will halted at non-secure state\n");
        
        __TZ_set_MSP_NS(0x30017FE0);
        
        /* Try to halted in non-secure state (SRAM) */
        M32(0x30017FF0) = 0xe7fee7ff;
        fp = (NonSecure_funcptr)0x30017FF0;
        fp(0);
        
        
        while(1);
    }}



void SYS_Init(void)
{
//    int32_t i;
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    
    /* Enable PLL */
    CLK->PLLCTL = CLK_PLLCTL_128MHz_HIRC;
    /* Waiting for PLL stable */
    while((CLK->STATUS & CLK_STATUS_PLLSTB_Msk) == 0);
    
    /* Set HCLK divider to 2 */
    CLK->CLKDIV0 = (CLK->CLKDIV0&(~CLK_CLKDIV0_HCLKDIV_Msk)) | 1;
    
    /* Switch HCLK clock source to PLL */
    CLK->CLKSEL0 = CLK_CLKSEL0_HCLKSEL_PLL;

    /* Enable All IP clock, so we can use them in non-secure code. */
    CLK->AHBCLK = 0xfffffffful;
    CLK->APBCLK0 = 0xfffffffful;
    CLK->APBCLK1 = 0xfffffffful;

    /* Select IP clock source */
    CLK->CLKSEL1 = CLK_CLKSEL1_UART0SEL_HIRC;
    CLK->CLKSEL3 = CLK_CLKSEL3_UART5SEL_HIRC;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    //SystemCoreClockUpdate();
    PllClock        = 64000000;            // PLL
    SystemCoreClock = 64000000;            // HCLK
    CyclesPerUs     = 64000000 / 1000000;  // For SYS_SysTickDelay()

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set multi-function pins for UART0 RXD and TXD */
    SYS->GPD_MFPL = UART0_RXD_PB12 | UART0_TXD_PB13;
    

#if 1 // ETM MFP
    SYS->GPE_MFPH &= ~(TRACE_CLK_PE12_Msk|TRACE_DATA0_PE11_Msk|TRACE_DATA1_PE10_Msk|TRACE_DATA2_PE9_Msk|TRACE_DATA3_PE8_Msk);
    SYS->GPE_MFPH |= TRACE_CLK_PE12|TRACE_DATA0_PE11|TRACE_DATA1_PE10|TRACE_DATA2_PE9|TRACE_DATA3_PE8;
#endif

    /* NOTE: Most peripherals are configured as non-secure in "partion_M2351.h" */

}

void DEBUG_PORT_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset IP */

    /* Configure UART0 and set UART0 Baudrate */
    DEBUG_PORT->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC, 115200);
    DEBUG_PORT->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}


void HardFault_Handlerx(void)
{
    printf("Secure Hard Fault Handler\n");
    while(1);
}
