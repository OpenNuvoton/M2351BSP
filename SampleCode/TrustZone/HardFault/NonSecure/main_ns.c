/*----------------------------------------------------------------------------
 * Name:    main_ns.c
 * Purpose: Main function non-secure mode
 *----------------------------------------------------------------------------*/

#include <arm_cmse.h>
#include "NuMicro.h"                      /* Device header */

/*----------------------------------------------------------------------------
  NonSecure Callable Functions from Secure Region
 *----------------------------------------------------------------------------*/
extern uint32_t GetSystemCoreClock(void);

void HardFault_Handler(void)__attribute__((noreturn));
/*----------------------------------------------------------------------------
  Main function
 *----------------------------------------------------------------------------*/

/*
   Non-secure Hard Fault is control by secure code.
   If secure code set it to secure, the non-secure hard fault handler is always handled
   by secure hard fault vector.

   If secure code set it to non-secure, the non-secure hard fault handler is handled
   by non-secure hard fault vector. However, if non-secure code access secure region
   and cases hard fault. It is still handled by secure hard fault vector for security.

*/


int main(void)
{
    uint32_t i;
    int32_t i32Ch;

    printf("\n");
    printf("+---------------------------------------------+\n");
    printf("|    Nonsecure code is running ...            |\n");
    printf("+---------------------------------------------+\n");

    /* Call secure API to get system core clock */
    SystemCoreClock = GetSystemCoreClock();
    printf("System core clock = %d\n", SystemCoreClock);

    do
    {
        printf("+-------------------------------------------------------+\n");
        printf("| [0] Write address 0x10040000 to generate hard fault   |\n");
        printf("| [1] Write secure I/O port to generate hard fault      |\n");
        printf("| [2] Read secure SRAM to generate hard fault           |\n");
        printf("| [3] Toggle non-secure I/O                             |\n");
        printf("+-------------------------------------------------------+\n");
        i32Ch = getchar();

        switch(i32Ch)
        {
            case '0':
                /*
                    Non-secure code access non-secure region and cause bus error.
                    This would cause non-secure or secure hard fault,
                    dependent on the setting in AIRCR[13]. (AIRCR is secure register)
                */
                M32(0x10040000) = 0;
                break;
            case '1':
                /*
                    GPIO Port B is configured as secure port. Access it in non-secure code
                    will cause secure hard fault.
                */
                PB0 = 0;
                PB1 = 0;
                break;
            case '2':
                /*
                    0x20000000 is configured as secure SRAM. Access it in non-secure code
                    will cause secure hard fault.
                */
                M32(0x20000000);
                break;
            case '3':
                printf("LED blinking...\n");
                PA10_NS = 0;
                PA11_NS = 1;
                while(1)
                {
                    PA10_NS ^= 1;
                    PA11_NS ^= 1;
                    i = 0x10000;
                    while(i-- > 0);

                }

                //break;
            default:
                break;
        }
    }
    while(1);
}

void HardFault_Handler(void)
{
    PA10_NS = 0;
    printf("Non-secure Hand Fault Handler\n");
    while(1);
}

