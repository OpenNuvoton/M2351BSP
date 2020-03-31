/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Show MKROM Non-secure API in Secure region.
 *
 * Copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <arm_cmse.h>
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"
#include "partition_M2351.h"


#define NEXT_BOOT_BASE  0x10040000
#define JUMP_HERE       0xe7fee7ff      /* Instruction Code of "B ." */

/* typedef for NonSecure callback functions */
typedef __NONSECURE_CALL int32_t (*NonSecure_funcptr)(uint32_t);

void Nonsecure_Init(void);
uint32_t GetSystemCoreClock(void);
void SYS_Init(void);
void UART_Init(void);

/*----------------------------------------------------------------------------
  Secure function for NonSecure callbacks exported to NonSecure application
  Must place in Non-secure Callable
 *----------------------------------------------------------------------------*/
__NONSECURE_ENTRY
uint32_t GetSystemCoreClock(void)
{
    return SystemCoreClock;
}


void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Set PLL frequency */
    CLK->PLLCTL = CLK_PLLCTL_128MHz_HIRC;

    /* Waiting for PLL stable */
    CLK_WaitClockReady(CLK_STATUS_PLLSTB_Msk);

    /* Select HCLK clock source as PLL and HCLK source divider as 2 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_PLL, CLK_CLKDIV0_HCLK(2));

    /* Set SysTick source to HCLK/2*/
    CLK_SetSysTickClockSrc(CLK_CLKSEL0_STCLKSEL_HCLK_DIV2);
    
    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & (~(UART0_RXD_PB12_Msk | UART0_TXD_PB13_Msk))) | UART0_RXD_PB12 | UART0_TXD_PB13;
}

void UART_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure UART and set UART Baudrate */
    DEBUG_PORT->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC, 115200);
    DEBUG_PORT->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}

/*----------------------------------------------------------------------------
  Main function
 *----------------------------------------------------------------------------*/
int main(void)
{
    uint32_t u32Addr;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* UART is configured as Non-secure for debug in both secure and Non-secure region */
    UART_Init();

    printf("\n\n");
    printf("+--------------------------------------------------------+\n");
    printf("|    MKROM Non-secure API Sample Code in Secure Region   |\n");
    printf("+--------------------------------------------------------+\n\n");

    
    /* The setting of Non-secure flash base address must be the same as SAU setting. */
    printf("Maximum APROM flash size = %d bytes.\n", BL_EnableFMC());
    printf("Non-secure flash base = %d (0x%08x).\n", BL_GetNSBoundary(), SCU->FNSADDR);
    if(SCU->FNSADDR != FMC_SECURE_ROM_SIZE)
    {
        printf("Set Non-secure base address fail!\n");
        while(1) {}
    }
    printf("\n");

    
    /* Show basic chip information */
    printf("Basic chip info:\n");
    printf("PID:    0x%08x\n", BL_ReadPID());
    printf("UID-0:  0x%08x\n", BL_ReadUID(0));
    printf("UID-1:  0x%08x\n", BL_ReadUID(1));
    printf("UID-2:  0x%08x\n", BL_ReadUID(2));
    printf("UCID-0: 0x%08x\n", BL_ReadUCID(0));
    printf("UCID-1: 0x%08x\n", BL_ReadUCID(1));
    printf("UCID-2: 0x%08x\n", BL_ReadUCID(2));
    printf("UCID-3: 0x%08x\n", BL_ReadUCID(3));
    printf("\n");
    
    
    /* Prepare Non-secure flash data */
    u32Addr = 0x10070000UL;
    printf("Page erase [0x%08x] ... ", u32Addr);
    if(BL_FlashPageErase(u32Addr))
    {
        printf("Fail!\n");
        while(1) {}
    }
    else
    {
        printf("Ok.\nProgram and verify data on [0x%08x] ... ", u32Addr);
        BL_FlashWrite(u32Addr, 0x12345678UL);
        if(BL_FlashRead(u32Addr) == 0x12345678UL)
        {
            printf("Pass.\n");
        }
        else
        {
            printf("Fail!\n");
            while(1) {}
        }
    }
    printf("\n");

    
    /* Jump to perform MKROM Non-secure API in Non-secure region */
    printf("Jump to perform MKROM Non-secure API in Non-secure region.\n\n");
    //printf("Hit any key, then jump to perform MKROM Non-secure API in Non-secure region.\n\n");
    //getchar();
    
    Nonsecure_Init(); /* Jump to Non-secure code */
    
    while(1) {}
}

void Nonsecure_Init(void)
{
    NonSecure_funcptr fp;

    /* SCB_NS.VTOR points to the Non-secure vector table base address. */
    SCB_NS->VTOR = NEXT_BOOT_BASE;

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
        printf("Execute Non-secure code ...\n");
        fp(0); /* Non-secure function call */
    }
    else
    {
        /* Something went wrong */
        printf("No code in Non-secure region!\n");
        printf("CPU will halted at Non-secure state\n");

        /* Set nonsecure MSP in nonsecure region */
        __TZ_set_MSP_NS(NON_SECURE_SRAM_BASE + 512);

        /* Try to halted in Non-secure state (SRAM) */
        M32(NON_SECURE_SRAM_BASE) = JUMP_HERE;
        fp = (NonSecure_funcptr)(NON_SECURE_SRAM_BASE + 1);
        fp(0);

        while(1) {}
    }
}

/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/
