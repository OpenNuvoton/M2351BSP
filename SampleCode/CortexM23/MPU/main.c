/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Demonstrate the usage of Cortex-M23 MPU.
 *
 *
 * @copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

void SYS_Init(void);
void UART_Init(void);
void HardFault_Handler(void);
uint32_t ReadMemCore(uint32_t address);
void MPU_Test(void);


/* MPU Region Base Address Register: Shareablity Definition */
#define MPU_RBAR_SH_NON_SHAREABLE        (0x0UL << MPU_RBAR_SH_Pos)
#define MPU_RBAR_SH_OUTER_SHAREABLE      (0x1UL << MPU_RBAR_SH_Pos)
#define MPU_RBAR_SH_INNER_SHAREABLE      (0x2UL << MPU_RBAR_SH_Pos)

/* MPU Region Base Address Register: Access Permission Definition */
#define MPU_RBAR_AP_PRI_RW_USER_NO       0x0UL
#define MPU_RBAR_AP_PRI_RW_USER_RW       0x2UL
#define MPU_RBAR_AP_PRI_RO_USER_NO       0x4UL
#define MPU_RBAR_AP_PRI_RO_USER_RO       0x6UL

/* MPU Attribute Indirection Register: Memory Attribute Definition */
/* Normal Memory */
#define NORMAL_MEM_OUTER_WT_T            0x00UL /* Outer Write-Through transient (RW!=0b00) */
#define NORMAL_MEM_OUTER_NC              0x40UL /* Outer Non-cacheable */
#define NORMAL_MEM_OUTER_WB_T            0x40UL /* Outer Write-Back Transient (RW!=0b00) */
#define NORMAL_MEM_OUTER_WT_NT           0x80UL /* Outer Write-Through Non-transient */
#define NORMAL_MEM_OUTER_WB_NT           0xC0UL /* Outer Write-Back Non-transient */
#define NORMAL_MEM_INNER_WT_T            0x00UL /* Inner Write-Through Transient (RW!=0b00) */
#define NORMAL_MEM_INNER_NC              0x04UL /* Inner Non-cacheable */
#define NORMAL_MEM_INNER_WB_T            0x04UL /* Inner Write-Back Transient (RW!=0b00) */
#define NORMAL_MEM_INNER_WT_NT           0x08UL /* Inner Write-Through Non-transient */
#define NORMAL_MEM_INNER_WB_NT           0x0CUL /* Inner Write-Back Non-transient */
/* Device Memory */
#define DEVICE_MEM_NG_NR_NE              0x00UL /* Non-Gathering, Non-Reordering, Non-Early-Write-Acknowledgement */
#define DEVICE_MEM_NG_NR_E               0x04UL /* Non-Gathering, Non-Reordering, Early-Write-Acknowledgement */
#define DEVICE_MEM_NG_R_E                0x08UL /* Non-Gathering, Reordering, Early-Write-Acknowledgement */
#define DEVICE_MEM_G_R_E                 0x0CUL /* Gathering, Reordering, Early-Write-Acknowledgement */




uint32_t ReadMemCore(uint32_t address)
{
    __IO uint32_t val = 0;
    uint32_t *a = (uint32_t*) address;
    val = *a;

    return val;
}

/* NOTE: Cortex-M23 does not supported the MemManage exception.
         It can only use the HardFault exception for MemManage fault. */
void HardFault_Handler(void)
{
    /* NOTE1: Disable MPU to allow simple return from HardFault handler
              MemManage fault typically indicates code failure, and would
              be resolved by reset or terminating faulty thread in OS.
       NOTE2: The code set MPU->CTRL below will allow the code touch
              illegal address to be executed after return from
              HardFault_Handler(). If this line is comment out, this code
              will keep enter HardFault_Handler() */
    MPU->CTRL = 0x0;

    printf("\n Memory Fault !!\n");
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Switch HCLK clock source to HIRC */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable PLL */
    CLK->PLLCTL = CLK_PLLCTL_128MHz_HIRC;

    /* Waiting for PLL stable */
    CLK_WaitClockReady(CLK_STATUS_PLLSTB_Msk);

    /* Select HCLK clock source as PLL and HCLK source divider as 2 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_PLL, CLK_CLKDIV0_HCLK(2));

    /* Set SysTick source to HCLK/2*/
    CLK_SetSysTickClockSrc(CLK_CLKSEL0_STCLKSEL_HCLK_DIV2);

    /* Configure UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & (~(UART0_RXD_PB12_Msk | UART0_TXD_PB13_Msk))) | UART0_RXD_PB12 | UART0_TXD_PB13;
}

void UART_Init()
{
    UART_Open(UART0, 115200);
}

void MPU_Test(void)
{
    int32_t i32TestItem = 0;
    uint32_t u32MemAttr0;

    /*-------------------------------
       Configure MPU memory attribute
      -------------------------------*/
    /*
      Attribute 0
      Memory Type = Normal
      Attribute = Outer Non-cacheable, Inner Non-cacheable
    */
    u32MemAttr0 = NORMAL_MEM_OUTER_NC | NORMAL_MEM_INNER_NC;

    /* Configure Memory attribute */
    MPU->MAIR0 |= u32MemAttr0;

    /*------------------------------
       Configure MPU memory regions
      ------------------------------*/
    /*
      Region 1 (Flash Memory Space)
      Start address = 0x0
      Shareablity = Non-shareable
      Size = 128KB
      Permission = Full access
    */

    /* Select Region 1 */
    MPU->RNR = 1;
    /* Base address = Base address :OR: Attribute = Full access :OR: Shareablity = Non-shareable */
    MPU->RBAR = ((0x00000000 & MPU_RBAR_BASE_Msk) | MPU_RBAR_AP_PRI_RW_USER_RW | MPU_RBAR_SH_NON_SHAREABLE);
    /* Limit address = Limit address :OR: Attribute Index = 0 :OR: ENABLE */
    MPU->RLAR = ((0x0001FFFF & MPU_RLAR_LIMIT_Msk) | (0x0UL << MPU_RLAR_AttrIndx_Pos) | MPU_RLAR_EN_Msk);

    /*
      Region 2 (SRAM Memory Space)
      Start address = 0x20000000
      Shareablity = Non-shareable
      Size = 16KB
      Permission = Full access
    */

    /* Select Region 2 */
    MPU->RNR = 2;
    /* Base address = Base address :OR: Attribute = Full access :OR: Shareablity = Non-shareable */
    MPU->RBAR = ((0x20000000 & MPU_RBAR_BASE_Msk) | MPU_RBAR_AP_PRI_RW_USER_RW | MPU_RBAR_SH_NON_SHAREABLE);
    /* Limit address = Limit address :OR: Attribute Index = 0 :OR: ENABLE */
    MPU->RLAR = ((0x20003FFF & MPU_RLAR_LIMIT_Msk) | (0x0UL << MPU_RLAR_AttrIndx_Pos) | MPU_RLAR_EN_Msk);

    /*
      Region 3 (Test Memory Space)
      Start address = 0x20004000
      Shareablity = Non-shareable
      Size = 1KB
      Permission = No write access
    */

    /* Select Region 3 */
    MPU->RNR = 3;
    /* Base address = Base address :OR: Attribute = No write access :OR: Shareablity = Non-shareable */
    MPU->RBAR = ((0x20004000 & MPU_RBAR_BASE_Msk) | MPU_RBAR_AP_PRI_RO_USER_RO | MPU_RBAR_SH_NON_SHAREABLE);
    /* Limit address = Limit address :OR: Attribute Index = 0 :OR: ENABLE */
    MPU->RLAR = ((0x200043FF & MPU_RLAR_LIMIT_Msk) | (0x0UL << MPU_RLAR_AttrIndx_Pos) | MPU_RLAR_EN_Msk);

    /* Enable MPU */
    MPU->CTRL = MPU_CTRL_PRIVDEFENA_Msk | MPU_CTRL_ENABLE_Msk;

    printf("\n\n ==============================================\n");
    printf(" Memory Region 1 (Flash Memory) configuration:\n");
    printf(" ==============================================\n");
    printf(" Start address : 0x00000000\n");
    printf(" End address   : 0x0001FFFF\n");
    printf(" Size          : 128 KB\n");
    printf(" Memory Type   : Normal\n");
    printf(" Permission    : Full access\n");
    printf(" ----------------------------------------------\n");
    printf(" Please Press '1' to read memory successfully from region 1 (Flash Memory).\n");

    while(i32TestItem != '1') i32TestItem = getchar();

    printf("\n Read value from 0x00000000 is 0x%08X.\n", ReadMemCore(0x00000000));

    printf("\n\n ==============================================\n");
    printf(" Memory Region 2 (SRAM Memory) configuration:\n");
    printf(" ==============================================\n");
    printf(" Start address : 0x20000000\n");
    printf(" End address   : 0x20003FFF\n");
    printf(" Size          : 16 KB\n");
    printf(" Memory Type   : Normal\n");
    printf(" Permission    : Full access\n");
    printf(" ----------------------------------------------\n");
    printf(" Please Press '2' to read memory successfully from region 2 (SRAM Memory).\n");

    while(i32TestItem != '2') i32TestItem = getchar();

    printf("\n Read value from 0x20000000 is 0x%08X.\n", ReadMemCore(0x20000000));

    printf("\n\n ==============================================\n");
    printf(" Memory Region 3 (Test Memory) configuration:\n");
    printf(" ==============================================\n");
    printf(" Start address : 0x20004000\n");
    printf(" End address   : 0x200043FF\n");
    printf(" Size          : 1 KB\n");
    printf(" Memory Type   : Normal\n");
    printf(" Permission    : No write access\n");
    printf(" ----------------------------------------------\n");
    printf(" Please Press '3' to write memory to region 3 (Test Memory).\n");
    printf(" (It should trigger a memory fault exception!)\n");

    while(i32TestItem != '3') i32TestItem = getchar();

    /* Write address 0x20004000 */
    M32(0x20004000) = 0;
}

int main()
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART for printf */
    UART_Init();

    printf("\n Start MPU test: \n");

    MPU_Test();

    while(1);
}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
