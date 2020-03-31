/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief
 *           Show how to select SRAM power mode in system Power-down mode.
 *
 *
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"



extern int IsDebugFifoEmpty(void);
static volatile uint8_t s_u8IsINTEvent;

void WDT_IRQHandler(void);
void PowerDownFunction(void);
void SYS_Init(void);
void UART0_Init(void);

/*---------------------------------------------------------------------------------------------------------*/
/*  WDT IRQ Handler                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
void WDT_IRQHandler(void)
{

    if(WDT_GET_TIMEOUT_INT_FLAG())
    {
        /* Clear WDT time-out interrupt flag */
        WDT_CLEAR_TIMEOUT_INT_FLAG();
    }

    if(WDT_GET_TIMEOUT_WAKEUP_FLAG())
    {
        /* Clear WDT time-out wake-up flag */
        WDT_CLEAR_TIMEOUT_WAKEUP_FLAG();
    }

    s_u8IsINTEvent = 1;

}

/*---------------------------------------------------------------------------------------------------------*/
/*  Function for System Entry to Power Down Mode                                                           */
/*---------------------------------------------------------------------------------------------------------*/
void PowerDownFunction(void)
{
    /* To check if all the debug messages are finished */
    while(IsDebugFifoEmpty() == 0);

    /* Enter to Power-down mode */
    CLK_PowerDown();
}


void SYS_Init(void)
{

    /* Set PF multi-function pins for XT1_OUT(PF.2) and XT1_IN(PF.3) */
    SYS->GPF_MFPL = (SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF2MFP_Msk)) | SYS_GPF_MFPL_PF2MFP_XT1_OUT;
    SYS->GPF_MFPL = (SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF3MFP_Msk)) | SYS_GPF_MFPL_PF3MFP_XT1_IN;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and HCLK source divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable HXT clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Wait for HXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Enable PLL */
    CLK->PLLCTL = CLK_PLLCTL_128MHz_HIRC;

    /* Wait for PLL stable */
    CLK_WaitClockReady(CLK_STATUS_PLLSTB_Msk);

    /* Select HCLK clock source as PLL and HCLK source divider as 2 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_PLL, CLK_CLKDIV0_HCLK(2));

    /* Enable module clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(CRC_MODULE);
    CLK_EnableModuleClock(WDT_MODULE);

    /* Set module clock */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HXT, CLK_CLKDIV0_UART0(1));
    CLK_SetModuleClock(WDT_MODULE, CLK_CLKSEL1_WDTSEL_LIRC, 0);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

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
    /* Reset UART0 */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    uint32_t au32SRAMCheckSum[8] = {0};
    uint32_t au32SRAMSize[8] = { 8192, 8192, 8192, 8192, 16384, 16384, 16384, 16384};
    uint32_t u32Idx, u32Addr, u32SRAMStartAddr = 0;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);
    printf("+---------------------------------------+\n");
    printf("|       SRAM Power Mode Sample Code     |\n");
    printf("+---------------------------------------+\n\n");

    /*
        SRAM power mode in system Power-down mode can select as normal mode, retention mode and power shut down mode.
        If SRAM power mode select as power shut down mode, SRAM data will not kept after Power-down and wake-up.
        This sample code will set SRAM bank0 (0x2000000 - 0x20007FFF) in normal mode and set SRAM bank1 (0x20008000 - 0x20017FFF) in power shut down mode.
        The SRAM bank1 checksum after wake-up will be different with checksum before entering to Power-down mode.
    */


    /* Unlock protected registers before setting SRAM power mode */
    SYS_UnlockReg();

    /* Calculate SRAM checksum before entering to Power-down mode */
    printf("Calculate SRAM checksum before Power-down:\n");

    /* Specify SRAM region start address */
    u32SRAMStartAddr = 0x20000000;

    /* Calculate SRAM checksum */
    for(u32Idx = 0; u32Idx < 8; u32Idx++)
    {
        /* Configure CRC controller for CRC-CRC32 mode */
        CRC_Open(CRC_32, (CRC_WDATA_RVS | CRC_CHECKSUM_RVS | CRC_CHECKSUM_COM), 0xFFFFFFFF, CRC_CPU_WDATA_32);

        /* Start to execute CRC-CRC32 operation */
        for(u32Addr = u32SRAMStartAddr; u32Addr < (u32SRAMStartAddr + au32SRAMSize[u32Idx]); u32Addr += 4)
        {
            CRC_WRITE_DATA(CRC, inpw(u32Addr));
        }

        /* Record checksum result */
        au32SRAMCheckSum[u32Idx] = CRC_GetChecksum();

        /* Specify next SRAM region start address */
        u32SRAMStartAddr = u32Addr;
    }

    printf("SRAM Bank0 Region 0 Checksum [0x%08X]\n", au32SRAMCheckSum[0]);
    printf("SRAM Bank0 Region 1 Checksum [0x%08X]\n", au32SRAMCheckSum[1]);
    printf("SRAM Bank0 Region 2 Checksum [0x%08X]\n", au32SRAMCheckSum[2]);
    printf("SRAM Bank0 Region 3 Checksum [0x%08X]\n", au32SRAMCheckSum[3]);
    printf("SRAM Bank1 Region 0 Checksum [0x%08X]\n", au32SRAMCheckSum[4]);
    printf("SRAM Bank1 Region 1 Checksum [0x%08X]\n", au32SRAMCheckSum[5]);
    printf("SRAM Bank1 Region 2 Checksum [0x%08X]\n", au32SRAMCheckSum[6]);
    printf("SRAM Bank1 Region 3 Checksum [0x%08X]\n\n", au32SRAMCheckSum[7]);

    /* Select SRAM power mode in system Power-down Mode */
    SYS_SetSSRAMPowerMode(SYS_SRAMPCTL_SRAM0PM0_Msk, SYS_SRAMPCTL_SRAM_NORMAL);
    SYS_SetSSRAMPowerMode(SYS_SRAMPCTL_SRAM0PM1_Msk, SYS_SRAMPCTL_SRAM_NORMAL);
    SYS_SetSSRAMPowerMode(SYS_SRAMPCTL_SRAM0PM2_Msk, SYS_SRAMPCTL_SRAM_NORMAL);
    SYS_SetSSRAMPowerMode(SYS_SRAMPCTL_SRAM0PM3_Msk, SYS_SRAMPCTL_SRAM_NORMAL);
    SYS_SetSSRAMPowerMode(SYS_SRAMPCTL_SRAM1PM0_Msk, SYS_SRAMPCTL_SRAM_POWER_SHUT_DOWN);
    SYS_SetSSRAMPowerMode(SYS_SRAMPCTL_SRAM1PM1_Msk, SYS_SRAMPCTL_SRAM_POWER_SHUT_DOWN);
    SYS_SetSSRAMPowerMode(SYS_SRAMPCTL_SRAM1PM2_Msk, SYS_SRAMPCTL_SRAM_POWER_SHUT_DOWN);
    SYS_SetSSRAMPowerMode(SYS_SRAMPCTL_SRAM1PM3_Msk, SYS_SRAMPCTL_SRAM_POWER_SHUT_DOWN);

    /* Enter to Power-down mode and wake-up by WDT interrupt */
    printf("Enter to Power-down mode ... ");

    /* Enable WDT NVIC */
    NVIC_EnableIRQ(WDT_IRQn);

    /* Configure WDT settings and start WDT counting */
    WDT_Open(WDT_TIMEOUT_2POW14, (uint32_t)NULL, FALSE, TRUE);

    /* Enable WDT interrupt function */
    WDT_EnableInt();

    /* Unlock protected registers before entering Power-down mode */
    SYS_UnlockReg();

    /* Enter to Power-down mode */
    PowerDownFunction();

    /* Check if WDT time-out interrupt and wake-up occurred or not */
    while(s_u8IsINTEvent == 0);
    printf("wake-up!\n\n");

    /* Calculate SRAM checksum after wake-up from Power-down mode */
    printf("Calculate SRAM CheckSum after wake-up:\n");

    /* Specify SRAM region start address */
    u32SRAMStartAddr = 0x20000000;

    /* Calculate SRAM checksum */
    for(u32Idx = 0; u32Idx < 8; u32Idx++)
    {
        /* Configure CRC controller for CRC-CRC32 mode */
        CRC_Open(CRC_32, (CRC_WDATA_RVS | CRC_CHECKSUM_RVS | CRC_CHECKSUM_COM), 0xFFFFFFFF, CRC_CPU_WDATA_32);

        /* Start to execute CRC-CRC32 operation */
        for(u32Addr = u32SRAMStartAddr; u32Addr < u32SRAMStartAddr + au32SRAMSize[u32Idx]; u32Addr += 4)
        {
            CRC_WRITE_DATA(CRC, inpw(u32Addr));
        }

        /* Record checksum result */
        au32SRAMCheckSum[u32Idx] = CRC_GetChecksum();

        /* Specify next SRAM region start address */
        u32SRAMStartAddr = u32Addr;
    }

    printf("SRAM Bank0 Region 0 Checksum [0x%08X]\n", au32SRAMCheckSum[0]);
    printf("SRAM Bank0 Region 1 Checksum [0x%08X]\n", au32SRAMCheckSum[1]);
    printf("SRAM Bank0 Region 2 Checksum [0x%08X]\n", au32SRAMCheckSum[2]);
    printf("SRAM Bank0 Region 3 Checksum [0x%08X]\n", au32SRAMCheckSum[3]);
    printf("SRAM Bank1 Region 0 Checksum [0x%08X]\n", au32SRAMCheckSum[4]);
    printf("SRAM Bank1 Region 1 Checksum [0x%08X]\n", au32SRAMCheckSum[5]);
    printf("SRAM Bank1 Region 2 Checksum [0x%08X]\n", au32SRAMCheckSum[6]);
    printf("SRAM Bank1 Region 3 Checksum [0x%08X]\n", au32SRAMCheckSum[7]);

    while(1);

}

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
