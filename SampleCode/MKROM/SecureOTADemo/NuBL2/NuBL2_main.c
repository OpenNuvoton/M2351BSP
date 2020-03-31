/**************************************************************************//**
 * @file     NuBL2_main.c
 * @version  V1.00
 * @brief    Demonstrate to do secure OTA update for NuBL32 and NuBL33 firmware by NuBL2 with MKROM library.
 *
 * @copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <arm_cmse.h>
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"
#include "partition_M2351.h"

#include "NuBL_common.h"
#include "NuBL_crypto.h"
#include "NuBL2.h"

#include "ota.h"

volatile uint32_t gNuBL2_32Key[8], gNuBL2_33Key[8];

static volatile ISP_INFO_T s_NuBL2ISPInfo = {0};

#define PLL_CLOCK       64000000

/* For ECC NIST: Curve P-256 */
const uint32_t g_au32Eorder[] = {
    0xFC632551, 0xF3B9CAC2, 0xA7179E84, 0xBCE6FAAD, 0xFFFFFFFF, 0xFFFFFFFF, 0x00000000, 0xFFFFFFFF
};

void WDT_IRQHandler(void);
void SYS_Init(void);
void UART_Init(void);
void HardFault_Handler(void)__attribute__((noreturn));
void WdtEnableAndCheck(void);
int8_t NuBLTrustBootInit(void);

/**
 * @brief       IRQ Handler for WDT Interrupt
 * @param       None
 * @return      None
 * @details     The WDT_IRQHandler is default IRQ of WDT, declared in startup_M2351.s.
 */
void WDT_IRQHandler(void)
{
    printf("WDT_IRQHandler\n");
#if (WDT_RST_ENABLE)
    if(g_u32WakeupCounts < 10)
    {
        WDT_RESET_COUNTER();
    }

    if(WDT_GET_TIMEOUT_INT_FLAG() == 1)
    {
        /* Clear WDT time-out interrupt flag */
        WDT_CLEAR_TIMEOUT_INT_FLAG();

        g_u8IsINTEvent = 1;
    }

    if(WDT_GET_TIMEOUT_WAKEUP_FLAG() == 1)
    {
        /* Clear WDT time-out wake-up flag */
        WDT_CLEAR_TIMEOUT_WAKEUP_FLAG();

        g_u32WakeupCounts++;
    }
#endif
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

    /* Select HCLK clock source as HIRC and HCLK source divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable HXT clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Wait for HXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Enable PLL */
    CLK->PLLCTL = CLK_PLLCTL_128MHz_HIRC;

    /* Waiting for PLL stable */
    CLK_WaitClockReady(CLK_STATUS_PLLSTB_Msk);

    /* Select HCLK clock source as PLL and HCLK source divider as 2 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_PLL, CLK_CLKDIV0_HCLK(2));

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART module clock source as HIRC */
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
    /* Reset UART module */
    SYS_ResetModule(UART0_RST);

    /* Configure UART and set UART Baudrate */
    UART_Open(DEBUG_PORT, 115200);
}

void HardFault_Handler(void)
{
    printf("NuBL2 HardFault!!\n");
    while(1) {}
}

/**
  * @brief      Enable WDT and check if reset by WDT
  * @param      None
  * @return     None
  * @details    Enable WDT and check if system reset by WDT
  */
void WdtEnableAndCheck(void)
{
    SYS_UnlockReg();
    FMC_Open();
    FMC_ENABLE_AP_UPDATE();
#if (WDT_RST_ENABLE)
    /* Check WDT config */
    u32Cfg = FMC_Read(FMC_CONFIG_BASE);
    if((u32Cfg & (BIT31|BIT4|BIT3)) == ((BIT31|BIT4|BIT3)))
    {
        printf("[WRAN]WDT was disabled\n");
        FMC_ENABLE_CFG_UPDATE();
        u32Cfg &= ~((BIT31|BIT4|BIT3));
        FMC_Write(FMC_CONFIG_BASE, u32Cfg);

        //NVIC_SystemReset();
        SYS_ResetChip();
        while(1){}
    }
    else
        printf("WDT was enabled\n");
#endif
    if (SYS_IS_WDT_RST())
    {
        /* upgrade done, clear OTA status */
        FMC_Write(OTA_STATUS_BASE, 0x1);

        SYS_ClearResetSrc(SYS_RSTSTS_WDTRF_Msk);
        printf("*** System has been reset by WDT time-out event 0***\n\n");
        SYS_ResetChip();
        while(1){}
    }

    /* To check if system has been reset by WDT time-out reset or not */
    if(WDT_GET_RESET_FLAG() == 1)
    {
        WDT_CLEAR_RESET_FLAG();

        /* upgrade done, clear OTA status */
        FMC_Write(OTA_STATUS_BASE, 0x1);

        printf("*** System has been reset by WDT time-out event ***\n\n");
        SYS_ResetChip();
        while(1) {}
    }
}

/**
  * @brief      Trust boot initialization
  * @param      None
  * @retval     0           Success
  * @retval     Other       Fail
  * @details    Create FW INFO
  */
//FwSign.exe .\NuBL2_FW.bin\FLASH .\NuBL2_FW.bin\FWINFO
int8_t NuBLTrustBootInit(void)
{
    /* Init NuBL2 FW INFO */
    extern const uint32_t g_InitialFWinfo[];
    //uint32_t cfg = g_InitialFWinfo.mData.u32AuthCFGs;
    uint32_t cfg = g_InitialFWinfo[16];
    printf("\n[AuthCFG: 0x%08x]\n", cfg);

    return 0;
}


/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
#if (OTA_UPGRADE_FROM_SD)
int main(void)
{
    uint8_t u8FailNuBL3x; /* Bit0 = 1: NuBL32, Bit1 = 1: NuBL33 */
    uint8_t u32NeedReset = 0;
#if 0
    uint32_t    u32NuBL2ISPInfoAddr;
    ISP_INFO_T  *pu32NuBL2ISPInfoAddr;
#endif
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART for printf */
    UART_Init();
    printf("\n\n[HCLK %d Hz] (%s, %s)\n", SystemCoreClock, __DATE__, __TIME__);
    printf("+-------------------------------+\n");
    printf("|    M2351 NuBL2 OTA Sample Code    |\n");
    printf("+-------------------------------+\n\n");
    CLK_SysTickDelay(200000);

    /* Create FW INFO */
    NuBLTrustBootInit();

    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Enable APROM Update Function */
    FMC_Open();
    FMC_ENABLE_AP_UPDATE();

    /* Enable WDT and check if system reset by WDT */
    WdtEnableAndCheck();

    /* Init u8FailNuBL3x value as 0 */
    u8FailNuBL3x = 0;
    /* check OTA status of system firmware */
    printf("check OTA status: SYS:%d, APP:%d\n", FMC_Read(SYS_FW_OTA_STATUS_BASE), FMC_Read(APP_FW_OTA_STATUS_BASE));
    if (FMC_Read(SYS_FW_OTA_STATUS_BASE) == 1)
    {
        int32_t i32Ret = 0;

        /* init OTA and SD Host */
        //OTA_Init(__HSI);
        /* Init SD Host */
        OTA_API_SDInit();

        /* verify NuBL32 identity by FW INFO and update FW */
        i32Ret = OTA_VerifyAndUpdateNuBL3xFromSD(0x1);
        if (i32Ret == 0)
        {
            /* if update done, clear OTA status, reboot for secure boot. */
            u32NeedReset = TRUE;
        }
        else
        {
            printf("NuBL32 upgrade failed(%d) \n", i32Ret);
            /* if update failed, clear OTA status, then normal boot to BL3x. */
            u32NeedReset = FALSE;
        }
        /* upgrade done, clear OTA status */
        FMC_Erase(SYS_FW_OTA_STATUS_BASE);
    }
    /* check OTA status of application firmware */
    if (FMC_Read(APP_FW_OTA_STATUS_BASE) == 1)
    {
        int32_t i32Ret = 0;

        /* init OTA and SD card */
//        OTA_Init(__HSI);
        /* Init SD Host */
        OTA_API_SDInit();
        /* verify NuBL33 identity by FW INFO and update FW */
        i32Ret = OTA_VerifyAndUpdateNuBL3xFromSD(0x2);
        if (i32Ret == 0)
        {
            /* if update done, clear OTA status, reboot for secure boot. */
            u32NeedReset = TRUE;
        }
        else
        {
            printf("NuBL33 upgrade failed(%d) \n", i32Ret);
            /* if update failed, clear OTA status, then normal boot to NuBL3x. */
            u32NeedReset = FALSE;
        }
        /* upgrade done, clear OTA status */
        FMC_Erase(APP_FW_OTA_STATUS_BASE);
    }

    if (u32NeedReset)
    {
        /* Reset CPU only to reset to new vector page */
        goto reset;
    }

    /* normal boot */
    /* verify application firmware identity and integrity */
    NuBL2_Init();
    /* verify system firmware identity and integrity */
    if(NuBL2_ExecuteVerifyNuBL3x(NULL, 0) != 0)
    {
        /* if verify failed, do OTA update right now */
        printf("\n\nNuBL2 verifies NuBL32 FAIL.\n\n");
        /* set this flag for reset NuBL32 firmware version to 0. */
        u8FailNuBL3x = BIT0;

        goto _VERIFY_FAIL;
    }
    else
    {
        printf("\n\nNuBL2 verifies NuBL32 PASS.\n\n");
    }

    if(NuBL2_ExecuteVerifyNuBL3x(NULL, 1) != 0)
    {
        /* if verify failed, do OTA update right now */
        printf("\n\nNuBL2 verifies NuBL33 FAIL.\n\n");
        /* set this flag for reset NuBL33 firmware version to 0. */
        u8FailNuBL3x = BIT1;

        goto _VERIFY_FAIL;
    }
    else
    {
        printf("\n\nNuBL2 verifies NuBL33 PASS.\n\n");
    }

    printf("\n[Executing in NuBL2] execute NuBL32 F/W on 0x%08x.\n", (uint32_t)NUBL32_FW_BASE);

    __set_PRIMASK(1); /* Disable all interrupt */
    FMC_SetVectorPageAddr(NUBL32_FW_BASE);

    /* TODO: Clear Secure SRAM */

    /* Reset CPU only to reset to new vector page */
    SYS_ResetCPU();
    while(1) {}

_VERIFY_FAIL:
#if 1
    {/* debug */
        while(1) {}
    }
#else
    /* if verify failed, modified NuBL3x firmware version to 0, and do OTA update after reboot */
    if (u8FailNuBL3x&BIT0)
    {
        FW_INFO_T FwInfo;

        /* Clear NuBL3x firmware version to 0 */
        FwInfo.mData.au32ExtInfo[0] = 0;
        /* NuBL32 */
        if (NuBL2_UpdateNuBL3xFwInfo((uint32_t *)&FwInfo, sizeof(FW_INFO_T), 0, NUBL32_FW_INFO_BASE) != 0)
        {
            printf("\nClear NuBL32 F/W version failed.\n");
        }
    }

    if ((u8FailNuBL3x&BIT1))
    {
        FW_INFO_T FwInfo;

        /* Clear NuBL3x firmware version to 0 */
        FwInfo.mData.au32ExtInfo[0] = 0;
        /* NuBL33 */
        if (NuBL2_UpdateNuBL3xFwInfo((uint32_t *)&FwInfo, sizeof(FW_INFO_T), 1, NUBL33_FW_INFO_BASE) != 0)
        {
            printf("\nClear NuBL33 F/W version failed.\n");
        }
    }

    SYS_UnlockReg();
    FMC_Open();
    FMC_ENABLE_AP_UPDATE();
    /* init OTA */
    OTA_Init(__HSI, (ISP_INFO_T *)&s_NuBL2ISPInfo);
    if (OTA_TaskProcess() == 0)
    {
        /* check OTA status and re-boot for update firmware */
        printf("OTA update for BL3x verify failed: SYS:%d, APP:%d\n", FMC_Read(SYS_FW_OTA_STATUS_BASE), FMC_Read(APP_FW_OTA_STATUS_BASE));
        if ((FMC_Read(SYS_FW_OTA_STATUS_BASE) == 1)||FMC_Read(APP_FW_OTA_STATUS_BASE) == 1)
        {
            u32NeedReset = TRUE;
        }
    }
    goto reset;
#endif

reset:
    while(!(UART_IS_TX_EMPTY(DEBUG_PORT)));

    /* Reset CPU only to reset to new vector page */
    SYS_ResetChip();

    while(1) {}
}
#else /* update on the fly */
int main(void)
{
#if 0
    uint32_t u32NeedReset = 0;
#endif
#if (WDT_RST_ENABLE)
    uint32_t u32Cfg;
#endif
    uint8_t u8FailNuBL3x; /* Bit0 = 1: NuBL32, Bit1 = 1: NuBL33 */

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART for printf */
    UART_Init();

    printf("\n\n[HCLK %d Hz]\n", SystemCoreClock);
    printf("+------------------------------------+\n");
    printf("|    M2351 Secure OTA Sample Code    |\n");
    printf("+------------------------------------+\n\n");
    CLK_SysTickDelay(200000);

    /* Create FW INFO */
    NuBLTrustBootInit();

    SYS_UnlockReg();
    FMC_Open();
    FMC_ENABLE_AP_UPDATE();

    #if (OTA_DEMO_WITH_APP)
    {/* for App Demo */
        FMC_Erase(OTA_STATUS_BASE);
        FMC_Write(OTA_STATUS_BASE, 0x1UL);
    }
    #endif

    /* Enable WDT and check if system reset by WDT */
    WdtEnableAndCheck();

    printf("check OTA status:%d\n", FMC_Read(OTA_STATUS_BASE));

    /* check OTA status for entry update mode */
    if (FMC_Read(OTA_STATUS_BASE) == 1)
    {
        /* init wifi module and connect to OTA server */
        OTA_Init(__HSI, (ISP_INFO_T *)(uint32_t)&s_NuBL2ISPInfo);
        if (OTA_TaskProcess() == 0)
        {
            SYS_UnlockReg();
            FMC_Open();
            FMC_ENABLE_AP_UPDATE();
            /* upgrade done, clear OTA status */
            FMC_Erase(OTA_STATUS_BASE);

            printf("update done. Check OTA status: %d\n", FMC_Read(OTA_STATUS_BASE));
            if ((FMC_Read(OTA_STATUS_BASE) == 1))
            {
                printf("[ERR]clear OTA status error\n");
            }
            #if (OTA_DEMO_WITH_APP)
            {/* for App Demo */
                __set_PRIMASK(1); /* Disable all interrupt */
                FMC_SetVectorPageAddr(NUBL32_FW_BASE);

                /* TODO: Clear Secure SRAM */

                /* Reset CPU only to reset to new vector page */
                SYS_ResetCPU();
                while(1) {}
            }
            #endif
            goto reset;
        }
    }

    /* normal boot */
    /* verify application firmware identity and integrity */
    NuBL2_Init();
    /* verify system firmware identity and integrity */
    if(NuBL2_ExecuteVerifyNuBL3x(NULL, 0) != 0)
    {
        /* if verify failed, do OTA update right now */
        printf("\n\nNuBL2 verifies NuBL32 FAIL.\n\n");
        /* set this flag for reset NuBL32 firmware version to 0. */
        u8FailNuBL3x = BIT0;

        goto _VERIFY_FAIL;
    }
    else
    {
        printf("\n\nNuBL2 verifies NuBL32 PASS.\n\n");
    }

    if(NuBL2_ExecuteVerifyNuBL3x(NULL, 1) != 0)
    {
        /* if verify failed, do OTA update right now */
        printf("\n\nNuBL2 verifies NuBL33 FAIL.\n\n");
        /* set this flag for reset NuBL33 firmware version to 0. */
        u8FailNuBL3x = BIT1;

        goto _VERIFY_FAIL;
    }
    else
    {
        printf("\n\nNuBL2 verifies NuBL33 PASS.\n\n");
    }

    printf("\n[Executing in NuBL2] execute NuBL32 F/W on 0x%08x.\n", (uint32_t)NUBL32_FW_BASE);

    __set_PRIMASK(1); /* Disable all interrupt */
    FMC_SetVectorPageAddr(NUBL32_FW_BASE);

    /* TODO: Clear Secure SRAM */

    /* Reset CPU only to reset to new vector page */
    SYS_ResetCPU();
    while(1) {}

_VERIFY_FAIL:
#if 1
    {/* debug */
        while(1) {}
    }
#else
    /* if verify failed, modified NuBL3x firmware version to 0, and do OTA update after reboot */
    if (u8FailNuBL3x&BIT0)
    {
        FW_INFO_T FwInfo;

        /* Clear NuBL3x firmware version to 0 */
        FwInfo.mData.au32ExtInfo[0] = 0;
        /* NuBL32 */
        if (NuBL2_UpdateNuBL3xFwInfo((uint32_t *)&FwInfo, sizeof(FW_INFO_T), 0, NUBL32_FW_INFO_BASE) != 0)
        {
            printf("\nClear NuBL32 F/W version failed.\n");
        }
    }

    if ((u8FailNuBL3x&BIT1))
    {
        FW_INFO_T FwInfo;

        /* Clear NuBL3x firmware version to 0 */
        FwInfo.mData.au32ExtInfo[0] = 0;
        /* NuBL33 */
        if (NuBL2_UpdateNuBL3xFwInfo((uint32_t *)&FwInfo, sizeof(FW_INFO_T), 1, NUBL33_FW_INFO_BASE) != 0)
        {
            printf("\nClear NuBL33 F/W version failed.\n");
        }
    }

    SYS_UnlockReg();
    FMC_Open();
    FMC_ENABLE_AP_UPDATE();

    /* update OTA status for upgrade right now. */
    FMC_Write(OTA_STATUS_BASE, 0x1UL);

    /* init OTA */
    OTA_Init(__HSI, (ISP_INFO_T *)&s_NuBL2ISPInfo);
    if (OTA_TaskProcess() == 0)
    {
        /* check OTA status and re-boot for update firmware */
        printf("OTA update for NuBL3x verify failed: SYS:%d, APP:%d\n", FMC_Read(SYS_FW_OTA_STATUS_BASE), FMC_Read(APP_FW_OTA_STATUS_BASE));
        if ((FMC_Read(SYS_FW_OTA_STATUS_BASE) == 1)||FMC_Read(APP_FW_OTA_STATUS_BASE) == 1)
        {
            u32NeedReset = TRUE;
        }
    }
    goto reset;
#endif

reset:
    while(!(UART_IS_TX_EMPTY(DEBUG_PORT)));

    /* Reset CPU only to reset to new vector page */
    SYS_ResetChip();

    while(1) {}
}
#endif
////////////////////////////////////////////////////////////////////////////////////////////

/**
  * @brief      Initial NuBL2
  * @param      None
  * @retval     0           Success
  * @retval     -1          Failed
  */
int32_t NuBL2_Init(void)
{
    uint32_t au32CFG[4], u32XOMStatus;

    if((sizeof(METADATA_T)%16) != 0)
        return -1;

    if((sizeof(FW_INFO_T)%16) != 0)
        return -2;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable FMC ISP function */
    FMC_Open();

    FMC_ReadConfig(au32CFG, 4);

    printf("Chip info:\n");
    printf("\t[Secure boundary : 0x%08x]\n", SCU->FNSADDR);
    printf("\t[CONFIG0         : 0x%08x]", au32CFG[0]);
    if(!(au32CFG[0]&BIT5))
    {
        printf("\tBoot from MaskROM -> %s.\n", (au32CFG[0]&BIT7)?"APROM":"LDROM");
    }
    else
    {
        if((au32CFG[0]&BIT7))
            printf("\tBoot from APROM.\n");
        else
            printf("\tBoot from LDROM.\n");
    }

    u32XOMStatus = (uint32_t)FMC_GetXOMState(XOMR0);
    printf("\t[XOM0            : %s]", (u32XOMStatus==1)?"Active":"Inactive");
    printf("\tBase: 0x%08x; Page count: 0x%08x.\n", (FMC->XOMR0STS>>8), (FMC->XOMR0STS&0xFF));
    u32XOMStatus = (uint32_t)FMC_GetXOMState(XOMR1);
    printf("\t[XOM1            : %s]", (u32XOMStatus==1)?"Active":"Inactive");
    printf("\tBase: 0x%08x; Page count: 0x%08x.\n", (FMC->XOMR1STS>>8), (FMC->XOMR1STS&0xFF));

    return 0;
}

/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/
