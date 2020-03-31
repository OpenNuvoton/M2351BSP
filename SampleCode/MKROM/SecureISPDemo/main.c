/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate how to initial SecureISP function in secure code.
 *
 * @copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"
#include "ProcessVendorCmd.h"

/*
    Increase Stack_Size=0x1800 for SecureISP application.
*/


/*---------------------------------------------------------------------------------------------------------*/
/* Global variables for initial SecureISP function                                                         */
/*---------------------------------------------------------------------------------------------------------*/
ISP_INFO_T      g_ISPInfo = {0};
static BL_USBD_INFO_T  s_USBDInfo;
extern void Exec_VendorFunction(uint32_t *pu32Buf, uint32_t u32Len);

void USBD_IRQHandler(void);
void UART1_IRQHandler(void);
void EnableXOM0(void);
void SYS_Init(void);
void UART_Init(void);

/**
 * @brief       IRQ Handler for USBD Interrupt
 * @param       None
 * @return      None
 * @details     The USBD_IRQHandler is default IRQ of USBD, declared in startup_M2351.s.
 */
void USBD_IRQHandler(void)
{
    /* Process USBD data */
    BL_ProcessUSBDInterrupt((uint32_t *)g_ISPInfo.pfnUSBDEP, (uint32_t *)&g_ISPInfo, (uint32_t *)(uint32_t)&s_USBDInfo);
}

/**
 * @brief       IRQ Handler for UART1 Interrupt
 * @param       None
 * @return      None
 * @details     The UART1_IRQHandler is default IRQ of UART1, declared in startup_M2351.s.
 */
void UART1_IRQHandler(void)
{
    /* Process UART1 data */
    BL_ProcessUART1Interrupt((uint32_t *)&g_ISPInfo);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Configure USBD function                                                                                */
/*---------------------------------------------------------------------------------------------------------*/
static int32_t ConfigureUSBDISP(void)
{
    printf("[Configure USBD]\n");
    
    /* Enable Internal RC 48MHz clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRC48EN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRC48STB_Msk);

    /* Use HIRC48 as USB clock source */
    CLK_SetModuleClock(USBD_MODULE, CLK_CLKSEL0_USBSEL_HIRC48, CLK_CLKDIV0_USB(1));

    /* Select USBD */
    SYS->USBPHY = (SYS->USBPHY & ~SYS_USBPHY_USBROLE_Msk) | SYS_USBPHY_OTGPHYEN_Msk | SYS_USBPHY_SBO_Msk;

    /* Enable IP clock */
    CLK_EnableModuleClock(USBD_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* USBD multi-function pins for VBUS, D+, D-, and ID pins */
    SYS->GPA_MFPH &= ~(SYS_GPA_MFPH_PA12MFP_Msk | SYS_GPA_MFPH_PA13MFP_Msk | SYS_GPA_MFPH_PA14MFP_Msk | SYS_GPA_MFPH_PA15MFP_Msk);
    SYS->GPA_MFPH |= (SYS_GPA_MFPH_PA12MFP_USB_VBUS | SYS_GPA_MFPH_PA13MFP_USB_D_N | SYS_GPA_MFPH_PA14MFP_USB_D_P | SYS_GPA_MFPH_PA15MFP_USB_OTG_ID);
    
    printf("\n");
    
    return 0;
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Configure UART1 function                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
static int32_t ConfigureUART1ISP(uint32_t mode)
{
    printf("[Configure UART1]\n");
    
    switch(mode)
    {
        case 0:
            printf("UART1: RX = PB.6, TX = PB.7\n");
            /* UART1: TX = PB.7, RX = PB.6 */
            SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB7MFP_Msk | SYS_GPB_MFPL_PB6MFP_Msk);
            SYS->GPB_MFPL |= (UART1_TXD_PB7 | UART1_RXD_PB6);
            break;

        case 1:
            printf("UART1: TX = PA.9, RX = PA.8\n");
            /* UART1: TX = PA.9, RX = PA.8 */
            SYS->GPA_MFPH &= ~(SYS_GPA_MFPH_PA9MFP_Msk | SYS_GPA_MFPH_PA8MFP_Msk);
            SYS->GPA_MFPH |= (UART1_TXD_PA9 | UART1_RXD_PA8);
            break;

        case 2:
            printf("UART1: TX = PF.0, RX = PF.1\n");
            /* UART1: TX = PF.0, RX = PF.1 */
            SYS->GPF_MFPL &= ~(SYS_GPF_MFPL_PF0MFP_Msk | SYS_GPF_MFPL_PF1MFP_Msk);
            SYS->GPF_MFPL |= (UART1_TXD_PF0 | UART1_RXD_PF1);
            break;

        case 3:
            printf("UART1: TX = PB.3, RX = PB.2\n");
            /* UART1: TX = PB.3, RX = PB.2 */
            SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB3MFP_Msk | SYS_GPB_MFPL_PB2MFP_Msk);
            SYS->GPB_MFPL |= (UART1_TXD_PB3 | UART1_RXD_PB2);
            break;

        /* Others */
        default:
            printf("UART1: TX = PA.3, RX = PA.2\n");
            /* UART1: TX = PA.3, RX = PA.2 */
            SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA3MFP_Msk | SYS_GPA_MFPL_PA2MFP_Msk);
            SYS->GPA_MFPL |= (UART1_TXD_PA3 | UART1_RXD_PA2);
            break;
    }

    /* Enable UART1 module clock */
    CLK_EnableModuleClock(UART1_MODULE);
    CLK_SetModuleClock(UART1_MODULE, CLK_CLKSEL1_UART1SEL_HIRC, CLK_CLKDIV0_UART1(1));

    printf("\n");
    
    return __HIRC;
}

void EnableXOM0(void)
{
    int32_t i32Status;
    uint32_t u32Base = 0x10000;
    uint8_t u8Page = 2;
    
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable FMC ISP function and enable APROM active */
    FMC_ENABLE_ISP();
    FMC_ENABLE_AP_UPDATE();
    
    if((FMC->XOMSTS & 0x1) != 0x1)
    {
        printf("\nXOM0 base: 0x%x, page count: %d.\n\n", u32Base, u8Page);
        
        if(FMC_GetXOMState(XOMR0) == 0)
        {
            i32Status = FMC_ConfigXOM(XOMR0, u32Base, u8Page);
            if(i32Status == 0)
            {
                printf("Configure XOM0 Success.\n");
            }
            else
            {
                printf("Configure XOM0 FAIL.\n");
                while(1) {}
            }
        }
        else
        {
            printf("Get XOM0 status FAIL.\n\n");
            while(1) {}
        }

        printf("Reset chip to enable XOM region.\n\n");
        UART_WAIT_TX_EMPTY((UART_T *)DEBUG_PORT);

        /* Reset chip to enable XOM region. */
        SYS_ResetChip();
        while(1) {}
    }
    else
    {
        printf("XOM0 region is already actived.\n\n");
    }
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
    /* Reset UART module */
    SYS_ResetModule(UART0_RST);

    /* Configure UART and set UART Baudrate */
    UART_Open(DEBUG_PORT, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    int32_t     ret = 0;
    uint32_t    u32ISPmode = 0, u32UartClkFreq;
    
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART for printf */
    UART_Init();

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+---------------------------------------------------+\n");
    printf("|    Initialize USBD/UART1 SecureISP Sample Code    |\n");
    printf("+---------------------------------------------------+\n\n");
    
// Remove XO setting in Keil v5.26.2.0 (Nuvoton Edition)
//    /* Enable XOM0 */
//    EnableXOM0();

    /* Configure USBD ISP */
    ConfigureUSBDISP();
    
    /* Configure UART1 ISP */
    u32UartClkFreq = (uint32_t)ConfigureUART1ISP(0x0);
                
    //printf("\n[Hit any key to enter SecureISP]\n\n");
    //getchar();    

    while(1) 
    {
        if(ret == 0x8000) // 0x8000 is Re-sync command
        {
            printf("Enter SecureISP again......\n");
            
            memset((void *)&g_ISPInfo.au32AESKey[0], 0x0, sizeof(g_ISPInfo.au32AESKey));
            memset((void *)&g_ISPInfo.au32AESIV[0], 0x0, sizeof(g_ISPInfo.au32AESIV));
            memset((void *)&g_ISPInfo.sign, 0x0, sizeof(ECDSA_SIGN_T));
            g_ISPInfo.UARTDataIdx = 0;
            g_ISPInfo.IsUSBDataReady = FALSE;
            g_ISPInfo.IsUARTDataReady = FALSE;
                        
            u32ISPmode |= RESYNC_ISP;
        }
        else
        {
            printf("Initialize and enter SecureISP......\n");
            
            /* Clear global variables */   
            memset((void *)&g_ISPInfo, 0x0, sizeof(ISP_INFO_T));
            memset((void *)&s_USBDInfo, 0x0, sizeof(BL_USBD_INFO_T));
            
            u32ISPmode = USB_UART_MODE;
        }

        /* Configure UART1 ISP */
        g_ISPInfo.UARTClockFreq = u32UartClkFreq;
                
        /* Configure user's vendor function */
        g_ISPInfo.pfnVendorFunc = Exec_VendorFunction;
        
        /* Configure time-out time for checking the SecureISP Tool connection */
        g_ISPInfo.timeout = SystemCoreClock;

        /* Configure to mask specify command */
        g_ISPInfo.u32CmdMask = 0;
                
        /* Initialize and start USBD and UART1 SecureISP function */
        ret = BL_SecureISPInit(&g_ISPInfo, &s_USBDInfo, (E_ISP_MODE)u32ISPmode);  
        if(ret == 0x8000) // 0x8000 is Re-sync command
            continue;
                
        break;
    }

    printf("\nExit SecureISP. (Status: %d)\n", ret);

    while(1) {}
}

/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/
