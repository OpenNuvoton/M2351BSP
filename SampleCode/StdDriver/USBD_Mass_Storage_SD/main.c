/***************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Use SD card as storage to implement a USB Mass-Storage device.
 *
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"
#include "massstorage.h"

#define CRYSTAL_LESS        1
#define TRIM_INIT           (SYS_BASE+0x10C)

#define CLK_PLLCTL_144MHz_HXT   (CLK_PLLCTL_PLLSRC_HXT  | CLK_PLLCTL_NR(2) | CLK_PLLCTL_NF( 12) | CLK_PLLCTL_NO_1)

/*--------------------------------------------------------------------------*/
uint8_t volatile g_u8SdInitFlag = 0;

/*--------------------------------------------------------------------------*/
void SDH0_IRQHandler(void);
void SYS_Init(void);
void PowerDown(void);
int IsDebugFifoEmpty(void);

void SDH0_IRQHandler(void)
{
    unsigned int volatile isr;

    // FMI data abort interrupt
    if(SDH0->GINTSTS & SDH_GINTSTS_DTAIF_Msk)
    {
        /* ResetAllEngine() */
        SDH0->GCTL |= SDH_GCTL_GCTLRST_Msk;
    }

    //----- SD interrupt status
    isr = SDH0->INTSTS;
    if(isr & SDH_INTSTS_BLKDIF_Msk)
    {
        // block down
        g_u8SDDataReadyFlag = TRUE;
        SDH0->INTSTS = SDH_INTSTS_BLKDIF_Msk;
    }

    if(isr & SDH_INTSTS_CDIF_Msk)  // card detect
    {
        //----- SD interrupt status
        // it is work to delay 50 times for SD_CLK = 200KHz
        {
            int volatile i;         // delay 30 fail, 50 OK
            for(i = 0; i < 0x500; i++); // delay to make sure got updated value from REG_SDISR.
            isr = SDH0->INTSTS;
        }

        if(isr & SDH_INTSTS_CDSTS_Msk)
        {
            printf("\n***** card remove !\n");
            SD0.IsCardInsert = FALSE;   // SDISR_CD_Card = 1 means card remove for GPIO mode
            memset(&SD0, 0, sizeof(SDH_INFO_T));
        }
        else
        {
            printf("***** card insert !\n");
            SDH_Open(SDH0, CardDetect_From_GPIO);
            if(SDH_Probe(SDH0))
            {
                g_u8SdInitFlag = 0;
                printf("SD initial fail!!\n");
            }
            else
            {
                g_u8SdInitFlag = 1;
                g_u32TotalSectors = SD0.totalSectorN;
            }
        }
        SDH0->INTSTS = SDH_INTSTS_CDIF_Msk;
    }

    // CRC error interrupt
    if(isr & SDH_INTSTS_CRCIF_Msk)
    {
        if(!(isr & SDH_INTSTS_CRC16_Msk))
        {
            //printf("***** ISR sdioIntHandler(): CRC_16 error !\n");
            // handle CRC error
        }
        else if(!(isr & SDH_INTSTS_CRC7_Msk))
        {
            if(!g_u8R3Flag)
            {
                //printf("***** ISR sdioIntHandler(): CRC_7 error !\n");
                // handle CRC error
            }
        }
        SDH0->INTSTS = SDH_INTSTS_CRCIF_Msk;      // clear interrupt flag
    }

    if(isr & SDH_INTSTS_DITOIF_Msk)
    {
        printf("***** ISR: data in timeout !\n");
        SDH0->INTSTS |= SDH_INTSTS_DITOIF_Msk;
    }

    // Response in timeout interrupt
    if(isr & SDH_INTSTS_RTOIF_Msk)
    {
        printf("***** ISR: response in timeout !\n");
        SDH0->INTSTS |= SDH_INTSTS_RTOIF_Msk;
    }
}


void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
#if (!CRYSTAL_LESS)
    /* Enable external XTAL 12MHz clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Waiting for 12MHz clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Set PLL frequency */
    CLK->PLLCTL = CLK_PLLCTL_144MHz_HXT;

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_STATUS_PLLSTB_Msk);

    /* Switch HCLK clock source to PLL */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_PLL, CLK_CLKDIV0_HCLK(3));

    /* Use PLL as USB clock source */
    CLK_SetModuleClock(USBD_MODULE, CLK_CLKSEL0_USBSEL_PLL, CLK_CLKDIV0_USB(3));
#else
    /* Enable Internal RC 48MHz clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRC48EN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRC48STB_Msk);

    /* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC48, CLK_CLKDIV0_HCLK(1));

    /* Use HIRC48 as USB clock source */
    CLK_SetModuleClock(USBD_MODULE, CLK_CLKSEL0_USBSEL_HIRC48, CLK_CLKDIV0_USB(1));
#endif

    /* Select USBD */
    SYS->USBPHY = (SYS->USBPHY & ~SYS_USBPHY_USBROLE_Msk) | SYS_USBPHY_OTGPHYEN_Msk | SYS_USBPHY_SBO_Msk;

    /* Enable IP clock */
    CLK_EnableModuleClock(USBD_MODULE);

    /* Select IP clock source */
    CLK_SetModuleClock(SDH0_MODULE, CLK_CLKSEL0_SDH0SEL_HCLK, CLK_CLKDIV0_SDH0(2));
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Enable IP clock */
    CLK_EnableModuleClock(SDH0_MODULE);
    CLK_EnableModuleClock(UART0_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* USBD multi-function pins for VBUS, D+, D-, and ID pins */
    SYS->GPA_MFPH &= ~(SYS_GPA_MFPH_PA12MFP_Msk | SYS_GPA_MFPH_PA13MFP_Msk | SYS_GPA_MFPH_PA14MFP_Msk | SYS_GPA_MFPH_PA15MFP_Msk);
    SYS->GPA_MFPH |= (SYS_GPA_MFPH_PA12MFP_USB_VBUS | SYS_GPA_MFPH_PA13MFP_USB_D_N | SYS_GPA_MFPH_PA14MFP_USB_D_P | SYS_GPA_MFPH_PA15MFP_USB_OTG_ID);

    /* Set multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & (~(UART0_RXD_PB12_Msk | UART0_TXD_PB13_Msk))) | UART0_RXD_PB12 | UART0_TXD_PB13;

    /* Select multi-function pins */
    SYS->GPE_MFPL &= ~(SYS_GPE_MFPL_PE2MFP_Msk | SYS_GPE_MFPL_PE3MFP_Msk | SYS_GPE_MFPL_PE4MFP_Msk | SYS_GPE_MFPL_PE5MFP_Msk |
                       SYS_GPE_MFPL_PE6MFP_Msk | SYS_GPE_MFPL_PE7MFP_Msk);
    SYS->GPD_MFPH &= ~SYS_GPD_MFPH_PD13MFP_Msk;
    SYS->GPE_MFPL |= (SYS_GPE_MFPL_PE2MFP_SD0_DAT0 | SYS_GPE_MFPL_PE3MFP_SD0_DAT1 | SYS_GPE_MFPL_PE4MFP_SD0_DAT2 | SYS_GPE_MFPL_PE5MFP_SD0_DAT3 |
                      SYS_GPE_MFPL_PE6MFP_SD0_CLK | SYS_GPE_MFPL_PE7MFP_SD0_CMD);
    SYS->GPD_MFPH |= SYS_GPD_MFPH_PD13MFP_SD0_nCD;
}

void PowerDown(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    printf("Enter power down ...\n");
    while(!IsDebugFifoEmpty());

    /* Wakeup Enable */
    USBD_ENABLE_INT(USBD_INTEN_WKEN_Msk);

    CLK_PowerDown();

    /* Clear PWR_DOWN_EN if it is not clear by itself */
    if(CLK->PWRCTL & CLK_PWRCTL_PDEN_Msk)
        CLK->PWRCTL ^= CLK_PWRCTL_PDEN_Msk;

    printf("device wakeup!\n");

    /* Lock protected registers */
    SYS_LockReg();
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
#if CRYSTAL_LESS
    uint32_t u32TrimInit;
#endif

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Open(UART0, 115200);

    printf("+-------------------------------------------------------+\n");
    printf("|          NuMicro USB MassStorage Sample Code          |\n");
    printf("+-------------------------------------------------------+\n");

    printf("NuMicro USB MassStorage Start!\n");

    /* initial SD card */
    SDH_Open(SDH0, CardDetect_From_GPIO);
    if(SDH_Probe(SDH0))
    {
        g_u8SdInitFlag = 0;
        printf("SD initial fail!!\n");
    }
    else
        g_u8SdInitFlag = 1;

    USBD_Open(&gsInfo, MSC_ClassRequest, NULL);

    USBD_SetConfigCallback(MSC_SetConfig);

    /* Endpoint configuration */
    MSC_Init();
    USBD_Start();

#if CRYSTAL_LESS
    /* Backup default trim */
    u32TrimInit = M32(TRIM_INIT);
#endif

    /* Clear SOF */
    USBD->INTSTS = USBD_INTSTS_SOFIF_Msk;

    NVIC_EnableIRQ(SDH0_IRQn);
    NVIC_SetPriority(USBD_IRQn, 1); // Because of USBD interrupt will make SDH read abnormal, so set USBD interrupt priority lower than SDH.
    NVIC_EnableIRQ(USBD_IRQn);

    while(1)
    {
#if CRYSTAL_LESS
        /* Start USB trim if it is not enabled. */
        if((SYS->TCTL48M & SYS_TCTL48M_FREQSEL_Msk) != 1)
        {
            /* Start USB trim only when SOF */
            if(USBD->INTSTS & USBD_INTSTS_SOFIF_Msk)
            {
                /* Clear SOF */
                USBD->INTSTS = USBD_INTSTS_SOFIF_Msk;

                /* Re-enable crystal-less */
                SYS->TCTL48M = 0x01;
                SYS->TCTL48M |= SYS_TCTL48M_REFCKSEL_Msk;
            }
        }

        /* Disable USB Trim when error */
        if(SYS->TISTS48M & (SYS_TISTS48M_CLKERRIF_Msk | SYS_TISTS48M_TFAILIF_Msk))
        {
            /* Init TRIM */
            M32(TRIM_INIT) = u32TrimInit;

            /* Disable crystal-less */
            SYS->TCTL48M = 0;

            /* Clear error flags */
            SYS->TISTS48M = SYS_TISTS48M_CLKERRIF_Msk | SYS_TISTS48M_TFAILIF_Msk;

            /* Clear SOF */
            USBD->INTSTS = USBD_INTSTS_SOFIF_Msk;
        }
#endif

        /* Enter power down when USB suspend */
        if(g_u8Suspend)
            PowerDown();

        MSC_ProcessCmd();
    }
}

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
