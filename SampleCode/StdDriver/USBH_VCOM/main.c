/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Use USB Host core driver and CDC driver. This sample demonstrates how
 *           to connect a CDC class VCOM device.
 *
 *
 * @copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <string.h>

#include "NuMicro.h"

#include "usbh_lib.h"
#include "usbh_cdc.h"

#define CLK_PLLCTL_192MHz_HXT   (CLK_PLLCTL_PLLSRC_HXT  | CLK_PLLCTL_NR(2) | CLK_PLLCTL_NF( 16) | CLK_PLLCTL_NO_1)

static char s_achLine[64];             /* Console input buffer */

static volatile int  g_i8RxReady = 0;

extern int kbhit(void);                        /* function in retarget.c                 */

static volatile uint32_t  s_u32TickCnt;

void SysTick_Handler(void);
void enable_sys_tick(int ticks_per_second);
void  dump_buff_hex(uint8_t *pu8Buff, int i8Bytes);
void  vcom_status_callback(CDC_DEV_T *cdev, uint8_t *pu8RData, int u8DataLen);
void  vcom_rx_callback(CDC_DEV_T *cdev, uint8_t *pu8RData, int u8DataLen);
void show_line_coding(LINE_CODING_T *lc);
int  init_cdc_device(CDC_DEV_T *cdev);
void SYS_Init(void);
void UART0_Init(void);


void SysTick_Handler(void)
{
    s_u32TickCnt++;
}

void enable_sys_tick(int ticks_per_second)
{
    s_u32TickCnt = 0;
    if(SysTick_Config(SystemCoreClock / (uint32_t)ticks_per_second))
    {
        /* Setup SysTick Timer for 1 second interrupts  */
        printf("Set system tick error!!\n");
        while(1);
    }
}

uint32_t get_ticks()
{
    return s_u32TickCnt;
}

/*
 *  This function is necessary for USB Host library.
 */
void delay_us(int usec)
{
    /*
     *  Configure Timer0, clock source from XTL_12M. Prescale 12
     */
    /* TIMER0 clock from HXT */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & (~CLK_CLKSEL1_TMR0SEL_Msk)) | CLK_CLKSEL1_TMR0SEL_HXT;
    CLK->APBCLK0 |= CLK_APBCLK0_TMR0CKEN_Msk;
    TIMER0->CTL = 0;        /* disable timer */
    TIMER0->INTSTS = (TIMER_INTSTS_TIF_Msk | TIMER_INTSTS_TWKF_Msk);   /* write 1 to clear for safety */
    TIMER0->CMP = usec;
    TIMER0->CTL = (11 << TIMER_CTL_PSC_Pos) | TIMER_ONESHOT_MODE | TIMER_CTL_CNTEN_Msk;

    while(!TIMER0->INTSTS);
}

void  dump_buff_hex(uint8_t *pu8Buff, int i8Bytes)
{
    int     i8Idx, i8Cnt;

    i8Idx = 0;
    while(i8Bytes > 0)
    {
        printf("0x%04X  ", i8Idx);
        for(i8Cnt = 0; (i8Cnt < 16) && (i8Bytes > 0); i8Cnt++)
        {
            printf("%02x ", pu8Buff[i8Idx + i8Cnt]);
            i8Bytes--;
        }
        i8Idx += 16;
        printf("\n");
    }
    printf("\n");
}

void  vcom_status_callback(CDC_DEV_T *cdev, uint8_t *pu8RData, int u8DataLen)
{
    int  i8Cnt;
	  
    (void)cdev;
    printf("[VCOM STS] ");
    for(i8Cnt = 0; i8Cnt < u8DataLen; i8Cnt++)
        printf("0x%02x ", pu8RData[i8Cnt]);
    printf("\n");
}

void  vcom_rx_callback(CDC_DEV_T *cdev, uint8_t *pu8RData, int u8DataLen)
{
    int  i8Cnt;
    (void)cdev; 

    //printf("[VCOM DATA %d] ", u8DataLen);
    for(i8Cnt = 0; i8Cnt < u8DataLen; i8Cnt++)
    {
        //printf("0x%02x ", pu8RData[i8Cnt]);
        printf("%c", pu8RData[i8Cnt]);
    }
    //printf("\n");

    g_i8RxReady = 1;
}

void show_line_coding(LINE_CODING_T *lc)
{
    printf("[CDC device line coding]\n");
    printf("====================================\n");
    printf("Baud rate:  %d bps\n", lc->baud);
    printf("Parity:     ");
    switch(lc->parity)
    {
        case 0:
            printf("None\n");
            break;
        case 1:
            printf("Odd\n");
            break;
        case 2:
            printf("Even\n");
            break;
        case 3:
            printf("Mark\n");
            break;
        case 4:
            printf("Space\n");
            break;
        default:
            printf("Invalid!\n");
            break;
    }
    printf("Data Bits:  ");
    switch(lc->data_bits)
    {
        case 5 :
        case 6 :
        case 7 :
        case 8 :
        case 16:
            printf("%d\n", lc->data_bits);
            break;
        default:
            printf("Invalid!\n");
            break;
    }
    printf("Stop Bits:  %s\n\n", (lc->stop_bits == 0) ? "1" : ((lc->stop_bits == 1) ? "1.5" : "2"));
}

int  init_cdc_device(CDC_DEV_T *cdev)
{
    int     i8Ret;
    LINE_CODING_T  line_code;

    printf("\n\n==================================\n");
    printf("  Init CDC device : 0x%x\n", (int)cdev);
    printf("  VID: 0x%x, PID: 0x%x\n\n", cdev->udev->descriptor.idVendor, cdev->udev->descriptor.idProduct);

    i8Ret = usbh_cdc_get_line_coding(cdev, &line_code);
    if(i8Ret < 0)
    {
        printf("Get Line Coding command failed: %d\n", i8Ret);
    }
    else
        show_line_coding(&line_code);

    line_code.baud = 115200;
    line_code.parity = 0;
    line_code.data_bits = 8;
    line_code.stop_bits = 0;

    i8Ret = usbh_cdc_set_line_coding(cdev, &line_code);
    if(i8Ret < 0)
    {
        printf("Set Line Coding command failed: %d\n", i8Ret);
    }

    i8Ret = usbh_cdc_get_line_coding(cdev, &line_code);
    if(i8Ret < 0)
    {
        printf("Get Line Coding command failed: %d\n", i8Ret);
    }
    else
    {
        printf("New line coding =>\n");
        show_line_coding(&line_code);
    }

    usbh_cdc_set_control_line_state(cdev, 1, 1);

    printf("usbh_cdc_start_polling_status...\n");
    usbh_cdc_start_polling_status(cdev, vcom_status_callback);

    printf("usbh_cdc_start_to_receive_data...\n");
    usbh_cdc_start_to_receive_data(cdev, vcom_rx_callback);

    return 0;
}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode */
    PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);

    /* Enable HXT clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Wait for HXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Switch HCLK clock source to HXT */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HXT, CLK_CLKDIV0_HCLK(1));

    /* Set PLL frequency */
    CLK->PLLCTL = CLK_PLLCTL_192MHz_HXT;

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_STATUS_PLLSTB_Msk);

    /* Switch HCLK clock source to PLL */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_PLL, CLK_CLKDIV0_HCLK(3));

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART module clock source as HXT and UART module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HXT, CLK_CLKDIV0_UART0(1));

    /* Enable USBH module clock */
    CLK_EnableModuleClock(USBH_MODULE);

    /* USB Host desired input clock is 48 MHz. Set as PLL divided by 4 (192/4 = 48) */
    CLK->CLKDIV0 = (CLK->CLKDIV0 & ~CLK_CLKDIV0_USBDIV_Msk) | CLK_CLKDIV0_USB(4);

    /* Enable USBD and OTG clock */
    CLK->APBCLK0 |= CLK_APBCLK0_USBDCKEN_Msk | CLK_APBCLK0_OTGCKEN_Msk;

    /* Set OTG as USB Host role */
    SYS->USBPHY = SYS_USBPHY_OTGPHYEN_Msk | SYS_USBPHY_SBO_Msk | (0x1 << SYS_USBPHY_USBROLE_Pos);

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & (~(UART0_RXD_PB12_Msk | UART0_TXD_PB13_Msk))) | UART0_RXD_PB12 | UART0_TXD_PB13;

    /* USB_VBUS_EN (USB 1.1 VBUS power enable pin) multi-function pin - PB.15     */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~SYS_GPB_MFPH_PB15MFP_Msk) | SYS_GPB_MFPH_PB15MFP_USB_VBUS_EN;

    /* USB_VBUS_ST (USB 1.1 over-current detect pin) multi-function pin - PB.14   */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~SYS_GPB_MFPH_PB14MFP_Msk) | SYS_GPB_MFPH_PB14MFP_USB_VBUS_ST;

    /* USB 1.1 port multi-function pin VBUS, D+, D-, and ID pins */
    SYS->GPA_MFPH &= ~(SYS_GPA_MFPH_PA12MFP_Msk | SYS_GPA_MFPH_PA13MFP_Msk |
                       SYS_GPA_MFPH_PA14MFP_Msk | SYS_GPA_MFPH_PA15MFP_Msk);
    SYS->GPA_MFPH |= SYS_GPA_MFPH_PA12MFP_USB_VBUS | SYS_GPA_MFPH_PA13MFP_USB_D_N |
                     SYS_GPA_MFPH_PA14MFP_USB_D_P | SYS_GPA_MFPH_PA15MFP_USB_OTG_ID;

    /* Lock protected registers */
    SYS_LockReg();
}

void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}


/*----------------------------------------------------------------------------
  MAIN function
 *----------------------------------------------------------------------------*/
int32_t main(void)
{
    CDC_DEV_T   *cdev;
    int         i8Ret;

    SYS_Init();                        /* Init System, IP clock and multi-function I/O */

    UART0_Init();                      /* Initialize UART0 */

    enable_sys_tick(100);

    printf("\n");
    printf("+----------------------------------------------------------+\n");
    printf("|            M2351 USB Host VCOM sample program            |\n");
    printf("+----------------------------------------------------------+\n");
    printf("|   (NOTE: This sample supports only one CDC device, but   |\n");
    printf("|          driver supports multiple CDC devices. If you    |\n");
    printf("|          want to support multiple CDC devices, you       |\n");
    printf("|          have to modify this sample.                     |\n");
    printf("+----------------------------------------------------------+\n");

    usbh_core_init();
    usbh_cdc_init();
    usbh_memory_used();

    while(1)
    {
        if(usbh_pooling_hubs())              /* USB Host port detect polling and management */
        {
            usbh_memory_used();              /* print out USB memory allocating information */

            cdev = usbh_cdc_get_device_list();
            if(cdev == NULL)
                continue;

            while(cdev != NULL)
            {
                init_cdc_device(cdev);

                if(cdev != NULL)
                    cdev = cdev->next;
            }
        }

        cdev = usbh_cdc_get_device_list();
        if(cdev == NULL)
            continue;

        if(g_i8RxReady)
        {
            g_i8RxReady = 0;

            if(cdev->rx_busy == 0)
                usbh_cdc_start_to_receive_data(cdev, vcom_rx_callback);
        }

        /*
         *  Check user input and send to CDC device immediately
         *  (You can also modify it send multiple characters at one time.)
         */
        if(kbhit() == 0)
        {
            s_achLine[0] = (char)getchar();
            i8Ret = usbh_cdc_send_data(cdev, (uint8_t *)s_achLine, 1);
            if(i8Ret != 0)
                printf("\n!! Send data failed, 0x%x!\n", i8Ret);
        }
    }
}


/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/
