/******************************************************************************
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate how to implement a USB multi virtual COM port device.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2024 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "vcom_serial.h"

#define CRYSTAL_LESS        1
#define TRIM_INIT           (SYS_BASE+0x10C)
#define TRIM_THRESHOLD      16      /* Each value is 0.125%, max 2% */

#define CLK_PLLCTL_144MHz_HXT   (CLK_PLLCTL_PLLSRC_HXT  | CLK_PLLCTL_NR(2) | CLK_PLLCTL_NF( 12) | CLK_PLLCTL_NO_1)

#if CRYSTAL_LESS
static volatile uint32_t s_u32DefaultTrim, s_u32LastTrim;
#endif

/*--------------------------------------------------------------------------*/
STR_VCOM_LINE_CODING g_LineCoding0 = {115200, 0, 0, 8};   /* Baud rate : 115200    */
STR_VCOM_LINE_CODING g_LineCoding1 = {115200, 0, 0, 8};   /* Baud rate : 115200    */
STR_VCOM_LINE_CODING g_LineCoding2 = {115200, 0, 0, 8};   /* Baud rate : 115200    */
/* Stop bit     */
/* parity       */
/* data bits    */
uint16_t g_u16CtrlSignal0 = 0;     /* BIT0: DTR(Data Terminal Ready) , BIT1: RTS(Request To Send) */
uint16_t g_u16CtrlSignal1 = 0;     /* BIT0: DTR(Data Terminal Ready) , BIT1: RTS(Request To Send) */
uint16_t g_u16CtrlSignal2 = 0;     /* BIT0: DTR(Data Terminal Ready) , BIT1: RTS(Request To Send) */

/*--------------------------------------------------------------------------*/
#define RXBUFSIZE           512 /* RX buffer size */
#define TXBUFSIZE           512 /* RX buffer size */

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
#if (VCOM_CNT >= 1)
/* UART0 */
static volatile uint8_t s_au8ComRbuf0[RXBUFSIZE];
volatile uint16_t g_u16ComRbytes0 = 0;
volatile uint16_t g_u16ComRhead0 = 0;
volatile uint16_t g_u16ComRtail0 = 0;

static volatile uint8_t s_au8ComTbuf0[TXBUFSIZE];
volatile uint16_t g_u16ComTbytes0 = 0;
volatile uint16_t g_u16ComThead0 = 0;
volatile uint16_t g_u16ComTtail0 = 0;

static uint8_t s_au8RxBuf0[64] = {0};
uint8_t *g_pu8RxBuf0 = 0;
uint32_t g_u32RxSize0 = 0;
uint32_t g_u32TxSize0 = 0;

volatile int8_t g_i8BulkOutReady0 = 0;
#endif

#if (VCOM_CNT >= 2)
/* UART1 */
static volatile uint8_t s_au8ComRbuf1[RXBUFSIZE];
volatile uint16_t g_u16ComRbytes1 = 0;
volatile uint16_t g_u16ComRhead1 = 0;
volatile uint16_t g_u16ComRtail1 = 0;

static volatile uint8_t s_au8ComTbuf1[TXBUFSIZE];
volatile uint16_t g_u16ComTbytes1 = 0;
volatile uint16_t g_u16ComThead1 = 0;
volatile uint16_t g_u16ComTtail1 = 0;

static uint8_t s_au8RxBuf1[64] = {0};
uint8_t *g_pu8RxBuf1 = 0;
uint32_t g_u32RxSize1 = 0;
uint32_t g_u32TxSize1 = 0;

volatile int8_t g_i8BulkOutReady1 = 0;
#endif

#if (VCOM_CNT >= 3)
/* UART2 */
static volatile uint8_t s_au8ComRbuf2[RXBUFSIZE];
volatile uint16_t g_u16ComRbytes2 = 0;
volatile uint16_t g_u16ComRhead2 = 0;
volatile uint16_t g_u16ComRtail2 = 0;

static volatile uint8_t s_au8ComTbuf2[TXBUFSIZE];
volatile uint16_t g_u16ComTbytes2 = 0;
volatile uint16_t g_u16ComThead2 = 0;
volatile uint16_t g_u16ComTtail2 = 0;

static uint8_t s_au8RxBuf2[64] = {0};
uint8_t *g_pu8RxBuf2 = 0;
uint32_t g_u32RxSize2 = 0;
uint32_t g_u32TxSize2 = 0;

volatile int8_t g_i8BulkOutReady2 = 0;
#endif

void SYS_Init(void);
void UART0_Init(void);
void UART1_Init(void);
void UART2_Init(void);
void UART0_IRQHandler(void);
void UART1_IRQHandler(void);
void UART2_IRQHandler(void);
void PowerDown(void);
/*---------------------------------------------------------------------------------------------------------*/

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

    /* Enable UART module clock and select UART module clock source */
#if (VCOM_CNT >= 1)
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));
#endif

#if (VCOM_CNT >= 2)
    CLK_EnableModuleClock(UART1_MODULE);
    CLK_SetModuleClock(UART1_MODULE, CLK_CLKSEL1_UART1SEL_HIRC, CLK_CLKDIV0_UART1(1));
#endif

#if (VCOM_CNT >= 3)
    CLK_EnableModuleClock(UART2_MODULE);
    CLK_SetModuleClock(UART2_MODULE, CLK_CLKSEL3_UART2SEL_HIRC, CLK_CLKDIV4_UART2(1));
#endif

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* USBD multi-function pins for VBUS, D+, D-, and ID pins */
    SET_USB_VBUS_PA12();
    SET_USB_D_N_PA13();
    SET_USB_D_P_PA14();
    SET_USB_OTG_ID_PA15();

    /* Set multi-function pins for UART RXD and TXD */
#if (VCOM_CNT >= 1)
    SET_UART0_RXD_PB12();
    SET_UART0_TXD_PB13();
#endif

#if (VCOM_CNT >= 2)
    SET_UART1_RXD_PA2();
    SET_UART1_TXD_PA3();
#endif

#if (VCOM_CNT >= 3)
    SET_UART2_RXD_PC4();
    SET_UART2_TXD_PC5();
#endif
}

#if (VCOM_CNT >= 1)
void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART */
    SYS_ResetModule(UART0_RST);

    /* Configure UART and set UART Baudrate */
    UART_Open(UART0, 115200);

    /* Enable Interrupt and install the call back function */
    UART_ENABLE_INT(UART0, (UART_INTEN_RDAIEN_Msk | UART_INTEN_THREIEN_Msk | UART_INTEN_RXTOIEN_Msk));
}
#endif

#if (VCOM_CNT >= 2)
void UART1_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART */
    SYS_ResetModule(UART1_RST);

    /* Configure UART and set UART Baudrate */
    UART_Open(UART1, 115200);

    /* Enable Interrupt and install the call back function */
    UART_ENABLE_INT(UART1, (UART_INTEN_RDAIEN_Msk | UART_INTEN_THREIEN_Msk | UART_INTEN_RXTOIEN_Msk));
}
#endif

#if (VCOM_CNT >= 3)
void UART2_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART */
    SYS_ResetModule(UART2_RST);

    /* Configure UART and set UART Baudrate */
    UART_Open(UART2, 115200);

    /* Enable Interrupt and install the call back function */
    UART_ENABLE_INT(UART2, (UART_INTEN_RDAIEN_Msk | UART_INTEN_THREIEN_Msk | UART_INTEN_RXTOIEN_Msk));
}
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* UART Callback function                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
#if (VCOM_CNT >= 1)
void UART0_IRQHandler(void)
{
    uint8_t u8InChar;
    int32_t i32Size;
    uint32_t u32IntStatus;

    u32IntStatus = UART0->INTSTS;

    if((u32IntStatus & UART_INTSTS_RDAIF_Msk) || (u32IntStatus & UART_INTSTS_RXTOIF_Msk))
    {
        /* Receiver FIFO threshold level is reached or Rx time out */

        /* Get all the input characters */
        while(!(UART0->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk))
        {
            /* Get the character from UART Buffer */
            u8InChar = (uint8_t)UART0->DAT;

            /* Check if buffer full */
            if(g_u16ComRbytes0 < RXBUFSIZE)
            {
                /* Enqueue the character */
                s_au8ComRbuf0[g_u16ComRtail0++] = u8InChar;
                if(g_u16ComRtail0 >= RXBUFSIZE)
                {
                    g_u16ComRtail0 = 0;
                }
                g_u16ComRbytes0++;
            }
            else
            {
                /* FIFO over run */
            }
        }
    }

    if(u32IntStatus & UART_INTSTS_THREIF_Msk)
    {

        if(g_u16ComTbytes0 && (UART0->INTEN & UART_INTEN_THREIEN_Msk))
        {
            /* Fill the Tx FIFO */
            i32Size = g_u16ComTbytes0;
            if(i32Size >= UART0_FIFO_SIZE)
            {
                i32Size = UART0_FIFO_SIZE;
            }

            while(i32Size)
            {
                u8InChar = s_au8ComTbuf0[g_u16ComThead0++];
                UART0->DAT = u8InChar;
                if(g_u16ComThead0 >= TXBUFSIZE)
                {
                    g_u16ComThead0 = 0;
                }
                g_u16ComTbytes0--;
                i32Size--;
            }
        }
        else
        {
            /* No more data, just stop Tx (Stop work) */
            UART0->INTEN &= ~UART_INTEN_THREIEN_Msk;
        }
    }
}
#endif

#if (VCOM_CNT >= 2)
void UART1_IRQHandler(void)
{
    uint8_t u8InChar;
    int32_t i32Size;
    uint32_t u32IntStatus;

    u32IntStatus = UART1->INTSTS;

    if((u32IntStatus & UART_INTSTS_RDAIF_Msk) || (u32IntStatus & UART_INTSTS_RXTOIF_Msk))
    {
        /* Receiver FIFO threshold level is reached or Rx time out */

        /* Get all the input characters */
        while(!(UART1->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk))
        {
            /* Get the character from UART Buffer */
            u8InChar = (uint8_t)UART1->DAT;

            /* Check if buffer full */
            if(g_u16ComRbytes1 < RXBUFSIZE)
            {
                /* Enqueue the character */
                s_au8ComRbuf1[g_u16ComRtail1++] = u8InChar;
                if(g_u16ComRtail1 >= RXBUFSIZE)
                {
                    g_u16ComRtail1 = 0;
                }
                g_u16ComRbytes1++;
            }
            else
            {
                /* FIFO over run */
            }
        }
    }

    if(u32IntStatus & UART_INTSTS_THREIF_Msk)
    {

        if(g_u16ComTbytes1 && (UART1->INTEN & UART_INTEN_THREIEN_Msk))
        {
            /* Fill the Tx FIFO */
            i32Size = g_u16ComTbytes1;
            if(i32Size >= UART1_FIFO_SIZE)
            {
                i32Size = UART1_FIFO_SIZE;
            }

            while(i32Size)
            {
                u8InChar = s_au8ComTbuf1[g_u16ComThead1++];
                UART1->DAT = u8InChar;
                if(g_u16ComThead1 >= TXBUFSIZE)
                    g_u16ComThead1 = 0;
                g_u16ComTbytes1--;
                i32Size--;
            }
        }
        else
        {
            /* No more data, just stop Tx (Stop work) */
            UART1->INTEN &= ~UART_INTEN_THREIEN_Msk;
        }
    }
}
#endif

#if (VCOM_CNT >= 3)
void UART2_IRQHandler(void)
{
    uint8_t u8InChar;
    int32_t i32Size;
    uint32_t u32IntStatus;

    u32IntStatus = UART2->INTSTS;

    if((u32IntStatus & UART_INTSTS_RDAIF_Msk) || (u32IntStatus & UART_INTSTS_RXTOIF_Msk))
    {
        /* Receiver FIFO threshold level is reached or Rx time out */

        /* Get all the input characters */
        while(!(UART2->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk))
        {
            /* Get the character from UART Buffer */
            u8InChar = (uint8_t)UART2->DAT;

            /* Check if buffer full */
            if(g_u16ComRbytes2 < RXBUFSIZE)
            {
                /* Enqueue the character */
                s_au8ComRbuf2[g_u16ComRtail2++] = u8InChar;
                if(g_u16ComRtail2 >= RXBUFSIZE)
                {
                    g_u16ComRtail2 = 0;
                }
                g_u16ComRbytes2++;
            }
            else
            {
                /* FIFO over run */
            }
        }
    }

    if(u32IntStatus & UART_INTSTS_THREIF_Msk)
    {

        if(g_u16ComTbytes2 && (UART2->INTEN & UART_INTEN_THREIEN_Msk))
        {
            /* Fill the Tx FIFO */
            i32Size = g_u16ComTbytes2;
            if(i32Size >= UART2_FIFO_SIZE)
            {
                i32Size = UART2_FIFO_SIZE;
            }

            while(i32Size)
            {
                u8InChar = s_au8ComTbuf2[g_u16ComThead2++];
                UART2->DAT = u8InChar;
                if(g_u16ComThead2 >= TXBUFSIZE)
                {
                    g_u16ComThead2 = 0;
                }
                g_u16ComTbytes2--;
                i32Size--;
            }
        }
        else
        {
            /* No more data, just stop Tx (Stop work) */
            UART2->INTEN &= ~UART_INTEN_THREIEN_Msk;
        }
    }
}
#endif

void VCOM_TransferData(void)
{
    uint32_t i, u32Len;

    /* Check whether USB is ready for next packet or not */
#if (VCOM_CNT >= 1)
    if(g_u32TxSize0 == 0)
    {
        /* Check whether we have new COM Rx data to send to USB or not */
        if(g_u16ComRbytes0)
        {
            u32Len = g_u16ComRbytes0;
            if(u32Len > EP2_MAX_PKT_SIZE)
            {
                u32Len = EP2_MAX_PKT_SIZE;
            }

            for(i = 0; i < u32Len; i++)
            {
                s_au8RxBuf0[i] = s_au8ComRbuf0[g_u16ComRhead0++];
                if(g_u16ComRhead0 >= RXBUFSIZE)
                {
                    g_u16ComRhead0 = 0;
                }
            }

            __set_PRIMASK(1);
            g_u16ComRbytes0 -= u32Len;
            __set_PRIMASK(0);

            g_u32TxSize0 = u32Len;
            USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP2)), (uint8_t *)s_au8RxBuf0, u32Len);
            USBD_SET_PAYLOAD_LEN(EP2, u32Len);
        }
        else
        {
            /* Prepare a zero packet if previous packet size is EP2_MAX_PKT_SIZE and
               no more data to send at this moment to note Host the transfer has been done */
            u32Len = USBD_GET_PAYLOAD_LEN(EP2);
            if(u32Len == EP2_MAX_PKT_SIZE)
                USBD_SET_PAYLOAD_LEN(EP2, 0);
        }
    }
#endif

#if (VCOM_CNT >= 2)
    if(g_u32TxSize1 == 0)
    {
        /* Check whether we have new COM Rx data to send to USB or not */
        if(g_u16ComRbytes1)
        {
            u32Len = g_u16ComRbytes1;
            if(u32Len > EP7_MAX_PKT_SIZE)
            {
                u32Len = EP7_MAX_PKT_SIZE;
            }

            for(i = 0; i < u32Len; i++)
            {
                s_au8RxBuf1[i] = s_au8ComRbuf1[g_u16ComRhead1++];
                if(g_u16ComRhead1 >= RXBUFSIZE)
                {
                    g_u16ComRhead1 = 0;
                }
            }

            __set_PRIMASK(1);
            g_u16ComRbytes1 -= u32Len;
            __set_PRIMASK(0);

            g_u32TxSize1 = u32Len;
            USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP7)), (uint8_t *)s_au8RxBuf1, u32Len);
            USBD_SET_PAYLOAD_LEN(EP7, u32Len);
        }
        else
        {
            /* Prepare a zero packet if previous packet size is EP7_MAX_PKT_SIZE and
               no more data to send at this moment to note Host the transfer has been done */
            u32Len = USBD_GET_PAYLOAD_LEN(EP7);
            if(u32Len == EP7_MAX_PKT_SIZE)
                USBD_SET_PAYLOAD_LEN(EP7, 0);
        }
    }
#endif

#if (VCOM_CNT >= 3)
    if(g_u32TxSize2 == 0)
    {
        /* Check whether we have new COM Rx data to send to USB or not */
        if(g_u16ComRbytes2)
        {
            u32Len = g_u16ComRbytes2;
            if(u32Len > EP10_MAX_PKT_SIZE)
            {
                u32Len = EP10_MAX_PKT_SIZE;
            }

            for(i = 0; i < u32Len; i++)
            {
                s_au8RxBuf2[i] = s_au8ComRbuf2[g_u16ComRhead2++];
                if(g_u16ComRhead2 >= RXBUFSIZE)
                {
                    g_u16ComRhead2 = 0;
                }
            }

            __set_PRIMASK(1);
            g_u16ComRbytes2 -= u32Len;
            __set_PRIMASK(0);

            g_u32TxSize2 = u32Len;
            USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP10)), (uint8_t *)s_au8RxBuf2, u32Len);
            USBD_SET_PAYLOAD_LEN(EP10, u32Len);
        }
        else
        {
            /* Prepare a zero packet if previous packet size is EP7_MAX_PKT_SIZE and
               no more data to send at this moment to note Host the transfer has been done */
            u32Len = USBD_GET_PAYLOAD_LEN(EP10);
            if(u32Len == EP10_MAX_PKT_SIZE)
                USBD_SET_PAYLOAD_LEN(EP10, 0);
        }
    }
#endif

#if (VCOM_CNT >= 1)
    /* Process the Bulk out data when bulk out data is ready. */
    if(g_i8BulkOutReady0 && (g_u32RxSize0 <= TXBUFSIZE - g_u16ComTbytes0))
    {
        for(i = 0; i < g_u32RxSize0; i++)
        {
            s_au8ComTbuf0[g_u16ComTtail0++] = g_pu8RxBuf0[i];
            if(g_u16ComTtail0 >= TXBUFSIZE)
            {
                g_u16ComTtail0 = 0;
            }
        }

        __set_PRIMASK(1);
        g_u16ComTbytes0 += g_u32RxSize0;
        __set_PRIMASK(0);

        g_u32RxSize0 = 0;
        g_i8BulkOutReady0 = 0; /* Clear bulk out ready flag */

        /* Ready to get next BULK out */
        USBD_SET_PAYLOAD_LEN(EP3, EP3_MAX_PKT_SIZE);
    }
#endif

#if (VCOM_CNT >= 2)
    if(g_i8BulkOutReady1 && (g_u32RxSize1 <= TXBUFSIZE - g_u16ComTbytes1))
    {
        for(i = 0; i < g_u32RxSize1; i++)
        {
            s_au8ComTbuf1[g_u16ComTtail1++] = g_pu8RxBuf1[i];
            if(g_u16ComTtail1 >= TXBUFSIZE)
            {
                g_u16ComTtail1 = 0;
            }
        }

        __set_PRIMASK(1);
        g_u16ComTbytes1 += g_u32RxSize1;
        __set_PRIMASK(0);

        g_u32RxSize1 = 0;
        g_i8BulkOutReady1 = 0; /* Clear bulk out ready flag */

        /* Ready to get next BULK out */
        USBD_SET_PAYLOAD_LEN(EP6, EP6_MAX_PKT_SIZE);
    }
#endif

#if (VCOM_CNT >= 3)
    if(g_i8BulkOutReady2 && (g_u32RxSize2 <= TXBUFSIZE - g_u16ComTbytes2))
    {
        for(i = 0; i < g_u32RxSize2; i++)
        {
            s_au8ComTbuf2[g_u16ComTtail2++] = g_pu8RxBuf2[i];
            if(g_u16ComTtail2 >= TXBUFSIZE)
            {
                g_u16ComTtail2 = 0;
            }
        }

        __set_PRIMASK(1);
        g_u16ComTbytes2 += g_u32RxSize2;
        __set_PRIMASK(0);

        g_u32RxSize2 = 0;
        g_i8BulkOutReady2 = 0; /* Clear bulk out ready flag */

        /* Ready to get next BULK out */
        USBD_SET_PAYLOAD_LEN(EP9, EP9_MAX_PKT_SIZE);
    }
#endif

    /* Process the software Tx FIFO */
#if (VCOM_CNT >= 1)
    if(g_u16ComTbytes0)
    {
        /* Check if Tx is working */
        if((UART0->INTEN & UART_INTEN_THREIEN_Msk) == 0)
        {
            /* Send one bytes out */
            UART0->DAT = s_au8ComTbuf0[g_u16ComThead0++];
            if(g_u16ComThead0 >= TXBUFSIZE)
            {
                g_u16ComThead0 = 0;
            }

            g_u16ComTbytes0--;

            /* Enable Tx Empty Interrupt. (Trigger first one) */
            UART0->INTEN |= UART_INTEN_THREIEN_Msk;
        }
    }
#endif

#if (VCOM_CNT >= 2)
    if(g_u16ComTbytes1)
    {
        /* Check if Tx is working */
        if((UART1->INTEN & UART_INTEN_THREIEN_Msk) == 0)
        {
            /* Send one bytes out */
            UART1->DAT = s_au8ComTbuf1[g_u16ComThead1++];
            if(g_u16ComThead1 >= TXBUFSIZE)
            {
                g_u16ComThead1 = 0;
            }

            g_u16ComTbytes1--;

            /* Enable Tx Empty Interrupt. (Trigger first one) */
            UART1->INTEN |= UART_INTEN_THREIEN_Msk;
        }
    }
#endif

#if (VCOM_CNT >= 3)
    if(g_u16ComTbytes2)
    {
        /* Check if Tx is working */
        if((UART2->INTEN & UART_INTEN_THREIEN_Msk) == 0)
        {
            /* Send one bytes out */
            UART2->DAT = s_au8ComTbuf2[g_u16ComThead2++];
            if(g_u16ComThead2 >= TXBUFSIZE)
            {
                g_u16ComThead2 = 0;
            }

            g_u16ComTbytes2--;

            /* Enable Tx Empty Interrupt. (Trigger first one) */
            UART2->INTEN |= UART_INTEN_THREIEN_Msk;
        }
    }
#endif
}

void PowerDown(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Wakeup Enable */
    USBD_ENABLE_INT(USBD_INTEN_WKEN_Msk);

    CLK_PowerDown();

    /* Clear PWR_DOWN_EN if it is not clear by itself */
    if(CLK->PWRCTL & CLK_PWRCTL_PDEN_Msk)
    {
        CLK->PWRCTL ^= CLK_PWRCTL_PDEN_Msk;
    }

    /* Lock protected registers */
    SYS_LockReg();
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART */
#if (VCOM_CNT >= 1)
    UART0_Init();
#endif

#if (VCOM_CNT >= 2)
    UART1_Init();
#endif

#if (VCOM_CNT >= 3)
    UART2_Init();
#endif

    printf("\n");
    printf("+-------------------------------------------------------------+\n");
    printf("|        NuMicro USB Virtual COM Multi-Port Sample Code       |\n");
    printf("+-------------------------------------------------------------+\n");

    USBD_Open(&gsInfo, VCOM_ClassRequest, NULL);

    /* Endpoint configuration */
    VCOM_Init();
    USBD_Start();

#if (VCOM_CNT >= 1)
    NVIC_EnableIRQ(UART0_IRQn);
#endif

#if (VCOM_CNT >= 2)
    NVIC_EnableIRQ(UART1_IRQn);
#endif

#if (VCOM_CNT >= 3)
    NVIC_EnableIRQ(UART2_IRQn);
#endif

    NVIC_EnableIRQ(USBD_IRQn);

#if CRYSTAL_LESS
    /* Backup default trim */
    s_u32DefaultTrim = M32(TRIM_INIT);
    s_u32LastTrim = s_u32DefaultTrim;
#endif

    /* Clear SOF */
    USBD->INTSTS = USBD_INTSTS_SOFIF_Msk;

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
            /* Last TRIM */
            M32(TRIM_INIT) = s_u32LastTrim;

            /* Disable crystal-less */
            SYS->TCTL48M = 0;

            /* Clear error flags */
            SYS->TISTS48M = SYS_TISTS48M_CLKERRIF_Msk | SYS_TISTS48M_TFAILIF_Msk;

            /* Clear SOF */
            USBD->INTSTS = USBD_INTSTS_SOFIF_Msk;
        }

        /* Check trim value whether it is over the threshold */
        if((M32(TRIM_INIT) > (s_u32DefaultTrim + TRIM_THRESHOLD)) || (M32(TRIM_INIT) < (s_u32DefaultTrim - TRIM_THRESHOLD)))
        {
            /* Write updated value */
            M32(TRIM_INIT) = s_u32LastTrim;
        }
        else
        {
            /* Backup trim value */
            s_u32LastTrim = M32(TRIM_INIT);
        }
#endif

        /* Enter power down when USB suspend */
        if(g_u8Suspend)
        {
            PowerDown();
        }

        VCOM_TransferData();
    }
}
