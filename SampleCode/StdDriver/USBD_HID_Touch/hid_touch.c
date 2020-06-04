/******************************************************************************
 * @file     hid_touch.c
 * @brief    M2351 series USBD driver Sample file
 *
 * @note
 * Copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

/*!<Includes */
#include <string.h>
#include "NuMicro.h"
#include "hid_touch.h"

static uint8_t volatile s_u8EP2Ready = 0;
uint8_t volatile g_u8Suspend = 0;
static uint8_t s_u8Idle = 0, s_u8Protocol = 0;

void USBD_IRQHandler(void);

void USBD_IRQHandler(void)
{
    uint32_t u32IntSts = USBD_GET_INT_FLAG();
    uint32_t u32State = USBD_GET_BUS_STATE();

//------------------------------------------------------------------
    if(u32IntSts & USBD_INTSTS_FLDET)
    {
        // Floating detect
        USBD_CLR_INT_FLAG(USBD_INTSTS_FLDET);

        if(USBD_IS_ATTACHED())
        {
            /* USB Plug In */
            USBD_ENABLE_USB();
        }
        else
        {
            /* USB Un-plug */
            USBD_DISABLE_USB();
        }
    }

//------------------------------------------------------------------
    if(u32IntSts & USBD_INTSTS_WAKEUP)
    {
        /* Clear event flag */
        USBD_CLR_INT_FLAG(USBD_INTSTS_WAKEUP);
    }

//------------------------------------------------------------------
    if(u32IntSts & USBD_INTSTS_BUS)
    {
        /* Clear event flag */
        USBD_CLR_INT_FLAG(USBD_INTSTS_BUS);

        if(u32State & USBD_STATE_USBRST)
        {
            /* Bus reset */
            USBD_ENABLE_USB();
            USBD_SwReset();
            g_u8Suspend = 0;
        }
        if(u32State & USBD_STATE_SUSPEND)
        {
            /* Enter power down to wait USB attached */
            g_u8Suspend = 1;

            /* Enable USB but disable PHY */
            USBD_DISABLE_PHY();
        }
        if(u32State & USBD_STATE_RESUME)
        {
            /* Enable USB and enable PHY */
            USBD_ENABLE_USB();
            g_u8Suspend = 0;
        }
    }

//------------------------------------------------------------------
    if(u32IntSts & USBD_INTSTS_USB)
    {
        // USB event
        if(u32IntSts & USBD_INTSTS_SETUP)
        {
            // Setup packet
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_SETUP);

            /* Clear the data IN/OUT ready flag of control end-points */
            USBD_STOP_TRANSACTION(EP0);
            USBD_STOP_TRANSACTION(EP1);

            USBD_ProcessSetupPacket();
        }

        // EP events
        if(u32IntSts & USBD_INTSTS_EP0)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP0);
            // control IN
            USBD_CtrlIn();
        }

        if(u32IntSts & USBD_INTSTS_EP1)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP1);
            // control OUT
            USBD_CtrlOut();
        }

        if(u32IntSts & USBD_INTSTS_EP2)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP2);
            // Interrupt IN
            EP2_Handler();
        }

        if(u32IntSts & USBD_INTSTS_EP3)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP3);
        }

        if(u32IntSts & USBD_INTSTS_EP4)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP4);
        }

        if(u32IntSts & USBD_INTSTS_EP5)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP5);
        }

        if(u32IntSts & USBD_INTSTS_EP6)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP6);
        }

        if(u32IntSts & USBD_INTSTS_EP7)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP7);
        }

        if(u32IntSts & USBD_INTSTS_EP8)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP8);
        }

        if(u32IntSts & USBD_INTSTS_EP9)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP9);
        }

        if(u32IntSts & USBD_INTSTS_EP10)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP10);
        }

        if(u32IntSts & USBD_INTSTS_EP11)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP11);
        }
    }
}

void EP2_Handler(void)  /* Interrupt IN handler */
{
    s_u8EP2Ready = 1;
}


/*--------------------------------------------------------------------------*/
/**
  * @brief  USBD Endpoint Config.
  * @param  None.
  * @retval None.
  */
void HID_Init(void)
{
    /* Init setup packet buffer */
    /* Buffer range for setup packet -> [0 ~ 0x7] */
    USBD->STBUFSEG = SETUP_BUF_BASE;

    /*****************************************************/
    /* EP0 ==> control IN endpoint, address 0 */
    USBD_CONFIG_EP(EP0, USBD_CFG_CSTALL | USBD_CFG_EPMODE_IN | 0);
    /* Buffer range for EP0 */
    USBD_SET_EP_BUF_ADDR(EP0, EP0_BUF_BASE);

    /* EP1 ==> control OUT endpoint, address 0 */
    USBD_CONFIG_EP(EP1, USBD_CFG_CSTALL | USBD_CFG_EPMODE_OUT | 0);
    /* Buffer range for EP1 */
    USBD_SET_EP_BUF_ADDR(EP1, EP1_BUF_BASE);

    /*****************************************************/
    /* EP2 ==> Interrupt IN endpoint, address 1 */
    USBD_CONFIG_EP(EP2, USBD_CFG_EPMODE_IN | INT_IN_EP_NUM);
    /* Buffer range for EP2 */
    USBD_SET_EP_BUF_ADDR(EP2, EP2_BUF_BASE);

    /* Start to send IN data */
    s_u8EP2Ready = 1;
}

void HID_ClassRequest(void)
{
    uint8_t au8Buf[8];

    USBD_GetSetupPacket(au8Buf);

    if(au8Buf[0] & 0x80)    /* request data transfer direction */
    {
        // Device to host
        switch(au8Buf[1])
        {
            case GET_REPORT:
            {
                if(au8Buf[3] == HID_RPT_TYPE_INPUT)
                {
                    /* Report Type = input */
                    //DBG_PRINTF(" - Input\n");
                }
                else if(au8Buf[3] == HID_RPT_TYPE_FEATURE)
                {
                    /* Request Type = Feature */
                    /* report ID is 2 */
                    /* contact count maximum is 2 */
                    M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0)) = 2;
                    M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0) + 1) = 2;
                    /* Data stage */
                    USBD_SET_DATA1(EP0);
                    USBD_SET_PAYLOAD_LEN(EP0, 2);
                    /* Status stage */
                    USBD_PrepareCtrlOut(0, 0);
                }
                else
                {
                    // DBG_PRINTF(" - Unknown\n");
                    /* Setup error, stall the device */
                    USBD_SetStall(EP0);
                    USBD_SetStall(EP1);
                }
                break;
            }
            case GET_IDLE:
            {
                USBD_SET_PAYLOAD_LEN(EP1, au8Buf[6]);
                /* Data stage */
                USBD_PrepareCtrlIn(&s_u8Idle, au8Buf[6]);
                /* Status stage */
                USBD_PrepareCtrlOut(0, 0);
                break;
            }
            case GET_PROTOCOL:
            {
                USBD_SET_PAYLOAD_LEN(EP1, au8Buf[6]);
                /* Data stage */
                USBD_PrepareCtrlIn(&s_u8Protocol, au8Buf[6]);
                /* Status stage */
                USBD_PrepareCtrlOut(0, 0);
                break;
            }
            default:
            {
                /* Setup error, stall the device */
                USBD_SetStall(EP0);
                USBD_SetStall(EP1);
                break;
            }
        }
    }
    else
    {
        // Host to device
        switch(au8Buf[1])
        {
            case SET_REPORT:
            {
                if(au8Buf[3] == 2)
                {
                    /* Request Type = OUTPUT */
                    USBD_SET_DATA1(EP1);
                    USBD_SET_PAYLOAD_LEN(EP1, 0);
                }
                else if(au8Buf[3] == 3)
                {
                    /* Request Type = Feature */
                    USBD_SET_DATA1(EP1);
                    USBD_SET_PAYLOAD_LEN(EP1, 0);
                }
                break;
            }
            case SET_IDLE:
            {
                s_u8Idle = au8Buf[3];
                /* Status stage */
                USBD_SET_DATA1(EP0);
                USBD_SET_PAYLOAD_LEN(EP0, 0);
                break;
            }
            case SET_PROTOCOL:
            {
                s_u8Protocol = au8Buf[2];
                /* Status stage */
                USBD_SET_DATA1(EP0);
                USBD_SET_PAYLOAD_LEN(EP0, 0);
                break;
            }
            default:
            {
                // Stall
                /* Setup error, stall the device */
                USBD_SetStall(EP0);
                USBD_SetStall(EP1);
                break;
            }
        }
    }
}

/* two lines demo in Paint */
static uint32_t s_u32ReportCount = 0;
static uint8_t s_u8IsX1Send04 = 0;
void HID_UpdateTouchData(void)
{
    uint8_t *pu8Buf;
    static uint16_t u16X1 = 0x01f0, u16Y1 = 0x0100;

    if(s_u8EP2Ready)
    {

        pu8Buf = (uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP2));

        /* Report ID 1 */
        pu8Buf[0] = 1;
        pu8Buf[2] = 0;
        pu8Buf[8] = 1;


        s_u32ReportCount++;

        if((u16X1 >= 0x0200) && (u16X1 <= 0x0400))    // touchDown
        {
            s_u8IsX1Send04 = 1;

            pu8Buf[1] = 0x07;
            pu8Buf[3] = u16X1 & 0xff;
            pu8Buf[4] = (u16X1 >> 8) & 0xff;
            pu8Buf[5] = u16Y1 & 0xff;
            pu8Buf[6] = (u16Y1 >> 8) & 0xff;
            pu8Buf[7] = 0x07;
            pu8Buf[9] = u16X1 & 0xff;
            pu8Buf[10] = (u16X1 >> 8) & 0xff;
            pu8Buf[11] = (u16Y1 + 0x20) & 0xff;
            pu8Buf[12] = ((u16Y1 + 0x20) >> 8) & 0xff;
            pu8Buf[13] = 2;
        }
        else if(s_u8IsX1Send04)      // touchUp
        {
            s_u8IsX1Send04 = 0;
            pu8Buf[1] = 0x04;
            pu8Buf[3] = 0x00;
            pu8Buf[4] = 0x04;
            pu8Buf[5] = u16Y1 & 0xff;
            pu8Buf[6] = (u16Y1 >> 8) & 0xff;
            pu8Buf[7] = 0x04;
            pu8Buf[9] = 0x00;
            pu8Buf[10] = 0x04;
            pu8Buf[11] = (u16Y1 + 0x20) & 0xff;
            pu8Buf[12] = ((u16Y1 + 0x20) >> 8) & 0xff;
            pu8Buf[13] = 2;

            if(u16Y1 == 0x0100)
            {
                pu8Buf[5] = 0xE0;
                pu8Buf[6] = 0x02;
                pu8Buf[11] = 0x00;
                pu8Buf[12] = 0x03;
            }
            else
            {
                pu8Buf[5] = (u16Y1 - 0x50) & 0xff;
                pu8Buf[6] = ((u16Y1 - 0x50) >> 8) & 0xff;
                pu8Buf[11] = ((u16Y1 + 0x20) - 0x50) & 0xff;
                pu8Buf[12] = (((u16Y1 + 0x20) - 0x50) >> 8) & 0xff;
            }
        }
        else
        {
            pu8Buf[1] = 0;
            pu8Buf[3] = 0;
            pu8Buf[4] = 0;
            pu8Buf[5] = 0;
            pu8Buf[6] = 0;
            pu8Buf[7] = 0;
            pu8Buf[9] = 0;
            pu8Buf[10] = 0;
            pu8Buf[11] = 0;
            pu8Buf[12] = 0;
            pu8Buf[13] = 0;
        }

        if((s_u32ReportCount % 6) == 0)
            u16X1 += 0x3;

        if(u16X1 > 0x0400)
        {
            u16X1 = 0x01f0;
            u16Y1 += 0x50;
            if(u16Y1 > 0x0300)
                u16Y1 = 0x0100;
        }

        s_u8EP2Ready = 0;
        USBD_SET_PAYLOAD_LEN(EP2, 14);
    }
}

