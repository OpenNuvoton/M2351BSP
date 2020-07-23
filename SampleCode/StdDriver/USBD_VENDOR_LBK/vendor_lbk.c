/***************************************************************************//**
 * @file     vendor_lbk.c
 * @brief    M2351 series USBD Vendor LBK device driver Sample file
 * @version  2.0.0
 *
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

/*!<Includes */
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"
#include "vendor_lbk.h"

static uint8_t volatile s_u8EP3Ready = 0;      // EP3 for interrupt-out
static uint8_t volatile s_u8EP5Ready = 0;      // EP5 for isochronous-out
static uint8_t volatile s_u8EP7Ready = 0;      // EP7 for bulk-out

static volatile uint8_t  s_au8CtrlBuff[64];
static volatile uint8_t  s_au8IntBuff[64];
static volatile uint8_t  s_au8BulkBuff[64];
static volatile uint8_t  s_au8IsoBuff[256];

uint8_t volatile g_u8Suspend = 0;

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
    if(u32IntSts & USBD_INTSTS_WAKEUP)
    {
        /* Clear event flag */
        USBD_CLR_INT_FLAG(USBD_INTSTS_WAKEUP);
    }

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

            USBD_SET_PAYLOAD_LEN(EP2, EP2_MAX_PKT_SIZE);
        }

        if(u32IntSts & USBD_INTSTS_EP3)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP3);

            /* Interrupt-out data packet is ready */
            s_u8EP3Ready = 1;
        }

        if(u32IntSts & USBD_INTSTS_EP4)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP4);

            USBD_SET_PAYLOAD_LEN(EP4, EP4_MAX_PKT_SIZE);
        }

        if(u32IntSts & USBD_INTSTS_EP5)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP5);

            s_u8EP5Ready = 1;
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

            /* Bulk-out data packet is ready */
            s_u8EP7Ready = 1;
        }
    }
}

/*--------------------------------------------------------------------------*/
/**
  * @brief  USBD Endpoint Config.
  * @param  None.
  * @retval None.
  */
void VendorLBK_Init(void)
{
    memset((void *)(uint32_t)s_au8CtrlBuff, 0, sizeof(s_au8CtrlBuff));
    memset((void *)(uint32_t)s_au8IntBuff, 0, sizeof(s_au8IntBuff));
    memset((void *)(uint32_t)s_au8BulkBuff, 0, sizeof(s_au8BulkBuff));
    memset((void *)(uint32_t)s_au8IsoBuff, 0, sizeof(s_au8IsoBuff));

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
    /* EP2 ==> Interrupt IN endpoint, address 0x82 */
    USBD_CONFIG_EP(EP2, USBD_CFG_EPMODE_IN | INT_IN_EP_NUM);
    /* Buffer range for EP2 */
    USBD_SET_EP_BUF_ADDR(EP2, EP2_BUF_BASE);

    /* EP3 ==> Interrupt OUT endpoint, address 0x03 */
    USBD_CONFIG_EP(EP3, USBD_CFG_EPMODE_OUT | INT_OUT_EP_NUM);
    /* Buffer range for EP3 */
    USBD_SET_EP_BUF_ADDR(EP3, EP3_BUF_BASE);

    /*****************************************************/
    /* EP4 ==> Isochronous IN endpoint, address 0x84 */
    USBD_CONFIG_EP(EP4, USBD_CFG_EPMODE_IN | ISO_IN_EP_NUM | USBD_CFG_TYPE_ISO);
    /* Buffer range for EP4 */
    USBD_SET_EP_BUF_ADDR(EP4, EP4_BUF_BASE);

    /* EP5 ==> Isochronous OUT endpoint, address 0x05 */
    USBD_CONFIG_EP(EP5, USBD_CFG_EPMODE_OUT | ISO_OUT_EP_NUM | USBD_CFG_TYPE_ISO);
    /* Buffer range for EP5 */
    USBD_SET_EP_BUF_ADDR(EP5, EP5_BUF_BASE);

    /*****************************************************/
    /* EP6 ==> Bulk IN endpoint, address 0x86 */
    USBD_CONFIG_EP(EP6, USBD_CFG_EPMODE_IN | BULK_IN_EP_NUM);
    /* Buffer range for EP6 */
    USBD_SET_EP_BUF_ADDR(EP6, EP6_BUF_BASE);

    /* EP7 ==> Bulk OUT endpoint, address 0x07 */
    USBD_CONFIG_EP(EP7, USBD_CFG_EPMODE_OUT | BULK_OUT_EP_NUM);
    /* Buffer range for EP5 */
    USBD_SET_EP_BUF_ADDR(EP7, EP7_BUF_BASE);

    /* trigger to send interrupt-in packet */
    USBD_SET_PAYLOAD_LEN(EP2, EP2_MAX_PKT_SIZE);

    /* trigger to receive interrupt-out packet */
    USBD_SET_PAYLOAD_LEN(EP3, EP3_MAX_PKT_SIZE);

    /* trigger to send isochronous-in packet */
    USBD_SET_PAYLOAD_LEN(EP4, EP4_MAX_PKT_SIZE);

    /* trigger to receive isochronous-out packet */
    USBD_SET_PAYLOAD_LEN(EP5, EP5_MAX_PKT_SIZE);

    /* trigger to receive bulk-out data */
    USBD_SET_PAYLOAD_LEN(EP7, EP7_MAX_PKT_SIZE);
}

void VendorLBK_ClassRequest(void)
{
    uint8_t   buf[8];
    uint32_t   u32DataLen;

    USBD_GetSetupPacket(buf);

    u32DataLen = (uint32_t)((buf[7] << 8) | buf[6]);

    if(buf[0] & 0x80)    /* request data transfer direction */
    {
        // Device to host
        switch(buf[1])
        {
            case REQ_GET_DATA:
                if(u32DataLen > 64)
                    u32DataLen = 64;
                //USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0)), (uint8_t *)&SendBuffer, g_SendLen);
                /* Data stage */
                USBD_PrepareCtrlIn((uint8_t *)(uint32_t)s_au8CtrlBuff, u32DataLen);
                /* Status stage */
                USBD_PrepareCtrlOut(0, 0);
                break;

            default:
                /* Setup error, stall the device */
                USBD_SetStall(0);
                break;
        }
    }
    else
    {
        // Host to device
        switch(buf[1])
        {
            case REQ_SET_DATA:
                USBD_PrepareCtrlOut((uint8_t *)(uint32_t)s_au8CtrlBuff, buf[6]);

                /* Request Type = INPUT */
                USBD_SET_DATA1(EP1);
                USBD_SET_PAYLOAD_LEN(EP1, u32DataLen);
                /* Status stage */
                USBD_PrepareCtrlIn(0, 0);
                break;

            default:
                // Stall
                /* Setup error, stall the device */
                USBD_SetStall(0);
                break;
        }
    }
}

void VendorLBK_ProcessData(void)
{
    uint32_t  i, u32DataLen;

    /*
     * Process interrupt-out
     */
    if(s_u8EP3Ready)
    {
        s_u8EP3Ready = 0;
        u32DataLen = USBD_GET_PAYLOAD_LEN(EP3);
        if(u32DataLen > EP3_MAX_PKT_SIZE)
        {
            printf("Error! EP3 data overrun!\n");
            u32DataLen = EP3_MAX_PKT_SIZE;
        }

        /* Read interrupt-out packet from USBD SRAM to s_au8IntBuff[] */
        for(i = 0; i < u32DataLen; i++)
            s_au8IntBuff[i] = inpb(USBD_BUF_BASE + EP3_BUF_BASE + i);

        /* Write s_au8IntBuff[] to USBD SRAM as next interrupt-in packet */
        USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + EP2_BUF_BASE), (uint8_t *)(uint32_t)s_au8IntBuff, u32DataLen);

        USBD_SET_PAYLOAD_LEN(EP3, EP3_MAX_PKT_SIZE);
    }

    /*
     * Process isochronous-out
     */
    if(s_u8EP5Ready)
    {
        s_u8EP5Ready = 0;
        u32DataLen = USBD_GET_PAYLOAD_LEN(EP5);
        if(u32DataLen > EP5_MAX_PKT_SIZE)
        {
            printf("Error! EP5 data overrun!\n");
            u32DataLen = EP5_MAX_PKT_SIZE;
        }

        /* Read isochronous-out packet from USBD SRAM to s_au8IsoBuff[] */
        for(i = 0; i < u32DataLen; i++)
            s_au8IsoBuff[i] = inpb(USBD_BUF_BASE + EP5_BUF_BASE + i);

        /* Write s_au8IsoBuff[] to USBD SRAM as next isochronous-in packet */
        USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + EP4_BUF_BASE), (uint8_t *)(uint32_t)s_au8IsoBuff, u32DataLen);

        USBD_SET_PAYLOAD_LEN(EP5, EP5_MAX_PKT_SIZE);
    }

    /*
     * Process bulk-out
     */
    if(s_u8EP7Ready)
    {
        s_u8EP7Ready = 0;
        u32DataLen = USBD_GET_PAYLOAD_LEN(EP7);
        if(u32DataLen > EP7_MAX_PKT_SIZE)
        {
            printf("Error! EP7 data overrun!\n");
            u32DataLen = EP7_MAX_PKT_SIZE;
        }

        /* Read bulk-out packet from USBD SRAM to s_au8BulkBuff[] */
        for(i = 0; i < u32DataLen; i++)
            s_au8BulkBuff[i] = inpb(USBD_BUF_BASE + EP7_BUF_BASE + i);

        /* Write s_au8BulkBuff[] to USBD SRAM as next bulk-in packet */
        USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + EP6_BUF_BASE), (uint8_t *)(uint32_t)s_au8BulkBuff, u32DataLen);

        USBD_SET_PAYLOAD_LEN(EP6, u32DataLen);

        USBD_SET_PAYLOAD_LEN(EP7, EP7_MAX_PKT_SIZE);
    }
}


