/******************************************************************************
 * @file     ccid.c
 * @version  V2.00
 * @brief    M2351 USBD CCID sample file
 * @note
 * Copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
/*!<Includes */
#include <string.h>
#include "NuMicro.h"
#include "ccid.h"
#include "ccid_if.h"

static uint32_t volatile s_u32OutToggle = 0;

void USBD_IRQHandler(void);
void EP4_Handler(void);

/*--------------------------------------------------------------------------*/
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
            s_u32OutToggle = 0;
        }
        if(u32State & USBD_STATE_SUSPEND)
        {
            /* Enable USB but disable PHY */
            USBD_DISABLE_PHY();
        }
        if(u32State & USBD_STATE_RESUME)
        {
            /* Enable USB and enable PHY */
            USBD_ENABLE_USB();
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
            // Bulk IN
            EP2_Handler();
        }

        if(u32IntSts & USBD_INTSTS_EP3)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP3);
            // Bulk Out
            EP3_Handler();
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

void EP2_Handler(void)
{
    uint32_t u32Length;

    /* BULK IN transfer */
    if(gu8IsBulkInReady)
    {
        if(gi32UsbdMessageLength >= EP2_MAX_PKT_SIZE)
        {
            gu8IsBulkInReady = 1;
            USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP2)), pUsbMessageBuffer, EP2_MAX_PKT_SIZE);
            USBD_SET_PAYLOAD_LEN(EP2, EP2_MAX_PKT_SIZE);

            pUsbMessageBuffer += EP2_MAX_PKT_SIZE;
            gi32UsbdMessageLength -= EP2_MAX_PKT_SIZE;
        }
        else
        {
            u32Length = (uint32_t)gi32UsbdMessageLength;
            USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP2)), pUsbMessageBuffer, u32Length);
            USBD_SET_PAYLOAD_LEN(EP2, (uint32_t)gi32UsbdMessageLength);
            gi32UsbdMessageLength = 0;
            gu8IsBulkInReady = 0;
        }
    }
    if(!gu8IsBulkOutReady)
        USBD_SET_PAYLOAD_LEN(EP3, EP3_MAX_PKT_SIZE);
}

void EP3_Handler(void)
{
    /* BULK OUT */
    static int offset = 0;
    uint32_t len;

    if(s_u32OutToggle == (USBD->EPSTS0 & USBD_EPSTS0_EPSTS3_Msk))
    {
        USBD_SET_PAYLOAD_LEN(EP3, EP3_MAX_PKT_SIZE);
    }
    else
    {
        s_u32OutToggle = USBD->EPSTS0 & USBD_EPSTS0_EPSTS3_Msk;
        len = USBD_GET_PAYLOAD_LEN(EP3);

        USBD_MemCopy(&UsbMessageBuffer[offset], (uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP3)), len);

        if((len >= 0x0A && len != 0xFF) || offset != 0)
        {
            if(offset == 0)
            {
                /* Calculate number of byte to receive to finish the message  */
                gi32UsbdMessageLength = (int32_t)(USB_MESSAGE_HEADER_SIZE + make32(&UsbMessageBuffer[OFFSET_DWLENGTH]));
            }

            gi32UsbdMessageLength -= (int) len;
            /* Prepare next reception if whole message not received */
            if(gi32UsbdMessageLength > 0)
            {
                pUsbMessageBuffer = UsbMessageBuffer + len;
                offset += len;
            }

            if(gi32UsbdMessageLength == 0)
            {
                gu8IsBulkOutReady = 1;
                offset = 0;
            }
            if(gi32UsbdMessageLength < 0)
            {
                UsbMessageBuffer[OFFSET_DWLENGTH] = 0xFF;
                UsbMessageBuffer[OFFSET_DWLENGTH + 1] = 0xFF;
                UsbMessageBuffer[OFFSET_DWLENGTH + 2] = 0xFF;
                UsbMessageBuffer[OFFSET_DWLENGTH + 3] = 0xFF;
                gu8IsBulkOutReady = 1;
            }
        }
        CCID_DispatchMessage();

        /* trigger next out packet */
        if(gi32UsbdMessageLength > 0)
            USBD_SET_PAYLOAD_LEN(EP3, EP3_MAX_PKT_SIZE);
    }
}

void EP4_Handler(void)
{
    /* INT IN transfer */
    if(gu8IsDeviceReady)
    {
        RDR_to_PC_NotifySlotChange();
        USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP4)), pu8IntInBuf, 2);
        USBD_SET_PAYLOAD_LEN(EP4, 2);
        gu8IsDeviceReady = 0;
    }
    if(!gu8IsBulkOutReady)
        USBD_SET_PAYLOAD_LEN(EP3, EP3_MAX_PKT_SIZE);
}

/*--------------------------------------------------------------------------*/
/**
  * @brief  USBD Endpoint Config.
  * @param  None.
  * @retval None.
  */
void CCID_Init(void)
{
    /* Init setup packet buffer */
    /* Buffer for setup packet -> [0 ~ 0x7] */
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
    /* EP2 ==> Bulk IN endpoint, address 2 */
    USBD_CONFIG_EP(EP2, USBD_CFG_EPMODE_IN | BULK_IN_EP_NUM);
    /* Buffer offset for EP2 */
    USBD_SET_EP_BUF_ADDR(EP2, EP2_BUF_BASE);

    /* EP3 ==> Bulk Out endpoint, address 2 */
    USBD_CONFIG_EP(EP3, USBD_CFG_EPMODE_OUT | BULK_OUT_EP_NUM);
    /* Buffer offset for EP3 */
    USBD_SET_EP_BUF_ADDR(EP3, EP3_BUF_BASE);
    /* trigger receive OUT data */
    USBD_SET_PAYLOAD_LEN(EP3, EP3_MAX_PKT_SIZE);

    /* EP4 ==> Interrupt IN endpoint, address 3 */
    USBD_CONFIG_EP(EP4, USBD_CFG_EPMODE_IN | INT_IN_EP_NUM);
    /* Buffer offset for EP4 ->  */
    USBD_SET_EP_BUF_ADDR(EP4, EP4_BUF_BASE);

    /* check card state */
    gu8IsDeviceReady = 1;
    pu8IntInBuf = &UsbIntMessageBuffer[0];
    pUsbMessageBuffer = &UsbMessageBuffer[0];
    RDR_to_PC_NotifySlotChange();
    USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP4)), pu8IntInBuf, 2);
    USBD_SET_PAYLOAD_LEN(EP4, 2);
}

void CCID_ClassRequest(void)
{
    uint8_t buf[8];

    USBD_GetSetupPacket(buf);

    if(buf[0] & 0x80)    /* request data transfer direction */
    {
        // Device to host
        switch(buf[1])
        {
            case CCID_GET_CLOCK_FREQUENCIES:
            case CCID_GET_DATA_RATES:
            {
                uint8_t pData[1] = {0};
                USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0)), pData, sizeof(pData));
                /* Data stage */
                USBD_SET_DATA1(EP0);
                USBD_SET_PAYLOAD_LEN(EP0, sizeof(pData));
                /* Status stage */
                USBD_PrepareCtrlOut(0, 0);
                break;
            }
            default:
            {
                /* Setup error, stall the device */
                USBD_SetStall(0);
                break;
            }
        }
    }
    else
    {
        // Host to device
        switch(buf[1])
        {
            case CCID_ABORT:
            {
                /* Status stage */
                USBD_SET_DATA1(EP0);
                USBD_SET_PAYLOAD_LEN(EP0, 0);
                break;
            }
            default:
            {
                // Stall
                /* Setup error, stall the device */
                USBD_SetStall(0);
                break;
            }
        }
    }
}

void CCID_BulkInMessage(void)
{
    uint32_t u32Length;

    gi32UsbdMessageLength = (int32_t)(USB_MESSAGE_HEADER_SIZE + make32(&UsbMessageBuffer[OFFSET_DWLENGTH]));

    pUsbMessageBuffer = UsbMessageBuffer;

    if(gu8IsBulkInReady)
    {
        if(gi32UsbdMessageLength >= EP2_MAX_PKT_SIZE)
        {
            gu8IsBulkInReady = 1;
            USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP2)), pUsbMessageBuffer, EP2_MAX_PKT_SIZE);
            USBD_SET_PAYLOAD_LEN(EP2, EP2_MAX_PKT_SIZE);

            pUsbMessageBuffer += EP2_MAX_PKT_SIZE;
            gi32UsbdMessageLength -= EP2_MAX_PKT_SIZE;
        }
        else
        {
            u32Length = (uint32_t)gi32UsbdMessageLength;
            USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP2)), pUsbMessageBuffer, u32Length);
            USBD_SET_PAYLOAD_LEN(EP2, (uint32_t)gi32UsbdMessageLength);
            gi32UsbdMessageLength = 0;
            gu8IsBulkInReady = 0;
        }
    }
}

void CCID_DispatchMessage(void)
{
    uint8_t ErrorCode;

    if(gu8IsBulkOutReady)
    {
        switch(UsbMessageBuffer[OFFSET_BMESSAGETYPE])
        {
            case PC_TO_RDR_ICCPOWERON:
                ErrorCode = PC_to_RDR_IccPowerOn();
                RDR_to_PC_DataBlock(ErrorCode);
                break;
            case PC_TO_RDR_ICCPOWEROFF:
                ErrorCode = PC_to_RDR_IccPowerOff();
                RDR_to_PC_SlotStatus(ErrorCode);
                break;
            case PC_TO_RDR_GETSLOTSTATUS:
                ErrorCode = PC_to_RDR_GetSlotStatus();
                RDR_to_PC_SlotStatus(ErrorCode);
                break;
            case PC_TO_RDR_XFRBLOCK:
                ErrorCode = PC_to_RDR_XfrBlock();
                RDR_to_PC_DataBlock(ErrorCode);
                break;
            case PC_TO_RDR_GETPARAMETERS:
                ErrorCode = PC_to_RDR_GetParameters();
                RDR_to_PC_Parameters(ErrorCode);
                break;
            case PC_TO_RDR_RESETPARAMETERS:
                ErrorCode = PC_to_RDR_ResetParameters();
                RDR_to_PC_Parameters(ErrorCode);
                break;
            case PC_TO_RDR_SETPARAMETERS:
                ErrorCode = PC_to_RDR_SetParameters();
                RDR_to_PC_Parameters(ErrorCode);
                break;
            case PC_TO_RDR_ESCAPE:
                ErrorCode = PC_to_RDR_Escape();
                RDR_to_PC_Escape(ErrorCode);
                break;
            case PC_TO_RDR_ICCCLOCK:
                ErrorCode = PC_to_RDR_IccClock();
                RDR_to_PC_SlotStatus(ErrorCode);
                break;
            case PC_TO_RDR_ABORT:
                ErrorCode = PC_to_RDR_Abort();
                RDR_to_PC_SlotStatus(ErrorCode);
                break;
            case PC_TO_RDR_SETDATARATEANDCLOCKFREQUENCY:
            case PC_TO_RDR_SECURE:
            case PC_TO_RDR_T0APDU:
            case PC_TO_RDR_MECHANICAL:
            default:
                CmdNotSupported();
                break;
        }

        CCID_BulkInMessage();
        gu8IsBulkOutReady = 0;
    }
}

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
