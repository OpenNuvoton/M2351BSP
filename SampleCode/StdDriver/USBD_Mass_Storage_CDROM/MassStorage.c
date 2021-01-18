/******************************************************************************
 * @file     MassStorage.c
 * @brief    M2351 series USBD driver Sample file
 *
 * @note
 * Copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

/*!<Includes */
#include <string.h>
#include "NuMicro.h"
#include "massstorage.h"

/*--------------------------------------------------------------------------*/
/* Global variables for Control Pipe */
static uint32_t s_u32TotalSectors = 0;

static uint8_t volatile s_u8EP2Ready = 0;
static uint8_t volatile s_u8EP3Ready = 0;
static uint8_t volatile s_u8Remove = 0;
uint8_t volatile g_u8Suspend = 0;

/* USB flow control variables */
static uint8_t s_u8BulkState;
static uint8_t s_u8Prevent = 0;
static uint32_t s_u32Size;

static uint8_t s_au8SenseKey[4];

static uint32_t s_u32Address;
static uint32_t s_u32Length;
static uint32_t s_u32LbaAddress;
static uint32_t s_u32BytesInStorageBuf;

static uint32_t s_u32BulkBuf0, s_u32BulkBuf1;

/* CBW/CSW variables */
static struct CBW s_sCBW;
static struct CSW s_sCSW;

uint32_t g_au32MassBlock[MASS_BUFFER_SIZE / 4];
uint32_t g_au32StorageBlock[STORAGE_BUFFER_SIZE / 4];

/*--------------------------------------------------------------------------*/
static uint8_t s_au8InquiryID[36] =
{
    0x05,                   /* Peripheral Device Type : CD/DVD */
    0x80,                   /* RMB */
    0x00,                   /* ISO/ECMA, ANSI Version */
    0x32,                   /* Response Data Format */
    0x1F, 0x00, 0x00, 0x00, /* Additional Length */

    /* Vendor Identification */
    'N', 'u', 'v', 'o', 't', 'o', 'n', ' ',

    /* Product Identification */
    'U', 'S', 'B', ' ', 'C', 'D', 'R', 'O', 'M', ' ', ' ', ' ', ' ',

    /* Product Revision */
    '1', '.', '0', '0'
};

// code = 5Ah, Mode Sense 10
static uint8_t s_au8ModePage_01[12] =
{
    0x01, 0x0A, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00,
    0x03, 0x00, 0x00, 0x00
};

static uint8_t s_au8ModePage_05[32] =
{
    0x05, 0x1E, 0x13, 0x88, 0x08, 0x20, 0x02, 0x00,
    0x01, 0xF4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x05, 0x1E, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x01, 0x68, 0x00, 0x00
};

static uint8_t s_au8ModePage_1B[12] =
{
    0x1B, 0x0A, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00
};

static uint8_t s_au8ModePage_1C[8] =
{
    0x1C, 0x06, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00
};

static uint8_t  s_au8ReadTOC_LBA0[] =
{
    // TOC response header
    0x00, 0x12,
    0x01,
    0x01,
    // track descriptor 1
    0x00,
    0x14,
    0x01,
    0x00,
    0x00, 0x00, 0x00, 0x00,
    // track descriptor 2
    0x00,
    0x14,
    0xAA,
    0x00,
    0x00, 0x00, 0x08, 0x00
};

static uint8_t s_au8ReadTOC_LBA1[] =
{
    // TOC response header
    0x00, 0x0A,
    0x01,
    0x01,
    // track descriptor
    0x00,
    0x14,
    0x01,
    0x00,
    0x00, 0x00, 0x00, 0x00
};

static uint8_t s_au8ReadTOC_MSF0[] =
{
    // TOC response header
    0x00, 0x12,
    0x01,
    0x01,
    // track descriptor 1
    0x00,
    0x14,
    0x01,
    0x00,
    0x00, 0x00, 0x02, 0x00,
    // track descriptor 2
    0x00,
    0x14,
    0xAA,
    0x00,
    0x00, 0x00, 0x29, 0x23
};

static uint8_t s_au8ReadTOC_MSF2[] =
{
    // TOC response header
    0x00, 0x2E,             // Data Length
    0x01,                   // First Track
    0x01,                   // Last Track
    // track descriptors
    // Session ADR/CTRL  TNO   POINT  Min Sec Frame  ZERO PMIN PSEC PFRAME
    0x01,    0x14,   0x00, 0xA0, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00,
    0x01,    0x14,   0x00, 0xA1, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00,
    0x01,    0x14,   0x00, 0xA2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1d, 0x17,
    0x01,    0x14,   0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x00
};

static const uint8_t s_au8GetConfiguration[] =
{
    // Feature Header
    0x00, 0x00, 0x00, 0x4C, // Data Length
    0x00, 0x00,             // Reserved
    0x00, 0x00,             // Current Profile
    // Profile List
    0x00, 0x00,             // Feature Code (0000h = Profile List)
    0x03,                   // Version(0) /Persistent(1) /Current(1)
    0x04,                   // Additional Length (= number of profile descriptor x 4)
    // Profile Descriptor
    0x00, 0x08,             // Profile Number (CD-ROM)
    0x00,                   // CurrentP
    0x00,                   // Reserved
    // Features
    0x00, 0x01,             // Core Feature
    0x0B,                   // Version(2) /Persistent(1) /Current(1)
    0x08,                   // Additional Length
    0x00, 0x00, 0x00, 0x02, // Physical Interface Standard (0x00000002: ATAPI)
    0x01,                   // INQ2(0) / Device Busy Event(1)
    0x00, 0x00, 0x00,       // Reserved

    0x00, 0x02,             // Morphing Feature
    0x07,                   // Version(2) /Persistent(1) /Current(1)
    0x04,                   // Additional Length
    0x02,                   // OCEvent(1) / ASYNC(0)
    0x00, 0x00, 0x00,       // Reserved

    0x00, 0x03,             // Removable Medium Feature
    0x0B,                   // Version(0) /Persistent(1) /Current(1)
    0x04,                   // Additional Length
    0x29,                   // Loading mechanism (Tray:001h) /Eject(1) / Pvnt Jmpr(0) / Lock(1)
    0x00, 0x00, 0x00,       // Reserved

    0x00, 0x10,             // Random Readable Feature
    0x00,                   // Version(0) /Persistent(0) /Current(0)
    0x08,                   // Additional Length
    0x00, 0x00, 0x08, 0x00, // Logical Block Size
    0x00, 0x01,             // Blocking
    0x00,                   // Page Present
    0x00,                   // Reserved

    0x00, 0x1E,             // CD Read Feature
    0x08,                   // Version(2) /Persistent(0) /Current(0)
    0x04,                   // Additional Length
    0x00,                   // DAP(0) / C2 Flags(0) / CD-Text(0)
    0x00, 0x00, 0x00,       // Reserved

    0x01, 0x00,             // Power Management Feature
    0x03,                   // Version(0) /Persistent(1) /Current(1)
    0x00,                   // Additional Length

    0x01, 0x05,             // Timeout Feature
    0x07,                   // Version(1) /Persistent(1) /Current(1)
    0x04,                   // Additional Length
    0x00,                   // Group3
    0x00,                   // Reserved
    0x00, 0x00,             // Unit Length
};

static uint8_t s_au8GetEventStatusNotification_01[8] =
{
    0x00, 0x02, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00
};

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
            s_u8Remove = 0;
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
            // Bulk IN
            EP2_Handler();
        }

        if(u32IntSts & USBD_INTSTS_EP3)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP3);
            // Bulk OUT
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
    s_u8EP2Ready = 1;
    MSC_AckCmd();
}


void EP3_Handler(void)
{
    /* Bulk OUT */
    s_u8EP3Ready = 1;
}


void MSC_Init(void)
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
    /* EP2 ==> Bulk IN endpoint, address 2 */
    USBD_CONFIG_EP(EP2, USBD_CFG_EPMODE_IN | BULK_IN_EP_NUM);
    /* Buffer range for EP2 */
    USBD_SET_EP_BUF_ADDR(EP2, EP2_BUF_BASE);

    /* EP3 ==> Bulk Out endpoint, address 3 */
    USBD_CONFIG_EP(EP3, USBD_CFG_EPMODE_OUT | BULK_OUT_EP_NUM);
    /* Buffer range for EP3 */
    USBD_SET_EP_BUF_ADDR(EP3, EP3_BUF_BASE);

    /* trigger to receive OUT data */
    USBD_SET_PAYLOAD_LEN(EP3, EP3_MAX_PKT_SIZE);

    /*****************************************************/
    s_u32BulkBuf0 = EP3_BUF_BASE;
    s_u32BulkBuf1 = EP2_BUF_BASE;

    s_sCSW.dCSWSignature = CSW_SIGNATURE;
    s_u32TotalSectors = DATA_FLASH_STORAGE_SIZE / CDROM_BLOCK_SIZE;
}

void MSC_ClassRequest(void)
{
    uint8_t au8Buf[8];

    USBD_GetSetupPacket(au8Buf);

    if(au8Buf[0] & 0x80)    /* request data transfer direction */
    {
        // Device to host
        switch(au8Buf[1])
        {
            case GET_MAX_LUN:
            {
                /* Check interface number with cfg descriptor and check wValue = 0, wLength = 1 */
                if((((au8Buf[3] << 8) + au8Buf[2]) == 0) && (((au8Buf[5] << 8) + au8Buf[4]) == 0) && (((au8Buf[7] << 8) + au8Buf[6]) == 1))
                {
                    M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0)) = 0;
                    /* Data stage */
                    USBD_SET_DATA1(EP0);
                    USBD_SET_PAYLOAD_LEN(EP0, 1);
                    /* Status stage */
                    USBD_PrepareCtrlOut(0, 0);
                }
                else     /* Invalid Get MaxLun command */
                {
                    USBD_SetStall(EP0);
                    USBD_SetStall(EP1);
                }
                USBD_SET_DATA0(EP2);
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
            case BULK_ONLY_MASS_STORAGE_RESET:
            {
                /* Check interface number with cfg descriptor and check wValue = 0, wLength = 0 */
                if((((au8Buf[3] << 8) + au8Buf[2]) == 0) && (((au8Buf[5] << 8) + au8Buf[4]) == 0) && (((au8Buf[7] << 8) + au8Buf[6]) == 0))
                {
                    USBD_SET_DATA1(EP0);
                    USBD_SET_PAYLOAD_LEN(EP0, 0);

                    USBD_LockEpStall(0);

                    /* Clear ready */
                    USBD->EP[EP2].CFGP |= USBD_CFGP_CLRRDY_Msk;
                    USBD->EP[EP3].CFGP |= USBD_CFGP_CLRRDY_Msk;
                    USBD_SET_DATA0(EP2);

                    /* Prepare to receive the CBW */
                    s_u8EP3Ready = 0;
                    s_u8BulkState = BULK_CBW;

                    USBD_SET_DATA1(EP3);
                    USBD_SET_EP_BUF_ADDR(EP3, s_u32BulkBuf0);
                    USBD_SET_PAYLOAD_LEN(EP3, EP3_MAX_PKT_SIZE);
                }
                else     /* Invalid Reset command */
                {
                    USBD_SetStall(EP0);
                    USBD_SetStall(EP1);
                }
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

void MSC_ReadTOC(void)
{
    uint32_t u32Len;
    uint8_t u8format = (s_sCBW.u8LUN & 0x0F) | (s_sCBW.au8Data[7] >> 6);

    if(s_sCBW.u8LUN == 0x02)
    {
        switch(u8format)
        {
            case 0x00:
            {
                u32Len = s_sCBW.dCBWDataTransferLength;
                if(u32Len > sizeof(s_au8ReadTOC_MSF0))
                    u32Len = sizeof(s_au8ReadTOC_MSF0);
                s_u8BulkState = BULK_IN;
                USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP2)), s_au8ReadTOC_MSF0, u32Len);
                USBD_SET_PAYLOAD_LEN(EP2, u32Len);
                break;
            }
            case 0x02:
            {
                u32Len = s_sCBW.dCBWDataTransferLength;
                if(u32Len > sizeof(s_au8ReadTOC_MSF2))
                    u32Len = sizeof(s_au8ReadTOC_MSF2);
                s_u8BulkState = BULK_IN;
                USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP2)), s_au8ReadTOC_MSF2, u32Len);
                USBD_SET_PAYLOAD_LEN(EP2, u32Len);
                break;
            }
            default:
                s_au8SenseKey[0] = 0x05;
                s_au8SenseKey[1] = 0x24;
                s_au8SenseKey[2] = 0x00;
        }
    }
    else if(s_sCBW.u8LUN == 0x00)
    {
        switch(s_sCBW.au8Data[0])
        {
            case 0x01:
            {
                u32Len = s_sCBW.dCBWDataTransferLength;
                if(u32Len > sizeof(s_au8ReadTOC_LBA1))
                    u32Len = sizeof(s_au8ReadTOC_LBA1);
                s_u8BulkState = BULK_IN;
                USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP2)), s_au8ReadTOC_LBA1, u32Len);
                USBD_SET_PAYLOAD_LEN(EP2, u32Len);
                break;
            }
            case 0x00:
            {
                u32Len = s_sCBW.dCBWDataTransferLength;
                if(u32Len > sizeof(s_au8ReadTOC_LBA0))
                    u32Len = sizeof(s_au8ReadTOC_LBA0);
                s_u8BulkState = BULK_IN;
                USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP2)), s_au8ReadTOC_LBA0, u32Len);
                USBD_SET_PAYLOAD_LEN(EP2, u32Len);
                break;
            }
            default:
                s_au8SenseKey[0] = 0x05;
                s_au8SenseKey[1] = 0x24;
                s_au8SenseKey[2] = 0x00;
                break;
        }
    }
    else
    {
        s_au8SenseKey[0] = 0x05;
        s_au8SenseKey[1] = 0x24;
        s_au8SenseKey[2] = 0x00;

        USBD_SET_EP_STALL(EP2);
        s_u8Prevent = 1;
        s_sCSW.bCSWStatus = 0x01;
        s_sCSW.dCSWDataResidue = 0;
        s_u8BulkState = BULK_IN;
        MSC_AckCmd();
        return;
    }
}

void MSC_GetConfiguration(uint32_t u32Len, uint8_t *pu8Buff)
{
    uint32_t u32Index, u32FeatureLen;
    uint8_t  *pu8Ptr;

    if(s_u8Remove)
    {
        memset((int8_t *)pu8Buff, 0, u32Len);
        return;
    }

    if(s_sCBW.u8LUN == 0x02)
    {
        memcpy(pu8Buff, s_au8GetConfiguration, 8);
        u32FeatureLen = 0;
        // the first feature on the array
        u32Index = 8;

        // find the specified feature
        while(u32Index < sizeof(s_au8GetConfiguration))
        {
            if((s_au8GetConfiguration[u32Index] == s_sCBW.au8Data[0]) && (s_au8GetConfiguration[u32Index + 1] == s_sCBW.au8Data[1]))
            {
                // copy the feature
                u32FeatureLen = s_au8GetConfiguration[u32Index + 3] + 4;
                memcpy(pu8Buff + 8, &s_au8GetConfiguration[u32Index], u32FeatureLen);
                break;
            }
            else
            {
                u32Index += (s_au8GetConfiguration[u32Index + 3] + 4);
            }
        }
        // fix up return length
        u32Len = 8 + u32FeatureLen;
        pu8Buff[3] = (uint8_t)(u32Len - 4);
    }
    else if(s_sCBW.u8LUN == 0x01)
    {
        memcpy(pu8Buff, s_au8GetConfiguration, 8);
        pu8Ptr = pu8Buff + 8;

        // the first feature on the array
        u32Index = 8;

        // find current features
        while(u32Index < sizeof(s_au8GetConfiguration))
        {
            u32FeatureLen = s_au8GetConfiguration[u32Index + 3] + 4;
            if(s_au8GetConfiguration[u32Index + 2] & 0x01)     // check current bit
            {
                memcpy(pu8Ptr, &s_au8GetConfiguration[u32Index], u32FeatureLen);
                pu8Ptr += u32FeatureLen;
            }
            u32Index += u32FeatureLen;
        }
        u32Len = (uint32_t)(pu8Ptr - pu8Buff);
        pu8Buff[3] = (uint8_t)(u32Len - 4);
    }
    else if(s_sCBW.u8LUN == 0x00)
        memcpy((char *)pu8Buff, &s_au8GetConfiguration[0], sizeof(s_au8GetConfiguration));
    else
    {
        s_au8SenseKey[0] = 0x05;
        s_au8SenseKey[1] = 0x24;
        s_au8SenseKey[2] = 0x00;
    }
}

void MSC_GetEventStatusNotification(void)
{
    uint32_t u32Len;

    u32Len = s_sCBW.dCBWDataTransferLength;
    if(u32Len > 8)
        u32Len = 8;

    s_u8BulkState = BULK_IN;
    USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP2)), s_au8GetEventStatusNotification_01, u32Len);
    /* Trigger to send out the data packet */
    USBD_SET_PAYLOAD_LEN(EP2, u32Len);
}

void MSC_RequestSense(void)
{
    uint8_t au8Tmp[20];

    memset(au8Tmp, 0, 18);
    if(s_u8Prevent)
    {
        s_u8Prevent = 0;
        au8Tmp[0] = 0x70;
    }
    else
        au8Tmp[0] = 0xf0;

    au8Tmp[2] = s_au8SenseKey[0];
    au8Tmp[7] = 0x0a;
    au8Tmp[12] = s_au8SenseKey[1];
    au8Tmp[13] = s_au8SenseKey[2];
    USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP2)), au8Tmp, 20);

    s_au8SenseKey[0] = 0;
    s_au8SenseKey[1] = 0;
    s_au8SenseKey[2] = 0;
}

void MSC_ReadFormatCapacity(void)
{
    memset((uint8_t *)MassCMD_BUF, 0, 12);

    *((uint8_t *)(MassCMD_BUF + 0)) = 0x00;
    *((uint8_t *)(MassCMD_BUF + 1)) = 0x00;
    *((uint8_t *)(MassCMD_BUF + 2)) = 0x00;
    *((uint8_t *)(MassCMD_BUF + 3)) = 0x08;                    /* Capacity List Length */

    /* Block Count */
    *((uint8_t *)(MassCMD_BUF + 4)) = (s_u32TotalSectors >> 24) & 0xFF;
    *((uint8_t *)(MassCMD_BUF + 5)) = (s_u32TotalSectors >> 16) & 0xFF;
    *((uint8_t *)(MassCMD_BUF + 6)) = (s_u32TotalSectors >>  8) & 0xFF;
    *((uint8_t *)(MassCMD_BUF + 7)) = (s_u32TotalSectors >>  0) & 0xFF;

    /* Block Length */
    *((uint8_t *)(MassCMD_BUF + 8)) = 0x02;                    /* Descriptor Code: Formatted Media */
    *((uint8_t *)(MassCMD_BUF + 9)) = (CDROM_BLOCK_SIZE >> 16) & 0xFF;
    *((uint8_t *)(MassCMD_BUF + 10)) = (CDROM_BLOCK_SIZE >>  8) & 0xFF;
    *((uint8_t *)(MassCMD_BUF + 11)) = (CDROM_BLOCK_SIZE >>  0) & 0xFF;
}

void MSC_Read(void)
{
    uint32_t u32Len;

    if(USBD_GET_EP_BUF_ADDR(EP2) == s_u32BulkBuf1)
        USBD_SET_EP_BUF_ADDR(EP2, s_u32BulkBuf0);
    else
        USBD_SET_EP_BUF_ADDR(EP2, s_u32BulkBuf1);

    /* Trigger to send out the data packet */
    USBD_SET_PAYLOAD_LEN(EP2, s_u32Size);

    s_u32Length -= s_u32Size;
    s_u32BytesInStorageBuf -= s_u32Size;

    if(s_u32Length)
    {
        if(s_u32BytesInStorageBuf)
        {
            /* Prepare next data packet */
            s_u32Size = EP2_MAX_PKT_SIZE;
            if(s_u32Size > s_u32Length)
                s_u32Size = s_u32Length;

            if(USBD_GET_EP_BUF_ADDR(EP2) == s_u32BulkBuf1)
                USBD_MemCopy((uint8_t *)((uint32_t)USBD_BUF_BASE + s_u32BulkBuf0), (uint8_t *)s_u32Address, s_u32Size);
            else
                USBD_MemCopy((uint8_t *)((uint32_t)USBD_BUF_BASE + s_u32BulkBuf1), (uint8_t *)s_u32Address, s_u32Size);
            s_u32Address += s_u32Size;
        }
        else
        {
            u32Len = s_u32Length;
            if(u32Len > STORAGE_BUFFER_SIZE)
                u32Len = STORAGE_BUFFER_SIZE;

            MSC_ReadMedia(s_u32LbaAddress, u32Len, (uint8_t *)STORAGE_DATA_BUF);
            s_u32BytesInStorageBuf = u32Len;
            s_u32LbaAddress += u32Len;
            s_u32Address = STORAGE_DATA_BUF;

            /* Prepare next data packet */
            s_u32Size = EP2_MAX_PKT_SIZE;
            if(s_u32Size > s_u32Length)
                s_u32Size = s_u32Length;

            if(USBD_GET_EP_BUF_ADDR(EP2) == s_u32BulkBuf1)
                USBD_MemCopy((uint8_t *)((uint32_t)USBD_BUF_BASE + s_u32BulkBuf0), (uint8_t *)s_u32Address, s_u32Size);
            else
                USBD_MemCopy((uint8_t *)((uint32_t)USBD_BUF_BASE + s_u32BulkBuf1), (uint8_t *)s_u32Address, s_u32Size);
            s_u32Address += s_u32Size;
        }
    }
}

void MSC_ReadTrig(void)
{
    uint32_t u32Len;

    if(s_u32Length)
    {
        if(s_u32BytesInStorageBuf)
        {
            /* Prepare next data packet */
            s_u32Size = EP2_MAX_PKT_SIZE;
            if(s_u32Size > s_u32Length)
                s_u32Size = s_u32Length;

            if(USBD_GET_EP_BUF_ADDR(EP2) == s_u32BulkBuf1)
                USBD_MemCopy((uint8_t *)((uint32_t)USBD_BUF_BASE + s_u32BulkBuf0), (uint8_t *)s_u32Address, s_u32Size);
            else
                USBD_MemCopy((uint8_t *)((uint32_t)USBD_BUF_BASE + s_u32BulkBuf1), (uint8_t *)s_u32Address, s_u32Size);
            s_u32Address += s_u32Size;
        }
        else
        {
            u32Len = s_u32Length;
            if(s_u32LbaAddress <= (16 * CDROM_BLOCK_SIZE))   /* Logical Block Address > 32KB */
            {
                if(u32Len > STORAGE_BUFFER_SIZE)
                    u32Len = STORAGE_BUFFER_SIZE;
                s_u32Address = STORAGE_DATA_BUF;
            }

            MSC_ReadMedia(s_u32LbaAddress, u32Len, (uint8_t *)STORAGE_DATA_BUF);
            s_u32BytesInStorageBuf = u32Len;
            s_u32LbaAddress += u32Len;

            /* Prepare next data packet */
            s_u32Size = EP2_MAX_PKT_SIZE;
            if(s_u32Size > s_u32Length)
                s_u32Size = s_u32Length;

            if(USBD_GET_EP_BUF_ADDR(EP2) == s_u32BulkBuf1)
                USBD_MemCopy((uint8_t *)((uint32_t)USBD_BUF_BASE + s_u32BulkBuf0), (uint8_t *)s_u32Address, s_u32Size);
            else
                USBD_MemCopy((uint8_t *)((uint32_t)USBD_BUF_BASE + s_u32BulkBuf1), (uint8_t *)s_u32Address, s_u32Size);
            s_u32Address += s_u32Size;
        }

        /* DATA0/DATA1 Toggle */
        if(USBD_GET_EP_BUF_ADDR(EP2) == s_u32BulkBuf1)
            USBD_SET_EP_BUF_ADDR(EP2, s_u32BulkBuf0);
        else
            USBD_SET_EP_BUF_ADDR(EP2, s_u32BulkBuf1);

        /* Trigger to send out the data packet */
        USBD_SET_PAYLOAD_LEN(EP2, s_u32Size);

        s_u32Length -= s_u32Size;
        s_u32BytesInStorageBuf -= s_u32Size;
    }
    else
        USBD_SET_PAYLOAD_LEN(EP2, 0);
}


void MSC_ReadCapacity(void)
{
    uint32_t u32Tmp;

    memset((uint8_t *)MassCMD_BUF, 0, 8);

    u32Tmp = s_u32TotalSectors - 1;

    /* Last Logical Block */
    *((uint8_t *)(MassCMD_BUF + 0)) = (u32Tmp >> 24) & 0xFF;
    *((uint8_t *)(MassCMD_BUF + 1)) = (u32Tmp >> 16) & 0xFF;
    *((uint8_t *)(MassCMD_BUF + 2)) = (u32Tmp >>  8) & 0xFF;
    *((uint8_t *)(MassCMD_BUF + 3)) = (u32Tmp >>  0) & 0xFF;

    /* Block Length */
    *((uint8_t *)(MassCMD_BUF + 4)) = (CDROM_BLOCK_SIZE >> 24) & 0xFF;
    *((uint8_t *)(MassCMD_BUF + 5)) = (CDROM_BLOCK_SIZE >> 16) & 0xFF;
    *((uint8_t *)(MassCMD_BUF + 6)) = (CDROM_BLOCK_SIZE >>  8) & 0xFF;
    *((uint8_t *)(MassCMD_BUF + 7)) = (CDROM_BLOCK_SIZE >>  0) & 0xFF;
}

void MSC_ModeSense10(void)
{
    uint8_t i, j;
    uint8_t u8NumHead, u8NumSector;
    uint16_t u16NumCyl = 0;

    /* Clear the command buffer */
    *((uint32_t *)MassCMD_BUF) = 0;
    *((uint32_t *)MassCMD_BUF + 1) = 0;

    switch(s_sCBW.au8Data[0])
    {
        case 0x01:
            *((uint8_t *)MassCMD_BUF) = 19;
            i = 8;
            for(j = 0; j < 12; j++, i++)
                *((uint8_t *)(MassCMD_BUF + i)) = s_au8ModePage_01[j];
            break;

        case 0x05:
            *((uint8_t *)MassCMD_BUF) = 39;
            i = 8;
            for(j = 0; j < 32; j++, i++)
                *((uint8_t *)(MassCMD_BUF + i)) = s_au8ModePage_05[j];

            u8NumHead = 2;
            u8NumSector = 64;
            u16NumCyl = (uint16_t)(s_u32TotalSectors / 128);

            *((uint8_t *)(MassCMD_BUF + 12)) = u8NumHead;
            *((uint8_t *)(MassCMD_BUF + 13)) = u8NumSector;
            *((uint8_t *)(MassCMD_BUF + 16)) = (uint8_t)(u16NumCyl >> 8);
            *((uint8_t *)(MassCMD_BUF + 17)) = (uint8_t)(u16NumCyl & 0x00ff);
            break;

        case 0x1B:
            *((uint8_t *)MassCMD_BUF) = 19;
            i = 8;
            for(j = 0; j < 12; j++, i++)
                *((uint8_t *)(MassCMD_BUF + i)) = s_au8ModePage_1B[j];
            break;

        case 0x1C:
            *((uint8_t *)MassCMD_BUF) = 15;
            i = 8;
            for(j = 0; j < 8; j++, i++)
                *((uint8_t *)(MassCMD_BUF + i)) = s_au8ModePage_1C[j];
            break;

        case 0x3F:
            *((uint8_t *)MassCMD_BUF) = 0x47;
            i = 8;
            for(j = 0; j < 12; j++, i++)
                *((uint8_t *)(MassCMD_BUF + i)) = s_au8ModePage_01[j];
            for(j = 0; j < 32; j++, i++)
                *((uint8_t *)(MassCMD_BUF + i)) = s_au8ModePage_05[j];
            for(j = 0; j < 12; j++, i++)
                *((uint8_t *)(MassCMD_BUF + i)) = s_au8ModePage_1B[j];
            for(j = 0; j < 8; j++, i++)
                *((uint8_t *)(MassCMD_BUF + i)) = s_au8ModePage_1C[j];

            u8NumHead = 2;
            u8NumSector = 64;
            u16NumCyl = (uint16_t)(s_u32TotalSectors / 128);

            *((uint8_t *)(MassCMD_BUF + 24)) = u8NumHead;
            *((uint8_t *)(MassCMD_BUF + 25)) = u8NumSector;
            *((uint8_t *)(MassCMD_BUF + 28)) = (uint8_t)(u16NumCyl >> 8);
            *((uint8_t *)(MassCMD_BUF + 29)) = (uint8_t)(u16NumCyl & 0x00ff);
            break;

        default:
            s_au8SenseKey[0] = 0x05;
            s_au8SenseKey[1] = 0x24;
            s_au8SenseKey[2] = 0x00;
    }
}

void MSC_ProcessCmd(void)
{
    uint32_t u32Len;
    uint32_t i;
    uint32_t u32Hcount, u32Dcount;

    if(s_u8EP3Ready)
    {
        s_u8EP3Ready = 0;
        if(s_u8BulkState == BULK_CBW)
        {
            u32Len = USBD_GET_PAYLOAD_LEN(EP3);

            /* Check Signature & length of CBW */
            /* Bulk Out buffer */
            if((*(uint32_t *)((uint32_t)USBD_BUF_BASE + s_u32BulkBuf0) != CBW_SIGNATURE) || (u32Len != 31))
            {
                /* Invalid CBW */
                s_u8Prevent = 1;
                USBD_SET_EP_STALL(EP2);
                USBD_SET_EP_STALL(EP3);
                USBD_LockEpStall((1 << EP2) | (1 << EP3));
                return;
            }

            /* Get the CBW */
            for(i = 0; i < u32Len; i++)
                *((uint8_t *)(&s_sCBW.dCBWSignature) + i) = *(uint8_t *)((uint32_t)USBD_BUF_BASE + s_u32BulkBuf0 + i);

            /* Prepare to echo the tag from CBW to CSW */
            s_sCSW.dCSWTag = s_sCBW.dCBWTag;
            u32Hcount = s_sCBW.dCBWDataTransferLength;

            /* Parse Op-Code of CBW */
            switch(s_sCBW.u8OPCode)
            {
                case UFI_PREVENT_ALLOW_MEDIUM_REMOVAL:
                {
                    if(s_sCBW.au8Data[2] & 0x01)
                    {
                        s_au8SenseKey[0] = 0x05;  //INVALID COMMAND
                        s_au8SenseKey[1] = 0x24;
                        s_au8SenseKey[2] = 0;
                        s_u8Prevent = 1;
                    }
                    else
                        s_u8Prevent = 0;
                    s_u8BulkState = BULK_IN;
                    MSC_AckCmd();
                    return;
                }
                case UFI_TEST_UNIT_READY:
                {
                    if(u32Hcount != 0)
                    {
                        if(s_sCBW.bmCBWFlags == 0)      /* Ho > Dn (Case 9) */
                        {
                            s_u8Prevent = 1;
                            USBD_SET_EP_STALL(EP3);
                            s_sCSW.bCSWStatus = 0x1;
                            s_sCSW.dCSWDataResidue = u32Hcount;
                        }
                    }
                    else     /* Hn == Dn (Case 1) */
                    {
                        if(s_u8Remove)
                        {
                            s_sCSW.dCSWDataResidue = 0;
                            s_sCSW.bCSWStatus = 1;
                            s_au8SenseKey[0] = 0x02;    /* Not ready */
                            s_au8SenseKey[1] = 0x3A;
                            s_au8SenseKey[2] = 0;
                            s_u8Prevent = 1;
                        }
                        else
                        {
                            s_sCSW.dCSWDataResidue = 0;
                            s_sCSW.bCSWStatus = 0;
                        }
                    }
                    s_u8BulkState = BULK_IN;
                    MSC_AckCmd();
                    return;
                }
                case UFI_START_STOP:
                {
                    if((s_sCBW.au8Data[2] & 0x03) == 0x2)
                    {
                        s_u8Remove = 1;
                    }
                    s_u8BulkState = BULK_IN;
                    MSC_AckCmd();
                    return;
                }
                case UFI_VERIFY_10:
                {
                    s_u8BulkState = BULK_IN;
                    MSC_AckCmd();
                    return;
                }
                case UFI_REQUEST_SENSE:
                {
                    if((u32Hcount > 0) && (u32Hcount <= 18))
                    {
                        MSC_RequestSense();
                        USBD_SET_PAYLOAD_LEN(EP2, u32Hcount);
                        s_u8BulkState = BULK_IN;
                        s_sCSW.bCSWStatus = 0;
                        s_sCSW.dCSWDataResidue = 0;
                        return;
                    }
                    else
                    {
                        USBD_SET_EP_STALL(EP2);
                        s_u8Prevent = 1;
                        s_sCSW.bCSWStatus = 0x01;
                        s_sCSW.dCSWDataResidue = 0;
                        s_u8BulkState = BULK_IN;
                        MSC_AckCmd();
                        USBD_SET_DATA0(EP2);
                        return;
                    }
                }
                case UFI_READ_FORMAT_CAPACITY:
                {
                    if(s_u32Length == 0)
                    {
                        s_u32Length = s_sCBW.dCBWDataTransferLength;
                        s_u32Address = MassCMD_BUF;
                    }
                    MSC_ReadFormatCapacity();
                    s_u8BulkState = BULK_IN;
                    if(s_u32Length > 0)
                    {
                        if(s_u32Length > EP2_MAX_PKT_SIZE)
                            s_u32Size = EP2_MAX_PKT_SIZE;
                        else
                            s_u32Size = s_u32Length;

                        /* Bulk IN buffer */
                        USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + s_u32BulkBuf1), (uint8_t *)s_u32Address, s_u32Size);

                        s_u32Address += s_u32Size;
                        USBD_SET_EP_BUF_ADDR(EP2, s_u32BulkBuf0);
                        MSC_Read();
                    }
                    return;
                }
                case UFI_READ_CAPACITY:
                {
                    s_u32Length = s_sCBW.dCBWDataTransferLength;
                    s_u32Address = MassCMD_BUF;

                    MSC_ReadCapacity();
                    s_u8BulkState = BULK_IN;
                    if(s_u32Length > 0)
                    {
                        if(s_u32Length > EP2_MAX_PKT_SIZE)
                            s_u32Size = EP2_MAX_PKT_SIZE;
                        else
                            s_u32Size = s_u32Length;

                        /* Bulk IN buffer */
                        USBD_MemCopy((uint8_t *)((uint32_t)USBD_BUF_BASE + s_u32BulkBuf1), (uint8_t *)s_u32Address, s_u32Size);

                        s_u32Address += s_u32Size;
                        USBD_SET_EP_BUF_ADDR(EP2, s_u32BulkBuf0);
                        MSC_Read();
                    }
                    return;
                }
                case UFI_MODE_SENSE_10:
                {
                    if(s_u32Length == 0)
                    {
                        s_u32Length = s_sCBW.dCBWDataTransferLength;
                        s_u32Address = MassCMD_BUF;
                    }

                    MSC_ModeSense10();
                    s_u8BulkState = BULK_IN;
                    if(s_u32Length > 0)
                    {
                        if(s_u32Length > EP2_MAX_PKT_SIZE)
                            s_u32Size = EP2_MAX_PKT_SIZE;
                        else
                            s_u32Size = s_u32Length;
                        /* Bulk IN buffer */
                        USBD_MemCopy((uint8_t *)((uint32_t)USBD_BUF_BASE + s_u32BulkBuf1), (uint8_t *)s_u32Address, s_u32Size);

                        s_u32Address += s_u32Size;

                        USBD_SET_EP_BUF_ADDR(EP2, s_u32BulkBuf0);
                        MSC_Read();
                    }
                    return;
                }
                case UFI_INQUIRY:
                {
                    if(u32Hcount > 36 || u32Hcount == 0)
                    {
                        s_u8Prevent = 1;
                        s_sCSW.dCSWDataResidue = u32Hcount;
                        s_sCSW.bCSWStatus = 0x1;
                        USBD_SET_EP_STALL(EP2);
                        s_u8BulkState = BULK_IN;
                        MSC_AckCmd();
                        USBD_SET_DATA0(EP2);
                    }
                    else
                    {
                        /* Bulk IN buffer */
                        USBD_MemCopy((uint8_t *)((uint32_t)USBD_BUF_BASE + s_u32BulkBuf1), (uint8_t *)s_au8InquiryID, u32Hcount);
                        USBD_SET_PAYLOAD_LEN(EP2, u32Hcount);
                        s_u8BulkState = BULK_IN;
                        s_sCSW.bCSWStatus = 0;
                        s_sCSW.dCSWDataResidue = 0;
                    }
                    return;
                }
                case UFI_READ_12:
                case UFI_READ_10:
                {
                    /* Check if it is a new transfer */
                    if(s_u32Length == 0)
                    {
                        u32Dcount = (get_be32(&s_sCBW.au8Data[4]) >> 8) * CDROM_BLOCK_SIZE;
                        if(s_sCBW.bmCBWFlags == 0x80)       /* IN */
                        {
                            if(u32Hcount == u32Dcount)    /* Hi == Di (Case 6)*/
                            {
                            }
                            else if(u32Hcount < u32Dcount)      /* Hn < Di (Case 2) || Hi < Di (Case 7) */
                            {
                                if(u32Hcount)      /* Hi < Di (Case 7) */
                                {
                                    s_u8Prevent = 1;
                                    s_sCSW.bCSWStatus = 0x01;
                                    s_sCSW.dCSWDataResidue = 0;
                                }
                                else     /* Hn < Di (Case 2) */
                                {
                                    s_u8Prevent = 1;
                                    s_sCSW.bCSWStatus = 0x01;
                                    s_sCSW.dCSWDataResidue = 0;
                                    s_u8BulkState = BULK_IN;
                                    MSC_AckCmd();
                                    return;
                                }
                            }
                            else if(u32Hcount > u32Dcount)      /* Hi > Dn (Case 4) || Hi > Di (Case 5) */
                            {
                                s_u8Prevent = 1;
                                s_sCSW.bCSWStatus = 0x01;
                                s_sCSW.dCSWDataResidue = 0;
                            }
                        }
                        else     /* Ho <> Di (Case 10) */
                        {
                            s_u8Prevent = 1;
                            USBD_SET_EP_STALL(EP3);
                            s_sCSW.bCSWStatus = 0x01;
                            s_sCSW.dCSWDataResidue = u32Hcount;
                            s_u8BulkState = BULK_IN;
                            MSC_AckCmd();
                            return;
                        }
                    }

                    /* Get LBA address */
                    s_u32Address = get_be32(&s_sCBW.au8Data[0]);
                    s_u32LbaAddress = s_u32Address * CDROM_BLOCK_SIZE;
                    s_u32Length = s_sCBW.dCBWDataTransferLength;
                    s_u32BytesInStorageBuf = s_u32Length;

                    i = s_u32Length;
                    if(i > STORAGE_BUFFER_SIZE)
                        i = STORAGE_BUFFER_SIZE;

                    if(s_u32LbaAddress >= (16 * CDROM_BLOCK_SIZE))   /* Logical Block Address > 32KB */
                    {
                        /*
                            Because first 32KB of the ISO file are all '0', remove first 32KB data from ISO file
                            to reduce the code size instead of including ISO file directly.
                            The array - eprom is the data of ISO file with offset 32768
                         */
                        s_u32Address = (uint32_t)(&eprom[s_u32LbaAddress - 32768]);
                        s_u32LbaAddress += i;
                    }
                    else                                             /* Logical Block Address > 32KB */
                    {
                        memset((uint32_t*)g_au32StorageBlock, 0, i); /* First 32KB of ISO file are all 0 */
                        s_u32Address = STORAGE_DATA_BUF;
                    }
                    s_u32BytesInStorageBuf = i;

                    /* Indicate the next packet should be Bulk IN Data packet */
                    s_u8BulkState = BULK_IN;
                    if(s_u32BytesInStorageBuf > 0)
                    {
                        /* Set the packet size */
                        if(s_u32BytesInStorageBuf > EP2_MAX_PKT_SIZE)
                            s_u32Size = EP2_MAX_PKT_SIZE;
                        else
                            s_u32Size = s_u32BytesInStorageBuf;

                        /* Prepare the first data packet (DATA1) */
                        /* Bulk IN buffer */
                        USBD_MemCopy((uint8_t *)((uint32_t)USBD_BUF_BASE + s_u32BulkBuf1), (uint8_t *)s_u32Address, s_u32Size);
                        s_u32Address += s_u32Size;

                        /* kick - start */
                        USBD_SET_EP_BUF_ADDR(EP2, s_u32BulkBuf1);
                        /* Trigger to send out the data packet */
                        USBD_SET_PAYLOAD_LEN(EP2, s_u32Size);
                        s_u32Length -= s_u32Size;
                        s_u32BytesInStorageBuf -= s_u32Size;
                    }
                    return;
                }

                case UFI_WRITE_12:
                case UFI_WRITE_10:
                {
                    if(s_u32Length == 0)
                    {
                        u32Dcount = (get_be32(&s_sCBW.au8Data[4]) >> 8) * 512;
                        if(s_sCBW.bmCBWFlags == 0x00)       /* OUT */
                        {
                            if(u32Hcount == u32Dcount)    /* Ho == Do (Case 12)*/
                            {
                                s_sCSW.dCSWDataResidue = 0;
                                s_sCSW.bCSWStatus = 0;
                            }
                            else if(u32Hcount < u32Dcount)      /* Hn < Do (Case 3) || Ho < Do (Case 13) */
                            {
                                s_u8Prevent = 1;
                                s_sCSW.dCSWDataResidue = 0;
                                s_sCSW.bCSWStatus = 0x1;
                                if(u32Hcount == 0)     /* Hn < Do (Case 3) */
                                {
                                    s_u8BulkState = BULK_IN;
                                    MSC_AckCmd();
                                    return;
                                }
                            }
                            else if(u32Hcount > u32Dcount)      /* Ho > Do (Case 11) */
                            {
                                s_u8Prevent = 1;
                                s_sCSW.dCSWDataResidue = 0;
                                s_sCSW.bCSWStatus = 0x1;
                            }
                            s_u32Length = s_sCBW.dCBWDataTransferLength;
                            s_u32Address = STORAGE_DATA_BUF;
                        }
                        else     /* Hi <> Do (Case 8) */
                        {
                            s_u8Prevent = 1;
                            s_sCSW.dCSWDataResidue = u32Hcount;
                            s_sCSW.bCSWStatus = 0x1;
                            USBD_SET_EP_STALL(EP2);
                            s_u8BulkState = BULK_IN;
                            MSC_AckCmd();
                            USBD_SET_DATA0(EP2);
                            return;
                        }
                    }
                    USBD_SET_EP_STALL(EP3);
                    s_u8Prevent = 1;
                    s_sCSW.bCSWStatus = 0x01;
                    s_sCSW.dCSWDataResidue = 0;
                    s_u8BulkState = BULK_IN;
                    MSC_AckCmd();
                    return;
                }
                case UFI_READ_16:
                {
                    USBD_SET_EP_STALL(EP2);
                    s_u8Prevent = 1;
                    s_sCSW.bCSWStatus = 0x01;
                    s_sCSW.dCSWDataResidue = 0;
                    s_u8BulkState = BULK_IN;
                    MSC_AckCmd();
                    USBD_SET_DATA0(EP2);
                    return;
                }
                case UFI_READ_TOC:
                {
                    MSC_ReadTOC();
                    return;
                }
                case UFI_GET_CONFIGURATION:
                {
                    if(s_u32Length == 0)
                    {
                        // LBA
                        s_u32Address = get_be32(&s_sCBW.au8Data[0]);
                        s_u32Length = s_sCBW.dCBWDataTransferLength;
                        MSC_GetConfiguration(s_u32Length, (uint8_t *)g_au32MassBlock);
                    }
                    s_u32Address = (uint32_t)g_au32MassBlock;
                    s_u8BulkState = BULK_IN;
                    if(s_u32Length > 0)
                    {
                        if(s_u32Length > EP2_MAX_PKT_SIZE)
                            s_u32Size = EP2_MAX_PKT_SIZE;
                        else
                            s_u32Size = s_u32Length;

                        USBD_MemCopy((uint8_t *)((uint32_t)USBD_BUF_BASE + s_u32BulkBuf1), (uint8_t *)s_u32Address, s_u32Size);

                        USBD_SET_EP_BUF_ADDR(EP2, s_u32BulkBuf0);
                        /* Trigger to send out the data packet */
                        USBD_SET_PAYLOAD_LEN(EP2, s_u32Size);

                        s_u32Address += s_u32Size;
                        s_u32Length -= s_u32Size;

                        s_u32BytesInStorageBuf -= s_u32Size;
                    }
                    else
                        MSC_AckCmd();
                    return;
                }
                case UFI_SET_CDROM_SPEED:
                {
                    s_u8BulkState = BULK_IN;
                    MSC_AckCmd();
                    return;
                }
                case UFI_GET_EVENT_STATUS_NOTIFICATION:
                {
                    MSC_GetEventStatusNotification();
                    return;
                }
                case UFI_MODE_SENSE_6:
                {
                    *(uint8_t *)((uint32_t)USBD_BUF_BASE + s_u32BulkBuf1+0) = 0x3;
                    *(uint8_t *)((uint32_t)USBD_BUF_BASE + s_u32BulkBuf1+1) = 0x0;
                    *(uint8_t *)((uint32_t)USBD_BUF_BASE + s_u32BulkBuf1+2) = 0x0;
                    *(uint8_t *)((uint32_t)USBD_BUF_BASE + s_u32BulkBuf1+3) = 0x0;
                    USBD_SET_PAYLOAD_LEN(EP2, 4);
                    s_u8BulkState = BULK_IN;
                    s_sCSW.bCSWStatus = 0;
                    s_sCSW.dCSWDataResidue = u32Hcount - 4;
                    return;
                }
                default:
                {
                    /* Unsupported command */
                    s_au8SenseKey[0] = 0x05;
                    s_au8SenseKey[1] = 0x20;
                    s_au8SenseKey[2] = 0x00;

                    /* If CBW request for data phase, just return zero packet to end data phase */
                    if(s_sCBW.dCBWDataTransferLength > 0)
                    {
                        /* Data Phase, zero/short packet */
                        if((s_sCBW.bmCBWFlags & 0x80) != 0)
                        {
                            /* Data-In */
                            s_u8BulkState = BULK_IN;
                            USBD_SET_PAYLOAD_LEN(EP2, 0);
                        }
                    }
                    else
                    {
                        /* Status Phase */
                        s_u8BulkState = BULK_IN;
                        MSC_AckCmd();
                    }
                    return;
                }
            }
        }
    }
}

void MSC_AckCmd(void)
{
    /* Bulk IN */
    if(s_u8BulkState == BULK_CSW)
    {
        /* Prepare to receive the CBW */
        s_u8BulkState = BULK_CBW;

        USBD_SET_EP_BUF_ADDR(EP3, s_u32BulkBuf0);
        USBD_SET_PAYLOAD_LEN(EP3, 31);
    }
    else if(s_u8BulkState == BULK_IN)
    {
        switch(s_sCBW.u8OPCode)
        {
            case UFI_READ_12:
            case UFI_READ_10:
            {
                if(s_u32Length > 0)
                {
                    MSC_ReadTrig();
                    return;
                }
                break;
            }
            case UFI_READ_FORMAT_CAPACITY:
            case UFI_READ_CAPACITY:
            case UFI_MODE_SENSE_10:
            case UFI_GET_CONFIGURATION:
            case UFI_READ_CD:
            {
                if(s_u32Length > 0)
                {
                    MSC_Read();
                    return;
                }
                s_sCSW.dCSWDataResidue = 0;
                s_sCSW.bCSWStatus = 0;
                break;
            }

            case UFI_WRITE_12:
            case UFI_WRITE_10:
                break;
            case UFI_PREVENT_ALLOW_MEDIUM_REMOVAL:
            case UFI_VERIFY_10:
            case UFI_START_STOP:
            case UFI_GET_EVENT_STATUS_NOTIFICATION:
            case UFI_SET_CDROM_SPEED:
            {
                if (s_sCBW.dCBWDataTransferLength < STORAGE_BUFFER_SIZE)
                    s_sCSW.dCSWDataResidue = 0;
                else
                    s_sCSW.dCSWDataResidue = s_sCBW.dCBWDataTransferLength - STORAGE_BUFFER_SIZE;

                s_sCSW.bCSWStatus = 0;
                break;
            }
            case UFI_INQUIRY:
            case UFI_MODE_SENSE_6:
            case UFI_REQUEST_SENSE:
            case UFI_TEST_UNIT_READY:
            {
                break;
            }
            case UFI_READ_TOC:
            {
                s_sCSW.dCSWDataResidue = 0;
                s_sCSW.bCSWStatus = 0;
                break;
            }
            default:
            {
                /* Unsupported command. Return command fail status */
                s_sCSW.dCSWDataResidue = s_sCBW.dCBWDataTransferLength;
                s_sCSW.bCSWStatus = 0x01;
                break;
            }
        }

        /* Return the CSW */
        USBD_SET_EP_BUF_ADDR(EP2, s_u32BulkBuf1);

        /* Bulk IN buffer */
        USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + s_u32BulkBuf1), (uint8_t *)&s_sCSW.dCSWSignature, 16);

        s_u8BulkState = BULK_CSW;
        USBD_SET_PAYLOAD_LEN(EP2, 13);
    }
}

void MSC_ReadMedia(uint32_t u32Addr, uint32_t u32Size, uint8_t *pu8Buffer)
{
    (void)u32Addr;
    (void)u32Size;
    (void)pu8Buffer;
}

void MSC_WriteMedia(uint32_t u32Addr, uint32_t u32Size, uint8_t *pu8Buffer)
{
    (void)u32Addr;
    (void)u32Size;
    (void)pu8Buffer;
}



