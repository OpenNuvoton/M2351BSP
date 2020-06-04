/***************************************************************************//**
 * @file     MassStorage.c
 * @brief    M2351 series USBD driver Sample file
 * @version  V1.00
 *
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
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

/* USB flow control variables */
static uint8_t s_u8BulkState;
static uint8_t s_u8Prevent = 0;
static uint32_t s_u32Size;

static uint8_t s_au8SenseKey[4];

static uint32_t s_u32DataFlashStartAddr;
static uint32_t s_u32Address;
static uint32_t s_u32Length;
static uint32_t s_u32LbaAddress;
static uint32_t s_u32BytesInStorageBuf;

static uint32_t s_u32BulkBuf0, s_u32BulkBuf1;
static uint32_t volatile s_u32OutToggle = 0, s_u32OutSkip = 0;
static uint32_t volatile s_u32CbwStall = 0;

/* CBW/CSW variables */
static struct CBW s_sCBW;
static struct CSW s_sCSW;

uint32_t g_au32MassBlock[MASS_BUFFER_SIZE / 4];
uint32_t g_au32StorageBlock[STORAGE_BUFFER_SIZE / 4];

/*--------------------------------------------------------------------------*/
static uint8_t s_au8InquiryID[36] =
{
    0x00,                   /* Peripheral Device Type */
    0x80,                   /* RMB */
    0x00,                   /* ISO/ECMA, ANSI Version */
    0x00,                   /* Response Data Format */
    0x1F, 0x00, 0x00, 0x00, /* Additional Length */

    /* Vendor Identification */
    'N', 'u', 'v', 'o', 't', 'o', 'n', ' ',

    /* Product Identification */
    'U', 'S', 'B', ' ', 'M', 'a', 's', 's', ' ', 'S', 't', 'o', 'r', 'a', 'g', 'e',

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
            s_u32OutToggle = s_u32OutSkip = 0;
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
    if((s_u32OutToggle == (USBD->EPSTS0 & USBD_EPSTS0_EPSTS3_Msk)) && !s_u32CbwStall)
    {
        s_u32OutSkip = 1;
        USBD_SET_PAYLOAD_LEN(EP3, EP3_MAX_PKT_SIZE);
    }
    else
    {
        s_u8EP3Ready = 1;
        s_u32OutToggle = USBD->EPSTS0 & USBD_EPSTS0_EPSTS3_Msk;
        s_u32OutSkip = 0;

        s_u32CbwStall = 0;
    }
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
    s_u32TotalSectors = DATA_FLASH_STORAGE_SIZE / UDC_SECTOR_SIZE;
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
                if((au8Buf[4] == gsInfo.gu8ConfigDesc[LEN_CONFIG + 2]) && (au8Buf[2] + au8Buf[3] + au8Buf[6] + au8Buf[7] == 1))
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
                    USBD_SET_EP_STALL(EP1); // Stall when wrong parameter
                }
                s_u32OutToggle = 0;
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
                if((au8Buf[4] == gsInfo.gu8ConfigDesc[LEN_CONFIG + 2]) && (au8Buf[2] + au8Buf[3] + au8Buf[6] + au8Buf[7] == 0))
                {
                    USBD_SET_DATA1(EP0);
                    USBD_SET_PAYLOAD_LEN(EP0, 0);

                    s_u32Length = 0; // Reset all read/write data transfer
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
                    USBD_SET_PAYLOAD_LEN(EP3, 31);

                    /* Status stage */
                    USBD_SET_DATA1(EP0);
                    USBD_SET_PAYLOAD_LEN(EP0, 0);
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
    uint8_t *pu8Desc;

    pu8Desc = (uint8_t *)MassCMD_BUF;
    memset(pu8Desc, 0, 36);

    /*---------- Capacity List Header ----------*/
    // Capacity List Length
    pu8Desc[3] = 0x10;

    /*---------- Current/Maximum Capacity Descriptor ----------*/
    // Number of blocks (MSB first)
    pu8Desc[4] = *((uint8_t *)&s_u32TotalSectors + 3);
    pu8Desc[5] = *((uint8_t *)&s_u32TotalSectors + 2);
    pu8Desc[6] = *((uint8_t *)&s_u32TotalSectors + 1);
    pu8Desc[7] = *((uint8_t *)&s_u32TotalSectors + 0);

    // Descriptor Code:
    // 01b = Unformatted Media - Maximum formattable capacity for this cartridge
    // 10b = Formatted Media - Current media capacity
    // 11b = No Cartridge in Drive - Maximum formattable capacity for any cartridge
    pu8Desc[8] = 0x02;


    // Block Length. Fixed to be 512 (MSB first)
    pu8Desc[ 9] = 0;
    pu8Desc[10] = 0x02;
    pu8Desc[11] = 0;

    /*---------- Formattable Capacity Descriptor ----------*/
    // Number of Blocks
    pu8Desc[12] = *((uint8_t *)&s_u32TotalSectors + 3);
    pu8Desc[13] = *((uint8_t *)&s_u32TotalSectors + 2);
    pu8Desc[14] = *((uint8_t *)&s_u32TotalSectors + 1);
    pu8Desc[15] = *((uint8_t *)&s_u32TotalSectors + 0);

    // Block Length. Fixed to be 512 (MSB first)
    pu8Desc[17] = 0;
    pu8Desc[18] = 0x02;
    pu8Desc[19] = 0;

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

    memset((uint8_t *)MassCMD_BUF, 0, 36);

    u32Tmp = s_u32TotalSectors - 1;
    *((uint8_t *)(MassCMD_BUF + 0)) = *((uint8_t *)&u32Tmp + 3);
    *((uint8_t *)(MassCMD_BUF + 1)) = *((uint8_t *)&u32Tmp + 2);
    *((uint8_t *)(MassCMD_BUF + 2)) = *((uint8_t *)&u32Tmp + 1);
    *((uint8_t *)(MassCMD_BUF + 3)) = *((uint8_t *)&u32Tmp + 0);
    *((uint8_t *)(MassCMD_BUF + 6)) = 0x02;
}

void MSC_ReadCapacity16(void)
{
    uint32_t u32Tmp;

    memset((uint8_t *)MassCMD_BUF, 0, 36);

    u32Tmp = s_u32TotalSectors - 1;
    *((uint8_t *)(MassCMD_BUF + 0)) = 0;
    *((uint8_t *)(MassCMD_BUF + 1)) = 0;
    *((uint8_t *)(MassCMD_BUF + 2)) = 0;
    *((uint8_t *)(MassCMD_BUF + 3)) = 0;
    *((uint8_t *)(MassCMD_BUF + 4)) = *((uint8_t *)&u32Tmp + 3);
    *((uint8_t *)(MassCMD_BUF + 5)) = *((uint8_t *)&u32Tmp + 2);
    *((uint8_t *)(MassCMD_BUF + 6)) = *((uint8_t *)&u32Tmp + 1);
    *((uint8_t *)(MassCMD_BUF + 7)) = *((uint8_t *)&u32Tmp + 0);
    *((uint8_t *)(MassCMD_BUF + 10)) = 0x02;
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

void MSC_Write(void)
{
    uint32_t u32Lba, u32Len;

    if(s_u32OutSkip == 0)
    {
        if(s_u32Length > EP3_MAX_PKT_SIZE)
        {
            if(USBD_GET_EP_BUF_ADDR(EP3) == s_u32BulkBuf0)
            {
                USBD_SET_EP_BUF_ADDR(EP3, s_u32BulkBuf1);
                USBD_SET_PAYLOAD_LEN(EP3, EP3_MAX_PKT_SIZE);
                USBD_MemCopy((uint8_t *)s_u32Address, (uint8_t *)((uint32_t)USBD_BUF_BASE + s_u32BulkBuf0), EP3_MAX_PKT_SIZE);
            }
            else
            {
                USBD_SET_EP_BUF_ADDR(EP3, s_u32BulkBuf0);
                USBD_SET_PAYLOAD_LEN(EP3, EP3_MAX_PKT_SIZE);
                USBD_MemCopy((uint8_t *)s_u32Address, (uint8_t *)((uint32_t)USBD_BUF_BASE + s_u32BulkBuf1), EP3_MAX_PKT_SIZE);
            }

            s_u32Address += EP3_MAX_PKT_SIZE;
            s_u32Length -= EP3_MAX_PKT_SIZE;

            /* Buffer full. Writer it to storage first. */
            if(s_u32Address >= (STORAGE_DATA_BUF + STORAGE_BUFFER_SIZE))
            {
                DataFlashWrite(s_u32DataFlashStartAddr, STORAGE_BUFFER_SIZE, (uint32_t)STORAGE_DATA_BUF);

                s_u32Address = STORAGE_DATA_BUF;
                s_u32DataFlashStartAddr += STORAGE_BUFFER_SIZE;
            }
        }
        else
        {
            if(USBD_GET_EP_BUF_ADDR(EP3) == s_u32BulkBuf0)
                USBD_MemCopy((uint8_t *)s_u32Address, (uint8_t *)((uint32_t)USBD_BUF_BASE + s_u32BulkBuf0), s_u32Length);
            else
                USBD_MemCopy((uint8_t *)s_u32Address, (uint8_t *)((uint32_t)USBD_BUF_BASE + s_u32BulkBuf1), s_u32Length);
            s_u32Address += s_u32Length;
            s_u32Length = 0;


            if((s_sCBW.u8OPCode == UFI_WRITE_10) || (s_sCBW.u8OPCode == UFI_WRITE_12))
            {
                u32Lba = get_be32(&s_sCBW.au8Data[0]);
                u32Len = s_sCBW.dCBWDataTransferLength;

                u32Len = u32Lba * UDC_SECTOR_SIZE + s_sCBW.dCBWDataTransferLength - s_u32DataFlashStartAddr;
                if(u32Len)
                    DataFlashWrite(s_u32DataFlashStartAddr, u32Len, (uint32_t)STORAGE_DATA_BUF);
            }

            s_u8BulkState = BULK_IN;
            MSC_AckCmd();
        }
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

                s_u32CbwStall = 1;
                s_u8BulkState = BULK_CBW;
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
                            s_u32CbwStall = 1;
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
                    if(s_u32Length == 0)
                    {
                        s_u32Length = s_sCBW.dCBWDataTransferLength;
                        s_u32Address = MassCMD_BUF;
                    }

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
                case UFI_MODE_SELECT_6:
                case UFI_MODE_SELECT_10:
                {
                    s_u32Length = s_sCBW.dCBWDataTransferLength;
                    s_u32Address = MassCMD_BUF;

                    if(s_u32Length > 0)
                    {
                        USBD_SET_PAYLOAD_LEN(EP3, EP3_MAX_PKT_SIZE);
                        s_u8BulkState = BULK_OUT;
                    }
                    else
                    {
                        s_u8BulkState = BULK_IN;
                        MSC_AckCmd();
                    }
                    return;
                }
                case UFI_MODE_SENSE_6:
                {

                    *(uint8_t *)((uint32_t)USBD_BUF_BASE + s_u32BulkBuf1 + 0) = 0x3;
                    *(uint8_t *)((uint32_t)USBD_BUF_BASE + s_u32BulkBuf1 + 1) = 0x0;
                    *(uint8_t *)((uint32_t)USBD_BUF_BASE + s_u32BulkBuf1 + 2) = 0x0;
                    *(uint8_t *)((uint32_t)USBD_BUF_BASE + s_u32BulkBuf1 + 3) = 0x0;

                    USBD_SET_PAYLOAD_LEN(EP2, 4);
                    s_u8BulkState = BULK_IN;
                    s_sCSW.bCSWStatus = 0;
                    s_sCSW.dCSWDataResidue = u32Hcount - 4;
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
                        u32Dcount = (get_be32(&s_sCBW.au8Data[4]) >> 8) * 512;
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
                            s_u32CbwStall = 1;
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
                    s_u32LbaAddress = s_u32Address * UDC_SECTOR_SIZE;
                    s_u32Length = s_sCBW.dCBWDataTransferLength;
                    s_u32BytesInStorageBuf = s_u32Length;

                    i = s_u32Length;
                    if(i > STORAGE_BUFFER_SIZE)
                        i = STORAGE_BUFFER_SIZE;

                    MSC_ReadMedia(s_u32Address * UDC_SECTOR_SIZE, i, (uint8_t *)STORAGE_DATA_BUF);
                    s_u32BytesInStorageBuf = i;
                    s_u32LbaAddress += i;

                    s_u32Address = STORAGE_DATA_BUF;

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
                            s_u32DataFlashStartAddr = get_be32(&s_sCBW.au8Data[0]) * UDC_SECTOR_SIZE;
                        }
                        else     /* Hi <> Do (Case 8) */
                        {
                            s_u8Prevent = 1;
                            s_sCSW.dCSWDataResidue = u32Hcount;
                            s_sCSW.bCSWStatus = 0x1;
                            USBD_SET_EP_STALL(EP2);
                            s_u8BulkState = BULK_IN;
                            MSC_AckCmd();
                            return;
                        }
                    }

                    if((s_u32Length > 0))
                    {
                        USBD_SET_PAYLOAD_LEN(EP3, EP3_MAX_PKT_SIZE);
                        s_u8BulkState = BULK_OUT;
                    }
                    return;
                }
                case UFI_READ_CAPACITY_16:
                {
                    USBD_SET_EP_STALL(EP2);
                    s_u8Prevent = 1;
                    s_sCSW.bCSWStatus = 0x01;
                    s_sCSW.dCSWDataResidue = 0;
                    s_u8BulkState = BULK_IN;
                    MSC_AckCmd();
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
        else if(s_u8BulkState == BULK_OUT)
        {
            switch(s_sCBW.u8OPCode)
            {
                case UFI_WRITE_12:
                case UFI_WRITE_10:
                case UFI_MODE_SELECT_6:
                case UFI_MODE_SELECT_10:
                {
                    MSC_Write();
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
    DataFlashRead(u32Addr, u32Size, (uint32_t)pu8Buffer);
}

void MSC_WriteMedia(uint32_t u32Addr, uint32_t u32Size, uint8_t *pu8Buffer)
{
    (void)u32Addr;
    (void)u32Size;
    (void)pu8Buffer;
}

void MSC_SetConfig(void)
{
    // Clear stall status and ready
    USBD->EP[2].CFGP = 1;
    USBD->EP[3].CFGP = 1;
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


    USBD_LockEpStall(0);

    s_u8BulkState = BULK_CBW;
}

