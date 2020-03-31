/******************************************************************************
 * @file     MassStorage.c
 * @brief    M2351 series USBD driver Sample file
 *
 * @note
 * Copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

/*!<Includes */
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"
#include "massstorage.h"
#include "SDCARD.h"

#pragma clang diagnostic ignored "-Wdate-time"

#if 0
#define DBG_PRINTF      printf
#else
#define DBG_PRINTF(...)
#endif

/*--------------------------------------------------------------------------*/
/* Global variables for Control Pipe */
static uint32_t s_u32TotalSectors = 0;

static uint8_t volatile s_u8EP2Ready = 0;
static uint8_t volatile s_u8EP3Ready = 0;

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

/* CBW/CSW variables */
static struct CBW s_sCBW;
static struct CSW s_sCSW;

uint32_t MassBlock[MASS_BUFFER_SIZE / 4];
uint32_t Storage_Block[STORAGE_BUFFER_SIZE / 4];

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

// code = 5Ah, Mode Sense
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
void MSC_ReadCapacity16(void);


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
            s_u32OutToggle = s_u32OutSkip = 0;
            DBG_PRINTF("Bus reset\n");
        }
        if(u32State & USBD_STATE_SUSPEND)
        {
            /* Enable USB but disable PHY */
            USBD_DISABLE_PHY();
            DBG_PRINTF("Suspend\n");
        }
        if(u32State & USBD_STATE_RESUME)
        {
            /* Enable USB and enable PHY */
            USBD_ENABLE_USB();
            DBG_PRINTF("Resume\n");
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
    if(s_u32OutToggle == (USBD->EPSTS0 & USBD_EPSTS0_EPSTS3_Msk))
    {
        s_u32OutSkip = 1;
        USBD_SET_PAYLOAD_LEN(EP3, EP3_MAX_PKT_SIZE);
    }
    else
    {
        s_u8EP3Ready = 1;
        s_u32OutToggle = USBD->EPSTS0 & USBD_EPSTS0_EPSTS3_Msk;
        s_u32OutSkip = 0;
    }
}


void MSC_Init(void)
{
    int32_t i;
    uint8_t *pu8;
    char *pSerial = __TIME__;

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
    s_u32TotalSectors = GetLogicSector();

    /*
       Generate Mass-Storage Device serial number
       To compliant USB-IF MSC test, we must enable serial string descriptor.
       However, window may fail to recognize the devices if PID/VID and serial number are all the same
       when plug them to Windows at the sample time.
       Therefore, we must generate different serial number for each device to avoid conflict
       when plug more than 2 MassStorage devices to Windows at the same time.

       NOTE: We use compiler predefine macro "__TIME__" to generate different number for serial
       at each build but each device here for a demo.
       User must change it to make sure all serial number is different between each device.
     */
    pu8 = (uint8_t *)(uint32_t)gsInfo.gu8StringDesc[3];
    for(i = 0; i < 8; i++)
        pu8[pu8[0] - 16 + i * 2] = pSerial[i];

}

void MSC_ClassRequest(void)
{
    uint8_t buf[8];

    USBD_GetSetupPacket(buf);

    if(buf[0] & EP_INPUT)    /* request data transfer direction */
    {
        // Device to host
        switch(buf[1])
        {
            case GET_MAX_LUN:
            {
                /* Check interface number with cfg descriptor and check wValue = 0, wLength = 1 */
                if((buf[4] == gsInfo.gu8ConfigDesc[LEN_CONFIG + 2]) && (buf[2] + buf[3] + buf[6] + buf[7] == 1))
                {
                    M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0)) = 0;
                    /* Data stage */
                    USBD_SET_DATA1(EP0);
                    USBD_SET_PAYLOAD_LEN(EP0, 1);
                    /* Status stage */
                    USBD_PrepareCtrlOut(0, 0);
                }
                else
                    USBD_SET_EP_STALL(EP1); // Stall when wrong parameter

                break;
            }
            default:
            {
                /* Setup error, stall the device */
                USBD_SetStall(0);
                DBG_PRINTF("Unknown MSC req(0x%x). stall ctrl pipe\n", buf[1]);
                break;
            }
        }
    }
    else
    {
        // Host to device
        switch(buf[1])
        {
            case BULK_ONLY_MASS_STORAGE_RESET:
            {
                /* Check interface number with cfg descriptor and check wValue = 0, wLength = 0 */
                if((buf[4] == gsInfo.gu8ConfigDesc[LEN_CONFIG + 2]) && (buf[2] + buf[3] + buf[6] + buf[7] == 0))
                {

                    s_u32Length = 0; // Reset all read/write data transfer
                    USBD_LockEpStall(0);

                    /* Clear ready */
                    USBD->EP[EP2].CFGP |= USBD_CFGP_CLRRDY_Msk;
                    USBD->EP[EP3].CFGP |= USBD_CFGP_CLRRDY_Msk;

                    /* Prepare to receive the CBW */

                    s_u8EP3Ready = 0;
                    s_u8BulkState = BULK_CBW;

                    USBD_SET_DATA1(EP3);
                    USBD_SET_EP_BUF_ADDR(EP3, s_u32BulkBuf0);
                    USBD_SET_PAYLOAD_LEN(EP3, EP3_MAX_PKT_SIZE);

                }
                else
                {
                    /* Stall when wrong parameter */
                    USBD_SET_EP_STALL(EP1);
                }

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
                DBG_PRINTF("Unknown MSC req (0x%x). stall ctrl pipe\n", buf[1]);
                break;
            }
        }
    }
}


void MSC_RequestSense(void)
{
    uint8_t tmp[20];

    memset(tmp, 0, 18);
    if(s_u8Prevent)
    {
        s_u8Prevent = 0;
        tmp[0] = 0x70;
    }
    else
        tmp[0] = 0xf0;

    tmp[2] = s_au8SenseKey[0];
    tmp[7] = 0x0a;
    tmp[12] = s_au8SenseKey[1];
    tmp[13] = s_au8SenseKey[2];
    USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP2)), tmp, 20);

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
    pu8Desc[4] = _GET_BYTE3(s_u32TotalSectors);
    pu8Desc[5] = _GET_BYTE2(s_u32TotalSectors);
    pu8Desc[6] = _GET_BYTE1(s_u32TotalSectors);
    pu8Desc[7] = _GET_BYTE0(s_u32TotalSectors);

    // Descriptor Code:
    // 01b = Unformatted Media - Maximum formattable capacity for this cartridge
    // 10b = Formatted Media - Current media capacity
    // 11b = No Cartridge in Drive - Maximum formattable capacity for any cartridge
    pu8Desc[8] = 0x02;


    // Block Length. Fixed to be 512 (MSB first)
    pu8Desc[ 9] = _GET_BYTE2(512);
    pu8Desc[10] = _GET_BYTE1(512);
    pu8Desc[11] = _GET_BYTE0(512);

    /*---------- Formattable Capacity Descriptor ----------*/
    // Number of Blocks
    pu8Desc[12] = _GET_BYTE3(s_u32TotalSectors);
    pu8Desc[13] = _GET_BYTE2(s_u32TotalSectors);
    pu8Desc[14] = _GET_BYTE1(s_u32TotalSectors);
    pu8Desc[15] = _GET_BYTE0(s_u32TotalSectors);

    // Block Length. Fixed to be 512 (MSB first)
    pu8Desc[17] = _GET_BYTE2(512);
    pu8Desc[18] = _GET_BYTE1(512);
    pu8Desc[19] = _GET_BYTE0(512);

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
            s_u32LbaAddress += u32Len / UDC_SECTOR_SIZE;
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
            s_u32LbaAddress += u32Len / UDC_SECTOR_SIZE;
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
    uint32_t tmp;

    memset((uint8_t *)MassCMD_BUF, 0, 36);

    tmp = s_u32TotalSectors - 1;
    *((uint8_t *)(MassCMD_BUF + 0)) = *((uint8_t *)&tmp + 3);
    *((uint8_t *)(MassCMD_BUF + 1)) = *((uint8_t *)&tmp + 2);
    *((uint8_t *)(MassCMD_BUF + 2)) = *((uint8_t *)&tmp + 1);
    *((uint8_t *)(MassCMD_BUF + 3)) = *((uint8_t *)&tmp + 0);
    *((uint8_t *)(MassCMD_BUF + 6)) = 0x02;
}

void MSC_ReadCapacity16(void)
{
    uint32_t tmp;

    memset((uint8_t *)MassCMD_BUF, 0, 36);

    tmp = s_u32TotalSectors - 1;
    *((uint8_t *)(MassCMD_BUF + 0)) = 0;
    *((uint8_t *)(MassCMD_BUF + 1)) = 0;
    *((uint8_t *)(MassCMD_BUF + 2)) = 0;
    *((uint8_t *)(MassCMD_BUF + 3)) = 0;
    *((uint8_t *)(MassCMD_BUF + 4)) = *((uint8_t *)&tmp + 3);
    *((uint8_t *)(MassCMD_BUF + 5)) = *((uint8_t *)&tmp + 2);
    *((uint8_t *)(MassCMD_BUF + 6)) = *((uint8_t *)&tmp + 1);
    *((uint8_t *)(MassCMD_BUF + 7)) = *((uint8_t *)&tmp + 0);
    *((uint8_t *)(MassCMD_BUF + 10)) = 0x02;
}


void MSC_ModeSense10(void)
{
    uint8_t i, j;
    uint8_t NumHead, NumSector;
    uint16_t NumCyl = 0;

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

            NumHead = 2;
            NumSector = 64;
        NumCyl = (uint16_t)s_u32TotalSectors / 128;

            *((uint8_t *)(MassCMD_BUF + 12)) = NumHead;
            *((uint8_t *)(MassCMD_BUF + 13)) = NumSector;
            *((uint8_t *)(MassCMD_BUF + 16)) = (uint8_t)(NumCyl >> 8);
            *((uint8_t *)(MassCMD_BUF + 17)) = (uint8_t)(NumCyl & 0x00ff);
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

            NumHead = 2;
            NumSector = 64;
        NumCyl = (uint16_t)s_u32TotalSectors / 128;

            *((uint8_t *)(MassCMD_BUF + 24)) = NumHead;
            *((uint8_t *)(MassCMD_BUF + 25)) = NumSector;
            *((uint8_t *)(MassCMD_BUF + 28)) = (uint8_t)(NumCyl >> 8);
            *((uint8_t *)(MassCMD_BUF + 29)) = (uint8_t)(NumCyl & 0x00ff);
            break;

        default:
            s_au8SenseKey[0] = 0x05;
            s_au8SenseKey[1] = 0x24;
            s_au8SenseKey[2] = 0x00;
    }
}

void MSC_Write(void)
{
    uint32_t lba, len;

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
                //DataFlashWrite(s_u32DataFlashStartAddr, STORAGE_BUFFER_SIZE, (uint32_t)STORAGE_DATA_BUF);
                MSC_WriteMedia(s_u32DataFlashStartAddr, STORAGE_BUFFER_SIZE, (uint8_t *)STORAGE_DATA_BUF);
                s_u32Address = STORAGE_DATA_BUF;
                //s_u32DataFlashStartAddr += STORAGE_BUFFER_SIZE;
                s_u32DataFlashStartAddr += STORAGE_BUFFER_SIZE / UDC_SECTOR_SIZE;
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
                lba = get_be32(&s_sCBW.au8Data[0]);
                len = s_sCBW.dCBWDataTransferLength;

                //len = lba * UDC_SECTOR_SIZE + s_sCBW.dCBWDataTransferLength - s_u32DataFlashStartAddr;
                len = (lba - s_u32DataFlashStartAddr) * UDC_SECTOR_SIZE + s_sCBW.dCBWDataTransferLength;
                if(len)
                {
                    //DataFlashWrite(s_u32DataFlashStartAddr , len, (uint32_t)STORAGE_DATA_BUF);
                    MSC_WriteMedia(s_u32DataFlashStartAddr, len, (uint8_t *)STORAGE_DATA_BUF);
                }
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

    if(s_u8EP3Ready)
    {
        s_u8EP3Ready = 0;

        if(s_u8BulkState == BULK_CBW)
        {
            u32Len = USBD_GET_PAYLOAD_LEN(EP3);
            if(u32Len > 31) u32Len = 31;

            /* Check Signature & length of CBW */
            /* Bulk Out buffer */
            if((*(uint32_t *)((uint32_t)USBD_BUF_BASE + s_u32BulkBuf0) != CBW_SIGNATURE)/* || (u32Len != 31)*/)
            {

                USBD_SET_EP_STALL(EP2);
                USBD_SET_EP_STALL(EP3);
                USBD_LockEpStall(1 << EP3);

                s_u8BulkState = BULK_CBW;
                DBG_PRINTF("CBW signature fail. stall bulk out pipe\n");
                return;

            }

            /* Get the CBW */
            for(i = 0; i < u32Len; i++)
                *((uint8_t *)(&s_sCBW.dCBWSignature) + i) = *(uint8_t *)((uint32_t)USBD_BUF_BASE + s_u32BulkBuf0 + i);


            /* Prepare to echo the tag from CBW to CSW */
            s_sCSW.dCSWTag = s_sCBW.dCBWTag;

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
                case UFI_VERIFY_10:
                case UFI_START_STOP:
                case UFI_TEST_UNIT_READY:
                {
                    DBG_PRINTF("Test Unit\n");
                    s_u8BulkState = BULK_IN;
                    MSC_AckCmd();
                    return;
                }
                case UFI_REQUEST_SENSE:
                {
                    u32Len = s_sCBW.dCBWDataTransferLength;
                    if(u32Len > 18) u32Len = 18;

                    if(u32Len)
                    {
                        if(s_sCBW.dCBWDataTransferLength > u32Len)
                        {
                            /* Expecting a STALL after data phase completes with a zero-length or short packet */
                            USBD_SET_EP_STALL(EP2);
                            USBD_SET_EP_STALL(EP3);
                            USBD_LockEpStall((1 << EP2) | (1 << EP3));
                            return;
                        }

                        MSC_RequestSense();
                        s_u8BulkState = BULK_IN;
                        USBD_SET_PAYLOAD_LEN(EP2, u32Len);
                    }
                    else
                    {
                        /* Just skip data phase if zero data transfer length */
                        s_u8BulkState = BULK_IN;
                        MSC_AckCmd();
                    }
                    return;
                }
                case UFI_READ_FORMAT_CAPACITY:
                {
                    s_u32Length = s_sCBW.dCBWDataTransferLength;

                    /* format capacity descriptor length is fixed to be 12 bytes */
                    if(s_u32Length > 20) s_u32Length = 20;

                    s_u32Address = MassCMD_BUF;
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
                        s_u32BytesInStorageBuf = s_u32Size;

                        s_u32Address += s_u32Size;
                        USBD_SET_EP_BUF_ADDR(EP2, s_u32BulkBuf0);
                        MSC_Read();
                    }
                    return;
                }
                case UFI_READ_CAPACITY:
                case UFI_READ_CAPACITY_16:
                {
                    s_u32Length = s_sCBW.dCBWDataTransferLength;
                    if(s_u32Length > 36) s_u32Length = 36;
                    s_u32Address = MassCMD_BUF;

                    if(s_sCBW.u8OPCode == UFI_READ_CAPACITY)
                        MSC_ReadCapacity();
                    else
                        MSC_ReadCapacity16();

                    s_u8BulkState = BULK_IN;
                    if(s_u32Length > 0)
                    {
                        if(s_u32Length > EP2_MAX_PKT_SIZE)
                            s_u32Size = EP2_MAX_PKT_SIZE;
                        else
                            s_u32Size = s_u32Length;

                        /* Bulk IN buffer */
                        USBD_MemCopy((uint8_t *)((uint32_t)USBD_BUF_BASE + s_u32BulkBuf1), (uint8_t *)s_u32Address, s_u32Size);
                        s_u32BytesInStorageBuf = s_u32Size;

                        s_u32Address += s_u32Size;
                        USBD_SET_EP_BUF_ADDR(EP2, s_u32BulkBuf0);
                        MSC_Read();
                    }
                    return;
                }
                case UFI_MODE_SELECT_10:
                {
                    s_u32Length = s_sCBW.dCBWDataTransferLength;
                    s_u32Address = MassCMD_BUF;

                    if(s_u32Length > 0)
                    {
                        USBD_SET_PAYLOAD_LEN(EP3, EP3_MAX_PKT_SIZE);
                        s_u8BulkState = BULK_OUT;
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
                    uint8_t u8PageCode;


                    u32Len = s_sCBW.dCBWDataTransferLength;
                    /* Limit length */
                    if(u32Len > 36) u32Len = 36;

                    u8PageCode = s_sCBW.au8Data[0];


                    s_u8BulkState = BULK_IN;
                    if(u32Len)
                    {
                        /* u8PageCode should be zero */
                        if(u8PageCode)
                        {
                            /* Expecting a STALL after data phase completes with a zero-length or short packet */
                            //USBD_SET_EP_STALL(EP0);
                            USBD_SET_EP_STALL(EP3);
                            USBD_LockEpStall(1 << EP3);

                            DBG_PRINTF("INQUIRY page code = %d", u8PageCode);
                        }
                        /* Bulk IN buffer */
                        USBD_MemCopy((uint8_t *)((uint32_t)USBD_BUF_BASE + s_u32BulkBuf1), (uint8_t *)s_au8InquiryID, u32Len);
                        USBD_SET_PAYLOAD_LEN(EP2, u32Len);

                        DBG_PRINTF("Inquiry, len %d\n", u32Len);


                    }
                    else
                    {
                        /* Next is status phase if zero data length in data phase */
                        MSC_AckCmd();
                    }

                    return;
                }
                case UFI_READ_10:
                case UFI_READ_12:
                {
                    /* Check if it is a new transfer */
                    if(s_u32Length == 0)
                    {
                        /* Prepare the data for Bulk IN transfer */

                        /* Get LBA address */
                        s_u32Address = get_be32(&s_sCBW.au8Data[0]);
                        s_u32LbaAddress = s_u32Address;
                        s_u32Length = s_sCBW.dCBWDataTransferLength;
                        s_u32BytesInStorageBuf = s_u32Length;

                        DBG_PRINTF("Read addr=0x%x, len=0x%x\n", s_u32LbaAddress, s_u32Length);

                        /* Error check  */
                        if((s_u32LbaAddress > s_u32TotalSectors) || (s_u32LbaAddress + s_u32Length / UDC_SECTOR_SIZE > s_u32TotalSectors))
                        {

                            USBD_SET_EP_STALL(EP2);
                            USBD_SET_EP_STALL(EP3);
                            USBD_LockEpStall((1 << EP2) | (1 << EP3));

                            DBG_PRINTF("Stall ep2, ep3. addr=0x%x, len=0x%x\n", s_u32LbaAddress, s_u32Length);

                            return;
                        }


                        i = s_u32Length;
                        if(i > STORAGE_BUFFER_SIZE)
                            i = STORAGE_BUFFER_SIZE;

                        MSC_ReadMedia(s_u32LbaAddress, i, (uint8_t *)STORAGE_DATA_BUF);
                        s_u32BytesInStorageBuf = i;
                        s_u32LbaAddress += i / UDC_SECTOR_SIZE;
                    }
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
                case UFI_WRITE_10:
                case UFI_WRITE_12:
                {


                    if(s_u32Length == 0)
                    {
                        s_u32Length = s_sCBW.dCBWDataTransferLength;
                        s_u32Address = STORAGE_DATA_BUF;
                        //s_u32DataFlashStartAddr = get_be32(&s_sCBW.au8Data[0]) * UDC_SECTOR_SIZE;
                        s_u32DataFlashStartAddr = get_be32(&s_sCBW.au8Data[0]);
                    }
                    DBG_PRINTF("Write 0x%x  0x%x\n", s_u32Address, s_u32Length);

                    if((s_u32Length > 0))
                    {
                        USBD_SET_PAYLOAD_LEN(EP3, EP3_MAX_PKT_SIZE);
                        s_u8BulkState = BULK_OUT;
                    }
                    return;
                }
                case UFI_MODE_SENSE_6:
                {
                    uint32_t u32Data = 0x3;
                    s_u8BulkState = BULK_IN;
                    USBD_MemCopy((uint8_t *)((uint32_t)USBD_BUF_BASE + s_u32BulkBuf1), (uint8_t *)u32Data, 4);
                    USBD_SET_PAYLOAD_LEN(EP2, 4);
                    return;
                }
                case UFI_MODE_SELECT_6:
                {
                    s_u32Length = s_sCBW.dCBWDataTransferLength;
                    s_u32Address = MassCMD_BUF;

                    if(s_u32Length > 0)
                    {
                        USBD_SET_PAYLOAD_LEN(EP3, EP3_MAX_PKT_SIZE);
                        s_u8BulkState = BULK_OUT;
                    }
                    return;
                }
                default:
                {
                    /* Just stall for unknown command */
                    //USBD_SET_EP_STALL(EP2);
                    //USBD_SET_EP_STALL(EP3);
                    //USBD_LockEpStall((1 << EP2) | (1 << EP3));
                    /* Unknown command */

                    DBG_PRINTF("Unknown cmd 0x%x\n", s_sCBW.u8OPCode);

                    if(s_sCBW.bmCBWFlags & 0x80)
                        USBD_SET_PAYLOAD_LEN(EP2, 4);
                    s_u8BulkState = BULK_IN;
                    MSC_AckCmd();
                    return;
                }
            }
        }
        else if(s_u8BulkState == BULK_OUT)
        {
            switch(s_sCBW.u8OPCode)
            {
                case UFI_WRITE_10:
                case UFI_WRITE_12:
                case UFI_MODE_SELECT_10:
                {
                    MSC_Write();
                    return;
                }
                default:
                {
                    /* Bulk-out of unkonwn command. Just dorp them. */
                    if(s_u32Length > EP3_MAX_PKT_SIZE)
                    {
                        USBD_SET_EP_BUF_ADDR(EP3, s_u32BulkBuf0);
                        USBD_SET_PAYLOAD_LEN(EP3, EP3_MAX_PKT_SIZE);
                        s_u32Length -= EP3_MAX_PKT_SIZE;
                    }
                    else
                    {
                        s_u32Length = 0;
                        s_u8BulkState = BULK_IN;
                        MSC_AckCmd();
                    }

                    break;
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

        DBG_PRINTF("CSW ack\n");

    }
    else if(s_u8BulkState == BULK_IN)
    {
        switch(s_sCBW.u8OPCode)
        {
            case UFI_READ_FORMAT_CAPACITY:
            case UFI_READ_CAPACITY:
            case UFI_READ_CAPACITY_16:
            case UFI_MODE_SENSE_10:
            {
                if(s_u32Length > 0)
                {
                    MSC_Read();
                    return;
                }

                if(s_sCBW.dCBWDataTransferLength > 36)
                    s_sCSW.dCSWDataResidue = s_sCBW.dCBWDataTransferLength - 36;
                else
                    s_sCSW.dCSWDataResidue = 0;

                s_sCSW.bCSWStatus = 0;
                break;
            }
            case UFI_READ_10:
            case UFI_READ_12:
            {
                if(s_u32Length > 0)
                {
                    MSC_ReadTrig();
                    return;
                }
                break;
            }
            case UFI_REQUEST_SENSE:
            case UFI_INQUIRY:
            {
                if(s_sCBW.dCBWDataTransferLength > 36)
                {
                    // Stall EP2 after short packet
                    //USBD_SET_EP_STALL(EP2);

                    s_sCSW.dCSWDataResidue = s_sCBW.dCBWDataTransferLength - 36;
                    s_sCSW.bCSWStatus = 0;
                    DBG_PRINTF("Inquiry size > 36\n");
                }
                else
                {
                    s_sCSW.dCSWDataResidue = 0;
                    s_sCSW.bCSWStatus = 0;
                    DBG_PRINTF("Inquiry ack, %x\n", USBD->EP[2].CFGP);
                }
                break;
            }

            case UFI_PREVENT_ALLOW_MEDIUM_REMOVAL:
            case UFI_VERIFY_10:
            case UFI_START_STOP:
            case UFI_WRITE_10:
            case UFI_WRITE_12:
            {
                if (s_sCBW.dCBWDataTransferLength < STORAGE_BUFFER_SIZE)
                    s_sCSW.dCSWDataResidue = 0;
                else
                    s_sCSW.dCSWDataResidue = s_sCBW.dCBWDataTransferLength - STORAGE_BUFFER_SIZE;

                s_sCSW.bCSWStatus = 0;
                break;
            }
            case UFI_TEST_UNIT_READY:
            {
                s_sCSW.dCSWDataResidue = 0;
                s_sCSW.bCSWStatus = 0;
                break;
            }
            case UFI_MODE_SENSE_6:
            {
                s_sCSW.dCSWDataResidue = s_sCBW.dCBWDataTransferLength - 4;
                s_sCSW.bCSWStatus = 0;
                break;
            }
            default:
            {
                // Unknown command
                //USBD_SET_EP_STALL(EP2);
                //USBD_SET_EP_STALL(EP3);
                //USBD_LockEpStall(1 << EP3);
                s_sCSW.dCSWDataResidue = s_sCBW.dCBWDataTransferLength;
                s_sCSW.bCSWStatus = 1; // return command failed
                break;
            }
        }

        /* Return the CSW */
        USBD_SET_EP_BUF_ADDR(EP2, s_u32BulkBuf1);

        /* Bulk IN buffer */
        USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + s_u32BulkBuf1), (uint8_t *)&s_sCSW.dCSWSignature, 16);

        s_u8BulkState = BULK_CSW;
        USBD_SET_PAYLOAD_LEN(EP2, 13);

        DBG_PRINTF("Prepare CSW\n");
    }
    else
    {
        // This should be a DATA phase error.
        USBD_SET_EP_STALL(EP2);
        USBD_SET_EP_STALL(EP3);
        USBD_LockEpStall((1 << EP2) | (1 << EP3));

        DBG_PRINTF("Unexpected IN ack\n");
    }
}

void MSC_ReadMedia(uint32_t addr, uint32_t size, uint8_t *buffer)
{
    SpiRead(addr, size, buffer);
}

void MSC_WriteMedia(uint32_t addr, uint32_t size, uint8_t *buffer)
{
    SpiWrite(addr, size, buffer);
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


    DBG_PRINTF("Set config\n");

}

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
