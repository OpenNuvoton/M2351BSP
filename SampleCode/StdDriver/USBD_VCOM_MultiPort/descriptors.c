/******************************************************************************
 * @file     descriptors.c
 * @version  V3.00
 * @brief    USBD descriptor
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2024 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
/*!<Includes */
#include "NuMicro.h"
#include "vcom_serial.h"

/*----------------------------------------------------------------------------*/
/*!<USB Device Descriptor */
static uint8_t s_au8DeviceDescriptor[] =
{
    LEN_DEVICE,         /* bLength = 18 */
    DESC_DEVICE,        /* bDescriptorType */
    0x10, 0x01,         /* bcdUSB */
    0xEF,               /* bDeviceClass: IAD */
    0x02,               /* bDeviceSubClass */
    0x01,               /* bDeviceProtocol */
    EP0_MAX_PKT_SIZE,   /* bMaxPacketSize0 */
    /* idVendor */
    USBD_VID & 0x00FF,
    (USBD_VID & 0xFF00) >> 8,
    /* idProduct */
    USBD_PID & 0x00FF,
    (USBD_PID & 0xFF00) >> 8,
    0x00, 0x03,     /* bcdDevice */
    0x01,           /* iManufacture */
    0x02,           /* iProduct */
    0x03,           /* iSerialNumber */
    0x01            /* bNumConfigurations */
};

/*!<USB Configure Descriptor */
static uint8_t s_au8ConfigDescriptor[] =
{
    LEN_CONFIG,     /* bLength = 9          */
    DESC_CONFIG,    /* bDescriptorType      */
#if (VCOM_CNT == 1)
    0x4B, 0x00,     /* wTotalLength         */
    0x02,           /* bNumInterfaces       */
#elif (VCOM_CNT == 2)
    0x8D, 0x00,     /* wTotalLength         */
    0x04,           /* bNumInterfaces       */
#elif (VCOM_CNT == 3)
    0xCF, 0x00,     /* wTotalLength         */
    0x06,           /* bNumInterfaces       */
#endif
    0x01,           /* bConfigurationValue  */
    0x00,           /* iConfiguration       */
    0xC0,           /* bmAttributes         */
    0x32,           /* MaxPower             */

#if (VCOM_CNT >= 1)
    // IAD
    0x08,   // bLength: Interface Descriptor size
    0x0B,   // bDescriptorType: IAD
    0x00,   // bFirstInterface
    0x02,   // bInterfaceCount
    0x02,   // bFunctionClass: CDC
    0x02,   // bFunctionSubClass
    0x01,   // bFunctionProtocol
    0x02,   // iFunction

    /* VCOM - 1 */
    /* INTERFACE descriptor */
    LEN_INTERFACE,  /* bLength              */
    DESC_INTERFACE, /* bDescriptorType      */
    0x00,           /* bInterfaceNumber     */
    0x00,           /* bAlternateSetting    */
    0x01,           /* bNumEndpoints        */
    0x02,           /* bInterfaceClass      */
    0x02,           /* bInterfaceSubClass   */
    0x01,           /* bInterfaceProtocol   */
    0x00,           /* iInterface           */

    /* Communication Class Specified INTERFACE descriptor */
    0x05,           /* Size of the descriptor, in bytes */
    0x24,           /* CS_INTERFACE descriptor type */
    0x00,           /* Header functional descriptor subtype */
    0x10, 0x01,     /* Communication device compliant to the communication spec. ver. 1.10 */

    /* Communication Class Specified INTERFACE descriptor */
    0x05,           /* Size of the descriptor, in bytes */
    0x24,           /* CS_INTERFACE descriptor type */
    0x01,           /* Call management functional descriptor */
    0x00,           /* BIT0: Whether device handle call management itself. */
    /* BIT1: Whether device can send/receive call management information over a Data Class Interface 0 */
    0x01,           /* Interface number of data class interface optionally used for call management */

    /* Communication Class Specified INTERFACE descriptor */
    0x04,           /* Size of the descriptor, in bytes */
    0x24,           /* CS_INTERFACE descriptor type */
    0x02,           /* Abstract Control Management functional descriptor subtype */
    0x00,           /* bmCapabilities       */

    /* Communication Class Specified INTERFACE descriptor */
    0x05,           /* bLength              */
    0x24,           /* bDescriptorType: CS_INTERFACE descriptor type */
    0x06,           /* bDescriptorSubType   */
    0x00,           /* bMasterInterface     */
    0x01,           /* bSlaveInterface0     */

    /* ENDPOINT descriptor */
    LEN_ENDPOINT,                   /* bLength          */
    DESC_ENDPOINT,                  /* bDescriptorType  */
    (EP_INPUT | INT_IN_EP_NUM),     /* bEndpointAddress */
    EP_INT,                         /* bmAttributes     */
    EP4_MAX_PKT_SIZE, 0x00,         /* wMaxPacketSize   */
    0x01,                           /* bInterval        */

    /* INTERFACE descriptor */
    LEN_INTERFACE,  /* bLength              */
    DESC_INTERFACE, /* bDescriptorType      */
    0x01,           /* bInterfaceNumber     */
    0x00,           /* bAlternateSetting    */
    0x02,           /* bNumEndpoints        */
    0x0A,           /* bInterfaceClass      */
    0x00,           /* bInterfaceSubClass   */
    0x00,           /* bInterfaceProtocol   */
    0x00,           /* iInterface           */

    /* ENDPOINT descriptor */
    LEN_ENDPOINT,                   /* bLength          */
    DESC_ENDPOINT,                  /* bDescriptorType  */
    (EP_INPUT | BULK_IN_EP_NUM),    /* bEndpointAddress */
    EP_BULK,                        /* bmAttributes     */
    EP2_MAX_PKT_SIZE, 0x00,         /* wMaxPacketSize   */
    0x00,                           /* bInterval        */

    /* ENDPOINT descriptor */
    LEN_ENDPOINT,                   /* bLength          */
    DESC_ENDPOINT,                  /* bDescriptorType  */
    (EP_OUTPUT | BULK_OUT_EP_NUM),  /* bEndpointAddress */
    EP_BULK,                        /* bmAttributes     */
    EP3_MAX_PKT_SIZE, 0x00,         /* wMaxPacketSize   */
    0x00,                           /* bInterval        */
#endif

#if (VCOM_CNT >= 2)
    // IAD
    0x08,   // bLength: Interface Descriptor size
    0x0B,   // bDescriptorType: IAD
    0x02,   // bFirstInterface
    0x02,   // bInterfaceCount
    0x02,   // bFunctionClass: CDC
    0x02,   // bFunctionSubClass
    0x01,   // bFunctionProtocol
    0x02,   // iFunction

    /* VCOM - 2 */
    /* INTERFACE descriptor */
    LEN_INTERFACE,  /* bLength              */
    DESC_INTERFACE, /* bDescriptorType      */
    0x02,           /* bInterfaceNumber     */
    0x00,           /* bAlternateSetting    */
    0x01,           /* bNumEndpoints        */
    0x02,           /* bInterfaceClass      */
    0x02,           /* bInterfaceSubClass   */
    0x01,           /* bInterfaceProtocol   */
    0x00,           /* iInterface           */

    /* Communication Class Specified INTERFACE descriptor */
    0x05,           /* Size of the descriptor, in bytes */
    0x24,           /* CS_INTERFACE descriptor type */
    0x00,           /* Header functional descriptor subtype */
    0x10, 0x01,     /* Communication device compliant to the communication spec. ver. 1.10 */

    /* Communication Class Specified INTERFACE descriptor */
    0x05,           /* Size of the descriptor, in bytes */
    0x24,           /* CS_INTERFACE descriptor type */
    0x01,           /* Call management functional descriptor */
    0x00,           /* BIT0: Whether device handle call management itself. */
    /* BIT1: Whether device can send/receive call management information over a Data Class Interface 0 */
    0x03,           /* Interface number of data class interface optionally used for call management */

    /* Communication Class Specified INTERFACE descriptor */
    0x04,           /* Size of the descriptor, in bytes */
    0x24,           /* CS_INTERFACE descriptor type */
    0x02,           /* Abstract control management functional descriptor subtype */
    0x00,           /* bmCapabilities       */

    /* Communication Class Specified INTERFACE descriptor */
    0x05,           /* bLength              */
    0x24,           /* bDescriptorType: CS_INTERFACE descriptor type */
    0x06,           /* bDescriptorSubType   */
    0x02,           /* bMasterInterface     */
    0x03,           /* bSlaveInterface0     */

    /* ENDPOINT descriptor */
    LEN_ENDPOINT,                   /* bLength          */
    DESC_ENDPOINT,                  /* bDescriptorType  */
    (EP_INPUT | INT_IN_EP_NUM_1),   /* bEndpointAddress */
    EP_INT,                         /* bmAttributes     */
    EP5_MAX_PKT_SIZE, 0x00,         /* wMaxPacketSize   */
    0x01,                           /* bInterval        */

    /* INTERFACE descriptor */
    LEN_INTERFACE,  /* bLength              */
    DESC_INTERFACE, /* bDescriptorType      */
    0x03,           /* bInterfaceNumber     */
    0x00,           /* bAlternateSetting    */
    0x02,           /* bNumEndpoints        */
    0x0A,           /* bInterfaceClass      */
    0x00,           /* bInterfaceSubClass   */
    0x00,           /* bInterfaceProtocol   */
    0x00,           /* iInterface           */

    /* ENDPOINT descriptor */
    LEN_ENDPOINT,                    /* bLength          */
    DESC_ENDPOINT,                   /* bDescriptorType  */
    (EP_OUTPUT | BULK_OUT_EP_NUM_1), /* bEndpointAddress */
    EP_BULK,                         /* bmAttributes     */
    EP6_MAX_PKT_SIZE, 0x00,          /* wMaxPacketSize   */
    0x00,                            /* bInterval        */

    /* ENDPOINT descriptor */
    LEN_ENDPOINT,                    /* bLength          */
    DESC_ENDPOINT,                   /* bDescriptorType  */
    (EP_INPUT | BULK_IN_EP_NUM_1),   /* bEndpointAddress */
    EP_BULK,                         /* bmAttributes     */
    EP7_MAX_PKT_SIZE, 0x00,          /* wMaxPacketSize   */
    0x00,                            /* bInterval        */
#endif

#if (VCOM_CNT >= 3)
    // IAD
    0x08,   // bLength: Interface Descriptor size
    0x0B,   // bDescriptorType: IAD
    0x04,   // bFirstInterface
    0x02,   // bInterfaceCount
    0x02,   // bFunctionClass: CDC
    0x02,   // bFunctionSubClass
    0x01,   // bFunctionProtocol
    0x02,   // iFunction

    /* VCOM - 3 */
    /* INTERFACE descriptor */
    LEN_INTERFACE,  /* bLength              */
    DESC_INTERFACE, /* bDescriptorType      */
    0x04,           /* bInterfaceNumber     */
    0x00,           /* bAlternateSetting    */
    0x01,           /* bNumEndpoints        */
    0x02,           /* bInterfaceClass      */
    0x02,           /* bInterfaceSubClass   */
    0x01,           /* bInterfaceProtocol   */
    0x00,           /* iInterface           */

    /* Communication Class Specified INTERFACE descriptor */
    0x05,           /* Size of the descriptor, in bytes */
    0x24,           /* CS_INTERFACE descriptor type */
    0x00,           /* Header functional descriptor subtype */
    0x10, 0x01,     /* Communication device compliant to the communication spec. ver. 1.10 */

    /* Communication Class Specified INTERFACE descriptor */
    0x05,           /* Size of the descriptor, in bytes */
    0x24,           /* CS_INTERFACE descriptor type */
    0x01,           /* Call management functional descriptor */
    0x00,           /* BIT0: Whether device handle call management itself. */
    /* BIT1: Whether device can send/receive call management information over a Data Class Interface 0 */
    0x05,           /* Interface number of data class interface optionally used for call management */

    /* Communication Class Specified INTERFACE descriptor */
    0x04,           /* Size of the descriptor, in bytes */
    0x24,           /* CS_INTERFACE descriptor type */
    0x02,           /* Abstract control management functional descriptor subtype */
    0x00,           /* bmCapabilities       */

    /* Communication Class Specified INTERFACE descriptor */
    0x05,           /* bLength              */
    0x24,           /* bDescriptorType: CS_INTERFACE descriptor type */
    0x06,           /* bDescriptorSubType   */
    0x04,           /* bMasterInterface     */
    0x05,           /* bSlaveInterface0     */

    /* ENDPOINT descriptor */
    LEN_ENDPOINT,                   /* bLength          */
    DESC_ENDPOINT,                  /* bDescriptorType  */
    (EP_INPUT | INT_IN_EP_NUM_2),   /* bEndpointAddress */
    EP_INT,                         /* bmAttributes     */
    EP8_MAX_PKT_SIZE, 0x00,         /* wMaxPacketSize   */
    0x01,                           /* bInterval        */

    /* INTERFACE descriptor */
    LEN_INTERFACE,  /* bLength              */
    DESC_INTERFACE, /* bDescriptorType      */
    0x05,           /* bInterfaceNumber     */
    0x00,           /* bAlternateSetting    */
    0x02,           /* bNumEndpoints        */
    0x0A,           /* bInterfaceClass      */
    0x00,           /* bInterfaceSubClass   */
    0x00,           /* bInterfaceProtocol   */
    0x00,           /* iInterface           */

    /* ENDPOINT descriptor */
    LEN_ENDPOINT,                    /* bLength          */
    DESC_ENDPOINT,                   /* bDescriptorType  */
    (EP_OUTPUT | BULK_OUT_EP_NUM_2), /* bEndpointAddress */
    EP_BULK,                         /* bmAttributes     */
    EP9_MAX_PKT_SIZE, 0x00,          /* wMaxPacketSize   */
    0x00,                            /* bInterval        */

    /* ENDPOINT descriptor */
    LEN_ENDPOINT,                    /* bLength          */
    DESC_ENDPOINT,                   /* bDescriptorType  */
    (EP_INPUT | BULK_IN_EP_NUM_2),   /* bEndpointAddress */
    EP_BULK,                         /* bmAttributes     */
    EP10_MAX_PKT_SIZE, 0x00,         /* wMaxPacketSize   */
    0x00,                            /* bInterval        */
#endif
};

/*!<USB Language String Descriptor */
static uint8_t s_au8StringLang[4] =
{
    4,              /* bLength */
    DESC_STRING,    /* bDescriptorType */
    0x09, 0x04
};

/*!<USB Vendor String Descriptor */
static uint8_t s_au8VendorStringDesc[] =
{
    16,
    DESC_STRING,
    'N', 0, 'u', 0, 'v', 0, 'o', 0, 't', 0, 'o', 0, 'n', 0
};

/*!<USB Product String Descriptor */
static uint8_t s_au8ProductStringDesc[] =
{
    32,             /* bLength          */
    DESC_STRING,    /* bDescriptorType  */
    'U', 0, 'S', 0, 'B', 0, ' ', 0, 'V', 0, 'i', 0, 'r', 0, 't', 0, 'u', 0, 'a', 0, 'l', 0, ' ', 0, 'C', 0, 'O', 0, 'M', 0
};

static uint8_t *s_apu8UsbString[4] =
{
    s_au8StringLang,
    s_au8VendorStringDesc,
    s_au8ProductStringDesc,
    NULL
};

static uint8_t *s_apu8UsbHidReport[3] =
{
    NULL,
    NULL,
    NULL
};

static uint32_t s_au32UsbHidReportLen[3] =
{
    0,
    0,
    0
};

static uint32_t s_au32ConfigHidDescIdx[3] =
{
    0,
    0,
    0
};

const S_USBD_INFO_T gsInfo =
{
    (uint8_t *)s_au8DeviceDescriptor,
    (uint8_t *)s_au8ConfigDescriptor,
    (uint8_t **)s_apu8UsbString,
    (uint8_t **)s_apu8UsbHidReport,
    NULL,
    (uint32_t *)s_au32UsbHidReportLen,
    (uint32_t *)s_au32ConfigHidDescIdx
};
