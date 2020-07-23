/**************************************************************************//**
 * @file     descriptors.c
 * @brief    NuMicro series USBD driver source file
 *
 * @note
 * Copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
/*!<Includes */
#include "NuMicro.h"
#include "usbd_audio.h"

/*----------------------------------------------------------------------------*/
/*!<USB Device Descriptor */
static uint8_t s_au8DeviceDescriptor[] =
{
    LEN_DEVICE,     /* bLength */
    DESC_DEVICE,    /* bDescriptorType */
#ifdef SUPPORT_LPM
    0x01, 0x02,     /* bcdUSB >= 0x0201 to support LPM */
#else
    0x10, 0x01,     /* bcdUSB */
#endif
    0x00,           /* bDeviceClass */
    0x00,           /* bDeviceSubClass */
    0x00,           /* bDeviceProtocol */
    EP0_MAX_PKT_SIZE,   /* bMaxPacketSize0 */
    /* idVendor */
    USBD_VID & 0x00FF,
    ((USBD_VID & 0xFF00) >> 8),
    /* idProduct */
    USBD_PID & 0x00FF,
    ((USBD_PID & 0xFF00) >> 8),
    0x00, 0x01,     /* bcdDevice */
    0x01,           /* iManufacture */
    0x02,           /* iProduct */
    0x00,           /* iSerialNumber - no serial */
    0x01            /* bNumConfigurations */
};

/*!<USB Configure Descriptor */
static uint8_t s_au8ConfigDescriptor[] =
{

    LEN_CONFIG,     /* bLength */
    DESC_CONFIG,    /* bDescriptorType */
    0xBE, 0x00,     /* wTotalLength */
                    /*
                       Configuration Descriptor                    (0x09)
                       Interface Descriptor (Audio Class)          (0x09)
                       Audio Control Interface Header Descriptor   (0x0A)
                       Microphone - Audio Control
                         Audio Control Input Terminal Descriptor   (0x0C)
                         Audio Control Feature Unit Descriptor     (0x08)
                         Audio Control Output Terminal Descriptor  (0x09)
                       Speaker - Audio Control
                         Audio Control Input Terminal Descriptor   (0x0C)
                         Audio Control Feature Unit Descriptor     (0x0A)
                         Audio Control Output Terminal Descriptor  (0x09)
                       Microphone - Interface alternate 0
                         Standard AS interface                     (0x09)
                       Microphone - Interface alternate 1~6
                         Standard AS interface                                         (0x09)
                         Audio Streaming Class Specific Interface Descriptor           (0x07)
                         Audio Streaming Format Type Descriptor                        (0x0B)
                         Endpoint Descriptor                                           (0x07)
                         Audio Streaming Class Specific Audio Data Endpoint Descriptor (0x07)
                         *Each Interface alternate Summary                             (0x29)
                       Speaker - Interface alternate 0
                         Standard AS interface                     (0x09)
                       Speaker - Interface alternate 1~4
                         Standard AS interface                                         (0x09)
                         Audio Streaming Class Specific Interface Descriptor           (0x07)
                         Audio Streaming Format Type Descriptor                        (0x0B)
                         Endpoint Descriptor                                           (0x09)
                         Audio Streaming Class Specific Audio Data Endpoint Descriptor (0x07)
                         *Each Interface alternate Summary                             (0x2B)

                       0x09 + 0x09 + 0x0A + (0x0C + 0x08 + 0x09) + (0x0C + 0x0A + 0x09) +
                       0x09 + 0x29
                       0x09 + 0x2B = 0xBE
                    */
    0x03,           /* bNumInterfaces - Interface 0, Interface 1 (Microphone), Interface 2 (Speaker) */
    0x01,           /* bConfigurationValue */
    0x00,           /* iConfiguration */
    0x80,           /* bmAttributes */
    0x20,           /* Max power */

    /* Interface Descriptor (Audio Class) */
    0x09,           /* bLength */
    0x04,           /* bDescriptorType */
    0x00,           /* bInterfaceNumber */
    0x00,           /* bAlternateSetting */
    0x00,           /* bNumEndpoints */
    0x01,           /* bInterfaceClass:AUDIO */
    0x01,           /* bInterfaceSubClass:AUDIOCONTROL */
    0x00,           /* bInterfaceProtocol */
    0x00,           /* iInterface */

    /* Audio Control Interface Header Descriptor */
    0x0A,           /* bLength */
    0x24,           /* bDescriptorType:CS_INTERFACE */
    0x01,           /* bDescriptorSubType:HEADER */
    0x00, 0x01,     /* bcdADC:1.0 */
    0x46, 0x00,     /* wTotalLength
                       Audio Control Interface Header Descriptor  (0x0A)
                       Microphone - Audio Control
                         Audio Control Input Terminal Descriptor  (0x0C)
                         Audio Control Feature Unit Descriptor    (0x08)
                         Audio Control Output Terminal Descriptor (0x09)
                       Speaker - Audio Control
                         Audio Control Input Terminal Descriptor  (0x0C)
                         Audio Control Feature Unit Descriptor    (0x0A)
                         Audio Control Output Terminal Descriptor (0x09)

                         0x0A + (0x0C + 0x08 + 0x09) + (0x0C + 0x0A + 0x09) = 0x46
                    */
    0x02,           /* bInCollection */
    0x01,           /* baInterfaceNr(1) - Microphone */
    0x02,           /* baInterfaceNr(2) - Speaker */

    /* Audio Control Input Terminal Descriptor - Microphone (Terminal ID 4) */
    0x0C,           /* bLength */
    0x24,           /* bDescriptorType:CS_INTERFACE */
    0x02,           /* bDescriptorSubType:INPUT_TERMINAL*/
    0x04,           /* bTerminalID*/
    0x02, 0x04,     /* wTerminalType: Headset */
    0x00,           /* bAssocTerminal*/
    0x02,           /* bNrChannels : a number that specifies how many logical audio channels are present in the cluster */
    0x03, 0x00,     /* wChannelConfig: a bit field that indicates which spatial locations are present in the cluster.
                       The bit allocations are as follows:
                         D0: Left Front (L)
                         D1: Right Front (R)
                         D2: Center Front (C)
                         D3: Low Frequency Enhancement (LFE)
                         D4: Left Surround (LS)
                         D5: Right Surround (RS)
                         D6: Left of Center (LC)
                         D7: Right of Center (RC)
                         D8: Surround (S)
                         D9: Side Left (SL)
                         D10: Side Right (SR)
                         D11: Top (T)
                         D15..12: Reserved
                    */
    0x00,           /* iChannelNames*/
    0x00,           /* iTerminal*/

    /* Audio Control Feature Unit Descriptor - Microphone (UNIT ID 5 - Source 4) */
    0x08,           /* bLength */
    0x24,           /* bDescriptorType */
    0x06,           /* bDescriptorSubType */
    REC_FEATURE_UNITID, /* bUnitID */
    0x04,           /* bSourceID */
    0x01,           /* bControlSize - Size, in bytes, of the bmControls field: n */
    0x03,           /* bmaControls(0) */
                    /* A bit set to 1 indicates that the mentioned
                       Control is supported for master channel
                       0:
                       D0: Mute
                       D1: Volume
                       D2: Bass
                       D3: Mid
                       D4: Treble
                       D5: Graphic Equalizer
                       D6: Automatic Gain
                       D7: Delay
                       D8: Bass Boost
                       D9: Loudness
                       D10..(n*8-1): Reserved
                    */
    0x00,           /* iFeature */

    /* Audio Control Output Terminal Descriptor - Microphone (Terminal ID 2 - Source ID 5) */
    0x09,           /* bLength */
    0x24,           /* bDescriptorType:CS_INTERFACE */
    0x03,           /* bDescriptorSubType:OUTPUT_TERMINAL */
    0x02,           /* bTerminalID */
    0x01, 0x01,     /* wTerminalType: 0x0101 usb streaming */
    0x00,           /* bAssocTerminal */
    REC_FEATURE_UNITID, /* bSourceID */
    0x00,           /* iTerminal */

    /* Audio Control Input Terminal Descriptor - Speaker (Terminal ID 1) */
    0x0C,           /* bLength */
    0x24,           /* bDescriptorType:CS_INTERFACE */
    0x02,           /* bDescriptorSubType:INPUT_TERMINAL */
    0x01,           /* bTerminalID */
    0x01, 0x01,     /* wTerminalType: USB streaming */
    0x00,           /* bAssocTerminal */
    0x02,           /* bNrChannels : a number that specifies how many logical audio channels are present in the cluster */
    0x03, 0x00,     /* wChannelConfig: a bit field that indicates which spatial locations are present in the cluster.
                       The bit allocations are as follows:
                         D0: Left Front (L)
                         D1: Right Front (R)
                         D2: Center Front (C)
                         D3: Low Frequency Enhancement (LFE)
                         D4: Left Surround (LS)
                         D5: Right Surround (RS)
                         D6: Left of Center (LC)
                         D7: Right of Center (RC)
                         D8: Surround (S)
                         D9: Side Left (SL)
                         D10: Side Right (SR)
                         D11: Top (T)
                         D15..12: Reserved
                    */
    0x00,           /* iChannelNames */
    0x00,           /* iTerminal */

    /* Audio Control Feature Unit Descriptor - Speaker (UNIT ID 6 - Source ID 1) */
    0x0A,           /* bLength */
    0x24,           /* bDescriptorType */
    0x06,           /* bDescriptorSubType */
    PLAY_FEATURE_UNITID,/* bUnitID */
    0x01,           /* bSourceID */
    0x01,           /* bControlSize - Size, in bytes, of the bmControls field: n */
    0x01,           /* bmaControls(0) */
                    /* A bit set to 1 indicates that the mentioned
                       Control is supported for master channel
                       0:
                       D0: Mute
                       D1: Volume
                       D2: Bass
                       D3: Mid
                       D4: Treble
                       D5: Graphic Equalizer
                       D6: Automatic Gain
                       D7: Delay
                       D8: Bass Boost
                       D9: Loudness
                       D10..(n*8-1): Reserved
                    */
    0x02,           /* bmaControls(0) */
                    /* A bit set to 1 indicates that the mentioned
                       Control is supported for master channel
                       0:
                       D0: Mute
                       D1: Volume
                       D2: Bass
                       D3: Mid
                       D4: Treble
                       D5: Graphic Equalizer
                       D6: Automatic Gain
                       D7: Delay
                       D8: Bass Boost
                       D9: Loudness
                       D10..(n*8-1): Reserved
                    */
    0x02,           /* bmaControls(0) */
                    /* A bit set to 1 indicates that the mentioned
                       Control is supported for master channel
                       0:
                       D0: Mute
                       D1: Volume
                       D2: Bass
                       D3: Mid
                       D4: Treble
                       D5: Graphic Equalizer
                       D6: Automatic Gain
                       D7: Delay
                       D8: Bass Boost
                       D9: Loudness
                       D10..(n*8-1): Reserved
                    */
    0x00,           /* iFeature */

    /* Audio Control Output Terminal Descriptor - Speaker (UNIT ID 3 - Source ID 5) */
    0x09,           /* bLength*/
    0x24,           /* bDescriptorType:CS_INTERFACE*/
    0x03,           /* bDescriptorSubType:OUTPUT_TERMINAL*/
    0x03,           /* bTerminalID*/
    0x02, 0x04,     /* wTerminalType: Headset */
    0x00,           /* bAssocTerminal*/
    PLAY_FEATURE_UNITID,/* bSourceID*/
    0x00,           /* iTerminal*/

    /* Interface Descriptor - Interface 1, alternate 0 */
    0x09,           /* bLength */
    0x04,           /* bDescriptorType */
    0x01,           /* bInterfaceNumber */
    0x00,           /* bAlternateSetting */
    0x00,           /* bNumEndpoints */
    0x01,           /* bInterfaceClass:AUDIO */
    0x02,           /* bInterfaceSubClass:AUDIOSTREAMING */
    0x00,           /* bInterfaceProtocol */
    0x00,           /* iInterface */

    /* Interface Descriptor - Interface 1, alternate 1 */
    0x09,           /* bLength */
    0x04,           /* bDescriptorType */
    0x01,           /* bInterfaceNumber */
    0x01,           /* bAlternateSetting */
    0x01,           /* bNumEndpoints */
    0x01,           /* bInterfaceClass:AUDIO */
    0x02,           /* bInterfaceSubClass:AUDIOSTREAMING */
    0x00,           /* bInterfaceProtocol */
    0x00,           /* iInterface */

    /* Audio Streaming Class Specific Interface Descriptor (this interface's endpoint connect to Terminal ID 0x02 - Microphoe) */
    0x07,           /* bLength */
    0x24,           /* bDescriptorType:CS_INTERFACE */
    0x01,           /* bDescriptorSubType:AS_GENERAL */
    0x02,           /* bTernimalLink */
    0x01,           /* bDelay */
    0x01, 0x00,     /* wFormatTag:0x0001 PCM */

    /* Audio Streaming Format Type Descriptor */
    0x0B,           /* bLength */
    0x24,           /* bDescriptorType:CS_INTERFACE */
    0x02,           /* bDescriptorSubType:FORMAT_TYPE */
    0x01,           /* bFormatType:FORMAT_TYPE_I */
    /* Standard AS interface 1, alternate 1 */
    0x02,           /* bNrChannels    :  2 Channels */
    0x02,           /* bSubFrameSize  :  2 bytes per sample */
    0x10,           /* bBitResolution : 16 bits  per sample */
    0x01,           /* bSamFreqType :
                       0 Continuous sampling frequency
                       1 The number of discrete sampling frequencies */
    /* bSamFreqType  */
    (AUDIO_RATE_48K & 0xFF),
    ((AUDIO_RATE_48K >> 8) & 0xFF),
    ((AUDIO_RATE_48K >> 16) & 0xFF),

    /* Endpoint Descriptor (ISO IN Audio Data Endpoint - alternate 1) */
    0x07,                             /* bLength */
    0x05,                             /* bDescriptorType */
    ISO_IN_EP_NUM | EP_INPUT,         /* bEndpointAddress */
    0x05,                             /* bmAttributes */
    /* wMaxPacketSize note */
    EP2_MAX_PKT_SIZE & 0x00FF,
    ((EP2_MAX_PKT_SIZE & 0xFF00) >> 8),
    0x01,                             /* bInterval*/

    /* Audio Streaming Class Specific Audio Data Endpoint Descriptor */
    0x07,           /* bLength */
    0x25,           /* bDescriptorType:CS_ENDPOINT */
    0x01,           /* bDescriptorSubType:EP_GENERAL */
    0x01,           /* bmAttributes, Bit 0: Sampling Frequency */
    0x00,           /* bLockDelayUnits */
    0x00, 0x00,     /* wLockDelay */

    /* Interface Descriptor - Interface 2, alternate 0
        Interface 2 for Speaker & Microphone
    */
    0x09,           /* bLength */
    0x04,           /* bDescriptorType */
    0x02,           /* bInterfaceNumber */
    0x00,           /* bAlternateSetting */
    0x00,           /* bNumEndpoints */
    0x01,           /* bInterfaceClass:AUDIO */
    0x02,           /* bInterfaceSubClass:AUDIOSTREAMING */
    0x00,           /* bInterfaceProtocol */
    0x00,           /* iInterface */

    /* Interface Descriptor - Interface 2, alternate 1
        Interface 2 for Speaker & Microphone
    */
    0x09,           /* bLength */
    0x04,           /* bDescriptorType */
    0x02,           /* bInterfaceNumber */
    0x01,           /* bAlternateSetting */
    0x01,           /* bNumEndpoints */
    0x01,           /* bInterfaceClass:AUDIO */
    0x02,           /* bInterfaceSubClass:AUDIOSTREAMING */
    0x00,           /* bInterfaceProtocol */
    0x00,           /* iInterface */

    /* Audio Streaming Class Specific Interface Descriptor (this interface's endpoint connect to Terminal ID 0x01 - Speaker) */
    0x07,           /* bLength */
    0x24,           /* bDescriptorType:CS_INTERFACE */
    0x01,           /* bDescriptorSubType:AS_GENERAL */
    0x01,           /* bTernimalLink (Speaker) */
    0x01,           /* bDelay */
    0x01, 0x00,     /* wFormatTag:0x0001 PCM */

    /* Audio Streaming Format Type Descriptor */
    0x0B,           /* bLength */
    0x24,           /* bDescriptorType:CS_INTERFACE */
    0x02,           /* bDescriptorSubType:FORMAT_TYPE */
    0x01,           /* bFormatType:FORMAT_TYPE_I */
    /* Standard AS interface 2, alternate 1 */
    0x02,           /* bNrChannels    :  2 Channels */
    0x02,           /* bSubFrameSize  :  2 bytes per sample */
    0x10,           /* bBitResolution : 16 bits  per sample */
    0x01,           /* bSamFreqType :
                       0 Continuous sampling frequency
                       1 The number of discrete sampling frequencies */
    /* bSamFreqType  */
    (AUDIO_RATE_48K & 0xFF),
    ((AUDIO_RATE_48K >> 8) & 0xFF),
    ((AUDIO_RATE_48K >> 16) & 0xFF),

    /* Endpoint Descriptor (ISO OUT Audio Data Endpoint - alternate 1) */
    0x09,                             /* bLength */
    0x05,                             /* bDescriptorType */
    ISO_OUT_EP_NUM | EP_OUTPUT,       /* bEndpointAddress */
    0x09,                             /* bmAttributes */
    /* wMaxPacketSize note */
    EP3_MAX_PKT_SIZE & 0x00FF,
    ((EP3_MAX_PKT_SIZE & 0xFF00) >> 8),
    0x01,                             /* bInterval */
    0x00,                             /* bRefresh */
    0x00,                             /* bSynchAddress */

    /* Audio Streaming Class Specific Audio Data Endpoint Descriptor */
    0x07,           /* bLength */
    0x25,           /* bDescriptorType:CS_ENDPOINT */
    0x01,           /* bDescriptorSubType:EP_GENERAL */
    0x01,           /* bmAttributes, Bit 7: MaxPacketsOnly, Bit 0: Sampling Frequency */
    0x01,           /* bLockDelayUnits */
    0x01, 0x00,     /* wLockDelay */
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
    20,
    DESC_STRING,
    'U', 0, 'S', 0, 'B', 0, ' ', 0, 'A', 0, 'u', 0, 'd', 0, 'i', 0, 'o', 0
};

#ifdef SUPPORT_LPM
/*!<USB BOS Descriptor */
static uint8_t s_au8BOSDescriptor[] =
{
    LEN_BOS,        /* bLength */
    DESC_BOS,       /* bDescriptorType */
    /* wTotalLength */
    0x0C & 0x00FF,
    (0x0C & 0xFF00) >> 8,
    0x01,           /* bNumDeviceCaps */

    /* Device Capability */
    LEN_BOSCAP,     /* bLength */
    DESC_CAPABILITY,/* bDescriptorType */
    CAP_USB20_EXT,  /* bDevCapabilityType, 0x02 is USB 2.0 Extension */
    0x06, 0x04, 0x00, 0x00  /* bmAttributes, 32 bits */
                            /* bit 0 : Reserved. Must 0. */
                            /* bit 1 : 1 to support LPM. */
                            /* bit 2 : 1 to support BSL & Alternat HIRD. */
                            /* bit 3 : 1 to recommend Baseline BESL. */
                            /* bit 4 : 1 to recommand Deep BESL. */
                            /* bit 11:8 : Recommend Baseline BESL value. Ignore by bit3 is zero. */
                            /* bit 15:12 : Recommend Deep BESL value. Ignore by bit4 is zero. */
                            /* bit 31:16 : Reserved. Must 0. */
};
#endif

static uint8_t *s_apu8UsbString[4] =
{
    s_au8StringLang,
    s_au8VendorStringDesc,
    s_au8ProductStringDesc,
    NULL,
};

const S_USBD_INFO_T gsInfo =
{
    (uint8_t *)s_au8DeviceDescriptor,
    (uint8_t *)s_au8ConfigDescriptor,
    (uint8_t **)s_apu8UsbString,
    NULL,
#ifdef SUPPORT_LPM
    (uint8_t *)s_au8BOSDescriptor,
#else
    NULL,
#endif
    NULL,
    NULL
};



