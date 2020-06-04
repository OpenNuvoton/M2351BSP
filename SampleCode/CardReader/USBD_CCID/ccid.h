/******************************************************************************
 * @file     ccid.h
 * @brief    CCID driver header file
 * @version  V1.00
 *
 * @note
 * Copyright (C) 2014~2015 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __USBD_CCID_H__
#define __USBD_CCID_H__

/* Define the vendor id and product id */
#define USBD_VID        0x0416
#define USBD_PID        0xC146

/*!<Define CCID Class Specific Request */
/*!<Define CCID Class Specific Request */
#define     CCID_ABORT                              0x01
#define     CCID_GET_CLOCK_FREQUENCIES              0x02
#define     CCID_GET_DATA_RATES                     0x03

/*!<Define CCID Bulk-Out Message */
#define     PC_TO_RDR_ICCPOWERON                    0x62
#define     PC_TO_RDR_ICCPOWEROFF                   0x63
#define     PC_TO_RDR_GETSLOTSTATUS                 0x65
#define     PC_TO_RDR_XFRBLOCK                      0x6F
#define     PC_TO_RDR_GETPARAMETERS                 0x6C
#define     PC_TO_RDR_RESETPARAMETERS               0x6D
#define     PC_TO_RDR_SETPARAMETERS                 0x61
#define     PC_TO_RDR_ESCAPE                        0x6B
#define     PC_TO_RDR_ICCCLOCK                      0x6E
#define     PC_TO_RDR_T0APDU                        0x6A
#define     PC_TO_RDR_SECURE                        0x69
#define     PC_TO_RDR_MECHANICAL                    0x71
#define     PC_TO_RDR_ABORT                         0x72
#define     PC_TO_RDR_SETDATARATEANDCLOCKFREQUENCY  0x73

/*!<Define CCID Bulk-In Message */
#define     RDR_TO_PC_DATABLOCK                     0x80
#define     RDR_TO_PC_SLOTSTATUS                    0x81
#define     RDR_TO_PC_PARAMETERS                    0x82
#define     RDR_TO_PC_ESCAPE                        0x83
#define     RDR_TO_PC_DATARATEANDCLOCKFREQUENCY     0x84

/*!<Define CCID Interrupt-In Message */
#define     RDR_TO_PC_NOTIFYSLOTCHANGE              0x50
#define     RDR_TO_PC_HARDWAREERROR                 0x51

/*======================================================*/
/*      Variables and Definition of authorized values   */
/*======================================================*/
#define USB_MESSAGE_BUFFER_MAX_LENGTH       0x010F
#define ICC_MESSAGE_BUFFER_MAX_LENGTH       0x0105
#define USB_MESSAGE_HEADER_SIZE             10

/*!<Define Offsets in UsbMessageBuffer for Bulk Out messages */
#define OFFSET_BMESSAGETYPE             0x00
#define OFFSET_DWLENGTH                 0x01
#define OFFSET_BSLOT                    0x05
#define OFFSET_BSEQ                     0x06
#define OFFSET_BPOWERSELECT             0x07
#define OFFSET_BBWI                     0x07
#define OFFSET_BPROTOCOLNUM_OUT         0x07
#define OFFSET_BCLOCKCOMMAND            0x07
#define OFFSET_BMCHANGES                0x07
#define OFFSET_BFUNCTION                0x07
#define OFFSET_ABRFU_3B                 0x07
#define OFFSET_WLEVELPARAMETER          0x08
#define OFFSET_BCLASSGETRESPONSE        0x08
#define OFFSET_ABRFU_2B                 0x08
#define OFFSET_BCLASSENVELOPE           0x09
#define OFFSET_ABDATA                   0x0A

/*!<Define Offsets in UsbMessageBuffer for Bulk In messages */
#define OFFSET_BSTATUS                  0x07
#define OFFSET_BERROR                   0x08
#define OFFSET_BCHAINPARAMETER          0x09
#define OFFSET_BCLOCKSTATUS             0x09
#define OFFSET_BPROTOCOLNUM_IN          0x09
#define OFFSET_ABPROTOCOLDATASTRUCTURE  0x0A
#define OFFSET_BRFU                     0x09

/*!<Define Offsets in UsbMessageBuffer for Int In messages */
#define OFFSET_INT_BMESSAGETYPE         0x00
#define OFFSET_INT_BMSLOTICCSTATE       0x01
#define OFFSET_INT_BSLOT                0x01
#define OFFSET_INT_BSEQ                 0x02
#define OFFSET_INT_BHARDWAREERRORCODE   0x03

/*-------------------------------------------------------------*/
/* Define EP maximum packet size */
#define EP0_MAX_PKT_SIZE    16
#define EP1_MAX_PKT_SIZE    EP0_MAX_PKT_SIZE
#define EP2_MAX_PKT_SIZE    64
#define EP3_MAX_PKT_SIZE    64
#define EP4_MAX_PKT_SIZE    16

#define SETUP_BUF_BASE      0
#define SETUP_BUF_LEN       8
#define EP0_BUF_BASE        (SETUP_BUF_BASE + SETUP_BUF_LEN)
#define EP0_BUF_LEN         EP0_MAX_PKT_SIZE
#define EP1_BUF_BASE        (SETUP_BUF_BASE + SETUP_BUF_LEN)
#define EP1_BUF_LEN         EP1_MAX_PKT_SIZE
#define EP2_BUF_BASE        (EP1_BUF_BASE + EP1_BUF_LEN)
#define EP2_BUF_LEN         EP2_MAX_PKT_SIZE
#define EP3_BUF_BASE        (EP2_BUF_BASE + EP2_BUF_LEN)
#define EP3_BUF_LEN         EP3_MAX_PKT_SIZE
#define EP4_BUF_BASE        (EP3_BUF_BASE + EP3_BUF_LEN)
#define EP4_BUF_LEN         EP4_MAX_PKT_SIZE

/* Define the interrupt In EP number */
#define BULK_IN_EP_NUM      0x02
#define BULK_OUT_EP_NUM     0x02
#define INT_IN_EP_NUM       0x03


/* Define Descriptor information */
#define USBD_SELF_POWERED               0
#define USBD_REMOTE_WAKEUP              0
#define USBD_MAX_POWER                  50  /* The unit is in 2mA. ex: 50 * 2mA = 100mA */

/*-------------------------------------------------------------*/
__INLINE static uint32_t make32(uint8_t *buf)
{
    return ((uint32_t) buf[3] << 24) | ((uint32_t) buf[2] << 16) |
           ((uint32_t) buf[1] << 8) | ((uint32_t) buf[0]);
}

__INLINE static uint16_t make16(uint8_t *buf)
{
    return (uint16_t)(((uint16_t) buf[1] << 8) | ((uint16_t) buf[0]));
}
/*-------------------------------------------------------------*/
extern uint8_t UsbIntMessageBuffer[];
extern uint8_t UsbMessageBuffer[];

extern uint8_t volatile gu8IsDeviceReady;
extern uint8_t volatile gu8AbortRequestFlag;
extern uint8_t volatile gu8IsBulkOutReady;
extern uint8_t volatile gu8IsBulkInReady;

extern uint8_t *pu8IntInBuf;
extern uint8_t *pUsbMessageBuffer;
extern uint32_t volatile u32BulkSize;

extern int32_t volatile gi32UsbdMessageLength;


/*-------------------------------------------------------------*/
void CCID_Init(void);
void CCID_ClassRequest(void);

void EP2_Handler(void);
void EP3_Handler(void);
void CCID_TransferData(void);
void CCID_DispatchMessage(void);
void CCID_BulkInMessage(void);

#endif  /* __USBD_CCID_H_ */

/*** (C) COPYRIGHT 2014~2015 Nuvoton Technology Corp. ***/
