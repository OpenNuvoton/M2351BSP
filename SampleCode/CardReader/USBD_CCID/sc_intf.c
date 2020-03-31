/******************************************************************************
 * @file     sc_intf.c
 * @version  V2.00
 * @brief    M2351 USBD CCID smartcard interface control file
 * @note
 * Copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include "NuMicro.h"
#include "sc_intf.h"
#include "ccid.h"
#include "ccid_if.h"
#include "sclib.h"
#include <string.h>
#include <stdio.h>

//#define CCID_SC_DEBUG
#ifdef CCID_SC_DEBUG
#define CCIDSCDEBUG     printf
#else
#define CCIDSCDEBUG(...)
#endif

#define MIN_BUFFER_SIZE             271
// for T0 case 4 APDU
static uint8_t s_au8RBuf[MIN_BUFFER_SIZE];
static uint8_t s_au8RBufICC[MIN_BUFFER_SIZE];
static uint32_t s_u32RLen, s_u32RLenICC;
/* EMV for T=1 */
static uint8_t s_au8IfsReqFlag[SC_INTERFACE_NUM] = {0};

extern uint8_t UsbMessageBuffer[];
static volatile uint8_t IccTransactionType[SC_INTERFACE_NUM];


/*
 * 00h: response APDU begins and ends in this command
 * 01h: response APDU begins with this command and is to continue
 * 02h: abData field continues the response APDU and ends the response APDU
 * 03h: abData field continues the response APDU and another block is to follow
 * 10h: empty abDat field, continuation of the command APDU is expected in next PC_to_RDR_XfrBlock command
 */
uint8_t s_u8ChainParameter = 0x00;

typedef struct
{
    uint8_t FiDi;
    uint8_t Tcckst;
    uint8_t GuardTime;
    uint8_t WaitingInteger;
    uint8_t ClockStop;
    uint8_t Ifsc;                /* For protocol T=1 */
    uint8_t Nad;                 /* For protocol T=1 */
} Param;

static Param IccParameters[SC_INTERFACE_NUM];


/*---------------------------------------------------------------------------------------------------------*/
/* Clock rate conversion table according to ISO structure                                                  */
/*---------------------------------------------------------------------------------------------------------*/
typedef struct
{
    const unsigned long F;
    const unsigned long fs;

} S_SC_CLOCK_RATE_CONVERSION;

//
// Bit rate adjustment factor
// The layout of this table has been slightly modified due to
// the unavailibility of floating point math support in the kernel.
// The value D has beed devided into a numerator and a divisor.
//
typedef struct
{
    const unsigned long DNumerator;
    const unsigned long DDivisor;
} S_SC_BIT_RATE_ADJUSTMENT;

//
// The clock rate conversion table itself.
// All R(eserved)F(or Future)U(se) fields MUST be 0
//
static S_SC_CLOCK_RATE_CONVERSION ClockRateConversion[] =
{

    { 372,  4000000     },
    { 372,  5000000     },
    { 558,  6000000     },
    { 744,  8000000     },
    { 1116, 12000000    },
    { 1488, 16000000    },
    { 1860, 20000000    },
    { 0,    0            },
    { 0,    0            },
    { 512,  5000000     },
    { 768,  7500000     },
    { 1024, 10000000    },
    { 1536, 15000000    },
    { 2048, 20000000    },
    { 0,    0            },
    { 0,    0            }
};

//
// The bit rate adjustment table itself.
// All R(eserved)F(or)U(se) fields MUST be 0
//
static S_SC_BIT_RATE_ADJUSTMENT BitRateAdjustment[] =
{

    { 0,    0   },
    { 1,    1   },
    { 2,    1   },
    { 4,    1   },
    { 8,    1   },
    { 16,   1   },
    { 32,   1   },
    { 64,   1   },
    { 12,   1   },
    { 20,   1   },
    { 0,    0   },
    { 0,    0   },
    { 0,    0   },
    { 0,    0   },
    { 0,    0   },
    { 0,    0   }
};

uint8_t Intf_SC2CCIDErrorCode(int32_t u32Err);

/**
  * @brief  Transfer SC library status to CCID error code
  * @param  u32Err SC library's error code
  * @return Slot status error code
  */
uint8_t Intf_SC2CCIDErrorCode(int32_t u32Err)
{
    if(u32Err == SCLIB_ERR_TIME0OUT)
        return SLOTERR_ICC_MUTE;

    else if(u32Err == SCLIB_ERR_TIME2OUT)
        return SLOTERR_ICC_MUTE;

    else if(u32Err == SCLIB_ERR_AUTOCONVENTION)
        return SLOTERR_BAD_ATR_TS;

    else if(u32Err == SCLIB_ERR_ATR_INVALID_TCK)
        return SLOTERR_BAD_ATR_TCK;

    else if(u32Err == SCLIB_ERR_READ)
        return SLOTERR_XFR_OVERRUN;

    else if(u32Err == SCLIB_ERR_WRITE)
        return SLOTERR_HW_ERROR;

    else if(u32Err == SCLIB_ERR_T1_PARITY)
        return SLOTERR_XFR_PARITY_ERROR;

    else if(u32Err == SCLIB_ERR_PARITY_ERROR)
        return SLOTERR_XFR_PARITY_ERROR;

    else if(u32Err == SCLIB_ERR_CARD_REMOVED)
        return SLOTERR_ICC_MUTE;

    else if(u32Err == SCLIB_ERR_CARDBUSY)
        return SLOTERR_CMD_SLOT_BUSY;

    else if(u32Err == SCLIB_ERR_ATR_INVALID_PARAM)
        return SLOTERR_ICC_PROTOCOL_NOT_SUPPORTED;

    else if(u32Err == SCLIB_ERR_T0_PROTOCOL)
        return SLOTERR_ICC_PROTOCOL_NOT_SUPPORTED;

    else if(u32Err == SCLIB_ERR_T1_PROTOCOL)
        return SLOTERR_ICC_PROTOCOL_NOT_SUPPORTED;

    else if(u32Err == SCLIB_ERR_T1_ABORT_RECEIVED)
        return SLOTERR_CMD_ABORTED;

    else
        return SLOT_NO_ERROR;
}


/**
  * @brief  Set the default value applying to CCID protcol data structure
  * @param  intf Indicate which interface to open, ether 0 or 1
  * @return Slot status error code
  */
uint8_t Intf_Init(int32_t intf)
{
    if(intf != 0 && intf != 1 && intf != 2)
        return SLOTERR_BAD_SLOT;

    IccTransactionType[intf] = SCLIB_PROTOCOL_T0;

    // Not activate yet, give a dummy value. GetParameter will check real values from SCLIB
    IccParameters[intf].FiDi = DEFAULT_FIDI;
    IccParameters[intf].Tcckst = DEFAULT_T01CONVCHECKSUM;
    IccParameters[intf].GuardTime = DEFAULT_GUARDTIME;
    IccParameters[intf].WaitingInteger = DEFAULT_WAITINGINTEGER;
    IccParameters[intf].ClockStop = DEFAULT_CLOCKSTOP;
    IccParameters[intf].Ifsc = DEFAULT_IFSC;
    IccParameters[intf].Nad = DEFAULT_NAD;

    return SLOT_NO_ERROR;
}



/**
  * @brief  After got ATR and take ATR information to apply CCID protocol data structure
  * @param  intf Indicate which interface to open, ether 0 or 1
  * @return Slot status error code
  */
uint8_t Intf_ApplyParametersStructure(int32_t intf)
{
    SCLIB_CARD_INFO_T info;

    if(intf != 0 && intf != 1 && intf != 2)
        return SLOTERR_BAD_SLOT;

    if(SCLIB_GetCardInfo((uint32_t)intf, &info) != SCLIB_SUCCESS)
        return SLOTERR_ICC_MUTE;

    return SLOT_NO_ERROR;

#if 0
    SCLIB_CARD_INFO_T info;
    SCLIB_CARD_ATTRIB_T attrib;

    if(intf != 0 && intf != 1)
        return SLOTERR_BAD_SLOT;

    if(SCLIB_GetCardAttrib(intf, &attrib) != SCLIB_SUCCESS)
        return SLOTERR_ICC_MUTE;

    if(SCLIB_GetCardInfo(intf, &info) != SCLIB_SUCCESS)
        return SLOTERR_ICC_MUTE;

    IccParameters[intf].FiDi = (attrib.Fi << 4) | attrib.Di;
    IccParameters[intf].ClockStop = attrib.clkStop;

    if(info.T == SCLIB_PROTOCOL_T0)
    {
        IccTransactionType[intf] = SCLIB_PROTOCOL_T0;
        /* TCCKST */
        IccParameters[intf].Tcckst = attrib.conv ? 0x02 : 0x00;

        /* GuardTime */
        IccParameters[intf].GuardTime = attrib.GT - 12;
        /* WaitingInteger */
        IccParameters[intf].WaitingInteger = attrib.WI;
    }
    else if(info.T == SCLIB_PROTOCOL_T1)
    {
        IccTransactionType[intf] = SCLIB_PROTOCOL_T1;
        /* TCCKST */
        IccParameters[intf].Tcckst = 0x10;
        if(attrib.conv)
            IccParameters[intf].Tcckst |= 0x02;
        if(attrib.chksum)        // EDC = CRC scheme
            IccParameters[intf].Tcckst |= 0x01;
        /* GuardTime */
        if(attrib.GT == 11)
            IccParameters[intf].GuardTime = 0xFF;
        else
            IccParameters[intf].GuardTime = attrib.GT - 12;
        /* WaitingInteger */
        IccParameters[intf].WaitingInteger = (attrib.BWI << 4) | attrib.CWI;
        /* IFSC */
        IccParameters[intf].Ifsc = attrib.IFSC;
    }

    return SLOT_NO_ERROR;
#endif
}


/**
  * @brief  Check if hardware is busy or in other error condition
  * @param  intf Indicate which interface to open, ether 0 or 1
  * @return Slot status error code
  */
uint8_t Intf_GetHwError(int32_t intf)
{
    uint8_t ErrorCode;
    SCLIB_CARD_INFO_T info;

    if(intf != 0 && intf != 1 && intf != 2)
        return SLOTERR_BAD_SLOT;

    ErrorCode = SLOT_NO_ERROR;

    if(SCLIB_GetCardInfo((uint32_t)intf, &info) != SCLIB_SUCCESS)
    {
        ErrorCode = SLOTERR_ICC_MUTE;
        SCLIB_Deactivate((uint32_t)intf); // can remove....
    }

    return ErrorCode;
}


/**
  * @brief  Do cold-reset or warm-reset and return ATR information
  * @param  intf Indicate which interface to open, ether 0 or 1
  * @param  uint32_t Voltage class (AUTO, Class A, B, C)
  * @param  pu8AtrBuf Pointer uses to fill ATR information
  * @param  pu32AtrSize The length of ATR information
  * @return Slot status error code
  */
uint8_t Intf_IccPowerOn(int32_t intf,
                        uint32_t u32Volt,
                        uint8_t *pu8AtrBuf,
                        uint32_t *pu32AtrSize)
{
    int32_t ErrorCode;
    SCLIB_CARD_INFO_T info;
    SC_T *pSC;

    if(intf != 0 && intf != 1 && intf != 2)
        return SLOTERR_BAD_SLOT;

    ErrorCode = Intf_Init(intf);
    //Intf_ApplyParametersStructure();
    if(ErrorCode != SLOT_NO_ERROR)
        return (uint8_t)ErrorCode;

    if((__PC()&NS_OFFSET) == NS_OFFSET)
        pSC = (intf == 0) ? SC0_NS : (intf == 1) ? SC1_NS : SC2_NS;
    else
        pSC = (intf == 0) ? SC0 : (intf == 1) ? SC1 : SC2;

    SC_ResetReader(pSC);

    if(u32Volt == OPERATION_CLASS_AUTO)
    {
        if(SC_IsCardInserted(pSC) == TRUE)
        {
            //WRITE ME: Set interface voltage to class C
            ErrorCode = SCLIB_ColdReset((uint32_t)intf);
            if(ErrorCode != SCLIB_SUCCESS)
            {
                SCLIB_Deactivate((uint32_t)intf);
                //WRITE ME: Set interface voltage to class B
                ErrorCode = SCLIB_ColdReset((uint32_t)intf);
                if(ErrorCode != SCLIB_SUCCESS)
                {
                    SCLIB_Deactivate((uint32_t)intf);
                    //WRITE ME: Set interface voltage to class A
                    ErrorCode = SCLIB_ColdReset((uint32_t)intf);
                }
            }
        }
        else      // card removed
        {
            ErrorCode = SCLIB_ERR_CARD_REMOVED;
        }
    }
    // assign voltage
    else if((u32Volt == OPERATION_CLASS_C) || (u32Volt == OPERATION_CLASS_B) || (u32Volt == OPERATION_CLASS_A))
    {
        if(SC_IsCardInserted(pSC) == TRUE)     // Do cold-reset
        {
            //WRITE ME: Set interface voltage
            ErrorCode = SCLIB_ColdReset((uint32_t)intf);
        }
        else
        {
            ErrorCode = SCLIB_ERR_CARD_REMOVED;
        }
    }
    else
    {
        if(SC_IsCardInserted(pSC) == TRUE)
        {
            ErrorCode = SCLIB_ColdReset((uint32_t)intf);
        }
        else
        {
            ErrorCode = SCLIB_ERR_CARD_REMOVED;
        }
    }

    if(ErrorCode == SCLIB_ERR_ATR_INVALID_PARAM)
        ErrorCode = SCLIB_WarmReset((uint32_t)intf);

    if(ErrorCode != SCLIB_SUCCESS)
    {
        SCLIB_Deactivate((uint32_t)intf);
        return (uint8_t)ErrorCode;
    }

    // Get the ATR information
    if(SCLIB_GetCardInfo((uint32_t)intf, &info) != SCLIB_SUCCESS)
    {
        SCLIB_Deactivate((uint32_t)intf);
        return (uint8_t)ErrorCode;
    }

    *pu32AtrSize = info.ATR_Len;
    memcpy(pu8AtrBuf, &info.ATR_Buf, info.ATR_Len);

    ErrorCode = Intf_ApplyParametersStructure(intf);

    if(ErrorCode != SLOT_NO_ERROR)
        return (uint8_t)ErrorCode;

    s_au8IfsReqFlag[intf] = 1;

    return SLOT_NO_ERROR;
}



/**
  * @brief  According to hardware and ICC, those conditions decide the transmission protocol.
  * @param  intf Indicate which interface to open, ether 0 or 1
  * @param  pu8CmdBuf Command Data
  * @param  pu32CmdSize The size of command data
  * @return Slot status error code
  */
uint8_t Intf_XfrBlock(int32_t intf,
                      uint8_t *pu8CmdBuf,
                      uint32_t *pu32CmdSize)
{
    uint8_t ErrorCode = SLOT_NO_ERROR;

    s_u8ChainParameter = 0x00;

    if(UsbMessageBuffer[OFFSET_WLEVELPARAMETER] == 0)                 // Short APDU
    {
        if(IccTransactionType[intf] == SCLIB_PROTOCOL_T0)
            ErrorCode = Intf_XfrShortApduT0(intf, pu8CmdBuf, pu32CmdSize);
        else if(IccTransactionType[intf] == SCLIB_PROTOCOL_T1)
            ErrorCode = Intf_XfrShortApduT1(intf, pu8CmdBuf, pu32CmdSize);
    }

    if(ErrorCode != SLOT_NO_ERROR)
        return ErrorCode;

    return ErrorCode;
}




/**
  * @brief  Transmission by T=0 Short APDU Mode
  * @param  intf Indicate which interface to open, ether 0 or 1
  * @param  pu8CmdBuf Command Data
  * @param  pu32CmdSize The size of command data
  * @return Slot status error code
  */
uint8_t Intf_XfrShortApduT0(int32_t intf,
                            uint8_t *pu8CmdBuf,
                            uint32_t *pu32CmdSize)
{

    int32_t ErrorCode;

    uint8_t *buf = s_au8RBuf;
    uint32_t idx;
    uint8_t blockbuf[5];

    CCIDSCDEBUG("Intf_XfrShortApduT0: header=%02x %02x %02x %02x, len=%d\n",
                pBlockBuffer[0], pBlockBuffer[1], pBlockBuffer[2], pBlockBuffer[3], *pBlockSize);

    if(*pu32CmdSize == 0x4)
    {
        blockbuf[0] = pu8CmdBuf[0];
        blockbuf[1] = pu8CmdBuf[1];
        blockbuf[2] = pu8CmdBuf[2];
        blockbuf[3] = pu8CmdBuf[3];
        blockbuf[4] = 0x00;

        ErrorCode = SCLIB_StartTransmission((uint32_t)intf, &blockbuf[0], 0x5, &s_au8RBuf[0], &s_u32RLen);
        if(ErrorCode != SCLIB_SUCCESS)
            return Intf_SC2CCIDErrorCode(ErrorCode);

        // received data
        for(idx = 0; idx < s_u32RLen; idx++)
        {
            *pu8CmdBuf = s_au8RBuf[idx];
            pu8CmdBuf++;
        }
        // length of received data
        *pu32CmdSize = s_u32RLen;
    }
    else
    {
        ErrorCode = SCLIB_StartTransmission((uint32_t)intf, pu8CmdBuf, *pu32CmdSize, &s_au8RBuf[0], &s_u32RLen);
        if(ErrorCode != SCLIB_SUCCESS)
            return Intf_SC2CCIDErrorCode(ErrorCode);

        // check if wrong Le field error
        while((s_u32RLen == 2) && (s_au8RBuf[0] == 0x6C))
        {
            pu8CmdBuf[4] = s_au8RBuf[1];
            ErrorCode = SCLIB_StartTransmission((uint32_t)intf, pu8CmdBuf, *pu32CmdSize, &s_au8RBuf[0], &s_u32RLen);
            if(ErrorCode != SCLIB_SUCCESS)
                return Intf_SC2CCIDErrorCode(ErrorCode);
        }

        // check if data bytes still available
        if((s_u32RLen == 2) && (s_au8RBuf[0] == 0x61))
        {
            s_u32RLenICC = 0;
            while(1)
            {
                blockbuf[0] = pu8CmdBuf[0];    // Echo original class code
                blockbuf[1] = 0xC0;               // 0xC0 == Get response command
                blockbuf[2] = 0x00;               // 0x00
                blockbuf[3] = 0x00;               // 0x00
                blockbuf[4] = s_au8RBuf[s_u32RLen - 1];     // Licc = how many data bytes still available

                ErrorCode = SCLIB_StartTransmission((uint32_t)intf, blockbuf, 0x5, &s_au8RBuf[0], &s_u32RLen);
                if(ErrorCode != SCLIB_SUCCESS)
                    return Intf_SC2CCIDErrorCode(ErrorCode);
                // received data
                if(s_au8RBuf[s_u32RLen - 2] == 0x61)
                {
                    for(idx = 0; idx < s_u32RLen - 2; idx++)
                        s_au8RBufICC[s_u32RLenICC++] = s_au8RBuf[idx];
                }
                else
                {
                    for(idx = 0; idx < s_u32RLen; idx++)
                        s_au8RBufICC[s_u32RLenICC++] = s_au8RBuf[idx];
                }

                if(s_au8RBuf[s_u32RLen - 2] != 0x61) break;
            }
            s_u32RLen = s_u32RLenICC;
            buf = s_au8RBufICC;
        }


        CCIDSCDEBUG("Intf_XfrShortApduT0: dwLength = %d, Data = ", s_u32RLen);

        /* Check status bytes */
        if((buf[s_u32RLen - 2] & 0xF0) != 0x60 && (buf[s_u32RLen - 2] & 0xF0) != 0x90)
            return SLOTERR_ICC_PROTOCOL_NOT_SUPPORTED;

        // received data
        for(idx = 0; idx < s_u32RLen; idx++)
        {
            CCIDSCDEBUG("%02x", buf[idx]);
            *pu8CmdBuf = buf[idx];
            pu8CmdBuf++;
        }
        CCIDSCDEBUG("\n");

        // length of received data
        *pu32CmdSize = s_u32RLen;
    }

    return SLOT_NO_ERROR;

}


/**
  * @brief  Transmission by T=1 Short APDU Mode
  * @param  intf Indicate which interface to open, ether 0 or 1
  * @param  pu8CmdBuf Command Data
  * @param  pu32CmdSize The size of command data
  * @return Slot status error code
  */
uint8_t Intf_XfrShortApduT1(int32_t intf,
                            uint8_t *pu8CmdBuf,
                            uint32_t *pu32CmdSize)
{
    int32_t ErrorCode;
    uint32_t idx;

    CCIDSCDEBUG("Intf_XfrShortApduT1: header=%02x %02x %02x %02x, len=%d\n",
                pBlockBuffer[0], pBlockBuffer[1], pBlockBuffer[2], pBlockBuffer[3], *pBlockSize);

    /* IFS request only for EMV T=1 */
    /* First block (S-block IFS request) transmits after ATR */
    if(s_au8IfsReqFlag[intf] == 1)
    {
        s_au8IfsReqFlag[intf] = 0;
        ErrorCode = SCLIB_SetIFSD((uint32_t)intf, 0xFE);

        if(ErrorCode != SCLIB_SUCCESS)
            return Intf_SC2CCIDErrorCode(ErrorCode);
    }



    // Sending procedure
    ErrorCode = SCLIB_StartTransmission((uint32_t)intf, pu8CmdBuf, *pu32CmdSize, &s_au8RBuf[0], &s_u32RLen);
    if(ErrorCode != SCLIB_SUCCESS)
        return Intf_SC2CCIDErrorCode(ErrorCode);

    CCIDSCDEBUG("Intf_XfrShortApduT1: dwLength = %d, Data = ", s_u32RLen);

    // received data
    for(idx = 0; idx < s_u32RLen; idx++)
    {
        //CCIDSCDEBUG("Received Data: Data[%d]=0x%02x \n", idx, s_au8RBuf[idx]);
        CCIDSCDEBUG("%02x", s_au8RBuf[idx]);
        *pu8CmdBuf = s_au8RBuf[idx];
        pu8CmdBuf++;
    }
    CCIDSCDEBUG("\n");

    // length of received data
    *pu32CmdSize = s_u32RLen;

    return SLOT_NO_ERROR;

}



/**
  * @brief  Give slot's protocol data structure
  * @param  intf Indicate which interface to open, ether 0 or 1
  * @param  pu8Buf  Fill the protocol data structure continuously
  * @return 0x00 for T=0, 0x01 for T=1
  */
uint8_t Intf_GetParameters(int32_t intf, uint8_t *pu8Buf)
{

    CCIDSCDEBUG("\n**************Intf_GetParameters***************\n");
    CCIDSCDEBUG("FiDi = 0x%x \n", IccParameters[intf].FiDi);
    CCIDSCDEBUG("Tcckst = 0x%x \n", IccParameters[intf].Tcckst);
    CCIDSCDEBUG("GuardTime = 0x%x \n", IccParameters[intf].GuardTime);
    CCIDSCDEBUG("WaitingInteger = 0x%x \n", IccParameters[intf].WaitingInteger);
    CCIDSCDEBUG("ClockStop = 0x%x \n", IccParameters[intf].ClockStop);
    CCIDSCDEBUG("Ifsc = 0x%x \n", IccParameters[intf].Ifsc);
    CCIDSCDEBUG("Nad = 0x%x \n", IccParameters[intf].Nad);

    *pu8Buf = IccParameters[intf].FiDi;
    *(pu8Buf + 1) = IccParameters[intf].Tcckst;
    *(pu8Buf + 2) = IccParameters[intf].GuardTime;
    *(pu8Buf + 3) = IccParameters[intf].WaitingInteger;
    *(pu8Buf + 4) = IccParameters[intf].ClockStop;
    *(pu8Buf + 5) = IccParameters[intf].Ifsc;
    *(pu8Buf + 6) = IccParameters[intf].Nad;

    if(IccParameters[intf].Tcckst & 0x10)
        return 0x01;

    return 0x00;
}


/**
  * @brief  Set slot's protocol data structure and apply to hardware setting
  * @param  intf Indicate which interface to open, ether 0 or 1
  * @param  pu8Buf Protocol data structure
  * @param  u32T Specify if data structure is T=0 or T=1 type
  * @return Slot status error code
  */
uint8_t Intf_SetParameters(int32_t intf,
                           uint8_t *pu8Buf,
                           uint8_t u32T)
{
    Param NewIccParameters;

    if(intf != 0 && intf != 1)
        return SLOTERR_BAD_SLOT;

    CCIDSCDEBUG("\n**************Intf_SetParameters***************\n");
    CCIDSCDEBUG("FiDi = 0x%x \n", *pParamBuffer);
    CCIDSCDEBUG("Tcckst = 0x%x \n", *(pParamBuffer + 1));
    CCIDSCDEBUG("GuardTime = 0x%x \n", *(pParamBuffer + 2));
    CCIDSCDEBUG("WaitingInteger = 0x%x \n", *(pParamBuffer + 3));
    CCIDSCDEBUG("ClockStop = 0x%x \n", *(pParamBuffer + 4));
    CCIDSCDEBUG("Ifsc = 0x%x \n", *(pParamBuffer + 5));
    CCIDSCDEBUG("Nad = 0x%x \n", *(pParamBuffer + 6));


    NewIccParameters.FiDi = *pu8Buf;
    NewIccParameters.Tcckst = *(pu8Buf + 1);
    NewIccParameters.GuardTime = *(pu8Buf + 2);
    NewIccParameters.WaitingInteger = *(pu8Buf + 3);
    NewIccParameters.ClockStop = *(pu8Buf + 4);
    if(u32T == 0x01)
    {
        NewIccParameters.Ifsc = *(pu8Buf + 5);
        NewIccParameters.Nad = *(pu8Buf + 6);
    }
    else
    {
        NewIccParameters.Ifsc = 0x00;
        NewIccParameters.Nad = 0x00;
    }

    if((ClockRateConversion[(NewIccParameters.FiDi >> 4)].F == 0)
            || (BitRateAdjustment[(NewIccParameters.FiDi & 0x0F)].DNumerator == 0))
        return SLOTERR_BAD_FIDI;


    if((u32T == 0x00)
            && (NewIccParameters.Tcckst != 0x00)
            && (NewIccParameters.Tcckst != 0x02))
        return SLOTERR_BAD_T01CONVCHECKSUM;

    if((u32T == 0x01)
            && (NewIccParameters.Tcckst != 0x10)
            && (NewIccParameters.Tcckst != 0x11)
            && (NewIccParameters.Tcckst != 0x12)
            && (NewIccParameters.Tcckst != 0x13))
        return SLOTERR_BAD_T01CONVCHECKSUM;

    if((NewIccParameters.WaitingInteger >= 0xA0)         // condition: BWI > 0xA0 is reserved for future use
            && ((NewIccParameters.Tcckst & 0x10) == 0x10))
        return SLOTERR_BAD_WAITINGINTEGER;

    if((NewIccParameters.ClockStop != 0x00)
            && (NewIccParameters.ClockStop != 0x03))
        return SLOTERR_BAD_CLOCKSTOP;

    if(NewIccParameters.Nad != 0x00)
        return SLOTERR_BAD_NAD;

    IccParameters[intf].FiDi = NewIccParameters.FiDi;
    IccParameters[intf].Tcckst = NewIccParameters.Tcckst;
    IccParameters[intf].GuardTime = NewIccParameters.GuardTime;
    IccParameters[intf].WaitingInteger = NewIccParameters.WaitingInteger;
    IccParameters[intf].ClockStop = NewIccParameters.ClockStop;
    IccParameters[intf].Ifsc = NewIccParameters.Ifsc;
    IccParameters[intf].Nad = NewIccParameters.Nad;

    // Silently drop the request

    return SLOT_NO_ERROR;
}



/**
  * @brief  This function does nothing
  * @param  intf Indicate which interface to open, ether 0 or 1
  * @param  pu8CmdBuf Command Data
  * @param  pu32CmdSize The size of command data
  * @return Slot Status Error Code
  */
uint8_t Intf_Escape(int32_t intf,
                    uint8_t *pBlockBuffer,
                    uint32_t *pBlockSize)
{
    (void)pBlockBuffer;

    if(intf != 0 && intf != 1 && intf != 2)
        return SLOTERR_BAD_SLOT;

    *pBlockSize = 128;

    return SLOT_NO_ERROR;
}



/**
  * @brief  Enable/Disable Clock
  * @param  intf Indicate which interface to open, ether 0 or 1
  * @param  u32Cmd Specify to enable/disable clock
  * @return Slot Status Error Code
  */
uint8_t Intf_SetClock(int32_t intf, uint8_t u32Cmd)
{
    (void)u32Cmd;
    SC_T *pSC;

    if((__PC()&NS_OFFSET) == NS_OFFSET)
        pSC = (intf == 0) ? SC0_NS : (intf == 1) ? SC1_NS : SC2_NS;
    else
        pSC = (intf == 0) ? SC0 : (intf == 1) ? SC1 : SC2;

    if(intf != 0 && intf != 1 && intf != 2)
        return SLOTERR_BAD_SLOT;

    if(SC_IsCardInserted(pSC) == TRUE)
        return SLOTERR_ICC_MUTE;

    // Do nothing.

    return SLOT_NO_ERROR;
}



/**
  * @brief  Get card slot status
  * @param  intf Indicate which interface to open
  * @return Slot status error code
  */
uint8_t Intf_GetSlotStatus(int32_t intf)
{
    uint8_t Ret = 0x00;
    SC_T *pSC;

    if((__PC()&NS_OFFSET) == NS_OFFSET)
        pSC = (intf == 0) ? SC0_NS : (intf == 1) ? SC1_NS : SC2_NS;
    else
        pSC = (intf == 0) ? SC0 : (intf == 1) ? SC1 : SC2;

    if(intf != 0 && intf != 1 && intf != 2)
        return SLOTERR_BAD_SLOT;

    if(SC_IsCardInserted(pSC) == TRUE)
    {
        if(pSC->PINCTL & SC_PINCTL_CLKKEEP_Msk)
        {
            Ret = 0x00;
            CCIDSCDEBUG("Intf_GetSlotStatus:: Running ... \n");
        }
        else
        {
            Ret = 0x01;
            CCIDSCDEBUG("Intf_GetSlotStatus:: Stop ... \n");
        }
    }
    else
    {
        Ret = 0x02;
        CCIDSCDEBUG("Intf_GetSlotStatus:: Card absent ... \n");
    }

    return Ret;
}


/**
  * @brief  Get clock status of card slot
  * @param  intf Indicate which interface to open
  * @return Slot status error code
  */
uint8_t Intf_GetClockStatus(int32_t intf)
{
    SC_T *pSC;

    if((__PC()&NS_OFFSET) == NS_OFFSET)
        pSC = (intf == 0) ? SC0_NS : (intf == 1) ? SC1_NS : SC2_NS;
    else
        pSC = (intf == 0) ? SC0 : (intf == 1) ? SC1 : SC2;

    if(intf != 0 && intf != 1 && intf != 2)
        return SLOTERR_BAD_SLOT;

    if(pSC->PINCTL & SC_PINCTL_CLKKEEP_Msk)        // clock running
        return 0x00;
    else // clock stopped LOW
        return 0x01;

}

/**
  * @brief  Abort all transmitting or receiving process
  * @param  intf Indicate which interface to open, ether 0 or 1
  * @return Slot status error code
  */
uint8_t Intf_AbortTxRx(int32_t intf)
{
    SC_T *pSC;

    if((__PC()&NS_OFFSET) == NS_OFFSET)
        pSC = (intf == 0) ? SC0_NS : (intf == 1) ? SC1_NS : SC2_NS;
    else
        pSC = (intf == 0) ? SC0 : (intf == 1) ? SC1 : SC2;

    if(intf != 0 && intf != 1 && intf != 2)
        return SLOTERR_BAD_SLOT;

    // disable Tx interrupt
    pSC->INTEN &= ~SC_INTEN_TBEIEN_Msk;

    // Tx/Rx software reset
    SC_ClearFIFO(pSC);

    return SLOT_NO_ERROR;

}

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
