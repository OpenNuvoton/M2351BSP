/******************************************************************************
 * @file     sc_intf.h
 * @version  V2.00
 * @brief    USBD CCID smartcard interface control header
 *
 * @note
 * Copyright (C) 2014~2015 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __SC_INTF_H__
#define __SC_INTF_H__

#ifdef  __cplusplus
extern "C"
{
#endif

#define OPERATION_CLASS_AUTO                      (0x00)
#define OPERATION_CLASS_A                         (0x01)
#define OPERATION_CLASS_B                         (0x02)
#define OPERATION_CLASS_C                         (0x03)

#define DEFAULT_FIDI                (0x11)
#define DEFAULT_T01CONVCHECKSUM     (0x00)
#define DEFAULT_GUARDTIME           (0x00)
#define DEFAULT_WAITINGINTEGER      (0x0A)
#define DEFAULT_CLOCKSTOP           (0x03)
#define DEFAULT_IFSC                (0x20)
#define DEFAULT_NAD                 (0x00)

uint8_t Intf_Init(int32_t intf);
uint8_t Intf_ApplyParametersStructure(int32_t intf);
uint8_t Intf_GetHwError(int32_t intf);
uint8_t Intf_IccPowerOn(int32_t intf,
                        uint32_t u32Volt,
                        uint8_t *pu8AtrBuf,
                        uint32_t *pu32AtrSize);
uint8_t Intf_XfrBlock(int32_t intf,
                      uint8_t *pu8CmdBuf,
                      uint32_t *pu32CmdSize);
uint8_t Intf_XfrShortApduT0(int32_t intf,
                            uint8_t *pu8CmdBuf,
                            uint32_t *pu32CmdSize);
uint8_t Intf_XfrShortApduT1(int32_t intf,
                            uint8_t *pu8CmdBuf,
                            uint32_t *pu32CmdSize);
uint8_t Intf_GetParameters(int32_t intf, uint8_t *pu8Buf);
uint8_t Intf_SetParameters(int32_t intf,
                           uint8_t *pu8Buf,
                           uint8_t u32T);
uint8_t Intf_Escape(int32_t intf,
                    uint8_t *pu8CmdBuf,
                    uint32_t *pu32CmdSize);
uint8_t Intf_SetClock(int32_t intf, uint8_t ClockCmd);
uint8_t Intf_GetSlotStatus(int32_t intf);
uint8_t Intf_GetClockStatus(int32_t intf);
uint8_t Intf_AbortTxRx(int32_t intf);

extern uint8_t s_u8ChainParameter;

#ifdef  __cplusplus
}
#endif

#endif // __SC_INTF_H__

/*** (C) COPYRIGHT 2014~2015 Nuvoton Technology Corp. ***/


