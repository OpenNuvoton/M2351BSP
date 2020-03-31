/**************************************************************************//**
 * @file     ota_api.h
 * @version  V1.00
 * @brief    OTA porting API header file
 *
 * @note
 * Copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __OTA_API_H__
#define __OTA_API_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include "M2351.h"

#define ENABLE_DEBUG_MSG 1
#if (ENABLE_DEBUG_MSG)
#define DEBUG_MSG  printf
#else
#define DEBUG_MSG(...)
#endif

#define OTA_UPGRADE_FROM_SD 0 /* 1: Enable OTA update from SD card method */
                              /* 0: Enable OTA update on the fly method */

#define BUF_SIZE	128
#define ALIGN_BUFF_SIZE 64

extern uint8_t g_au8SendBuf[BUF_SIZE];

/**
  * @brief      OTA task routine
  * @param      None
  * @retval     0             Success
  * @retval     others        Failed
  * @details    OTA task routine
  */
int8_t OTA_API_TaskProcess(void);

/**
  * @brief       Disconnect transfer connection
  * @param       None
  * @return      None
  * @details     The function is used to disconnect transfer connection.
  */    
void OTA_API_TransferConnClose(void);
    
/**
  * @brief       Send frame data
  * @param[in]   pu8TxBuf        The buffer to send the data
  * @param[in]   u32Len          The data lengths
  * @return      None
  * @details     The function is to write frame data into send buffer to transmit data.
  */
void OTA_API_SendFrame(uint8_t* pu8Buff, uint32_t u32Len);

/**
  * @brief      OTA commands handler
  * @param[in]  pu8Buff        The pointer of packet buffer
  * @param[in]  u32Len         Valind data length
  * @param[in]  u32StartIdx    Valind data index in packet buffer
  * @param[in]  u32ValidLen    Valind data length
  * @return     None
  * @details    OTA commands handler
  */
int8_t OTA_API_RecvCallBack(uint8_t* pu8Buff, uint32_t u32Len, uint32_t u32StartIdx, uint32_t u32ValidLen);

/**
  * @brief      Init function for any hardware requirements(optional)
  * @param[in]  u32HSI        PLL Output Clock Frequency
  * @return     None
  * @details    This funcfion is to init function for any hardware requirements.
  */
void OTA_API_Init(uint32_t u32HSI);

/**
  * @brief      Get page size of flash
  * @param      None
  * @retval     0             Success
  * @retval     others        Failed
  * @details    This function is to get page size of flash.
  */
uint32_t OTA_API_GetFlashPageSize(void);

/**
  * @brief      Erase flash region
  * @param[in]  u32FlashAddr  Flash address
  * @retval     0             Success
  * @retval     others        Failed
  * @details    This function is to erase flash region.
  */
uint8_t OTA_API_EraseFlash(uint32_t u32FlashAddr);

/**
  * @brief      Write flash data
  * @param[in]  u32FlashAddr  Flash address
  * @param[in]  u32Data       data
  * @retval     0             Success
  * @retval     others        Failed
  * @details    This function is to write flash data.
  */
uint8_t OTA_API_WriteFlash(uint32_t u32FlashAddr, uint32_t u32Data);

/**
  * @brief      Get flag of firmware upgrade done
  * @param      None
  * @return     Flag of firmware upgrade done
  * @details    This function is to get flag of firmware upgrade done.
  */
uint8_t OTA_API_GetFwUpgradeDone(void);

/**
  * @brief      System tick handler for transfer task
  * @param[in]  u32Ticks  System ticks
  * @retval     0         success
  * @retval     Other     fail
  * @details    The function is the system tick handler for transfer task.
  */
uint8_t OTA_API_SysTickProcess(uint32_t u32Ticks);

/**
  * @brief        Read received transfer data
  * @param        None
  * @return       None
  * @details      The function is used to read Rx data from WiFi module.
  */
void OTA_API_WiFiProcess(void);

/**
  * @brief        Set Reset flag for transfer task
  * @param        None
  * @return       None
  * @details      The function is used to set reset flag for transfer task.
  */
void OTA_API_SetResetFlag(void);


#if (OTA_UPGRADE_FROM_SD)
/**
  * @brief        Init SD Host peripheral
  * @param        None
  * @return       None
  * @details      The function is used to init SD Host peripheral.
  */
int8_t OTA_API_SDInit(void);

/**
  * @brief        Open new NuBL3x firmware package file
  * @param        None
  * @return       None
  * @details      The function is used to open new NuBL3x firmware package file.
  */
uint8_t OTA_API_SDFwPackWriteOpen(uint8_t u8BLxSel);

/**
  * @brief  Write file data into SD card
  * @param[in]  pu8Buffer     Pointer of read data buffer
  * @param[in]  u32BufferLen  Data buffer length
  * @retval     0             Success
  * @retval     others        Failed
  * @details    Write file data into SD card.
  */
uint8_t OTA_API_SDWrite(uint8_t * pu8Buffer, uint32_t u32BufferLen);

/**
  * @brief  Close NuBL3x firmware package file
  * @param[in]  u8NuBLxSel  NuBL3x firmware package selection. Bit0 = 1: NuBL32, Bit1 = 1: NuBL33
  * @retval     0           Success
  * @retval     others      Failed
  * @details    Close NuBL3x firmware package file from SD card.
  */
uint8_t OTA_API_SDClose(uint8_t u8BLxSel);

/**
  * @brief  Open existed NuBL3x firmware package file
  * @param[in]  u8NuBLxSel  NuBL3x firmware package selection. Bit0 = 1: NuBL32, Bit1 = 1: NuBL33
  * @retval     0           Success
  * @retval     others      Failed
  * @details    Open existed NuBL3x firmware package file from SD card.
  */
uint8_t OTA_API_SDFwPackReadOpen(uint8_t u8BLxSel);

/**
  * @brief  Read file data from SD card
  * @param[in]  pu8Buffer     Pointer of read data buffer
  * @param[in]  u32BufferLen  Data buffer length
  * @param[in]  pu32ReadLen   Read data length
  * @retval     0             Success
  * @retval     others        Failed
  * @details    Read file data from SD card.
  */
uint8_t OTA_API_SDRead(uint8_t * pu8Buffer, uint32_t u32BufferLen, uint32_t *pu32ReadLen);

/**
  * @brief       IRQ Handler for SDH Interrupt
  * @param       None
  * @return      None
  * @details     The SDH_Process function is used for SDH interrupt handler.
  */
void SDH_Process(void);
#endif



#ifdef __cplusplus
}
#endif

#endif /* __OTA_API_H__ */

/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/
