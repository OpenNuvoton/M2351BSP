/**************************************************************************//**
 * @file     ota_transfer.h
 * @version  V1.00
 * @brief    OTA transfer header file
 *
 * @note
 * Copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __OTA_TRANSFER_H__
#define __OTA_TRANSFER_H__

#ifdef __cplusplus
extern "C"
{
#endif

#define INPUT_WIFI_PROMPT 0      /* Enable interface of manual input WiFi configure */
#define PERIODIC_CHK_NEW_VER 0   /* Enable checking new firmware version periodically */
/* Wifi network settings, replace with your settings */
#define WIFINAME            "VIZIO"
#define WIFIPASS            "9624212766"
#define WIFIIP				"192.168.1.100"
#define WIFIPORT            (1111U)

extern volatile uint32_t g_u32SendbytesLen;
extern volatile uint8_t g_u8SendbytesFlag;
extern volatile uint8_t g_u8ResetFlag;
extern volatile uint8_t g_u8DisconnFlag;

/**
  * @brief      Init hardware for transfer task
  * @param      None
  * @retval     None
  * @details    This API is used for init hardware configure of transfer task.
  */
void Transfer_Init(void);

/**
  * @brief Transfer task process
  * @param      None
  * @retval     0             Success
  * @retval     others        Failed
  * @details    Transfer task process.
  */
int8_t Transfer_Process(void);

/**
  * @brief        Send transfer data
  * @param[in]    pu8TxBuf        The buffer to send the data
  * @param[in]    u32Len          The data lengths
  * @return       None
  * @details      The function is to write data into send buffer to transmit data.
  */
void Transfer_SendBytes(uint8_t pu8TxBuf[], uint32_t u32Len);

/**
  * @brief        Read received transfer data
  * @param        None
  * @return       None
  * @details      The function is used to read Rx data from WiFi module.
  */
void Transfer_WiFiProcess(void);

/**
  * @brief      System tick handler for transfer task
  * @param[in]  u32Ticks  System ticks
  * @retval     0         success
  * @retval     Other     fail
  * @details    The function is the system tick handler for transfer task.
  */
uint8_t Transfer_SysTickProcess(uint32_t u32Ticks);

/**
  * @brief        Disconnect transfer connection
  * @param        None
  * @return       None
  * @details      The function is used to disconnect transfer connection.
  */
void Transfer_ConnClose(void);

/**
  * @brief        Set Reset flag for transfer task
  * @param        None
  * @return       None
  * @details      The function is used to set reset flag for transfer task.
  */
void Transfer_SetResetFlag(void);

/**
  * @brief        Set disconnect flag for transfer task
  * @param        None
  * @return       None
  * @details      The function is used to set disconnect flag for transfer task.
  */
void Transfer_SetDisconnFlag(void);

#ifdef __cplusplus
}
#endif

#endif /* __OTA_TRANSFER_H__ */

/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/
