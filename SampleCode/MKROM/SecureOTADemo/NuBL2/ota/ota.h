/**************************************************************************//**
 * @file     ota.h
 * @version  V1.00
 * @brief    OTA header file
 *
 * @note
 * Copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __OTA_H__
#define __OTA_H__

#ifdef __cplusplus
extern "C"
{
#endif
#include <arm_cmse.h>
#include "ota_api.h"
#include "NuBL_common.h"

#define OTA_CLIENT 1

#define OTA_DEMO_WITH_APP 0

#define WDT_RST_ENABLE 0
    
#define FW_INFO_SIZE     (0x150UL) /* Size of FW INFO */

#define OTA_STATUS_BASE (0x48000) /* Base address of OTA status for OTA update from SD card method */
#define SYS_FW_OTA_STATUS_BASE (0x48000 + FMC_FLASH_PAGE_SIZE) /* Base address of system(NuBL32) firmware OTA status for OTA update on the fly method */
#define APP_FW_OTA_STATUS_BASE (0x48000 + (FMC_FLASH_PAGE_SIZE * 2)) /* Base address of application(NuBL33) firmware OTA status for OTA update on the fly method */

#define SYS_FW_BASE          (NUBL32_FW_BASE) /* Base address of current system(NuBL32) firmware */
#define APP_FW_BASE          (NUBL33_FW_BASE)

/******************************************************************************/
/****** System Firmware Upgrade Definitions                              ******/
/******************************************************************************/
#define   SYS_NEW_FW_BLOCK_SIZE     (0x8000UL)           /* include system firmware, non-secure callable block, CRC checksum and firmware version */

/******************************************************************************/
/****** Application Firmware Upgrade Definitions                         ******/
/******************************************************************************/
#define   APP_NEW_FW_BLOCK_SIZE     (0x4000UL)           /*include application firmware ,CRC checksum and firmware version(default one flash page size is 2Kbytes) */


#define MAX_PAYLOAD_SIZE (48U) /* 44 bytes */
#define MAX_FRAME_SIZE (64U)

#define STATUS_SUCCESS           (0x0U)
#define STATUS_FAILED            (0x2U)

/* commands for firmware update operation */
/***************************************/
/* CMD                                 */
/***************************************/
#define CMD_CONNECT                 0x80
#define CMD_RESET                   0x81
#define CMD_WRITE                   0x83
#define CMD_DH_KEY                  0x86
#define CMD_AUTH_KEY                0x87

#define CMD_DISCONNECT              0x8E
#define CMD_GET_VERSION             0x8F

#define CMD_ECDH_PUB0               0x8600
#define CMD_ECDH_PUB1               0x8601
#define CMD_ECDH_GET_PUB0           0x8602
#define CMD_ECDH_GET_PUB1           0x8603
#define CMD_ECDH_RAND_PUB0          0x8604
#define CMD_ECDH_RAND_PUB1          0x8605
#define CMD_ECDH_GET_RAND_PUB0      0x8606
#define CMD_ECDH_GET_RAND_PUB1      0x8607
#define CMD_GET_RAND_IV             0x8608
#define CMD_SET_RAND_IV             0x8609
#define CMD_SET_MASS_WRITE          0x8300
#define CMD_MASS_WRITE              0x8301
#define CMD_WRITE_OTP               0x8D00
#define CMD_IDENTIFY_SERVER         0x8700
#define CMD_EXEC_VENDOR_FUNC        0x8FF0
#define CMD_ERASE_KPROM             0x9801
#define CMD_IS_MASKED               0x8888


/***************************************/
/* Upgrade Error Status                */
/***************************************/
#define STS_OK                      0x00
#define STS_REBOOT                  0x01
#define ERR_CMD_CONNECT             0x7F
#define ERR_CMD_INVALID             0x7E
#define ERR_CMD_CHECKSUM            0x7D
#define ERR_ISP_CONFIG              0x7C
#define ERR_ISP_WRITE               0x7B
#define ERR_INVALID_ADDRESS         0x7A
#define ERR_OVER_RANGE              0x79
#define ERR_PAGE_ALIGN              0x78
#define ERR_ISP_ERASE               0x77
#define ERR_DH_KEY                  0x76
#define ERR_DH_ARGUMENT             0x75
#define ERR_AUTH_KEY                0x74
#define ERR_AUTH_KEY_OVER           0x73
#define ERR_CMD_KEY_EXCHANGE        0x72
#define ERR_CMD_IDENTIFY            0x71
#define ERR_SPI_INVALID_PAGESIZE    0x70
#define ERR_TIMEOUT                 0x6F
#define ERR_OLD_FW_VER              0x6E


/* Command size is fixed as 64 bytes, the unuseless byte need fill with 0x0. */

/* Maximum 64-bytes */
typedef struct
{
    /* Word-0 */
    uint16_t        u16CRC;         /* CRC16 checksum of packet raw data */
    uint8_t         u8Cmd;          /* CMD ID */
    uint8_t         u8PacketID;
    /* Word-1 */
    uint32_t        u32DataLen;     /* Byte count */
    /* Word-2 ~ 15 */
    uint32_t        au32KeyHash[8]; /* Hash of authorization key */
    uint32_t        au32Reserved[6]; /* unuseless, write 0 */
} __attribute__((packed)) CMD_AUTH_KEY_REQ_T;

/* Maximum 64-bytes */
typedef struct
{
    /* Word-0 */
    uint16_t        u16CRC;         /* CRC16 checksum of packet raw data */
    uint8_t         u8DataLen;      /* Byte count */
    uint8_t         u8PacketID;
    /* Word-1 */
    uint32_t        u32Status;      /* Status */
    /* Word-2 ~ 15 */
    uint32_t        au32EnData[14]; /* Encrpyted data */
} __attribute__((packed)) CMD_AUTH_KEY_RSP_T;

#define MAX_PKT_SIZE            64

/**
  * @brief      OTA commands handler
  * @param[in]  pu8Buff        The pointer of packet buffer
  * @param[in]  u32Len         Valind data length
  * @param[in]  u32StartIdx    Valind data index in packet buffer
  * @param[in]  u32ValidLen    Valind data length
  * @return     None
  * @details    OTA commands handler
  */
void OTA_CallBackHandler(uint8_t* pu8Buff, uint32_t len, uint32_t u32StartIdx, uint32_t u32ValidLen);

/**
  * @brief OTA process initialization
  * @param[in]  u32HSI      PLL Output Clock Frequency
  * @param[in]  pISPInfo    The pointer of ISP Info
  * @retval     0           Success
  * @retval     others      Failed
  * @details    OTA process initialization
  */
__NONSECURE_ENTRY
int32_t OTA_Init(uint32_t u32HSI, ISP_INFO_T *pISPInfo);

/**
  * @brief OTA system tick interrupt process
  * @param[in]  u32Ticks    Tick value
  * @retval     0           Success
  * @retval     others      Failed
  * @details    OTA system tick interrupt process
  */
__NONSECURE_ENTRY
uint8_t OTA_SysTickProcess(uint32_t u32Ticks);

/**
  * @brief OTA package process
  * @param      None
  * @return     None
  * @details    OTA package process
  */
__NONSECURE_ENTRY
void OTA_WiFiProcess(void);

/**
  * @brief      OTA task routine
  * @param      None
  * @retval     0             Success
  * @retval     others        Failed
  * @details    OTA task routine
  */
__NONSECURE_ENTRY
int8_t OTA_TaskProcess(void);

/**
  * @brief      Get NuBL32 or NuBL33 F/W Version
  * @param[in]  * pu32FwVer F/W version write buffer  \n
  * @param[in]  i32Mode      F/W version of NuBL32 or NuBL33. bit-0: 0: NuBL32; 1: NuBL33  \n
  * @retval     0           Success
  * @retval     others      Failed
  * @details    This function is used to get F/W version of NuBL32 or NuBL33. \n
  *             Flow: \n
  *                 1. Decrypt or get NuBL3x info \n
  *                 2. Get NuBL3x F/W version (enclosed in F/W info) \n
  */
__NONSECURE_ENTRY
int32_t OTA_GetBLxFwVer(uint32_t * pu32FwVer, int32_t i32Mode);

/**
  * @brief      Get flag of firmware upgrade done
  * @param      None
  * @return     Flag of firmware upgrade done
  * @details    This function is to get flag of firmware upgrade done.
  */
uint8_t OTA_GetFwUpgradeDone(void);

#if (OTA_UPGRADE_FROM_SD)
/**
  * @brief Verify and Update NuBL32/33 from SD card.
  * @param[in]  u8BLxSel    NuBL3x selection. Bit0 = 1: NuBL32, Bit1 = 1: NuBL33
  * @retval     0           Success
  * @retval     others      Failed
  * @details    NuBL3x identity, authentication and firmware integrity.
  *             Update NuBL3x FW INFO and firmware.
  */
int32_t OTA_VerifyAndUpdateNuBL3xFromSD(uint8_t u8BLxSel);

/**
  * @brief SD Host interrupt process
  * @param      None
  * @return     None
  * @details    SD Host interrupt process
  */
void OTA_SDH_Process(void);
#endif

/**
  * @brief Force to execute OTA update
  * @param      None
  * @retval     0       Success
  * @retval     others  Failed
  * @details    Force to execute OTA update
  */
__NONSECURE_ENTRY
int32_t OTA_ForceUpdate(void);

#ifdef __cplusplus
}
#endif

#endif /* __OTA_H__ */

/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/
