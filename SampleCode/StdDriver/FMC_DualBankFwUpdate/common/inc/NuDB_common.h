/**************************************************************************//**
 * @file     NuDB_common.h
 * @version  V0.00
 * @brief    NuDB common header file
 *
 * @copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __NUDB_COMMON_H__
#define __NUDB_COMMON_H__

#ifdef __cplusplus
extern "C"
{
#endif

#define BOOT_BASE 0x0

#define TMP_PAGE_SIZE  0x800
/* CRC of each PAGE for bank0 */
#define BANK0_PAGE_CRC_BASE  0x3E800
/* CRC of each PAGE for bank1 */
#define BANK1_PAGE_CRC_BASE  0x3F000

/* Bank0 firmware related definition */
#define BANK0_FW_BASE  0x10000
#define BANK0_FW_SIZE  TMP_PAGE_SIZE*8
#define BANK0_FW_VER_BASE  (BANK0_PAGE_CRC_BASE + (BANK0_FW_SIZE/TMP_PAGE_SIZE)*4)
#define BANK0_FW_CRC_BASE  (BANK0_PAGE_CRC_BASE + (BANK0_FW_SIZE/TMP_PAGE_SIZE+1)*4)

/* Bank1 firmware related definition */
#define BANK1_FW_BASE  0x40000
#define BANK1_FW_SIZE  TMP_PAGE_SIZE*8
#define BANK1_FW_VER_BASE  (BANK1_PAGE_CRC_BASE + (BANK1_FW_SIZE/TMP_PAGE_SIZE)*4)
#define BANK1_FW_CRC_BASE  (BANK1_PAGE_CRC_BASE + (BANK1_FW_SIZE/TMP_PAGE_SIZE+1)*4)


/* For FW Swap tmp buffer, size is 1 page (2k bytes) */
#define TMP_PAGE_BASE  0x3F800

/* CRC calculate function*/
uint32_t  func_crc32(uint32_t start, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif /* __NUDB_COMMON_H__ */

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
