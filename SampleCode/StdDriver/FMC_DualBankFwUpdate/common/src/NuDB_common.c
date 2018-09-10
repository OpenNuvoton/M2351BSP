/**************************************************************************//**
 * @file     NuDB_common.c
 * @version  V3.00
 * @brief    NuDB common source file
 *
 * @copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

#include "NuDB_common.h"


/**
 * @brief      Checksum calculation function
 * @param[in]  start       check sum calculation start address
 * @param[in]  len         check sum calculation block size
 * @retval     sum         check sum value
 */

uint32_t  func_crc32(uint32_t start, uint32_t len)
{
    uint32_t  idx, data32 = 0UL;

    /* WDTAT_RVS, CHECKSUM_RVS, CHECKSUM_COM */
    for(idx = 0; idx < len; idx += 4)
    {
        data32 += *(uint32_t *)(start + idx);
    }
    data32 = 0xFFFFFFFF - data32 + 1UL;

    return data32;
}
/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/
