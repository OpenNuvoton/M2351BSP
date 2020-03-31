/**************************************************************************//**
 * @file     NuBL32_FwInfo.c
 * @version  V1.00
 * @brief    NuBL32 Firmware Info source code.
 *
 * @copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

#include "NuBL_common.h"

#define NUBL32_VER 2ul


//typedef struct
//{
//    ECC_PUBKEY_T    pubkey;             /* 64-bytes (256-bits + 256-bits) */
//    
//    METADATA_T      mData;              /* includes authenticate configuration, F/W regions and extend info */ 
//        
//    uint32_t        au32FwHash[8];      /* 32-bytes (256-bits) */
//    
//    ECDSA_SIGN_T    sign;               /* 64-bytes (256-bits R + 256-bits S) */    
//
//} __attribute__((packed)) FW_INFO_T;
const FW_INFO_T g_FWinfoInitial =
{
    {/* public key - 64-bytes (256-bits + 256-bits) */
        {0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF,
        0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF},
        {0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF,
        0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF}
    },
    {/* metadata data - includes authenticate configuration, F/W regions and extend info */
        0x00000001, 0x00000010, {{NUBL32_FW_BASE, 0x00000000},
        {0x00000000, 0x00000000}}, 0x00000094, {NUBL32_VER,
        0x00000000, 0x00000000, 0x00000000, 0x00000000,
        0x00000000, 0x00000000, 0x00000000, 0x00000000,
        0x00000000, 0x00000000, 0x00000000, 0x00000000,
        0x00000000, 0x00000000, 0x00000000, 0x00000000,
        0x00000000, 0x00000000, 0x00000000, 0x00000000,
        0x00000000, 0x00000000, 0x00000000, 0x00000000,
        0x00000000, 0x00000000, 0x00000000, 0x00000000,
        0x00000000, 0x00000000, 0x00000000, 0x00000000,
        0x00000000, 0x00000000, 0x00000000, 0x00000000}
    },
    {/* FW hash - 32-bytes (256-bits) */
        0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF,
        0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF
    },
    {/* FwInfo signature - 64-bytes (256-bits R + 256-bits S) */
        {0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF,
        0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF},
        {0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF,
        0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF}
    }
};

/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/
