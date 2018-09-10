/**************************************************************************//**
 * @file     NuBL2_FWinfo.c
 * @version  V1.00
 * @brief    NuBL2 FW INFO structure.
 *
 * @copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

#include "NuBL_common.h"
#include "NuBL_crypto.h"
#include "NuBL2.h"


//typedef struct
//{
//    uint32_t        u32AuthCFGs;        /* 4-bytes */
//    uint32_t        u32FwRegionLen;     /* 4-bytes */
//    FW_REGION_T     au32FwRegion[2];    /* (8*2) bytes */
//    uint32_t        u32ExtInfoLen;      /* 4-bytes */
//    uint32_t        au32ExtInfo[5];     /* 164-bytes (includes extend info and dummy data to meet METADATA_T size limitation */    
//} __attribute__((packed)) METADATA_T;
/* Size MUST be multiple of 16 bytes due to AES256 encrypt/decrypt limitation */

//typedef struct
//{
//    ECC_PUBKEY_T    pubkey;             /* 64-bytes (256-bits + 256-bits) */
//    
//    METADATA_T      mData;              /* includes authenticate configuration, F/W regions and extend info */ 
//        
//    uint32_t        au32FwHash[8];      /* 32-bytes (256-bits) */
//    
//    ECDSA_SIGN_T    sign;               /* 64-bytes (256-bits R + 256-bits S) */    
//} __attribute__((packed)) FW_INFO_T;
const uint32_t g_InitialFWinfo[] =
{
        0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF,
        0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF,
        0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF,
        0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF,

        0x00000001, 0x00000010, NUBL2_FW_BASE, 0xFFFFFFFF,
        0xFFFFFFFF, 0xFFFFFFFF, 0x00000014, 0x55552468,
        0x66662222, 0x77772468, 0x88882468, 0x00000000,

        0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF,
        0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF,

        0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF,
        0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF,
        0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF,
        0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF
};

/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/
