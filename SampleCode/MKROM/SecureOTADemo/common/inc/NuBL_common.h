/**************************************************************************//**
 * @file     NuBL_common.h
 * @version  V1.00
 * @brief    NuBL common header file
 *
 * @copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __NUBL_COMMON_H__
#define __NUBL_COMMON_H__

#ifdef __cplusplus
extern "C"
{
#endif
    
#if defined(ENABLE_MSG)
 #define NUBL_MSG       printf  /* enable debug message */
#else    
 #define NUBL_MSG(...)
#endif
    
/*---------------------------------------------------------------------------------------------------------*/
/* FLASH memory layout definitions                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
#define NUBL2_FW_BASE           0x00000000ul    /* 0K, Secure code */
#define NUBL32_FW_BASE          0x00040000ul    /* 256K, Secure code; SRAM 32~48K */
#define NUBL33_FW_BASE          0x00060000ul    /* 384K, Non-secure code start address */
    
#define NUBL2_LIB_BASE          0x0002C000ul    /* 176K (0x2C000) */
#define NUBL32_LIB_BASE         0x00047000ul    /* 284K, non-Secure callable code; */

#define NUBL1_IB_BASE           (NUBL33_FW_BASE - (1024*2)) /* 382K, 0x5F800 */

#define KEY_STORAGE_BASE        (NUBL1_IB_BASE - (1024*2))  /* 380K, 0x5F000 (encrypt [NuBL32, NuBL33 and Host(for cmd) public keys]) */
#define NUBL2_FW_INFO_BASE      (NUBL1_IB_BASE - (1024*4))  /* 378K, 0x5E800 */
#define NUBL32_FW_INFO_BASE     (NUBL1_IB_BASE - (1024*6))  /* 376K, 0x5E000 */
#define NUBL33_FW_INFO_BASE     (NUBL1_IB_BASE - (1024*8))  /* 374K, 0x5D800 */


#if 1 // For debug...
/* 
    0: Use OTP regions to store NuBL2 public key Hash
    1: Emulate OTP regions by using NUBL2_PUBKEY_HASH_BASE region to store NuBL2 public key Hash
*/
#define EMULATE_OTP_REGION      1
#if (EMULATE_OTP_REGION == 1)    
 #define NUBL2_PUBKEY_HASH_BASE (NUBL2_FW_BASE - (1024*10)) // 118K
#endif
#define DIRECT_VERIFY_NUBL3x_PUBKEY
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* Status and error code definitions                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
#define NUBL_STS_OK             (0)
#define ERR_XOM_NOT_ACTIVE      (-1)
#define ERR_XOM_SIZE            (-2)
#define ERR_XOM_ADDRESS         (-3)
#define ERR_INIT_ECC            (-1001)
#define ERR_GEN_ECDH            (-1002)
#define ERR_AES_DECRYPT         (-1003)


/* typedef struct declaration */

typedef struct
{
    uint32_t        u32Start;   /* 32-bits */
    uint32_t        u32Size;    /* 32-bits */
/* 
    bit[28]:    Indicate Secure/Non-secure flash. 
                1: Non-secure flash / 0: Secure flash
    bit[30]:    1: Use Flash CRC32 checksum page by page to calculate Hash.
                0: Direct read flash data to calculate Hash.
*/
} __attribute__((packed)) FW_REGION_T;

typedef struct
{
    uint32_t        u32AuthCFGs;        /* 4-bytes */
                        /* 
                            bit[1:0]:   Verification method, 1: ECDSA / 0: ECDH
                            bit[2]:     1: Info Hash includes PDID / 0: Not include PDID
                            bit[3]:     1: Info Hash includes UID / 0: Not include UID
                            bit[4]:     1: Info Hash inculdes UICD / 0: Not include UICD
                            bit[31:5]:  Reserved
                        */
    uint32_t        u32FwRegionLen;     /* 4-bytes */
    FW_REGION_T     au32FwRegion[2];    /* (8*2) bytes */
    uint32_t        u32ExtInfoLen;      /* 4-bytes */
    uint32_t        au32ExtInfo[41-4];     /* 20-bytes (includes extend info and dummy data to meet METADATA_T size limitation */    
} __attribute__((packed)) METADATA_T;
/* Size MUST be multiple of 16 bytes due to AES256 encrypt/decrypt limitation */

typedef struct
{
    ECC_PUBKEY_T    pubkey;             /* 64-bytes (256-bits + 256-bits) */
    
    METADATA_T      mData;              /* includes authenticate configuration, F/W regions and extend info */ 
        
    uint32_t        au32FwHash[8];      /* 32-bytes (256-bits) */
    
    ECDSA_SIGN_T    sign;               /* 64-bytes (256-bits R + 256-bits S) */    
} __attribute__((packed)) FW_INFO_T;
/* Size MUST be multiple of 16 bytes due to AES256 encrypt/decrypt limitation */

extern const FW_INFO_T g_FWinfoInitial;

#ifdef __cplusplus
}
#endif

#endif /* __NUBL_COMMON_H__ */

/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/
