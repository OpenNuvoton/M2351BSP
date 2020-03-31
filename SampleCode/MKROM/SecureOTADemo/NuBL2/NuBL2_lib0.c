/**************************************************************************//**
 * @file     NuBL2_lib0.c
 * @version  V1.00
 * @brief    NuBL2 library source code.
 *
 * @copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

#include "NuBL_common.h"
#include "NuBL_crypto.h"
#include "NuBL2.h"

#define printf(...)

#define DEBUG_CMD 0


void NuBL2_BytesSwap(char *buf, int32_t len);
void SetECCRegisters(int32_t i32Mode, uint32_t *pPriv, uint32_t *pPub);
void GetNuBL2AES256Key(uint32_t *key);
uint16_t cmd_CalCRC16Sum(uint32_t *pu32buf, uint32_t len);
int32_t cmd_VerifyCRC16Sum(uint32_t *pu32buf);
int32_t GenCmdSessionKey(uint32_t key[]);
int32_t IdentifyPublicKey(uint32_t *p32Buf, int32_t i32Mode);

static uint32_t IsAccessDenied(void)
{
    /* TODO:
        1. Check KPROM status (identity check then unlock KPROM protect)
        2. XOM region status
        3. Secure locked status
        4. Secuar callable only

        * Clear SRAM, crypto and others secure information
    */
    return 0;
#if 0
    if(FMC_GetXOMState(XOMR0) != 1)
    {
        NUBL_MSG("\n[XOM attributes FAIL. XOM0 is not active]\n\n");
        return (uint32_t)-1;
    }

    if((FMC->XOMR0STS>>8) == NUBL2_LIB_BASE)
    {
        if((FMC->XOMR0STS&0xFF) == ((NUBL32_FW_BASE-NUBL2_LIB_BASE)/FMC_FLASH_PAGE_SIZE))
        {
            return 0;
        }
        else
        {
            NUBL_MSG("\n[XOM attributes FAIL. Invalid XOM0 region page counts]\n\n");
            return (uint32_t)-2;
        }
    }
    else
    {
        NUBL_MSG("\n[XOM attributes FAIL. Invalid XOM0 region base address]\n\n");
        return (uint32_t)-3;
    }
    #endif
}

static void SetNuBL2PrivKey(void)
{
    //Set NuBL2 private key
    //example:
    //  const char NuBL2_priv[] = "380a67fcfc01ca7073da7c2c54296a61327f77262a7d4674c3d8e29a63e3fa20";
    //  "380a67fc-fc01ca70-73da7c2c-54296a61-327f7726-2a7d4674-c3d8e29a-63e3fa20"
    CRPT->ECC_K[0] = 0x63e3fa20;
    CRPT->ECC_K[1] = 0xc3d8e29a;
    CRPT->ECC_K[2] = 0x2a7d4674;
    CRPT->ECC_K[3] = 0x327f7726;
    CRPT->ECC_K[4] = 0x54296a61;
    CRPT->ECC_K[5] = 0x73da7c2c;
    CRPT->ECC_K[6] = 0xfc01ca70;
    CRPT->ECC_K[7] = 0x380a67fc;
}


/* __REV(x), byte swap32 */
static uint32_t NuBL_Swap32(uint32_t value)
{
    volatile uint32_t val;

    val = (value<<24) | ((value<<8)&0xff0000) | ((value>>8)&0xff00) | (value>>24);
    return val;
}

void SetECCRegisters(int32_t i32Mode, uint32_t *pPriv, uint32_t *pPub)
{
    volatile int32_t    i;
    ECC_PUBKEY_T        *pPubKey;

    if(i32Mode == 0x00020000)
    {
        pPubKey = (ECC_PUBKEY_T *)pPub;
        SetNuBL2PrivKey();
        NuBL2_BytesSwap((char*)pPubKey->au32Key0, sizeof(pPubKey->au32Key0));
        NuBL2_BytesSwap((char*)pPubKey->au32Key1, sizeof(pPubKey->au32Key1));
        for(i=0; i<8; i++)
            CRPT->ECC_X1[i] = pPubKey->au32Key0[i];
        for(i=0; i<8; i++)
            CRPT->ECC_Y1[i] = pPubKey->au32Key1[i];
    }
    if(i32Mode == 0x000F000F)
    {
        pPubKey = (ECC_PUBKEY_T *)pPub;
        for(i=0; i<8; i++)
            CRPT->ECC_K[i] = pPriv[i];
        for(i=0; i<8; i++)
            CRPT->ECC_X1[i] = pPubKey->au32Key0[i];
        for(i=0; i<8; i++)
            CRPT->ECC_Y1[i] = pPubKey->au32Key1[i];
    }
}

//    //iv = "1000000000000000000000000000000a"
//    //CRPT->AES0_IV[0] = 0x10000000;
//    //CRPT->AES0_IV[1] = 0x00000000;
//    //CRPT->AES0_IV[2] = 0x00000000;
//    //CRPT->AES0_IV[3] = 0x0000000a;

//    //iv = "00000000000000000000000000000000"

/* These AES256 keys are used to encrypt/decrypt Public Key Storage in ECDSA verification mode */
void GetNuBL2AES256Key(uint32_t *key)
{
    key[0] = 0x12345678;
    key[1] = 0x22222222;
    key[2] = 0x33333333;
    key[3] = 0x44444444;
    key[4] = 0x55555555;
    key[5] = 0x66666666;
    key[6] = 0x77777777;
    key[7] = 0x89abcdef;
}


/**
  * @brief      Perform NuBL2 to verify NuBL32 or NuBL33 F/W
  * @param[in]  mode        bit-0: 0:verify NuBL32; 1:verify NuBL33 \n
  *                         bit-4: 0:Integrity check; 1:Identify check
  * @retval     0           Success
  * @retval     others      Failed
  * @details    This function is used to perform identify, authenticate and verify F/W integrity of NuBL32 or NuBL33. \n
  *             Flow: \n
  *                 1. Decrypt(use NuBL2 AES key) Key Storage to get NuBL32x public key \n
  *                 2. Decrypt(use NuBL2&NuBL3x ECDH key) NuBL3x info \n
  *                 3. Identify NuBL3x public key \n
  *                 4. Authenticate NuBL3x info Hash \n
  *                 5. Verify NuBL3x F/W Hash \n
  */
int32_t NuBL2_ExecuteVerifyNuBL3x(uint32_t *buf, int32_t mode)
{
    volatile int32_t    ret = -1000;
    uint32_t            *keybuf, *infobuf, base, len, start, end;
    ECC_PUBKEY_T        PubKey;
    FW_INFO_T           FwInfo;
    uint32_t            AESkey[8], Hash[8], HashValue[8];
    volatile uint32_t    i;

    if(!((mode == 0) || (mode == 1) || (mode == 0x10) || (mode == 0x11)))
    {
        NUBL_MSG("\nNuBL2 verify NuBL3x FAIL. Invalid mode: 0x%x.\n\n", mode);
        return ret;
    }

    if((ret=(int32_t)IsAccessDenied()) != 0)
        return ret;

    NUBL_MSG("\nNuBL2 verify NuBL3%d. \n\n", ((mode&BIT0)==0)?2:3);

    memset(&AESkey, 0x0, sizeof(AESkey));
    memset(&PubKey, 0x0, sizeof(ECC_PUBKEY_T));
    memset(&FwInfo, 0x0, sizeof(FW_INFO_T));

    if(buf == NULL)
        infobuf = (uint32_t *)(uint32_t)&FwInfo;
    else
        infobuf = buf;

    /* Check NuBL3x Key Storage Hash */
    /* Get encrypted NuBL3x public key from Key Storage */
    len = sizeof(ECC_PUBKEY_T);
    if((((uint32_t)mode)&BIT0) == 0)
    {
        extern uint32_t g_NuBL32EnCryptPubKeyBase;  /* declared in LoadKeyStorage.s */
        base = (uint32_t)&g_NuBL32EnCryptPubKeyBase;  /* encrypted NuBL32 pub key address */
    }
    else
    {
        extern uint32_t g_NuBL33EnCryptPubKeyBase;  /* declared in LoadKeyStorage.s */
        base = (uint32_t)&g_NuBL33EnCryptPubKeyBase;  /* encrypted NuBL33 pub key address */
    }
    keybuf = (uint32_t *)(uint32_t)&PubKey;
    for(i=0; i<(len/4); i++)
        keybuf[i] = inpw(base + (i*4));

    start = (uint32_t )&PubKey;
    end   = (uint32_t )&PubKey+len;
    NuBL_CalculateSHA256(start, end, (uint32_t *)Hash, SHA_ONESHOT, SHA_SRC_SRAM);

    /* Get encrypted NuBL3x public key from Key Storage Hash base */
    len = sizeof(ECC_PUBKEY_T);
    if((((uint32_t)mode)&BIT0) == 0)
    {
        extern uint32_t g_NuBL32EnCryptPubKeyHashBase;  /* declared in LoadKeyStorage.s */
        base = (uint32_t)&g_NuBL32EnCryptPubKeyHashBase;  /* encrypted NuBL32 pub key hash address */
    }
    else
    {
        extern uint32_t g_NuBL33EnCryptPubKeyHashBase;  /* declared in LoadKeyStorage.s */
        base = (uint32_t)&g_NuBL33EnCryptPubKeyHashBase;  /* encrypted NuBL33 pub key hash address */
    }
    start = base;
    end   = base + sizeof(Hash);
    keybuf = (uint32_t *)&HashValue;
    for(i=0; i<(sizeof(HashValue)/4); i++)
        keybuf[i] = inpw(base + (i*4));

    if(memcmp(&Hash[0], &HashValue[0], (HASH_SIZE/8)) != 0)
    {
        uint32_t *tmp;
        tmp = (uint32_t *)&Hash;
        NUBL_MSG("Verify NuBL3%d F/W Hash [FAIL]\n\n", ((mode&BIT0)==0)?2:3);
        NUBL_MSG("F/W Hash:\n");
        for(i=0; i<((HASH_SIZE/8)/4); i++)
            NUBL_MSG("\t%d: 0x%08x - 0x%08x.\n", i, Hash[i], HashValue[i]);
        ret = -5001;
        goto _exit_NuBL2_ExecuteVerifyNuBL3x;
    }
    NUBL_MSG("Verify NuBL3%d Key Storage Hash [PASS]\n\n", ((mode&BIT0)==0)?2:3);

/* Step 1. Decrypt NuBL3x public key */
    /* Get AES256key for follows AES decryption */
    GetNuBL2AES256Key(AESkey);
    NUBL_MSG("AES256 decryption keys:\n");
    for(i=0; i<8; i++)
        NUBL_MSG("\t0x%08x.\n", AESkey[i]);

    /* Get encrypted NuBL3x public key from Key Storage */
    len = sizeof(ECC_PUBKEY_T);
    if((((uint32_t)mode)&BIT0) == 0)
    {
        extern uint32_t g_NuBL32EnCryptPubKeyBase;  /* declared in LoadKeyStorage.s */
        base = (uint32_t)&g_NuBL32EnCryptPubKeyBase;  /* encrypted NuBL32 pub key address */
    }
    else
    {
        extern uint32_t g_NuBL33EnCryptPubKeyBase;  /* declared in LoadKeyStorage.s */
        base = (uint32_t)&g_NuBL33EnCryptPubKeyBase;  /* encrypted NuBL33 pub key address */
    }
    keybuf = (uint32_t *)(uint32_t)&PubKey;
    for(i=0; i<(len/4); i++)
        keybuf[i] = inpw(base + (i*4));

    /* Trigger decryption... */
    if(NuBL_AES256Decrypt(keybuf, keybuf, len, AESkey) != 0)
    {
        ret = -1003;
        goto _exit_NuBL2_ExecuteVerifyNuBL3x;
    }
    NUBL_MSG("Decrypt NuBL3%d public key [Done]\n\n", ((mode&BIT0)==0)?2:3);


/* Step 2. Decrypt or get NuBL3x info */
    /* Get NuBL3x F/W info */
    len = sizeof(FW_INFO_T);
    if((((uint32_t)mode)&BIT4) != BIT4)
    {
        if((((uint32_t)mode)&BIT0) == 0)
            base = NUBL32_FW_INFO_BASE;   // NuBL32 F/W info address
        else
            base = NUBL33_FW_INFO_BASE;   // NuBL33 F/W info address
        for(i=0; i<(len/4); i++)
            infobuf[i] = inpw(base + (i*4));
    }
    NUBL_MSG("Get NuBL3%d F/W info [Done]\n\n", ((mode&BIT0)==0)?2:3);


/* Step 3. Identify NuBL3x public key (encloed in Key Storage and F/W info) */
    memcpy(&FwInfo, infobuf, sizeof(FW_INFO_T));
#if defined(DIRECT_VERIFY_NUBL3x_PUBKEY)
    if(memcmp(&PubKey.au32Key0[0], &FwInfo.pubkey.au32Key0[0], sizeof(ECC_PUBKEY_T)) != 0)
    {
        NUBL_MSG("Identify NuBL3%d public key [FAIL]\n\n", ((mode&BIT0)==0)?2:3);
        ret = -3001;
        goto _exit_NuBL2_ExecuteVerifyNuBL3x;
    }
#else
    start = (uint32_t )&PubKey.au32Key0[0];
    end   = start + sizeof(ECC_PUBKEY_T)/2;
    NuBL_CalculateSHA256(start, end, (uint32_t *)Hash, SHA_ONESHOT, SHA_SRC_SRAM);
    if(memcmp((uint32_t *)Hash, &FwInfo.pubkey.au32Key0[0], sizeof(Hash)) != 0)
    {
        NUBL_MSG("Identify NuBL3%d public key [FAIL]\n\n", ((mode&BIT0)==0)?2:3);
        ret = -3001;
        goto _exit_NuBL2_ExecuteVerifyNuBL3x;
    }
    start = (uint32_t )&PubKey.au32Key1[0];
    end   = start + sizeof(ECC_PUBKEY_T)/2;
    NuBL_CalculateSHA256(start, end, (uint32_t *)Hash, SHA_ONESHOT, SHA_SRC_SRAM);
    if(memcmp((uint32_t *)Hash, &FwInfo.pubkey.au32Key1[0], sizeof(Hash)) != 0)
    {
        NUBL_MSG("Identify NuBL3%d public key [FAIL]\n\n", ((mode&BIT0)==0)?2:3);
        ret = -3001;
        goto _exit_NuBL2_ExecuteVerifyNuBL3x;
    }
#endif
    NUBL_MSG("Identify NuBL3%d public key [PASS]\n\n", ((mode&BIT0)==0)?2:3);


/* Step 4. Authenticate NuBL3x info Hash */
    /* Calculate NuBL3x info Hash */
    start = (uint32_t )&FwInfo;
    end   = (uint32_t )&FwInfo.sign.au32R[0];
    NuBL_CalculateSHA256(start, end, (uint32_t *)Hash, SHA_ONESHOT, SHA_SRC_SRAM);

    if(NuBL2_VerifyNuBL3xECDSASignature((uint32_t *)Hash, (ECDSA_SIGN_T *)&FwInfo.sign.au32R[0], (ECC_PUBKEY_T *)&PubKey.au32Key0[0], mode) != 0)
    {
        uint32_t *tmp0;
        tmp0 = (uint32_t *)&FwInfo.sign.au32R[0];
        NUBL_MSG("Authenticate NuBL3%d info signature [FAIL]\n\n", ((mode&BIT0)==0)?2:3);
        NUBL_MSG("Get signature:\n");
        for(i=0; i<(sizeof(ECDSA_SIGN_T)/4); i++)
            NUBL_MSG("\t%d: 0x%08x.\n", i, tmp0[i]);
        ret = -4001;
        goto _exit_NuBL2_ExecuteVerifyNuBL3x;
    }
    NUBL_MSG("Authenticate NuBL3%d info signature [PASS]\n\n", ((mode&BIT0)==0)?2:3);



/* Step 5. Verify NuBL3x F/W Hash */
    if((((uint32_t)mode)&BIT4) != BIT4) /* if BIT4 == 0, dnubo not check F/W integrity */
    {
        uint32_t au32Hash[8];

        start = FwInfo.mData.au32FwRegion[1].u32Start;
        end   = start + FwInfo.mData.au32FwRegion[1].u32Size;
        /* if start address and size are both 0x0, it means no second firmware. */
        if ((start == 0x0)&&(end == 0x0))
        {
            start = FwInfo.mData.au32FwRegion[0].u32Start;
            end   = start + FwInfo.mData.au32FwRegion[0].u32Size;
            /* Calculate NuBL3x F/W Hash (use PubKey buffer to store NuBL3x F/W Hash) */
            memset(&au32Hash, 0x0, sizeof(au32Hash));
            NuBL_CalculateSHA256(start, end, (uint32_t *)au32Hash, SHA_ONESHOT, SHA_SRC_FLASH);
        }
        else
        {
            start = FwInfo.mData.au32FwRegion[0].u32Start;
            end   = start + FwInfo.mData.au32FwRegion[0].u32Size;
            /* Calculate NuBL3x F/W Hash (use PubKey buffer to store NuBL3x F/W Hash) */
            memset(&au32Hash, 0x0, sizeof(au32Hash));
            NuBL_CalculateSHA256(start, end, (uint32_t *)au32Hash, SHA_CONTI_START, SHA_SRC_FLASH);

            start = FwInfo.mData.au32FwRegion[1].u32Start;
            end   = start + FwInfo.mData.au32FwRegion[1].u32Size;
            NuBL_CalculateSHA256(start, end, (uint32_t *)au32Hash, SHA_CONTI_END, SHA_SRC_FLASH);
        }

        if(memcmp(&au32Hash[0], &FwInfo.au32FwHash[0], (HASH_SIZE/8)) != 0)
        {
            uint32_t *tmp;
            tmp = (uint32_t *)&au32Hash;
            NUBL_MSG("Verify NuBL3%d F/W Hash [FAIL]\n\n", ((mode&BIT0)==0)?2:3);
            NUBL_MSG("F/W Hash:\n");
            for(i=0; i<((HASH_SIZE/8)/4); i++)
                NUBL_MSG("\t%d: 0x%08x - 0x%08x.\n", i, tmp[i], FwInfo.au32FwHash[i]);
            ret = -5001;
            goto _exit_NuBL2_ExecuteVerifyNuBL3x;
        }
        NUBL_MSG("Verify NuBL3%d F/W Hash [PASS]\n\n", ((mode&BIT0)==0)?2:3);
    }
    ret = 0;

_exit_NuBL2_ExecuteVerifyNuBL3x:
    SYS_ResetModule(CRPT_RST);

    memset(&AESkey, 0x0, sizeof(AESkey));
    memset(&Hash,   0x0, sizeof(Hash));
    memset(&PubKey, 0x0, sizeof(ECC_PUBKEY_T));
    memset(&FwInfo, 0x0, sizeof(FW_INFO_T));

    return ret;
}

/*
    Return
        (NuBL2_priv * NuBL32_pub) ECDH key in Key32[]
        (NuBL2_priv * NuBL33_pub) ECDH key in Key33[]
*/
int32_t NuBL2_GetNuBL3xECDHKeys(uint32_t Key32[], uint32_t Key33[])
{
    volatile int32_t    ret = -1000;
    uint32_t            *keybuf, base, len;
    ECC_PUBKEY_T        PubKey;
    uint32_t            AESkey[8];
    volatile uint32_t    i;

    if((ret=(int32_t)IsAccessDenied()) != 0)
        return ret;

    NUBL_MSG("\nCalculate (NuBL2*NuBL3) and (NuBL2*NuBL33) ECDH keys.\n\n");

    GetNuBL2AES256Key(AESkey);

    for(i=0; i<8; i++)
        NUBL_MSG("\tSharedKey: 0x%08x. (NuBL2_priv * NuBL1_pub)\n", AESkey[i]);

    /* Get encrypted NuBL32 public key from Key Storage */
    len = sizeof(ECC_PUBKEY_T);
    extern uint32_t g_NuBL32EnCryptPubKeyBase;  /* declared in LoadKeyStorage.s */
    base = (uint32_t)&g_NuBL32EnCryptPubKeyBase;  /* encrypted NuBL32 pub key address */
    keybuf = (uint32_t *)(uint32_t)&PubKey;
    for(i=0; i<(len/4); i++)
        keybuf[i] = FMC_Read(base + (i*4));

    /* Trigger decryption... */
    if(NuBL_AES256Decrypt(keybuf, keybuf, len, AESkey) != 0)
    {
        ret = -1003;
        goto _exit_NuBL2_GetNuBL3xECDHKeys;
    }

/* Step 2. Calculate (NuBL2 * NuBL32) ECDH key */
    /* Init ECC */
    if(NuBL_ECCInitCurve() != 0)
    {
        ret = -2001;
        goto _exit_NuBL2_GetNuBL3xECDHKeys;
    }

    /* Select NuBL2_priv * NuBL32_pub */
    SetECCRegisters(0x00020000, NULL, (uint32_t *)(uint32_t)&PubKey);

    /* Calculate ECDH shared key */
    if(NuBL_ECCGenerateECDHKey() != 0)
    {
        ret = -2002;
        goto _exit_NuBL2_GetNuBL3xECDHKeys;
    }

    /* Get ECDH shared key */
    memcpy(Key32, (void *)(uint32_t)&CRPT->ECC_X1[0], sizeof(AESkey));
    NuBL2_BytesSwap((char *)Key32, sizeof(AESkey));
    for(i=0; i<8; i++)
        Key32[i] = NuBL_Swap32(Key32[i]);
    for(i=0; i<8; i++)
        NUBL_MSG("\tSharedKey: 0x%08x. (NuBL2_priv * NuBL32_pub)\n", Key32[i]);


/* Step 3. Decrypt NuBL33 public key */
    /* Get encrypted NuBL33 public key from Key Storage */
    len = sizeof(ECC_PUBKEY_T);
    extern uint32_t g_NuBL33EnCryptPubKeyBase;  /* declared in LoadKeyStorage.s */
    base = (uint32_t)&g_NuBL33EnCryptPubKeyBase;  /* encrypted NuBL33 pub key address */
    keybuf = (uint32_t *)(uint32_t)&PubKey;
    for(i=0; i<(len/4); i++)
        keybuf[i] = FMC_Read(base + (i*4));

    /* Trigger decryption... */
    if(NuBL_AES256Decrypt(keybuf, keybuf, len, AESkey) != 0)
    {
        ret = -3001;
        goto _exit_NuBL2_GetNuBL3xECDHKeys;
    }

/* Step 4. Calculate (NuBL2 * NuBL33) ECDH key */
    /* Init ECC */
    if(NuBL_ECCInitCurve() != 0)
    {
        ret = -4001;
        goto _exit_NuBL2_GetNuBL3xECDHKeys;
    }

    /* Select NuBL2_priv * NuBL33_pub */
    SetECCRegisters(0x00020000, NULL, (uint32_t *)(uint32_t)&PubKey);

    /* Calculate ECDH shared key */
    if(NuBL_ECCGenerateECDHKey() != 0)
    {
        ret = -4002;
        goto _exit_NuBL2_GetNuBL3xECDHKeys;
    }

    /* Get ECDH shared key */
    memcpy(Key33, (void *)(uint32_t)&CRPT->ECC_X1[0], sizeof(AESkey));
    NuBL2_BytesSwap((char *)Key33, sizeof(AESkey));
    for(i=0; i<8; i++)
        Key33[i] = NuBL_Swap32(Key33[i]);
    for(i=0; i<8; i++)
        NUBL_MSG("\tSharedKey: 0x%08x. (NuBL2_priv * NuBL33_pub)\n", Key33[i]);

    ret = 0;

_exit_NuBL2_GetNuBL3xECDHKeys:
    SYS_ResetModule(CRPT_RST);

    memset(&AESkey, 0x0, sizeof(AESkey));
    memset(&PubKey, 0x0, sizeof(PubKey));

    return ret;
}

/*
    Update NuBL3x F/W info to specify destination address
*/
int32_t NuBL2_UpdateNuBL3xFwInfo(uint32_t *pFwInfo, uint32_t size, int32_t i32Mode, uint32_t dest)
{
    (void)dest;
    volatile int32_t    ret = -1000;
    uint32_t            base;
    volatile uint32_t   i;

    if((i32Mode == -1) || (pFwInfo == NULL))
        return -1;

    if((ret=(int32_t)IsAccessDenied()) != 0)
        return ret;

    NUBL_MSG("\nNuBL2 update NuBL3%d F/W info.\n\n", (i32Mode==0)?2:3);

    if(i32Mode == 0)
        base = NUBL32_FW_INFO_BASE;
    else
        base = NUBL33_FW_INFO_BASE;

    for(i=0; i<(size+(FMC_FLASH_PAGE_SIZE-1))/FMC_FLASH_PAGE_SIZE; i++)
        FMC_Erase(base + (i*FMC_FLASH_PAGE_SIZE));

    NUBL_MSG("\tProgram data to 0x%x, and total %d bytes.\n", base, size);
    for(i=0; i<(size/4); i++)
    {
        FMC_Write(base + (i*4), pFwInfo[i]);
        if(FMC_Read(base + (i*4)) != pFwInfo[i])
        {
            NUBL_MSG("[FAIL] W: 0x%x, R: 0x%x on addr 0x%x.\n\n", pFwInfo[i], FMC_Read(base + (i*4)), base + (i*4));
            return -1001;
        }
    }
    NUBL_MSG("\t[Done]\n\n");

    return 0;
}

void NuBL2_BytesSwap(char *buf, int32_t len)
{
    int32_t i;
    char    tmp;

    for(i=0; i<(len/2); i++)
    {
        tmp = buf[len-i-1];
        buf[len-i-1] = buf[i];
        buf[i] = tmp;
    }
}

/*
    To verify ECDSA (R, S) signature
*/
int32_t NuBL2_VerifyNuBL3xECDSASignature(uint32_t *msg, ECDSA_SIGN_T *sign, ECC_PUBKEY_T *pubkey, int32_t mode)
{
    volatile int32_t    ret = 0;
    uint32_t    m[8], k0[8], k1[8], r[8], s[8];

    if(mode == -1)
        return ret;

    if((ret=(int32_t)IsAccessDenied()) != 0)
        return ret;

    NUBL_MSG("\nNuBL2 verify NuBL3%d FW info signature.\n", ((mode&BIT0)==0)?2:3);

    memcpy(&m,  &msg[0], sizeof(m));
    memcpy(&k0, (uint32_t *)&pubkey->au32Key0[0], sizeof(k0));
    memcpy(&k1, (uint32_t *)&pubkey->au32Key1[0], sizeof(k1));
    memcpy(&r,  &sign->au32R[0], sizeof(r));
    memcpy(&s,  &sign->au32S[0], sizeof(s));
    NuBL2_BytesSwap((char*)m,  sizeof(m));
    NuBL2_BytesSwap((char*)k0, sizeof(k0));
    NuBL2_BytesSwap((char*)k1, sizeof(k1));
    NuBL2_BytesSwap((char*)r,  sizeof(r));
    NuBL2_BytesSwap((char*)s,  sizeof(s));

    ret = NuBL_VerifyECCSignature(m, k0, k1, r, s);

    return ret;
}

uint16_t cmd_CalCRC16Sum(uint32_t *pu32buf, uint32_t len)
{
    volatile uint32_t   i;
    uint16_t            *pu16buf;

    if(len > 56) // data byte count
        return (uint16_t)-1;

    CLK->AHBCLK |= CLK_AHBCLK_CRCCKEN_Msk;
    CRC->SEED = 0xFFFFFFFFul;
    CRC->CTL = (CRC_16 | CRC_CPU_WDATA_16) | CRC_CTL_CRCEN_Msk;
    CRC->CTL |= CRC_CTL_CHKSINIT_Msk;

    pu16buf = (uint16_t *)pu32buf;
    CRC->DAT = *(pu16buf+1);
    CRC->DAT = *(pu16buf+2);
    CRC->DAT = *(pu16buf+3);

    for(i=0; i<(len/2); i++)
        CRC->DAT = *(pu16buf+4+i);

    *(pu16buf+0) = (CRC->CHECKSUM & 0xFFFF);

    /* Clear CRC checksum */
    CRC->SEED = 0xFFFFFFFFul;
    CRC->CTL |= CRC_CTL_CHKSINIT_Msk;

    return *(pu16buf+0);
}

int32_t cmd_VerifyCRC16Sum(uint32_t *pu32buf)
{
    volatile uint32_t   i, len;
    uint16_t            *pu16buf, ChkSum0, ChkSum1;

    pu16buf = (uint16_t *)pu32buf;
    len = pu32buf[1];
    if(len > 56) // data byte count
        return -1;

    CLK->AHBCLK |= CLK_AHBCLK_CRCCKEN_Msk;
    CRC->SEED = 0xFFFFFFFFul;
    CRC->CTL = (CRC_16 | CRC_CPU_WDATA_16) | CRC_CTL_CRCEN_Msk;
    CRC->CTL |= CRC_CTL_CHKSINIT_Msk;

    CRC->DAT = *(pu16buf+1);
    CRC->DAT = *(pu16buf+2);
    CRC->DAT = *(pu16buf+3);

    for(i=0; i<(len/2); i++)
        CRC->DAT = *(pu16buf+4+i);

    ChkSum0 = *(pu16buf+0);
    ChkSum1 = (CRC->CHECKSUM & 0xFFFF);

    /* Clear CRC checksum */
    CRC->SEED = 0xFFFFFFFFul;
    CRC->CTL |= CRC_CTL_CHKSINIT_Msk;

    /* Verify CRC16 checksum */
    if (ChkSum0 == ChkSum1)
        return 0;       /* Verify CRC16 Pass */
    else
        return ChkSum1; /* Verify CRC16 Fail */
}

int32_t GenCmdSessionKey(uint32_t key[])
{
    volatile int32_t    ret = -1000;
    uint32_t            *keybuf, base, len, start, end;
    ECC_PUBKEY_T        PubKey;
    uint32_t            AESkey[8], Hash[8], HashValue[8];
    volatile uint32_t    i;

    memset(&AESkey, 0x0, sizeof(AESkey));

    /* Check Host Key Storage Hash */
    /* Get encrypted Host public key from Key Storage */
    len = sizeof(ECC_PUBKEY_T);
    extern uint32_t g_HostEnCryptPubKeyBase;  /* declared in LoadHostKey.s */
    base = (uint32_t)&g_HostEnCryptPubKeyBase;  /* encrypted Host pub key address */
    keybuf = (uint32_t *)(uint32_t)&PubKey;
    for(i=0; i<(len/4); i++)
        keybuf[i] = inpw(base + (i*4));

    start = (uint32_t )&PubKey;
    end   = (uint32_t )&PubKey+len;
    NuBL_CalculateSHA256(start, end, (uint32_t *)Hash, SHA_ONESHOT, SHA_SRC_SRAM);

    /* Get Hash value of encrypted Host public key from Key Storage Hash base */
    len = sizeof(ECC_PUBKEY_T);
    extern uint32_t g_HostEnCryptPubKeyHashBase;  /* declared in LoadKeyStorage.s */
    base = (uint32_t)&g_HostEnCryptPubKeyHashBase;  /* encrypted Host pub key hash address */
    start = base;
    end   = base + sizeof(Hash);
    keybuf = (uint32_t *)&HashValue;
    for(i=0; i<(sizeof(HashValue)/4); i++)
        keybuf[i] = inpw(base + (i*4));

    if(memcmp(&Hash[0], &HashValue[0], (HASH_SIZE/8)) != 0)
    {
        uint32_t *tmp;
        tmp = (uint32_t *)&Hash;
        NUBL_MSG("Verify Host pub key Hash [FAIL]\n\n");
        NUBL_MSG("Key Hash:\n");
        for(i=0; i<((HASH_SIZE/8)/4); i++)
            NUBL_MSG("\t%d: 0x%08x - 0x%08x.\n", i, Hash[i], HashValue[i]);
        ret = -5001;
        goto _exit_GenCmdSessionKey;
    }
    NUBL_MSG("Verify Host Key Storage Hash [PASS]\n\n");

    
    GetNuBL2AES256Key(AESkey);

    /* Get encrypted Host public key from Key Storage */
    len = sizeof(ECC_PUBKEY_T);
    base = (uint32_t)&g_HostEnCryptPubKeyBase;  /* encrypted Host pub key address */

    keybuf = (uint32_t *)(uint32_t)&PubKey;
    for(i=0; i<(len/4); i++)
        keybuf[i] = FMC_Read(base + (i*4));

    /* Trigger decryption... */
    if(NuBL_AES256Decrypt(keybuf, keybuf, len, AESkey) != 0)
    {
        ret = -1003;
        goto _exit_GenCmdSessionKey;
    }


/* Step 2. Calculate (NuBL2 * Host) ECDH key */
    /* Init ECC */
    if(NuBL_ECCInitCurve() != 0)
    {
        ret = -2001;
        goto _exit_GenCmdSessionKey;
    }

    /* Select NuBL2_priv * Host_pub */
    SetECCRegisters(0x00020000, NULL, (uint32_t *)(uint32_t)&PubKey);

    /* Calculate ECDH shared key */
    if(NuBL_ECCGenerateECDHKey() != 0)
    {
        ret = -2002;
        goto _exit_GenCmdSessionKey;
    }

    /* Get ECDH shared key for follows AES encrypt/decrypt */
    memcpy(key, (void *)(uint32_t)&CRPT->ECC_X1[0], sizeof(AESkey));
#if (DEBUG_CMD == 1)
    for(i=0; i<8; i++)
        NUBL_MSG("\tSharedKey: 0x%08x. (NuBL2_priv * Host_pub)\n", key[i]);
#endif

    ret = 0;

_exit_GenCmdSessionKey:
    SYS_ResetModule(CRPT_RST);

    memset(&AESkey, 0x0, sizeof(AESkey));
    memset(&PubKey, 0x0, sizeof(ECC_PUBKEY_T));

    return ret;
}

/*
    Identify public keys mached or mismatch
        i32Mode:
            0: p32Buf means public keys
            1: p32Buf means public key Hash
    Return:
        bit-0:
            0: No NuBL32 pubic key;
            1: NuBL32 public key match
        bit-1:
            0: No NuBL33 pubic key;
            1: NuBL33 public key match
        bit-2:
            0: No Host pubic key;
            1: Host public key match
*/
int32_t IdentifyPublicKey(uint32_t *p32Buf, int32_t i32Mode)
{
    volatile int32_t    ret = -1000, sts = 0;
    uint32_t            *keybuf, base, len, start, end;
    ECC_PUBKEY_T        PubKey, *pInPubKey;
    uint32_t            AESkey[8], Hash[8];
    volatile uint32_t    i;

    if(i32Mode == -1)
        return ret;

    memset(&AESkey, 0x0, sizeof(AESkey));
    memset(&Hash, 0x0, sizeof(Hash));
    pInPubKey = (ECC_PUBKEY_T *)p32Buf;

    GetNuBL2AES256Key(AESkey);

    /* decrypt NuBL32 public key */
    len = sizeof(ECC_PUBKEY_T);
    extern uint32_t g_NuBL32EnCryptPubKeyBase;  /* declared in LoadKeyStorage.s */
    base = (uint32_t)&g_NuBL32EnCryptPubKeyBase;  /* encrypted NuBL32 pub key address */

    keybuf = (uint32_t *)(uint32_t)&PubKey;
    for(i=0; i<(len/4); i++)
        keybuf[i] = FMC_Read(base + (i*4));
    /* Trigger decryption... */
    if(NuBL_AES256Decrypt(keybuf, keybuf, len, AESkey) != 0)
    {
        ret = -1003;
        goto _exit_IdentifyPublicKey;
    }
    if(i32Mode == 0)
    {
        /* Identify NuBL32 public key */
        if(memcmp(&PubKey.au32Key0[0], &pInPubKey->au32Key0[0], sizeof(ECC_PUBKEY_T)) == 0)
        {
            sts |= BIT0;
        }
    }
    else
    {
        start = (uint32_t )&PubKey.au32Key0[0];
        end   = start + len;
        NuBL_CalculateSHA256(start, end, (uint32_t *)&Hash[0], SHA_ONESHOT, SHA_SRC_SRAM);
        /* Identify NuBL32 public key Hash */
        if(memcmp(&Hash[0], &pInPubKey->au32Key0[0], sizeof(Hash)) == 0)
        {
            sts |= BIT0;
        }
    }

    /* decrypt NuBL33 public key */
    len = sizeof(ECC_PUBKEY_T);
    extern uint32_t g_NuBL33EnCryptPubKeyBase;  /* declared in LoadKeyStorage.s */
    base = (uint32_t)&g_NuBL33EnCryptPubKeyBase;  /* encrypted NuBL33 pub key address */

    keybuf = (uint32_t *)(uint32_t)&PubKey;
    for(i=0; i<(len/4); i++)
        keybuf[i] = FMC_Read(base + (i*4));
    /* Trigger decryption... */
    if(NuBL_AES256Decrypt(keybuf, keybuf, len, AESkey) != 0)
    {
        ret = -1003;
        goto _exit_IdentifyPublicKey;
    }
    if(i32Mode == 0)
    {
        /* Identify NuBL33 public key */
        if(memcmp(&PubKey.au32Key0[0], &pInPubKey->au32Key0[0], sizeof(ECC_PUBKEY_T)) == 0)
        {
            sts |= BIT1;
        }
    }
    else
    {
        start = (uint32_t )&PubKey.au32Key0[0];
        end   = start + len;
        NuBL_CalculateSHA256(start, end, (uint32_t *)&Hash[0], SHA_ONESHOT, SHA_SRC_SRAM);
        /* Identify NuBL33 public key Hash */
        if(memcmp(&Hash[0], &pInPubKey->au32Key0[0], sizeof(Hash)) == 0)
        {
            sts |= BIT1;
        }
    }

    /* decrypt Host public key */
    len = sizeof(ECC_PUBKEY_T);
    extern uint32_t g_HostEnCryptPubKeyBase;  /* declared in LoadHostKey.s */
    base = (uint32_t)&g_HostEnCryptPubKeyBase;  /* encrypted Host pub key address */

    keybuf = (uint32_t *)(uint32_t)&PubKey;
    for(i=0; i<(len/4); i++)
        keybuf[i] = FMC_Read(base + (i*4));
    /* Trigger decryption... */
    if(NuBL_AES256Decrypt(keybuf, keybuf, len, AESkey) != 0)
    {
        ret = -1003;
        goto _exit_IdentifyPublicKey;
    }
    if(i32Mode == 0)
    {
        /* Identify Host public key */
        if(memcmp(&PubKey.au32Key0[0], &pInPubKey->au32Key0[0], sizeof(ECC_PUBKEY_T)) == 0)
        {
            sts |= BIT2;
        }
    }
    else
    {
        start = (uint32_t )&PubKey.au32Key0[0];
        end   = start + len;
        NuBL_CalculateSHA256(start, end, (uint32_t *)&Hash[0], SHA_ONESHOT, SHA_SRC_SRAM);
        /* Identify Host public key Hash */
        if(memcmp(&Hash[0], &pInPubKey->au32Key0[0], sizeof(Hash)) == 0)
        {
            sts |= BIT2;
        }
    }

    ret = sts;

_exit_IdentifyPublicKey:
    SYS_ResetModule(CRPT_RST);

    memset(&AESkey, 0x0, sizeof(AESkey));
    memset(&PubKey, 0x0, sizeof(ECC_PUBKEY_T));
    memset(&Hash, 0x0, sizeof(Hash));

    return ret;
}

/**
  * @brief      Perform NuBL2 to verify the hash value of Host public key
  * @param[in]  u32KeyHash  key hash of HOST
  * @retval     1,2,3       Success, Bit0:NuBL32, Bit1:NuBL33, Bit1|Bit0:both NuBL32 and NuBL33
  * @retval     -1      Failed
  * @details    This function is used to perform identify, authenticate and verify F/W integrity of NuBL32 or NuBL33. \n
  *             Flow: \n
  *                 1. Decrypt(use NuBL2 AES Key )Key Storage to get NuBL32x public key \n
  *                 2. Decrypt(use NuBL2&NuBL3x ECDH key) NuBL3x info \n
  *                 3. Identify NuBL3x public key \n
  */
int32_t VerifyNuBL3xKeyHash(uint32_t * pu32KeyHash)
{
    volatile int32_t    ret = -1000;
    uint32_t            *keybuf, *infobuf, base, len, start, end;
    ECC_PUBKEY_T        PubKey;
    FW_INFO_T           FwInfo;
    uint32_t            AESkey[8];
    uint32_t            au32HashBuf[8];
    volatile uint32_t    i;

    NUBL_MSG("\nNuBL2 verify NuBL3x.\n\n");

    memset(&AESkey, 0x0, sizeof(AESkey));
    memset(&PubKey, 0x0, sizeof(ECC_PUBKEY_T));
    memset(&FwInfo, 0x0, sizeof(FW_INFO_T));

    infobuf = (uint32_t *)(uint32_t)&FwInfo;

    /* reset crypto */
    SYS->IPRST0 |= SYS_IPRST0_CRPTRST_Msk;
    SYS->IPRST0 &= ~SYS_IPRST0_CRPTRST_Msk;
    NVIC_DisableIRQ(CRPT_IRQn);

    GetNuBL2AES256Key(AESkey);
/* Step 1. Decrypt NuBL3x public key */
    for(i=0; i<8; i++)
        NUBL_MSG("\tSharedKey: 0x%08x. (NuBL2_priv * NuBL1_pub)\n", AESkey[i]);

    /* Get encrypted NuBL3x public key from Key Storage */
    len = sizeof(ECC_PUBKEY_T);
    extern uint32_t g_NuBL32EnCryptPubKeyBase;  /* declared in LoadKeyStorage.s */
    base = (uint32_t)&g_NuBL32EnCryptPubKeyBase;  /* encrypted NuBL32 pub key address */

    keybuf = (uint32_t *)(uint32_t)&PubKey;
    for(i=0; i<(len/4); i++)
        keybuf[i] = FMC_Read(base + (i*4));

    /* Trigger decryption... */
    if(NuBL_AES256Decrypt(keybuf, keybuf, len, AESkey) != 0)
    {
        ret = -1003;
        goto _exit_NuBL2_ExecuteVerifyNuBL3x;
    }
    NUBL_MSG("Decrypt NuBL32 public key [Done]\n\n");

    start = (uint32_t )&keybuf[0];
    end   = start + len;
    NuBL_CalculateSHA256(start, end, (uint32_t *)&au32HashBuf[0], SHA_ONESHOT, SHA_SRC_SRAM);

    /* verify NuBL32 key hash */
    if(memcmp(&au32HashBuf, pu32KeyHash, sizeof(au32HashBuf)) == 0)
    {
        NUBL_MSG("Identify NuBL32 public key [PASS]\n\n");
        ret = BIT0;
        goto _exit_NuBL2_ExecuteVerifyNuBL3x;
    }

    extern uint32_t g_NuBL33EnCryptPubKeyBase;  /* declared in LoadKeyStorage.s */
    base = (uint32_t)&g_NuBL33EnCryptPubKeyBase;  /* encrypted NuBL33 pub key address */

    keybuf = (uint32_t *)(uint32_t)&PubKey;
    for(i=0; i<(len/4); i++)
        keybuf[i] = FMC_Read(base + (i*4));

    /* Trigger decryption... */
    if(NuBL_AES256Decrypt(keybuf, keybuf, len, AESkey) != 0)
    {
        ret = -1003;
        goto _exit_NuBL2_ExecuteVerifyNuBL3x;
    }
    NUBL_MSG("Decrypt NuBL32 public key [Done]\n\n");

    NuBL_CalculateSHA256(start, end, (uint32_t *)&au32HashBuf[0], SHA_ONESHOT, SHA_SRC_SRAM);

    /* verify BL33 key hash */
    if(memcmp(&au32HashBuf, pu32KeyHash, sizeof(au32HashBuf)) == 0)
    {
        NUBL_MSG("Identify NuBL33 public key [PASS]\n\n");
        ret = BIT1;
        goto _exit_NuBL2_ExecuteVerifyNuBL3x;
    }

    ret = 0;

_exit_NuBL2_ExecuteVerifyNuBL3x:
    SYS_ResetModule(CRPT_RST);

    memset(&AESkey, 0x0, sizeof(AESkey));
    memset(&PubKey, 0x0, sizeof(ECC_PUBKEY_T));
    memset(&FwInfo, 0x0, sizeof(FW_INFO_T));

    return ret;
}

/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/
