/**************************************************************************//**
 * @file     NuBL_crypto.h
 * @version  V1.00
 * @brief    NuBL crypto header file
 *
 * @copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __NuBL_CRYPTO_H__
#define __NuBL_CRYPTO_H__

#ifdef __cplusplus
extern "C"
{
#endif
    

/*  Constant definitions */
#define HASH_SIZE           256 /* bits */

    
/* typedef enum declaration */
typedef enum
{
    SHA_ONESHOT     = 0,    /* One shop SHA encrypt */
    SHA_CONTI_START,        /* Start continuous SHA encrypt */
    SHA_CONTI_ING,          /* Continuous SHA encrypt of SHA_CONTI_START */
    SHA_CONTI_END,          /* Last SHA encrypt of SHA_CONTI_START */
} E_SHA_OP_MODE;

typedef enum
{
    SHA_SRC_SRAM    = 0,
    SHA_SRC_FLASH,
} E_SHA_SRC;

/* For ECC NIST: Curve P-256. Declared in NuBL2_main.c */
extern const uint32_t g_au32Eorder[];

/* Function declaration */
/**
  * @brief      Open SHA encrypt function
  * @param[in]  start       SHA encrypt start address
  * @param[in]  end         SHA encrypt end address
  * @param[out] digest      The SHA encrypt output digest
  * @param[in]  mode        SHA operation mode, including:
  *                             - \ref SHA_ONESHOT
  *                             - \ref SHA_CONTI_START
  *                             - \ref SHA_CONTI_ING
  *                             - \ref SHA_CONTI_END
  * @retval     0           Success
  * @retval     -1          Failed
  */
int32_t NuBL_CalculateSHA256(uint32_t start, uint32_t end, uint32_t digest[], E_SHA_OP_MODE mode, E_SHA_SRC src);

/**
  * @brief      Initial ECC Curve P-256
  */
int32_t NuBL_ECCInitCurve(void);

/**
  * @brief      Generate ECDH key
  */
int32_t NuBL_ECCGenerateECDHKey(void);

/**
  * @brief      Perform AES-256 CFB NoPadding decrypt
  */
int32_t NuBL_AES256Decrypt(uint32_t *in, uint32_t *out, uint32_t len, uint32_t *key);

/*
    To verify ECDSA (R, S) signature
*/
int32_t NuBL_VerifyECCSignature(uint32_t *msg, uint32_t *Qx, uint32_t *Qy, uint32_t *R, uint32_t *S);

#ifdef __cplusplus
}
#endif

#endif /* __NuBL_CRYPTO_H__ */

/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/
