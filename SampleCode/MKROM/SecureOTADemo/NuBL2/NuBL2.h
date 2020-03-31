/**************************************************************************//**
 * @file     NuBL2.h
 * @version  V1.00
 * @brief    NuBL2 header file
 *
 * @copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __NUBL2_H__
#define __NUBL2_H__

#ifdef __cplusplus
extern "C"
{
#endif

extern volatile uint32_t gNuBL2_32Key[8], gNuBL2_33Key[8];
extern const uint32_t g_InitialFWinfo[];

/* Function declaration */
int32_t NuBL2_Init(void);
int32_t NuBL2_DecryptNuBL2FwInfo(uint32_t *buf, uint32_t *len, int32_t mode);
int32_t NuBL2_ExecuteVerifyNuBL3x(uint32_t *buf, int32_t mode);
int32_t NuBL2_GetNuBL3xECDHKeys(uint32_t Key32[], uint32_t Key33[]);
int32_t NuBL2_UpdateNuBL3xFwInfo(uint32_t *pFwInfo, uint32_t size, int32_t mode, uint32_t dest);

int32_t NuBL2_VerifyNuBL3xECDSASignature(uint32_t *msg, ECDSA_SIGN_T *sign, ECC_PUBKEY_T *pubkey, int32_t mode);

int32_t VerifyNuBL3xKeyHash(uint32_t * pu32KeyHash);

#ifdef __cplusplus
}
#endif

#endif /* __NUBL2_H__ */

/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/
