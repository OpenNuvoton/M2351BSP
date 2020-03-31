/**************************************************************************//**
 * @file     XOMRegion.c
 * @version  V3.00
 * @brief    Provide APIs and store private keys in XOM0 region.
 *
 * @copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

#define printf(...)


//#define ECC_CURVE_TYPE      CURVE_P_256
//#define ECC_KEY_SIZE        256     /* Select ECC P-256 curve, 256-bits key length */

#if defined ( __ICCARM__ )
#pragma default_function_attributes = __root
#endif

int32_t XOM_CalIDsECDSA(char *pOutR, char *pOutS);

static uint8_t Byte2Char(uint8_t c)
{
    if(c < 10)
        return (c + '0');
    if(c < 16)
        return (c - 10 + 'a');

    return 0;
}

static void SetPrivKey(char *d)
{
    uint32_t au32PrivKey[8];
    
    //const char g_acPub0[] = "755b3819f05a3e9f32d4d599062834aac5220f75955378414a8f63716a152ce2";
    //const char g_acPub1[] = "91c413f1915ed7b47473fd797647ba3d83e8224377909af5b30c530eaad79fd7";
    //const char g_acPriv[] = "380a67fcfc01ca7073da7c2c54296a61327f77262a7d4674c3d8e29a63e3fa20";
    // Set private key
    au32PrivKey[0] = 0x63e3fa20;
    au32PrivKey[1] = 0xc3d8e29a;
    au32PrivKey[2] = 0x2a7d4674;
    au32PrivKey[3] = 0x327f7726;
    au32PrivKey[4] = 0x54296a61;
    au32PrivKey[5] = 0x73da7c2c;
    au32PrivKey[6] = 0xfc01ca70;
    au32PrivKey[7] = 0x380a67fc;
    
    // Convert register value to HEX(ASCII) for XECC API
    XECC_Reg2Hex(64, au32PrivKey, d);
    
    memset((void *)au32PrivKey, 0x0, sizeof(au32PrivKey));
}

static void SetMessage(char *m)
{
    uint32_t au32Message[8];
    
    BL_EnableFMC();
    
    au32Message[0] = BL_ReadUCID(3);
    au32Message[1] = BL_ReadUCID(2);
    au32Message[2] = BL_ReadUCID(1);
    au32Message[3] = BL_ReadUCID(0);
    au32Message[4] = BL_ReadUID(2);
    au32Message[5] = BL_ReadUID(1);
    au32Message[6] = BL_ReadUID(0);
    au32Message[7] = SYS->PDID;
    
    // Convert register value to HEX(ASCII) for XECC API
    XECC_Reg2Hex(64, au32Message, m);
}

int32_t XOM_CalIDsECDSA(char *pOutR, char *pOutS)
{
    char    d[70], k[70], m[70];
    uint32_t i, j, u32NBits;
    BL_RNG_T rng;
    uint8_t au8r[32]; // (ECC_KEY_SIZE/8)
    
    memset(d, 0x0, sizeof(d));
    memset(k, 0x0, sizeof(k));
    memset(m, 0x0, sizeof(k));
    
    CLK_EnableModuleClock(CRPT_MODULE);

    ECC_ENABLE_INT(CRPT);
    
    u32NBits = 256; //ECC_KEY_SIZE;

    /* Initial TRNG */
    BL_RandomInit(&rng, BL_RNG_PRNG | BL_RNG_LIRC32K);

    do
    {
        /* Generate random number for private key */
        BL_Random(&rng, au8r, u32NBits / 8);

        for(i = 0, j = 0; i < u32NBits / 8; i++)
        {
            k[j++] = Byte2Char(au8r[i] & 0xf);
            k[j++] = Byte2Char(au8r[i] >> 4);
        }
        k[j] = 0; // NULL end

        /* Check if the private key valid */
        if(XECC_IsPrivateKeyValid(XCRPT, CURVE_P_256, k))
        {
            //printf("Private key check ok\n");
            break;
        }
        else
        {
            /* Invalid key */
            //printf("Current private key is not valid. Need a new one.\n");
        }
    }
    while(1);        
    printf("k: %s\n", k);
    
    SetPrivKey(d);
    printf("d: %s\n", d);
    
    SetMessage(m);
    printf("m: %s\n", m);
    
    if(XECC_GenerateSignature(XCRPT, CURVE_P_256, m, d, k, pOutR, pOutS) < 0)
    {
        memset(d, 0x0, sizeof(d));
        memset(k, 0x0, sizeof(k));
        memset(m, 0x0, sizeof(k));
        printf("ECC signature generation failed!!\n");
        return -1;
    }
    
    memset(d, 0x0, sizeof(d));
    memset(k, 0x0, sizeof(k));
    memset(m, 0x0, sizeof(k));
    return 0;
}

#if defined ( __ICCARM__ )
#pragma default_function_attributes = 
#endif

/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/
