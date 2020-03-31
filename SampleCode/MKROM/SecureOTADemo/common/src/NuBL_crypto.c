/**************************************************************************//**
 * @file     NuBL_crypto.c
 * @version  V1.00
 * @brief    NuBL crypto source file
 *
 * @copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

#include "NuBL_common.h"
#include "NuBL_crypto.h"

#define printf(...)
#if 0
static uint32_t Swap32(uint32_t val)
{
    return (val<<24) | ((val<<8)&0xff0000) | ((val>>8)&0xff00) | (val>>24);
}
#endif
static void ResetCrypto(void)
{
    SYS->IPRST0 |= SYS_IPRST0_CRPTRST_Msk;
    SYS->IPRST0 &= ~SYS_IPRST0_CRPTRST_Msk;
    NVIC_DisableIRQ(CRPT_IRQn);
}

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
int32_t NuBL_CalculateSHA256(uint32_t start, uint32_t end, uint32_t digest[], E_SHA_OP_MODE mode, E_SHA_SRC src)
{
    volatile int32_t    i, bytes;
    uint32_t            *ptr, addr, data = 0;

    bytes   = (int32_t)(end - start);
    ptr     = (uint32_t *)start;
    addr    = (uint32_t)ptr;

    if((mode == SHA_ONESHOT) || (mode == SHA_CONTI_START))
    {
        NUBL_MSG("\n[Start SHA256 from 0x%x to 0x%x. (size: %d) (mode: %d)] (%s)\n", start, end, bytes, mode, (src==SHA_SRC_SRAM)?"SRAM":"Flash");
        /* Reset CRYPTO module */
        ResetCrypto();

        /* CRYPTO */
        CLK->AHBCLK |= CLK_AHBCLK_CRPTCKEN_Msk;

        CRPT->HMAC_CTL = (SHA_MODE_SHA256 << CRPT_HMAC_CTL_OPMODE_Pos) | CRPT_HMAC_CTL_INSWAP_Msk | CRPT_HMAC_CTL_OUTSWAP_Msk;
        CRPT->HMAC_DMACNT = 64;
        CRPT->HMAC_CTL |= CRPT_HMAC_CTL_START_Msk;
    }
    else
    {
        NUBL_MSG("[Continue SHA256 from 0x%x to 0x%x. (size: %d) (mode: %d)]\n", start, end, bytes, mode);
    }

    /* Start to calculate ... */
    while(bytes > 0)
    {
        if(bytes < 64)
            CRPT->HMAC_DMACNT = (uint32_t)bytes;

        if(CRPT->HMAC_STS & CRPT_HMAC_STS_DATINREQ_Msk)
        {
            if(src == SHA_SRC_SRAM)
            {
                data = *ptr++;
                bytes -= 4;
            }
            else if(src == SHA_SRC_FLASH)
            {
                data = FMC_Read(addr&0x0FFFFFFF);
                addr += 4;
                bytes -= 4;
            }
            if(bytes <= 0)
                bytes = 0;

            /* bytes means remain byte counts */
            if(bytes != 0)
            {
                CRPT->HMAC_DATIN = data;
            }
            else
            {
                if((mode == SHA_ONESHOT) || (mode == SHA_CONTI_END)) /* The last SHA operation */
                {
                    /* It's last word ... *-* */
                    CRPT->HMAC_CTL |= CRPT_HMAC_CTL_START_Msk | CRPT_HMAC_CTL_DMALAST_Msk;
                    CRPT->HMAC_DATIN = data;
                    while(CRPT->HMAC_STS & CRPT_HMAC_STS_BUSY_Msk);

                    for(i=0; i<8; i++)
                        digest[i] = *(uint32_t *)((uint32_t)&(CRPT->HMAC_DGST[0]) + (uint32_t)(i*4));

                    NUBL_MSG("\t[SHA256 done]\n");

                    /* Reset CRYPTO module */
                    ResetCrypto();
                    break;
                }
                else
                {
                    CRPT->HMAC_DATIN = data;
                    /* Exit to wait next SHA operation */
                }
            }
        }
    }

    return 0;
/*
    Verify:
        Input:
            {0x30313233,0x34353637} or String "32107654"
        Result:
            6952CF8EACE972CD4F10567331B46D85104E9E57402364F205876D13F84F7E42
            ==> Arraty {0x6952CF8E, 0xACE972CD, 0x4F105673....}
*/
}

/**
  * @brief      Perform AES-256 CFB NoPadding decrypt
  */
int32_t NuBL_AES256Decrypt(uint32_t *in, uint32_t *out, uint32_t len, uint32_t *KEY)
{
    uint32_t au32AESIV[4] = {0};

    /* reset crypto */
    SYS->IPRST0 |= SYS_IPRST0_CRPTRST_Msk;
    SYS->IPRST0 &= ~SYS_IPRST0_CRPTRST_Msk;
    NVIC_DisableIRQ(CRPT_IRQn);

    CLK->AHBCLK |= CLK_AHBCLK_CRPTCKEN_Msk;

    CLK->AHBCLK |= CLK_AHBCLK_CRPTCKEN_Msk;

    /* KEY and IV are byte order (32 bit) reversed, __REV(x)) and stored in ISP_INFO_T */
    memcpy((void *)(uint32_t)&CRPT->AES0_KEY[0], KEY, (4 * 8));
    memcpy((void *)(uint32_t)&CRPT->AES0_IV[0], au32AESIV, (4 * 4));

    CRPT->AES0_SADDR = (uint32_t)in;
    CRPT->AES0_DADDR = (uint32_t)out;
    CRPT->AES0_CNT   = len;
    CRPT->AES_CTL = ((AES_KEY_SIZE_256 << CRPT_AES_CTL_KEYSZ_Pos) | (AES_IN_OUT_SWAP << CRPT_AES_CTL_OUTSWAP_Pos));
    CRPT->AES_CTL |= ((AES_MODE_CFB << CRPT_AES_CTL_OPMODE_Pos) | CRPT_AES_CTL_START_Msk | CRPT_AES_CTL_DMAEN_Msk);
//    CRPT->AES_CTL |= ((AES_MODE_ECB << CRPT_AES_CTL_OPMODE_Pos) | CRPT_AES_CTL_START_Msk | CRPT_AES_CTL_DMAEN_Msk);
    while(CRPT->AES_STS & CRPT_AES_STS_BUSY_Msk) {}

    return 0;
}


/*-----------------------------------------------------------------------------------------------*/
/*                                                                                               */
/*    ECC                                                                                        */
/*                                                                                               */
/*-----------------------------------------------------------------------------------------------*/
/* For ECC NIST: Curve P-256. Declared in NuBL2_main.c */
extern const uint32_t g_au32Eorder[];

#define ECC_CURVE_TYPE      CURVE_P_256
#define CURVE_P_SIZE        CURVE_P_256
#define ECC_KEY_SIZE        256 /* bits */

#define ECCOP_POINT_MUL     (0x0UL << CRPT_ECC_CTL_ECCOP_Pos)
#define ECCOP_MODULE        (0x1UL << CRPT_ECC_CTL_ECCOP_Pos)
#define ECCOP_POINT_ADD     (0x2UL << CRPT_ECC_CTL_ECCOP_Pos)
#define ECCOP_POINT_DOUBLE  (0x0UL << CRPT_ECC_CTL_ECCOP_Pos)

#define MODOP_DIV           (0x0UL << CRPT_ECC_CTL_MODOP_Pos)
#define MODOP_MUL           (0x1UL << CRPT_ECC_CTL_MODOP_Pos)
#define MODOP_ADD           (0x2UL << CRPT_ECC_CTL_MODOP_Pos)
#define MODOP_SUB           (0x3UL << CRPT_ECC_CTL_MODOP_Pos)

#define B2C(c)              (((uint8_t)(c)<10)? ((uint8_t)(c)+'0'):((uint8_t)(c)-10+'a'))
#if 0
static char ch2hex(char ch)
{
    if(ch <= '9')
    {
        return ch - '0';
    }
    else if((ch <= 'z') && (ch >= 'a'))
    {
        return ch - 'a' + 10U;
    }
    else
    {
        return ch - 'A' + 10U;
    }
}

static void Hex2Reg(char input[], uint32_t volatile reg[])
{
    char      hex;
    int       si, ri;
    uint32_t  i, val32;

    si = (int)strlen(input) - 1;
    ri = 0;

    while(si >= 0)
    {
        val32 = 0UL;
        for(i = 0UL; (i < 8UL) && (si >= 0); i++)
        {
            hex = ch2hex(input[si]);
            val32 |= (uint32_t)hex << (i * 4UL);
            si--;
        }
        reg[ri++] = val32;
    }
}

static int  get_nibble_value(char c)
{
    char ch;

    if((c >= '0') && (c <= '9'))
    {
        ch = '0';
        return ((int)c - (int)ch);
    }

    if((c >= 'a') && (c <= 'f'))
    {
        ch = 'a';
        return ((int)c - (int)ch - 10);
    }

    if((c >= 'A') && (c <= 'F'))
    {
        ch = 'A';
        return ((int)c - (int)ch - 10);
    }
    return 0;
}
#endif
static int32_t Trigger_ECC(uint32_t mode)
{
    volatile uint32_t i = 1000000;

    if((mode & CRPT_ECC_CTL_ECCOP_Msk) == ECCOP_MODULE)
    {
        CRPT->ECC_CTL = CRPT_ECC_CTL_FSEL_Msk;
    }
    else
    {
        /* CURVE_GF_P */
        CRPT->ECC_CTL = CRPT_ECC_CTL_FSEL_Msk;
    }

    CRPT->ECC_CTL |= (ECC_KEY_SIZE << CRPT_ECC_CTL_CURVEM_Pos) | (mode) | CRPT_ECC_CTL_START_Msk;
    while(CRPT->ECC_STS & CRPT_ECC_STS_BUSY_Msk)
    {
        if(i-- == 0)
        {
            return -1;
        }
    }

    return 0;
}

/**
  * @brief      Initial ECC Curve P-256
  */
int32_t NuBL_ECCInitCurve(void)
{
    volatile int32_t i, ret = 0;

    /* Reset CRYPTO module */
    ResetCrypto();

    CLK->AHBCLK |= CLK_AHBCLK_CRPTCKEN_Msk;
    ECC_ENABLE_INT(CRPT); /* Enable CRYPTO function */

    /* Use ECC polling mode */
    NVIC_DisableIRQ(CRPT_IRQn);

    for(i=0; i<18; i++)
    {
        CRPT->ECC_A[i]  = 0UL;
        CRPT->ECC_B[i]  = 0UL;
        CRPT->ECC_X1[i] = 0UL;
        CRPT->ECC_Y1[i] = 0UL;
        CRPT->ECC_N[i]  = 0UL;
        CRPT->ECC_K[i]  = 0UL;
    }

    //char  Ea[144];
    //char  Eb[144];
    //char  Px[144];
    //char  Py[144];
        //"FFFFFFFF-00000001-00000000-00000000-00000000-FFFFFFFF-FFFFFFFF-FFFFFFFC",  /* "0000000000000000000000000000000000000000000000000000000000000003" */
    CRPT->ECC_A[0] = 0xFFFFFFFC;
    CRPT->ECC_A[1] = 0xFFFFFFFF;
    CRPT->ECC_A[2] = 0xFFFFFFFF;
    CRPT->ECC_A[3] = 0x00000000;
    CRPT->ECC_A[4] = 0x00000000;
    CRPT->ECC_A[5] = 0x00000000;
    CRPT->ECC_A[6] = 0x00000001;
    CRPT->ECC_A[7] = 0xFFFFFFFF;
        //"5ac635d8-aa3a93e7-b3ebbd55-769886bc-651d06b0-cc53b0f6-3bce3c3e-27d2604b",
    CRPT->ECC_B[0] = 0x27d2604b;
    CRPT->ECC_B[1] = 0x3bce3c3e;
    CRPT->ECC_B[2] = 0xcc53b0f6;
    CRPT->ECC_B[3] = 0x651d06b0;
    CRPT->ECC_B[4] = 0x769886bc;
    CRPT->ECC_B[5] = 0xb3ebbd55;
    CRPT->ECC_B[6] = 0xaa3a93e7;
    CRPT->ECC_B[7] = 0x5ac635d8;
        //"6b17d1f2-e12c4247-f8bce6e5-63a440f2-77037d81-2deb33a0-f4a13945-d898c296",
    CRPT->ECC_X1[0] = 0xd898c296;
    CRPT->ECC_X1[1] = 0xf4a13945;
    CRPT->ECC_X1[2] = 0x2deb33a0;
    CRPT->ECC_X1[3] = 0x77037d81;
    CRPT->ECC_X1[4] = 0x63a440f2;
    CRPT->ECC_X1[5] = 0xf8bce6e5;
    CRPT->ECC_X1[6] = 0xe12c4247;
    CRPT->ECC_X1[7] = 0x6b17d1f2;
        //"4fe342e2-fe1a7f9b-8ee7eb4a-7c0f9e16-2bce3357-6b315ece-cbb64068-37bf51f5",
    CRPT->ECC_Y1[0] = 0x37bf51f5;
    CRPT->ECC_Y1[1] = 0xcbb64068;
    CRPT->ECC_Y1[2] = 0x6b315ece;
    CRPT->ECC_Y1[3] = 0x2bce3357;
    CRPT->ECC_Y1[4] = 0x7c0f9e16;
    CRPT->ECC_Y1[5] = 0x8ee7eb4a;
    CRPT->ECC_Y1[6] = 0xfe1a7f9b;
    CRPT->ECC_Y1[7] = 0x4fe342e2;

    /* pCurve->GF == (int)CURVE_GF_P */

    //char  Pp[176];
        //"FFFFFFFF-00000001-00000000-00000000-00000000-FFFFFFFF-FFFFFFFF-FFFFFFFF",  /* "115792089210356248762697446949407573530086143415290314195533631308867097853951" */
    CRPT->ECC_N[0] = 0xFFFFFFFF;
    CRPT->ECC_N[1] = 0xFFFFFFFF;
    CRPT->ECC_N[2] = 0xFFFFFFFF;
    CRPT->ECC_N[3] = 0x00000000;
    CRPT->ECC_N[4] = 0x00000000;
    CRPT->ECC_N[5] = 0x00000000;
    CRPT->ECC_N[6] = 0x00000001;
    CRPT->ECC_N[7] = 0xFFFFFFFF;

    return ret;
}

/**
  * @brief      Generate ECDH key
  */
int32_t NuBL_ECCGenerateECDHKey(void)
{
    /* set FSEL (Field selection) */
    CRPT->ECC_CTL = CRPT_ECC_CTL_FSEL_Msk; /* CURVE_GF_P */

    ECC_CLR_INT_FLAG(CRPT);
    CRPT->ECC_CTL |= (ECC_KEY_SIZE << CRPT_ECC_CTL_CURVEM_Pos) |
                     (0x0UL << CRPT_ECC_CTL_ECCOP_Pos) | CRPT_ECC_CTL_START_Msk;

    while(1)
    {
        if(ECC_GET_INT_FLAG(CRPT) != 0x0)
        {
            if((ECC_GET_INT_FLAG(CRPT)&CRPT_INTSTS_ECCEIF_Msk) == CRPT_INTSTS_ECCEIF_Msk)
            {
                NVIC_DisableIRQ(CRPT_IRQn);
                ECC_CLR_INT_FLAG(CRPT);
                return -1;
            }
            break;
        }
    }

    ECC_CLR_INT_FLAG(CRPT);

    return 0;
}

/*
    To verify ECDSA (R, S) signature
*/
int32_t NuBL_VerifyECCSignature(uint32_t *msg, uint32_t *Qx, uint32_t *Qy, uint32_t *R, uint32_t *S)
{
    volatile int32_t    i;
    volatile uint32_t   temp_result1[8], temp_result2[8];
    volatile uint32_t   temp_x[8], temp_y[8];

    /*
     *   1. Verify that r and s are integers in the interval [1, n-1]. If not, the signature is invalid
     *   2. Compute e = HASH (m), where HASH is the hashing algorithm in signature generation
     *      (1) Use SHA to calculate e
     */

    /*
     *   3. Compute w = s^-1 (mod n)
     *      (1) Write the curve order to N registers
     *      (2) Write 0x1 to Y1 registers
     *      (3) Write s to X1 registers
     *      (4) Set ECCOP(CRPT_ECC_CTL[10:9]) to 01
     *      (5) Set MOPOP(CRPT_ECC_CTL[12:11]) to 00
     *      (6) Set FSEL(CRPT_ECC_CTL[8]) according to used curve of prime field or binary field
     *      (7) Set START(CRPT_ECC_CTL[0]) to 1
     *      (8) Wait for BUSY(CRPT_ECC_STS[0]) be cleared
     *      (9) Read X1 registers to get w
     */

    /* Reset CRYPTO module */
    ResetCrypto();

    CLK->AHBCLK |= CLK_AHBCLK_CRPTCKEN_Msk;
    ECC_ENABLE_INT(CRPT); /* Enable CRYPTO function */

    /* Use ECC polling mode */
    NVIC_DisableIRQ(CRPT_IRQn);

    NuBL_ECCInitCurve();

    /*  3-(1) Write the curve order to N registers */
    for(i = 0; i < 8; i++)
    {
        CRPT->ECC_N[i] = g_au32Eorder[i];
    }

    /*  3-(2) Write 0x1 to Y1 registers */
    for(i = 0; i < 8; i++)
    {
        CRPT->ECC_Y1[i] = 0UL;
    }
    CRPT->ECC_Y1[0] = 0x1UL;

    /*  3-(3) Write s to X1 registers */
    CRPT->ECC_X1[0] = S[0];
    CRPT->ECC_X1[1] = S[1];
    CRPT->ECC_X1[2] = S[2];
    CRPT->ECC_X1[3] = S[3];
    CRPT->ECC_X1[4] = S[4];
    CRPT->ECC_X1[5] = S[5];
    CRPT->ECC_X1[6] = S[6];
    CRPT->ECC_X1[7] = S[7];

    if(Trigger_ECC(ECCOP_MODULE | MODOP_DIV) != 0)
    {
        ResetCrypto();
        return -1;
    }

    /*  3-(9) Read X1 registers to get w */
    for(i = 0; i < 8; i++)
    {
        temp_result2[i] = CRPT->ECC_X1[i];
    }

    /*
     *   4. Compute u1 = e ?w (mod n) and u2 = r ?w (mod n)
     *      (1) Write the curve order and curve length to N ,M registers
     *      (2) Write e, w to X1, Y1 registers
     *      (3) Set ECCOP(CRPT_ECC_CTL[10:9]) to 01
     *      (4) Set MOPOP(CRPT_ECC_CTL[12:11]) to 01
     *      (5) Set START(CRPT_ECC_CTL[0]) to 1
     *      (6) Wait for BUSY(CRPT_ECC_STS[0]) be cleared
     *      (7) Read X1 registers to get u1
     *      (8) Write the curve order and curve length to N ,M registers
     *      (9) Write r, w to X1, Y1 registers
     *      (10) Set ECCOP(CRPT_ECC_CTL[10:9]) to 01
     *      (11) Set MOPOP(CRPT_ECC_CTL[12:11]) to 01
     *      (12) Set START(CRPT_ECC_CTL[0]) to 1
     *      (13) Wait for BUSY(CRPT_ECC_STS[0]) be cleared
     *      (14) Read X1 registers to get u2
     */

    /*  4-(1) Write the curve order and curve length to N ,M registers */
    for(i = 0; i < 8; i++)
    {
        CRPT->ECC_N[i] = g_au32Eorder[i];
    }

    /* 4-(2) Write e, w to X1, Y1 registers */
    for(i = 0; i < 8; i++)
    {
        CRPT->ECC_X1[i] = 0UL;
    }
    CRPT->ECC_X1[0] = msg[0];
    CRPT->ECC_X1[1] = msg[1];
    CRPT->ECC_X1[2] = msg[2];
    CRPT->ECC_X1[3] = msg[3];
    CRPT->ECC_X1[4] = msg[4];
    CRPT->ECC_X1[5] = msg[5];
    CRPT->ECC_X1[6] = msg[6];
    CRPT->ECC_X1[7] = msg[7];

    for(i = 0; i < 8; i++)
    {
        CRPT->ECC_Y1[i] = temp_result2[i];
    }

    if(Trigger_ECC(ECCOP_MODULE | MODOP_MUL) != 0)
    {
        ResetCrypto();
        return -1;
    }

    /*  4-(7) Read X1 registers to get u1 */
    for(i = 0; i < 8; i++)
    {
        temp_result1[i] = CRPT->ECC_X1[i];
    }

    /*  4-(8) Write the curve order and curve length to N ,M registers */
    for(i = 0; i < 8; i++)
    {
        CRPT->ECC_N[i] = g_au32Eorder[i];
    }

    /* 4-(9) Write r, w to X1, Y1 registers */
    for(i = 0; i < 8; i++)
    {
        CRPT->ECC_X1[i] = 0UL;
    }
    CRPT->ECC_X1[0] = R[0];
    CRPT->ECC_X1[1] = R[1];
    CRPT->ECC_X1[2] = R[2];
    CRPT->ECC_X1[3] = R[3];
    CRPT->ECC_X1[4] = R[4];
    CRPT->ECC_X1[5] = R[5];
    CRPT->ECC_X1[6] = R[6];
    CRPT->ECC_X1[7] = R[7];

    for(i = 0; i < 8; i++)
    {
        CRPT->ECC_Y1[i] = temp_result2[i];
    }

    if(Trigger_ECC(ECCOP_MODULE | MODOP_MUL) != 0)
    {
        ResetCrypto();
        return -1;
    }

    /*  4-(14) Read X1 registers to get u2 */
    for(i = 0; i < 8; i++)
    {
        temp_result2[i] = CRPT->ECC_X1[i];
    }

    /*
     *   5. Compute X?(x1? y1? = u1 * G + u2 * Q
     *      (1) Write the curve parameter A, B, N, and curve length M to corresponding registers
     *      (2) Write the point G(x, y) to X1, Y1 registers
     *      (3) Write u1 to K registers
     *      (4) Set ECCOP(CRPT_ECC_CTL[10:9]) to 00
     *      (5) Set START(CRPT_ECC_CTL[0]) to 1
     *      (6) Wait for BUSY(CRPT_ECC_STS[0]) be cleared
     *      (7) Read X1, Y1 registers to get u1*G
     *      (8) Write the curve parameter A, B, N, and curve length M to corresponding registers
     *      (9) Write the public key Q(x,y) to X1, Y1 registers
     *      (10) Write u2 to K registers
     *      (11) Set ECCOP(CRPT_ECC_CTL[10:9]) to 00
     *      (12) Set START(CRPT_ECC_CTL[0]) to 1
     *      (13) Wait for BUSY(CRPT_ECC_STS[0]) be cleared
     *      (14) Write the curve parameter A, B, N, and curve length M to corresponding registers
     *      (15) Write the result data u1*G to X2, Y2 registers
     *      (16) Set ECCOP(CRPT_ECC_CTL[10:9]) to 10
     *      (17) Set START(CRPT_ECC_CTL[0]) to 1
     *      (18) Wait for BUSY(CRPT_ECC_STS[0]) be cleared
     *      (19) Read X1, Y1 registers to get X?x1? y1?
     *      (20) Write the curve order and curve length to N ,M registers
     *      (21) Write x1?to X1 registers
     *      (22) Write 0x0 to Y1 registers
     *      (23) Set ECCOP(CRPT_ECC_CTL[10:9]) to 01
     *      (24) Set MOPOP(CRPT_ECC_CTL[12:11]) to 10
     *      (25) Set START(CRPT_ECC_CTL[0]) to 1
     *      (26) Wait for BUSY(CRPT_ECC_STS[0]) be cleared
     *      (27) Read X1 registers to get x1?(mod n)
     *
     *   6. The signature is valid if x1?= r, otherwise it is invalid
     */

    /*
     *  (1) Write the curve parameter A, B, N, and curve length M to corresponding registers
     *  (2) Write the point G(x, y) to X1, Y1 registers
     */
    NuBL_ECCInitCurve();

    /* (3) Write u1 to K registers */
    for(i = 0; i < 8; i++)
    {
        CRPT->ECC_K[i] = temp_result1[i];
    }

    if(Trigger_ECC(ECCOP_POINT_MUL) != 0)
    {
        ResetCrypto();
        return -1;
    }

    /* (7) Read X1, Y1 registers to get u1*G */
    for(i = 0; i < 8; i++)
    {
        temp_x[i] = CRPT->ECC_X1[i];
        temp_y[i] = CRPT->ECC_Y1[i];
    }

    /* (8) Write the curve parameter A, B, N, and curve length M to corresponding registers */
    NuBL_ECCInitCurve();

    /* (9) Write the public key Q(x,y) to X1, Y1 registers */
    for(i = 0; i < 8; i++)
    {
        CRPT->ECC_X1[i] = 0UL;
        CRPT->ECC_Y1[i] = 0UL;
    }

    for(i = 0; i < 8; i++)
    {
        CRPT->ECC_X1[i] = Qx[i];
        CRPT->ECC_Y1[i] = Qy[i];
    }

    /* (10) Write u2 to K registers */
    for(i = 0; i < 8; i++)
    {
        CRPT->ECC_K[i] = temp_result2[i];
    }

    if(Trigger_ECC(ECCOP_POINT_MUL) != 0)
    {
        ResetCrypto();
        return -1;
    }

    for(i = 0; i < 8; i++)
    {
        temp_result1[i] = CRPT->ECC_X1[i];
        temp_result2[i] = CRPT->ECC_Y1[i];
    }

    /* (14) Write the curve parameter A, B, N, and curve length M to corresponding registers */
    NuBL_ECCInitCurve();

    /* Write the result data u2*Q to X1, Y1 registers */
    for(i = 0; i < 8; i++)
    {
        CRPT->ECC_X1[i] = temp_result1[i];
        CRPT->ECC_Y1[i] = temp_result2[i];
    }

    /* (15) Write the result data u1*G to X2, Y2 registers */
    for(i = 0; i < 8; i++)
    {
        CRPT->ECC_X2[i] = temp_x[i];
        CRPT->ECC_Y2[i] = temp_y[i];
    }

    if(Trigger_ECC(ECCOP_POINT_ADD) != 0)
    {
        ResetCrypto();
        return -1;
    }

    /* (19) Read X1, Y1 registers to get X?x1? y1? */
    for(i = 0; i < 8; i++)
    {
        temp_x[i] = CRPT->ECC_X1[i];
        temp_y[i] = CRPT->ECC_Y1[i];
    }

    /*  (20) Write the curve order and curve length to N ,M registers */
    for(i = 0; i < 8; i++)
    {
        CRPT->ECC_N[i] = g_au32Eorder[i];
    }

    /*
     *  (21) Write x1?to X1 registers
     *  (22) Write 0x0 to Y1 registers
     */
    for(i = 0; i < 8; i++)
    {
        CRPT->ECC_X1[i] = temp_x[i];
        CRPT->ECC_Y1[i] = 0UL;
    }

    if(Trigger_ECC(ECCOP_MODULE | MODOP_ADD) != 0)
    {
        ResetCrypto();
        return -1;
    }

    /*  (27) Read X1 registers to get x1?(mod n) */

    /* 6. The signature is valid if x1?= r, otherwise it is invalid */

    /* Compare with test pattern to check if r is correct or not */
    for(i = 0; i < 8; i++)
    {
        if(CRPT->ECC_X1[i] != R[i])
        {
            ResetCrypto();
            return -1;
        }
    }

    return 0;
}

/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/
