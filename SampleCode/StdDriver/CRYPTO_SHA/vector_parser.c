/**************************************************************************//**
 * @file     vector_parser.c
 * @version  V1.00
 * @brief    CRYPTO SHA test vector parser
 *
 * @note
 * Copyright (C) 2016 M2351 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <string.h>

#include "NuMicro.h"


extern uint32_t g_u32VectorDataBase, g_u32VectorDataLimit;
static uint8_t *g_pu8FileBase;
static uint32_t g_u32FileIdx, g_u32FileSize;
static char  g_pi8LineBuff[20 * 1024];

#ifdef __ICCARM__
#pragma data_alignment=32
uint8_t g_au8ShaDataPool[8192] ;
#else
__attribute__((aligned(32))) uint8_t g_au8ShaDataPool[8192] ;
#endif


uint8_t *g_au8ShaData;
uint8_t g_au8ShaDigest[64];
int32_t g_i32DataLen;

void OpenTestVector(void)
{
    /* Get test vector base */
    g_pu8FileBase = (uint8_t *)&g_u32VectorDataBase;
#ifdef __ICCARM__
    _u32FileSize = 0x213f;
#else
    /* Get vector size */
    g_u32FileSize = (uint32_t)&g_u32VectorDataLimit - (uint32_t)&g_u32VectorDataBase;
#endif
    g_u32FileIdx = 0;
}

static int32_t ReadFile(uint8_t *pu8Buff, int i32Len)
{
    if(g_u32FileIdx + 1 >= g_u32FileSize)
        return -1;
    memcpy(pu8Buff, &g_pu8FileBase[g_u32FileIdx], i32Len);
    g_u32FileIdx += i32Len;
    return 0;
}


int32_t GetLine(void)
{
    int         i;
    uint8_t     ch;

    if(g_u32FileIdx + 1 >= g_u32FileSize)
    {
        return -1;
    }

    memset(g_pi8LineBuff, 0, sizeof(g_pi8LineBuff));

    for(i = 0;  ; i++)
    {
        if(ReadFile(&ch, 1) < 0)
            return 0;

        if((ch == 0x0D) || (ch == 0x0A))
            break;

        g_pi8LineBuff[i] = ch;
    }

    while(1)
    {
        if(ReadFile(&ch, 1) < 0)
            return 0;

        if((ch != 0x0D) && (ch != 0x0A))
            break;
    }
    g_u32FileIdx--;
    return 0;
}


int32_t IsHexChar(char c)
{
    if((c >= '0') && (c <= '9'))
        return 1;
    if((c >= 'a') && (c <= 'f'))
        return 1;
    if((c >= 'A') && (c <= 'F'))
        return 1;
    return 0;
}


uint8_t  char_to_hex(uint8_t c)
{
    if((c >= '0') && (c <= '9'))
        return c - '0';
    if((c >= 'a') && (c <= 'f'))
        return c - 'a' + 10;
    if((c >= 'A') && (c <= 'F'))
        return c - 'A' + 10;
    return 0;
}


int32_t Str2Hex(uint8_t *str, uint8_t *hex, int swap)
{
    int         i, count = 0, actual_len;
    uint8_t     val8;

    while(*str)
    {
        if(!IsHexChar(*str))
        {
            return count;
        }

        val8 = char_to_hex(*str);
        str++;

        if(!IsHexChar(*str))
        {
            return count;
        }

        val8 = (val8 << 4) | char_to_hex(*str);
        str++;

        hex[count] = val8;
        count++;
    }

    actual_len = count;

    for(; count % 4 ; count++)
        hex[count] = 0;

    if(!swap)
        return actual_len;

    // SWAP
    for(i = 0; i < count; i += 4)
    {
        val8 = hex[i];
        hex[i] = hex[i + 3];
        hex[i + 3] = val8;

        val8 = hex[i + 1];
        hex[i + 1] = hex[i + 2];
        hex[i + 2] = val8;
    }

    return actual_len;
}


int32_t Str2Dec(uint8_t *str)
{
    int         val32;

    val32 = 0;
    while(*str)
    {
        if((*str < '0') || (*str > '9'))
        {
            return val32;
        }
        val32 = (val32 * 10) + (*str - '0');
        str++;
    }
    return val32;
}


int GetNextPattern(void)
{
    int32_t line_num = 1;
    uint8_t *p;

    g_au8ShaData = (uint8_t *)g_au8ShaDataPool;

    while(GetLine() == 0)
    {
        line_num++;

        if(g_pi8LineBuff[0] == '#')
            continue;
    
        /* Get Length */
        if(strncmp(g_pi8LineBuff, "Len", 3) == 0)
        {
            p = (uint8_t *)&g_pi8LineBuff[3];
            while((*p < '0') || (*p > '9'))
                p++;

            g_i32DataLen = Str2Dec(p);
            continue;
        }

        /* Get Msg data */
        if(strncmp(g_pi8LineBuff, "Msg", 3) == 0)
        {
            p = (uint8_t *)&g_pi8LineBuff[3];
            while(!IsHexChar(*p)) p++;
            Str2Hex(p, &g_au8ShaData[0], 0);
            continue;
        }
        
        /* Get golden result */
        if(strncmp(g_pi8LineBuff, "MD", 2) == 0)
        {
            p = (uint8_t *)&g_pi8LineBuff[2];
            while(!IsHexChar(*p)) p++;
            Str2Hex(p, &g_au8ShaDigest[0], 1);
            return 0;
        }
    }
    return -1;
}




