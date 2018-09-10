/******************************************************************************
 * @file     main.c
 * @version  V3.00
 * $Revision: 3 $
 * $Date: 16/10/17 2:06p $
 * @brief    XOM library  --  Add function
 * @note
 * Copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

int32_t XOM_Add(int32_t a, int32_t b)
{
    uint32_t c;
    uint32_t d = 0x7654321;
    c =  a + b + d;

    return c;
}

int32_t XOM_Sub(int32_t a, int32_t b)
{
    uint32_t c;

    c =  a - b;

    return c;
}

int32_t XOM_Mul(int32_t a, int32_t b)
{
    uint32_t c;


    c =  a * b;

    return c;
}

int32_t XOM_Div(int32_t a, int32_t b)
{
    uint32_t c;

    c =  a / b;

    return c;
}


int32_t XOM_Sum(int32_t *pbuf, int32_t n)
{
    int32_t i;
    int32_t i32Sum;

    i32Sum = 0;
    for(i = 0; i < n; i++)
    {
        i32Sum += pbuf[i];
    }

    return i32Sum;
}





