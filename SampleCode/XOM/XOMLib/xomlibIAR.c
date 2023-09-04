/******************************************************************************
 * @file     xomlibIAR.c
 * @version  V3.00
 * $Revision: 1 $
 * $Date: 16/10/17 2:06p $
 * @brief    Function pointer for XOM APIs.
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2017-2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include "NuMicro.h"
 
int32_t (*XOM_Add)(int32_t a, int32_t b) = (int32_t (*)(int32_t a, int32_t b))(0x00010001);
int32_t (*XOM_Div)(int32_t a, int32_t b) = (int32_t (*)(int32_t a, int32_t b))(0x0001000d);
int32_t (*XOM_Mul)(int32_t a, int32_t b) = (int32_t (*)(int32_t a, int32_t b))(0x00010009);
int32_t (*XOM_Sub)(int32_t a, int32_t b) = (int32_t (*)(int32_t a, int32_t b))(0x00010005);
int32_t (*XOM_Sum)(int32_t *pbuf, int32_t n) = (int32_t (*)(int32_t *pbuf, int32_t n))(0x00010013);
        
        
    