/******************************************************************************
 * @file     main.c
 * @version  V3.00
 * $Revision: 3 $
 * $Date: 16/10/17 2:06p $
 * @brief    Show how to use XOM Lirbary
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2017-2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

uint32_t Lib_XOM_ADD(uint32_t a, uint32_t b);

uint32_t Lib_XOM_ADD(uint32_t a, uint32_t b)
{
    uint32_t c;
    c =  a + b;
    return c;
}
