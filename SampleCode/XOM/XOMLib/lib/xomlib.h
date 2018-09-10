/******************************************************************************
 * @file     xomlib.h
 * @version  V3.00
 * $Revision: 1 $
 * $Date: 16/10/17 2:06p $
 * @brief    Header file for XOM APIs.
 * @note
 * Copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#ifndef _XOMLIB_H_
#define _XOMLIB_H_

extern int32_t (*XOM_Add)(int32_t a, int32_t b);
extern int32_t (*XOM_Div)(int32_t a, int32_t b);
extern int32_t (*XOM_Mul)(int32_t a, int32_t b);
extern int32_t (*XOM_Sub)(int32_t a, int32_t b);
extern int32_t (*XOM_Sum)(int32_t *pbuf, int32_t n);


#endif //_XOMLIB_H_