/******************************************************************************
 * @file     uart_transfer.h
 * @brief    General UART ISP slave header file
 * @version  1.0.0
 * @date     22, Sep, 2014
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __UART_TRANS_H__
#define __UART_TRANS_H__
#include <stdint.h>

/*-------------------------------------------------------------*/
/* Define maximum packet size */
#define MAX_PKT_SIZE        	64

/*-------------------------------------------------------------*/

extern uint8_t g_au8uart_rcvbuf[];
extern uint8_t volatile g_u8bUartDataReady;
extern uint8_t volatile g_u8bufhead;

/*-------------------------------------------------------------*/
void UART_Init(void);
void UART1_IRQHandler(void);
void PutString(void);

#endif  /* __UART_TRANS_H__ */

/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/