/******************************************************************************
 * @file     isr.c
 * @version  V1.00
 * @brief   ISR source file
 *
 * @note
 * Copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

#include "config.h"

extern volatile uint8_t g_u8PCMBuffer_Full[2];
extern volatile uint8_t g_u8PCMBuffer_Playing;

void PDMA0_IRQHandler(void)
{
    uint32_t u32Status = PDMA_GET_INT_STATUS(PDMA0);

    if(u32Status & 0x2)    /* done */
    {
        if(PDMA_GET_TD_STS(PDMA0) & 0x4)
        {
            g_u8PCMBuffer_Full[g_u8PCMBuffer_Playing] = 0;       //set empty flag
            g_u8PCMBuffer_Playing ^= 1;
        }
        PDMA_CLR_TD_FLAG(PDMA0, PDMA_TDSTS_TDIF2_Msk);
    }
    else if(u32Status & 0x400)   /* Timeout */
    {
        PDMA_CLR_TMOUT_FLAG(PDMA0, PDMA_TDSTS_TDIF2_Msk);
    }

}

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
