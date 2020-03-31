/******************************************************************************
 * @file     isr.c
 * @version  V1.00
 * @brief    M2351 ISR source file
 *
 * @note
 * Copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

#include "config.h"

static uint32_t g_au32PcmBuff[32] = {0};

void I2S0_IRQHandler(void);

void I2S0_IRQHandler(void)
{
    uint32_t u32Reg;
    uint32_t u32Len, i;
    uint32_t *pBuffTx, *pBuffRx;

    // enable sound output
    PA4 = 0;

    u32Reg = I2S_GET_INT_FLAG(I2S0, I2S_STATUS0_TXTHIF_Msk | I2S_STATUS0_RXTHIF_Msk);

    if(u32Reg & I2S_STATUS0_TXTHIF_Msk)
    {
        pBuffTx = &g_au32PcmBuff[0];

        /* Read Tx FIFO free size */
        u32Len = 8 - I2S_GET_TX_FIFO_LEVEL(I2S0);

        if(g_u32BuffPos >= 8)
        {
            for(i = 0; i < u32Len; i++)
            {
                I2S_WRITE_TX_FIFO(I2S0, pBuffTx[i]);
            }

            for(i = 0; i < BUFF_LEN - u32Len; i++)
            {
                pBuffTx[i] = pBuffTx[i + u32Len];
            }

            g_u32BuffPos -= u32Len;
        }
        else
        {
            for(i = 0; i < u32Len; i++)
            {
                I2S_WRITE_TX_FIFO(I2S0, 0x00);
            }
        }
    }

    if(u32Reg & I2S_STATUS0_RXTHIF_Msk)
    {
        if(g_u32BuffPos < (BUFF_LEN - 8))
        {
            pBuffRx = &g_au32PcmBuff[g_u32BuffPos];

            /* Read Rx FIFO Level */
            u32Len = I2S_GET_RX_FIFO_LEVEL(I2S0);

            for(i = 0; i < u32Len; i++)
            {
                pBuffRx[i] = I2S_READ_RX_FIFO(I2S0);
            }

            g_u32BuffPos += u32Len;

            if(g_u32BuffPos >= BUFF_LEN)
            {
                g_u32BuffPos =    0;
            }
        }
    }
}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
