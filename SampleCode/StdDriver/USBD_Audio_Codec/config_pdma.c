/******************************************************************************
 * @file     config_pdma.c
 * @brief    NuMicro series HSUSBD driver Sample file
 *
 * @note
 * Copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
/*!<Includes */
#include <string.h>
#include "NuMicro.h"
#include "usbd_audio.h"

/* DMA scatter-gather descriptor */
static DMA_DESC_T DMA_TXDESC[PDMA_TXBUFFER_CNT];
static DMA_DESC_T DMA_RXDESC[PDMA_RXBUFFER_CNT];

void PDMA0_IRQHandler(void);

/* PDMA Interrupt handler */
void PDMA0_IRQHandler(void)
{
    uint32_t u32Status = PDMA_GET_INT_STATUS(PDMA0);

    if(u32Status & 0x1)     /* abort */
    {
        if(PDMA_GET_ABORT_STS(PDMA0) & 0x4)
            PDMA_CLR_ABORT_FLAG(PDMA0, PDMA_ABTSTS_ABTIF2_Msk);
    }
    else if(u32Status & 0x2)
    {
        if(PDMA_GET_TD_STS(PDMA0) & 0x2)             /* channel 1 done, Tx */
        {
            PDMA_CLR_TD_FLAG(PDMA0, PDMA_TDSTS_TDIF1_Msk);

            /* Decrease number of full buffer */
            g_u8TxDataCntInBuffer --;
            /* Change to next buffer */
            g_u8PDMATxIdx ++;
            if(g_u8PDMATxIdx >= PDMA_TXBUFFER_CNT)
                g_u8PDMATxIdx = 0;
        }
        if(PDMA_GET_TD_STS(PDMA0) & 0x4)             /* channel 2 done, Rx */
        {
            PDMA_CLR_TD_FLAG(PDMA0, PDMA_TDSTS_TDIF2_Msk);

            /* Set PCM buffer full flag */
            g_au8PcmRxBufFull[g_u8PDMARxIdx] = 1;

            /* Change to next buffer */
            g_u8PDMARxIdx ++;
            if(g_u8PDMARxIdx >= PDMA_RXBUFFER_CNT)
                g_u8PDMARxIdx = 0;
        }
    }
}

// Configure PDMA to Scatter Gather mode */
void PDMA_Init(void)
{
    PDMA_WriteTxSGTable();
    PDMA_WriteRxSGTable();

    /* Open PDMA channel 1 for I2S TX and channel 2 for I2S RX */
    PDMA_Open(PDMA0, (1 << PDMA_I2S_TX_CH) | (1 << PDMA_I2S_RX_CH));

    /* We want to enable these channels at run time */
    PDMA0->CHCTL = 0;

    /* Configure PDMA transfer mode */
    PDMA_SetTransferMode(PDMA0, PDMA_I2S_TX_CH, PDMA_I2S0_TX, 1, (uint32_t)&DMA_TXDESC[0]);
    PDMA_SetTransferMode(PDMA0, PDMA_I2S_RX_CH, PDMA_I2S0_RX, 1, (uint32_t)&DMA_RXDESC[0]);

    /* Enable PDMA channel 1&2 interrupt */
    PDMA_EnableInt(PDMA0, PDMA_I2S_TX_CH, 0);
    PDMA_EnableInt(PDMA0, PDMA_I2S_RX_CH, 0);

    /* Enable PDMA interrupt */
    NVIC_EnableIRQ(PDMA0_IRQn);
}

/* init TX scatter-gather table */
void PDMA_WriteTxSGTable(void)
{
    uint16_t u16Cnt;

    /* Use PDMA_TXBUFFER_CNT scatter-gather tables and link with each other */
    for(u16Cnt = 0; u16Cnt < PDMA_TXBUFFER_CNT; u16Cnt++)
    {
        DMA_TXDESC[u16Cnt].ctl = ((g_u32BuffLen - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_WIDTH_32 | PDMA_SAR_INC | PDMA_DAR_FIX | PDMA_REQ_SINGLE | PDMA_OP_SCATTER;
        DMA_TXDESC[u16Cnt].src = (uint32_t)&g_au32PcmPlayBuff[u16Cnt];
        DMA_TXDESC[u16Cnt].dest = (uint32_t)&I2S0->TXFIFO;

        if(u16Cnt != PDMA_TXBUFFER_CNT - 1)
            DMA_TXDESC[u16Cnt].offset = (uint32_t)&DMA_TXDESC[u16Cnt + 1] - (PDMA0->SCATBA);
        else
            DMA_TXDESC[u16Cnt].offset = (uint32_t)&DMA_TXDESC[0] - (PDMA0->SCATBA);
    }
}

/* init RX scatter-gather table */
void PDMA_WriteRxSGTable(void)
{
    uint16_t u16Cnt;

    /* Use PDMA_RXBUFFER_CNT scatter-gather tables and link with each other */
    for(u16Cnt = 0; u16Cnt < PDMA_RXBUFFER_CNT; u16Cnt++)
    {
        DMA_RXDESC[u16Cnt].ctl = (((g_u32RxBuffLen / 4) - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_WIDTH_32 | PDMA_SAR_FIX | PDMA_DAR_INC | PDMA_REQ_SINGLE | PDMA_OP_SCATTER;
        DMA_RXDESC[u16Cnt].src = (uint32_t)&I2S0->RXFIFO;
        DMA_RXDESC[u16Cnt].dest = (uint32_t)&g_au8PcmRecBuff[u16Cnt];

        if(u16Cnt != (PDMA_RXBUFFER_CNT - 1))
            DMA_RXDESC[u16Cnt].offset = (uint32_t)&DMA_RXDESC[u16Cnt + 1] - (PDMA0->SCATBA);
        else
            DMA_RXDESC[u16Cnt].offset = (uint32_t)&DMA_RXDESC[0] - (PDMA0->SCATBA);
    }
}

