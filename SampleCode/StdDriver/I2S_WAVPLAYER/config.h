/**************************************************************************//**
 * @file     config.h
 * @version  V1.00
 * @brief    I2S Wave Player Sample Configuration Header File
 *
 * @note
 * Copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#ifndef CONFIG_H
#define CONFIG_H

#include "ff.h"
/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
#define BUFF_LEN    32
#define BUFF_HALF_LEN   (BUFF_LEN/2)

#define PCM_BUFFER_SIZE 4*1024

typedef struct dma_desc_t
{
    uint32_t ctl;
    uint32_t endsrc;
    uint32_t enddest;
    uint32_t offset;
} DMA_DESC_T;

extern uint8_t bAudioPlaying;
extern FATFS FatFs[];      /* File system object for logical drive */

extern signed int aiPCMBuffer[2][PCM_BUFFER_SIZE];
extern volatile uint8_t s_au8PCMBufferFull[2];
extern volatile uint8_t s_u8PCMBufferPlaying;

void PDMA_Reset_SCTable(uint8_t id);
void WAVPlayer(void);
void NAU88L25_ConfigSampleRate(uint32_t u32SampleRate);

#endif

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
