/**************************************************************************//**
 * @file     config.h
 * @version  V1.00
 * @brief    I2S Driver Sample Configuration Header File
 *
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2017-2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef CONFIG_H
#define CONFIG_H

// *** <<< Use Configuration Wizard in Context Menu >>> ***
// <h> I2S Data Format Setup
//  <e0.0> Enable 24-bit Data Width (Disable: 16-bit)
//   <i> Enable this option to set I2S data width to 24-bit. Disable this option to set I2S data width to 16-bit.
//   <o1.24..26> Data Format Selection
//    <i> Select the data format in 24-bit data width.
//    <0=> I2S      <1=> I2S_MSB    <2=> I2S_LSB
#define ENABLE_I2S_24BIT_DATA       0
#define I2S_FIFO_24BIT_FORMAT       0x00000000
//  </e>
// </h>
// *** <<< end of configuration section >>> ***

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
#if ENABLE_I2S_24BIT_DATA
#define BUFF_LEN        1024
#else
#define BUFF_LEN        512
#endif
#define BUFF_HALF_LEN   (BUFF_LEN/2)

#define I2S_CHWIDTH_16  (1 << I2S_CTL0_CHWIDTH_Pos)     /*!< I2S Channel Width is 16-bit */
#define I2S_CHWIDTH_32  (3 << I2S_CTL0_CHWIDTH_Pos)     /*!< I2S Channel Width is 32-bit */

typedef struct dma_desc_t
{
    uint32_t ctl;
    uint32_t src;
    uint32_t dest;
    uint32_t offset;
} DMA_DESC_T;

extern void PDMA_ResetTxSGTable(uint8_t id);
extern void PDMA_ResetRxSGTable(uint8_t id);
extern void PDMA_Init(void);
#endif

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
