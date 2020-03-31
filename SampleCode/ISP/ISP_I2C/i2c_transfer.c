/***************************************************************************//**
 * @file     i2c_transfer.c
 * @brief    ISP support function source file
 * @version  0x32
 * @date     14, June, 2017
 *
 * @note
 * Copyright (C) 2017-2018 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "targetdev.h"
#include "i2c_transfer.h"

#define I2C_ADDR 0x60

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t au8I2cRcvBuf[64];
volatile uint8_t u8I2cDataReady;

static volatile uint8_t s_u8SlvDataLen;

__STATIC_INLINE void I2C_SlaveTRx(I2C_T *i2c, uint32_t u32Status);

extern uint32_t u32Pclk0;
extern uint32_t u32Pclk1;

void I2C1_IRQHandler(void);

void I2C_Init(void)
{
    /* Reset I2C1 */
    SYS->IPRST1 |=  SYS_IPRST1_I2C1RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_I2C1RST_Msk;

    /* Open I2C1 and set clock to 100k */
    I2C1->CLKDIV = (uint32_t)(((u32Pclk1 * 10U) / (100000 * 4U) + 5U) / 10U - 1U); /* Compute proper divider for I2C clock */
    I2C1->CTL0 |= I2C_CTL0_I2CEN_Msk;

    /* Set I2C1 ADDR0 Slave Addresses */
    I2C1->ADDR0  = (I2C_ADDR << 1U) | I2C_GCMODE_DISABLE;
    I2C1->CTL0 |= I2C_CTL0_INTEN_Msk;

    /* I2C enter no address SLV mode */
    I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI_AA);
    NVIC_EnableIRQ(I2C1_IRQn);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C1 IRQ Handler                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void I2C1_IRQHandler(void)
{
    uint32_t u32Status;
    u32Status = I2C_GET_STATUS(I2C1);

    if (I2C_GET_TIMEOUT_FLAG(I2C1))
    {
        /* Clear I2C1 Timeout Flag */
        I2C1->TOCTL |= I2C_TOCTL_TOIF_Msk;
    }
    else
    {
        I2C_SlaveTRx(I2C1, u32Status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C TRx Callback Function                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_SlaveTRx(I2C_T *i2c, uint32_t u32Status)
{
    uint8_t u8data;

    if (u32Status == 0x60)                      /* Own SLA+W has been receive; ACK has been return */
    {
        u8I2cDataReady = 0;
        s_u8SlvDataLen = 0;
        I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI_AA);
    }
    else if (u32Status == 0x80)                 /* Previously address with own SLA address
                                                   Data has been received; ACK has been returned*/
    {
        au8I2cRcvBuf[s_u8SlvDataLen] = (uint8_t)I2C_GET_DATA(i2c);
        s_u8SlvDataLen++;
        s_u8SlvDataLen &= 0x3F;
        u8I2cDataReady = (s_u8SlvDataLen == 0);

        if (s_u8SlvDataLen == 0x3F)
        {
            I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI);
        }
        else
        {
            I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI_AA);
        }
    }
    else if (u32Status == 0xA8)                 /* Own SLA+R has been receive; ACK has been return */
    {
        s_u8SlvDataLen = 0;
        u8data = g_au8ResponseBuff[s_u8SlvDataLen];
        I2C_SET_DATA(i2c, u8data);
        s_u8SlvDataLen++;
        I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI_AA);
    }
    else if (u32Status == 0xB8)
    {
        u8data = g_au8ResponseBuff[s_u8SlvDataLen];
        I2C_SET_DATA(i2c, u8data);
        s_u8SlvDataLen++;
        s_u8SlvDataLen &= 0x3F;

        if (s_u8SlvDataLen == 0x00)
        {
            I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI);
        }
        else
        {
            I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI_AA);
        }
    }
    else if (u32Status == 0xC8)
    {
        I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI_AA);
    }
    else if (u32Status == 0xC0)                 /* Data byte or last data in I2CDAT has been transmitted
                                                   Not ACK has been received */
    {
        I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI_AA);
    }
    else if (u32Status == 0x88)                 /* Previously addressed with own SLA address; NOT ACK has
                                                   been returned */
    {
        au8I2cRcvBuf[s_u8SlvDataLen] = (uint8_t)I2C_GET_DATA(i2c);
        s_u8SlvDataLen++;
        u8I2cDataReady = (s_u8SlvDataLen == 64);
        s_u8SlvDataLen = 0;
        I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI_AA);
    }
    else if (u32Status == 0xA0)                 /* A STOP or repeated START has been received while still
                                                   addressed as Slave/Receiver*/
    {
        s_u8SlvDataLen = 0;
        I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI_AA);
    }
    else
    {
        /* TO DO */
        /* printf("Status 0x%x is NOT processed\n", u32Status); */
    }
}


