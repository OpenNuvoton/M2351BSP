/******************************************************************************
 * @file     main.c
 * @brief
 *           Demonstrate how to play/record audio.
 *           NAU88L25 is used in this sample code to play and record the audio data.
 * @note
 * Copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#ifndef OPTION
# define OPTION      0 // 0: Play sin, 1: rec/play loopback
#endif

#define I2C_PORT        I2C2
#define I2S             I2S0

void NAU88L25_Setup(void);
void SYS_Init(void);
void UART0_Init(void);
void I2C_Init(void);
void Codec_Delay(uint32_t delayCnt);
uint8_t I2cWrite_MultiByteforNAU88L25(uint8_t chipadd, uint16_t subaddr, const uint8_t *p, uint32_t len);
uint8_t I2C_WriteNAU88L25(uint16_t addr, uint16_t dat);
void I2S0_IRQHandler(void);

#if (OPTION == 0)

static __attribute__((aligned(32))) int16_t s_ai16sin[96] =
{
    0, 0, -134, -134, -266, -266, -393, -393, -513, -513, -625, -625, -726, -726, -814, -814, -889, -889, -948, -948, -991, -991, -1018, -1018, -1026, -1026, -1018, -1018, -991, -991, -948, -948, -889, -889, -814, -814, -726, -726, -625, -625, -513, -513, -393, -393, -266, -266, -134, -134, 0, 0, 134, 134, 266, 266, 393, 393, 513, 513, 625, 625, 726, 726, 814, 814, 889, 889, 948, 948, 991, 991, 1018, 1018, 1026, 1026, 1018, 1018, 991, 991, 948, 948, 889, 889, 814, 814, 726, 726, 625, 625, 513, 513, 393, 393, 266, 266, 134, 134
};

static uint32_t *g_pu32sin = (uint32_t *)(uint32_t)&s_ai16sin[0];
static uint32_t *g_pu32sin;
static int32_t g_i32Idx = 0;

void I2S0_IRQHandler(void)
{
    uint32_t u32I2SIntFlag;

    u32I2SIntFlag = I2S->STATUS0;//  I2S_GET_INT_FLAG(I2S, I2S_STATUS_I2STXINT_Msk | I2S_STATUS_I2SRXINT_Msk);
    if(u32I2SIntFlag & I2S_STATUS0_TXTHIF_Msk)
    {
        /* Force to play sin wave */
        /* Fill 4 word data when it is TX threshold interrupt */
        /* Play to I2S */
        I2S_WRITE_TX_FIFO(I2S, g_pu32sin[g_i32Idx++]);
        if(g_i32Idx >= 48) g_i32Idx = 0;
        I2S_WRITE_TX_FIFO(I2S, g_pu32sin[g_i32Idx++]);
        if(g_i32Idx >= 48) g_i32Idx = 0;
        I2S_WRITE_TX_FIFO(I2S, g_pu32sin[g_i32Idx++]);
        if(g_i32Idx >= 48) g_i32Idx = 0;
        I2S_WRITE_TX_FIFO(I2S, g_pu32sin[g_i32Idx++]);
        if(g_i32Idx >= 48) g_i32Idx = 0;

    }

    if(u32I2SIntFlag & I2S_STATUS0_RXTHIF_Msk)
    {
        I2S_READ_RX_FIFO(I2S);
        I2S_READ_RX_FIFO(I2S);
        I2S_READ_RX_FIFO(I2S);
        I2S_READ_RX_FIFO(I2S);
    }


}


#else

static volatile uint32_t s_au32Tmp[8] = {0};
void I2S0_IRQHandler(void)
{
    uint32_t u32I2SIntFlag;


    u32I2SIntFlag = I2S->STATUS0;//  I2S_GET_INT_FLAG(I2S, I2S_STATUS_I2STXINT_Msk | I2S_STATUS_I2SRXINT_Msk);
    if(u32I2SIntFlag & I2S_STATUS0_TXTHIF_Msk)
    {
        /* Force to play sin wave */
        /* Fill 4 word data when it is TX threshold interrupt */
        /* Play to I2S */
        I2S_WRITE_TX_FIFO(I2S, s_au32Tmp[0]);
        I2S_WRITE_TX_FIFO(I2S, s_au32Tmp[1]);
        I2S_WRITE_TX_FIFO(I2S, s_au32Tmp[2]);
        I2S_WRITE_TX_FIFO(I2S, s_au32Tmp[3]);

    }

    if(u32I2SIntFlag & I2S_STATUS0_RXTHIF_Msk)
    {
        s_au32Tmp[0] = I2S_READ_RX_FIFO(I2S);
        s_au32Tmp[1] = I2S_READ_RX_FIFO(I2S);
        s_au32Tmp[2] = I2S_READ_RX_FIFO(I2S);
        s_au32Tmp[3] = I2S_READ_RX_FIFO(I2S);

    }


}

#endif

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and HCLK source divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable HXT clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Wait for HXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Enable PLL */
    CLK->PLLCTL = CLK_PLLCTL_128MHz_HIRC;

    /* Waiting for PLL stable */
    CLK_WaitClockReady(CLK_STATUS_PLLSTB_Msk);

    /* Select HCLK clock source as PLL and HCLK source divider as 2 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_PLL, CLK_CLKDIV0_HCLK(2));

    /* Select UART module clock source as HXT and UART module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HXT, CLK_CLKDIV0_UART0(1));

    /* Enable UART peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable I2S0 peripheral clock */
    CLK_EnableModuleClock(I2S0_MODULE);

    /* Enable CODEC_I2C peripheral clock */
    CLK_EnableModuleClock(I2C2_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & (~(UART0_RXD_PB12_Msk | UART0_TXD_PB13_Msk))) | UART0_RXD_PB12 | UART0_TXD_PB13;

    /* Set multi function pin for I2S0 */
    /* PB0, PB1, PB2, PB3, PB4 */
    SYS->GPC_MFPL = (SYS->GPC_MFPL & ~(SYS_GPC_MFPL_PC0MFP_Msk | SYS_GPC_MFPL_PC1MFP_Msk | SYS_GPC_MFPL_PC2MFP_Msk | SYS_GPC_MFPL_PC3MFP_Msk | SYS_GPC_MFPL_PC4MFP_Msk)) |
                    (SYS_GPC_MFPL_PC0MFP_I2S0_LRCK | SYS_GPC_MFPL_PC1MFP_I2S0_DO | SYS_GPC_MFPL_PC2MFP_I2S0_DI | SYS_GPC_MFPL_PC3MFP_I2S0_MCLK | SYS_GPC_MFPL_PC4MFP_I2S0_BCLK);

    /* Set CODEC_I2C multi-function pins */
    SYS->GPD_MFPH &= ~(SYS_GPD_MFPH_PD8MFP_Msk | SYS_GPD_MFPH_PD9MFP_Msk);
    SYS->GPD_MFPH |= (SYS_GPD_MFPH_PD8MFP_I2C2_SDA | SYS_GPD_MFPH_PD9MFP_I2C2_SCL);
}



void UART0_Init(void)
{

    /* Configure UART0 and set UART0 Baudrate */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC, 115200);
    UART0->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}


void I2C_Init(void)
{
    /* Open I2C and set clock to 100k */
    I2C_Open(I2C_PORT, 100000);

    /* Get I2C Bus Clock */
    printf("I2C clock %d Hz\n", I2C_GetBusClockFreq(I2C_PORT));
}


/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
#if OPTION == 0
    int32_t i;
#endif
    /* Unlock Protected Regsiter */
    SYS_UnlockReg();

    /* Initial system & multi-function */
    SYS_Init();

    /* Initial UART0 for debug message */
    UART0_Init();

    printf("\n");
    printf("+-------------------------------------------------------+\n");
    printf("|          NuMicro USB Audio CODEC Sample Code          |\n");
    printf("+-------------------------------------------------------+\n");

    /* Init I2C to access NAU88L25 */
    I2C_Init();

    I2S_Open(I2S0, I2S_MODE_SLAVE, 48000, I2S_DATABIT_16, I2S_STEREO, I2S_FORMAT_I2S);

    /* Set MCLK and enable MCLK */
    I2S_EnableMCLK(I2S0, 12000000);

    I2S_SetFIFO(I2S0, 4, 4);

    /* Fill dummy data to I2S TX for start I2S iteration */
#if OPTION == 0
    for(i = 0; i < 16; i++)
        I2S_WRITE_TX_FIFO(I2S, 0);
#endif

    /* Start I2S play iteration */
    I2S_EnableInt(I2S0, I2S_IEN_TXTHIEN_Msk | I2S_IEN_RXTHIEN_Msk);

    NVIC_EnableIRQ(I2S0_IRQn);

    I2S_ENABLE_TX(I2S0);
    I2S_ENABLE_RX(I2S0);

    /* Initialize NAU88L25 codec */
    NAU88L25_Setup();

    while(1);

}


void Codec_Delay(uint32_t delayCnt)
{
    while(delayCnt--)
    {
        __NOP();
        __NOP();
    }
}
uint8_t I2cWrite_MultiByteforNAU88L25(uint8_t chipadd, uint16_t subaddr, const uint8_t *p, uint32_t len)
{
    (void)chipadd;
    (void)len;
    /* Send START */
    I2C_START(I2C_PORT);
    I2C_WAIT_READY(I2C_PORT);

    /* Send device address */
    I2C_SET_DATA(I2C_PORT, 0x1A << 1);
    I2C_SET_CONTROL_REG(I2C_PORT, I2C_CTL_SI);
    I2C_WAIT_READY(I2C_PORT);

    /* Send register number and MSB of data */
    I2C_SET_DATA(I2C_PORT, (uint8_t)((subaddr >> 8)));
    I2C_SET_CONTROL_REG(I2C_PORT, I2C_CTL_SI);
    I2C_WAIT_READY(I2C_PORT);

    I2C_SET_DATA(I2C_PORT, (uint8_t)((subaddr & 0x00FF)));
    I2C_SET_CONTROL_REG(I2C_PORT, I2C_CTL_SI);
    I2C_WAIT_READY(I2C_PORT);

    /* Send data */
    I2C_SET_DATA(I2C_PORT, p[0]);
    I2C_SET_CONTROL_REG(I2C_PORT, I2C_CTL_SI);
    I2C_WAIT_READY(I2C_PORT);

    I2C_SET_DATA(I2C_PORT, p[1]);
    I2C_SET_CONTROL_REG(I2C_PORT, I2C_CTL_SI);
    I2C_WAIT_READY(I2C_PORT);

    /* Send STOP */
    I2C_STOP(I2C_PORT);

    return  1;
}


uint8_t I2C_WriteNAU88L25(uint16_t addr, uint16_t dat)
{

    uint8_t Tx_Data0[2];

    Tx_Data0[0] = (uint8_t)(dat >> 8);
    Tx_Data0[1] = (uint8_t)(dat & 0x00FF);
    return (I2cWrite_MultiByteforNAU88L25(0x34, addr, &Tx_Data0[0], 2));
}

void NAU88L25_Setup(void)
{
    I2C_WriteNAU88L25(0,  0);   // Reset all registers
    Codec_Delay(10000);

    I2C_WriteNAU88L25(0x007F,  0x0);

    I2C_WriteNAU88L25(0x0003,  0x8053);
    I2C_WriteNAU88L25(0x0004,  0x0001);

    I2C_WriteNAU88L25(0x0005,  0x3126);
    I2C_WriteNAU88L25(0x0006,  0x0008);
    I2C_WriteNAU88L25(0x0007,  0x0010);
    I2C_WriteNAU88L25(0x0008,  0xC000);


    I2C_WriteNAU88L25(0x0009,  0x6000);
    I2C_WriteNAU88L25(0x000A,  0xF13C);
    I2C_WriteNAU88L25(0x000C,  0x0048);
    I2C_WriteNAU88L25(0x000D,  0x0000);
    I2C_WriteNAU88L25(0x000F,  0x0000);
    I2C_WriteNAU88L25(0x0010,  0x0000);
    I2C_WriteNAU88L25(0x0011,  0x0000);
    I2C_WriteNAU88L25(0x0012,  0xFFFF);
    I2C_WriteNAU88L25(0x0013,  0x0015);
    I2C_WriteNAU88L25(0x0014,  0x0110);
    I2C_WriteNAU88L25(0x0015,  0x0000);
    I2C_WriteNAU88L25(0x0016,  0x0000);
    I2C_WriteNAU88L25(0x0017,  0x0000);
    I2C_WriteNAU88L25(0x0018,  0x0000);
    I2C_WriteNAU88L25(0x0019,  0x0000);
    I2C_WriteNAU88L25(0x001A,  0x0000);
    I2C_WriteNAU88L25(0x001B,  0x0000);
    I2C_WriteNAU88L25(0x001C,  0x0002);
    I2C_WriteNAU88L25(0x001D,  0x301a);   //301A:Master, BCLK_DIV=12.288M/8=1.536M, LRC_DIV=1.536M/32=48K
    I2C_WriteNAU88L25(0x001E,  0x0000);
    I2C_WriteNAU88L25(0x001F,  0x0000);
    I2C_WriteNAU88L25(0x0020,  0x0000);
    I2C_WriteNAU88L25(0x0021,  0x0000);
    I2C_WriteNAU88L25(0x0022,  0x0000);
    I2C_WriteNAU88L25(0x0023,  0x0000);
    I2C_WriteNAU88L25(0x0024,  0x0000);
    I2C_WriteNAU88L25(0x0025,  0x0000);
    I2C_WriteNAU88L25(0x0026,  0x0000);
    I2C_WriteNAU88L25(0x0027,  0x0000);
    I2C_WriteNAU88L25(0x0028,  0x0000);
    I2C_WriteNAU88L25(0x0029,  0x0000);
    I2C_WriteNAU88L25(0x002A,  0x0000);
    I2C_WriteNAU88L25(0x002B,  0x0012);
    I2C_WriteNAU88L25(0x002C,  0x0082);
    I2C_WriteNAU88L25(0x002D,  0x0000);
    I2C_WriteNAU88L25(0x0030,  0x00CF);
    I2C_WriteNAU88L25(0x0031,  0x0000);
    I2C_WriteNAU88L25(0x0032,  0x0000);
    I2C_WriteNAU88L25(0x0033,  0x0000);
    I2C_WriteNAU88L25(0x0034,  0x0200);


    I2C_WriteNAU88L25(0x0038,  0x1486);
    I2C_WriteNAU88L25(0x0039,  0x0F12);
    I2C_WriteNAU88L25(0x003A,  0x25FF);
    I2C_WriteNAU88L25(0x003B,  0x3457);
    I2C_WriteNAU88L25(0x0045,  0x1486);
    I2C_WriteNAU88L25(0x0046,  0x0F12);
    I2C_WriteNAU88L25(0x0047,  0x25F9);
    I2C_WriteNAU88L25(0x0048,  0x3457);
    I2C_WriteNAU88L25(0x004C,  0x0000);
    I2C_WriteNAU88L25(0x004D,  0x0000);
    I2C_WriteNAU88L25(0x004E,  0x0000);
    I2C_WriteNAU88L25(0x0050,  0x2007);
    I2C_WriteNAU88L25(0x0051,  0x0000);
    I2C_WriteNAU88L25(0x0053,  0xC201);
    I2C_WriteNAU88L25(0x0054,  0x0C95);
    I2C_WriteNAU88L25(0x0055,  0x0000);
    I2C_WriteNAU88L25(0x0058,  0x1A14);
    I2C_WriteNAU88L25(0x0059,  0x00FF);
    I2C_WriteNAU88L25(0x0066,  0x0060);
    I2C_WriteNAU88L25(0x0068,  0xC300);
    I2C_WriteNAU88L25(0x0069,  0x0000);
    I2C_WriteNAU88L25(0x006A,  0x0083);
    I2C_WriteNAU88L25(0x0071,  0x0011);
    I2C_WriteNAU88L25(0x0072,  0x0260);
    I2C_WriteNAU88L25(0x0073,  0x332C);
    I2C_WriteNAU88L25(0x0074,  0x4502);
    I2C_WriteNAU88L25(0x0076,  0x3140);
    I2C_WriteNAU88L25(0x0077,  0x0000);
    I2C_WriteNAU88L25(0x007F,  0x553F);
    I2C_WriteNAU88L25(0x0080,  0x0420);
    I2C_WriteNAU88L25(0x0001,  0x07D4);

    I2C_WriteNAU88L25(0x0033,  0x00CF);
    I2C_WriteNAU88L25(0x0034,  0x02CF);

}

/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/

