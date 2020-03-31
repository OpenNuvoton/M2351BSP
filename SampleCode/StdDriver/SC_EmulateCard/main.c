/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Emulate a smartcard and send ATR to card reader.
 *
  * @copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"


#define PLL_CLOCK       64000000


#define DEF_DEBUG       0
#if (DEF_DEBUG == 1)
#define DbgPrintf   printf
#else
#define DbgPrintf(...)
#endif

/*
    Sample code Notes:
    1. Set smartcard transmit protocol.
        SEL_PROTOCOL_T
            (0UL): for emulate T=0 transmit protocol samrtcard
            (1UL): for emulate T=1 transmit protocol samrtcard
    2. Enable smartcard working frequency.
        SC_CLK_MODE_AUTO
            This mode is using TM2(PD.0) pin to calculate reader clock frequency in activation state.
            And smartcard will use this clock frequency to response ATR to reader.
        SC_CLK_MODE_x
            If this mode is selected, smartcard will use specify clock frequency(x) to response ATR to reader.
*/

/* Select card transmission protocol */
#define SEL_PROTOCOL_T          (0UL)


/* Select one of the below SC_CLK_MODE_ to set smartcard clock frequency */
//#define SC_CLK_MODE_AUTO    /* Get initial reader clock on TM2(PD.0) pin */
#if !defined(SC_CLK_MODE_AUTO)
// #define SC_CLK_MODE_3P58M      3580000
// #define SC_CLK_MODE_3P75M      3750000
#define SC_CLK_MODE_4M         4000000
// #define SC_CLK_MODE_4P8M       4800000
// #define SC_CLK_MODE_6M         6000000
#endif


#define CARD_CD                 PC12    /* [out] */
#define CARD_PWR                PB2     /* [in] */
#define CARD_RST                PB3     /* [in] */
#define CARD_DAT                PB4     /* [out/in] SC0_DAT pin */
#define CARD_CLK                PB5     /* [out] for debug ...*/


#define SC_INS_SELECT           0xA4
#define SC_INS_GET_RESPONSE     0xC0
#define SC_INS_GET_DATA         0xCA
#define SC_INS_0xCB             0xCB
#define SC_INS_READ_BINARY      0xB0
#define SC_INS_WRITE_BINARY     0xD6

#define ERR_INVALID_CMD         0x8888

/* Card operation state */
typedef enum
{
    SC_OP_IDLE = 0,
    SC_OP_WRITE,
    SC_OP_READ,
    SC_OP_WRITE_ATR,
    SC_OP_PPS_EXCHANGE,
    SC_OP_PPS_ERROR,
} E_SC_OP_STATE;

#pragma pack(push)
#pragma pack(1)
/* Card data structure */
typedef struct
{
    SC_T            *sc;

    uint8_t         u8Protocol;

    uint8_t         *pu8TxBuf;
    uint32_t        u32TxLen;
    uint32_t        u32TxPos;

    uint8_t         *pu8RxBuf;
    uint32_t        u32RxLen;
    uint32_t        u32RxPos;

    uint8_t         *pu8RespBuf;
    uint32_t        u32RespLen;

    volatile E_SC_OP_STATE  OP_State;

    volatile uint8_t        u8RestToATR;

    int32_t         i32CmdStatus;
} SC_INFO_T;
#pragma pack(pop)

static volatile SC_INFO_T s_sSCInfo;
static volatile uint8_t s_au8SCTxBuf[300] = {0};
static volatile uint8_t s_au8SCRxBuf[300] = {0};
static volatile uint8_t s_au8CardATRBuf[48] = {0};


#if (SEL_PROTOCOL_T == 0UL)
static const uint8_t s_au8CardATR_T0[] =
{
#if defined(DEF_ATR_WITH_PPS)
    0x3B,
    0x16,
    0x94,   /* With PPS exchange */
    /* Historical bytes, 6 */
    0x71, 0x01, 0x01, 0x06, 0x02, 0x00,
#else
    0x3B,
    0x17,
    0x11,   /* No PPS exchange */
    /* Historical bytes, 7 */
    0x89, 0x01, 0x02, 0x00, 0x00, 0x41, 0xC3,
#endif
};
#endif


#if (SEL_PROTOCOL_T == 1UL)
const uint8_t g_au8CardATR_T1[] =
{
    0x3B,
    0xF7,
    0x11,   /* No PPS exchange */
    0x00,
    0x00,
    0x81,
    0x71,
    0xFE,
    0x42,
    0x00,
    /* Historical bytes, 7 */
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0x55, /* TCK */
};
#endif


/** Clock rate conversion table according to ISO */
typedef struct
{
    uint32_t u32F;
    uint32_t u32FS;
} CLOCK_RATE_CONVERSION;
/* Fi */
static const CLOCK_RATE_CONVERSION s_ClockRateConversion[] =
{
    { 372,  4000000     },
    { 372,  5000000     },
    { 558,  6000000     },
    { 744,  8000000     },
    { 1116, 12000000    },
    { 1488, 16000000    },
    { 1860, 20000000    },
    { 0,    0           },
    { 0,    0           },
    { 512,  5000000     },
    { 768,  7500000     },
    { 1024, 10000000    },
    { 1536, 15000000    },
    { 2048, 20000000    },
    { 0,    0           },
    { 0,    0           }
};

/**
 * Bit rate adjustment factor
 * The layout of this table has been slightly modified due to
 * the unavailability of floating point math support in the kernel.
 * The value D has been divided into a numerator and a divisor.
 */
typedef struct
{
    uint32_t u32DNumerator;
    uint32_t u32DDivisor;
} BIT_RATE_ADJUSTMENT;
/* Di */
static const BIT_RATE_ADJUSTMENT s_BitRateAdjustment[] =
{
    { 0,    0   },
    { 1,    1   },
    { 2,    1   },
    { 4,    1   },
    { 8,    1   },
    { 16,   1   },
    { 32,   1   },
    { 64,   1   },
    { 12,   1   },
    { 20,   1   },
    { 0,    0   },
    { 0,    0   },
    { 0,    0   },
    { 0,    0   },
    { 0,    0   },
    { 0,    0   }
};

void Init_CardInfo_And_CardPin(SC_INFO_T *sc_info, SC_T *sc, uint8_t u8Protocol);
int32_t Card_AutoFreqCheck(void);
void Card_SetFreqAfterPPSExchange(uint32_t u32Fi, uint32_t u32Di);
int32_t Card_ResponseATR(SC_INFO_T *sc_info);
void ProcessIRQ_TxRx(SC_T *sc);
int32_t Process_T0(SC_INFO_T *sc_info);
void Card_SetFreqForATR(void);
void SYS_Init(void);
void UART_Init(void);
void SC0_IRQHandler(void);
void GPB_IRQHandler(void);

void Init_CardInfo_And_CardPin(SC_INFO_T *sc_info, SC_T *sc, uint8_t u8Protocol)
{
    /* Card info. */
    {
        sc_info->sc         = sc;
        sc_info->u8Protocol = u8Protocol;

        sc_info->pu8TxBuf   = NULL;
        sc_info->u32TxLen   = 0;
        sc_info->u32TxPos   = 0;
        sc_info->pu8RxBuf   = NULL;
        sc_info->u32RxLen   = 0;
        sc_info->u32RxPos   = 0;
        sc_info->pu8RespBuf = NULL;
        sc_info->u32RespLen = 0;

        sc_info->OP_State = SC_OP_IDLE;

        sc_info->u8RestToATR = 0;

        sc_info->i32CmdStatus = -1;
    }

    /* Configure card pins */
    {
        SYS->GPB_MFPL &= ~(SC0_PWR_PB2_Msk | SC0_RST_PB3_Msk | SC0_CLK_PB5_Msk | SC0_DAT_PB4_Msk);
        SYS->GPB_MFPL |= (SC0_DAT_PB4);
        //SYS->GPB_MFPL |= (SC0_CLK_PB5); // for debug active CLK

        CARD_CD  = 1; // [out]
        CARD_PWR = 0; // [in]
        CARD_RST = 0; // [in]
        GPIO_SetMode(PC, BIT12, GPIO_MODE_OUTPUT);
        GPIO_SetMode(PB, BIT3, GPIO_MODE_INPUT);
        GPIO_SetMode(PB, BIT2, GPIO_MODE_INPUT);
    }

    /* Clear Rx/Tx buffer */
    memset((void *)(uint32_t)s_au8SCRxBuf, 0x0, sizeof(s_au8SCRxBuf));
    memset((void *)(uint32_t)s_au8SCTxBuf, 0x0, sizeof(s_au8SCRxBuf));
}


int32_t Card_AutoFreqCheck(void)
{
    uint32_t u32CAP, u32CardFreq;
    uint64_t u64WorkFreq;

    /* Set Timer0 event counting pin PD.0 */
    SYS->GPD_MFPL |= TM2_PD0;

    /* Enable TIMER peripheral clock */
    CLK_EnableModuleClock(TMR2_MODULE);
    CLK_EnableModuleClock(TMR3_MODULE);
    CLK_SetModuleClock(TMR2_MODULE, CLK_CLKSEL1_TMR2SEL_PCLK1, 0);
    CLK_SetModuleClock(TMR3_MODULE, CLK_CLKSEL1_TMR3SEL_PCLK1, 0);

    /* Give a dummy target frequency here. Will over write prescale and compare value with macro */
    TIMER_Open(TIMER2, TIMER_ONESHOT_MODE, 100);
    /* Update prescale and compare value. Calculate average frequency every 1000 events */
    TIMER_SET_PRESCALE_VALUE(TIMER2, 0);
    TIMER_SET_CMP_VALUE(TIMER2, 10000);
    /* Update Timer 1 prescale value. So Timer 0 clock is 12MHz */
    TIMER_SET_PRESCALE_VALUE(TIMER3, 0);
    TIMER_EnableFreqCounter(TIMER2, 0, 0, FALSE);
    while(TIMER_GetCaptureIntFlag(TIMER3) == 0) {}
    u32CAP = TIMER3->CAP;
    TIMER_ClearCaptureIntFlag(TIMER3);

    u64WorkFreq = ((uint64_t)SystemCoreClock * (uint64_t)10000) / (uint64_t)(u32CAP);
    u64WorkFreq = (u64WorkFreq / 1000) * 1000;
    u32CardFreq = (uint32_t)u64WorkFreq;

    if((u32CardFreq >= 3000000) && (u32CardFreq <= 12000000))
    {
        CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1UL));
        CLK_EnablePLL(CLK_PLLCTL_PLLSRC_HXT, (u32CardFreq * 10)); // set PLL from 30M to 120M
        if(u32CardFreq <= 4800000)
        {
            CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_PLL, CLK_CLKDIV0_HCLK(1UL));
        }
        else if((u32CardFreq > 4800000) && (u32CardFreq <= 9600000))
        {
            CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_PLL, CLK_CLKDIV0_HCLK(2UL));
        }
        else
        {
            CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_PLL, CLK_CLKDIV0_HCLK(3UL));
        }
        CLK_SetModuleClock(SC0_MODULE, CLK_CLKSEL3_SC0SEL_PLL, CLK_CLKDIV1_SC0(10));

        DbgPrintf("F: %d.\n", u32CardFreq);
        return (int32_t)u32CardFreq;
    }
    else
    {
        DbgPrintf("F: X. (%d)\n", u32CardFreq);
        return -1;
    }
}


void Card_SetFreqAfterPPSExchange(uint32_t u32Fi, uint32_t u32Di)
{
    (void)u32Di;

    if(s_ClockRateConversion[u32Fi].u32FS != 0)
    {
        CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1UL));
        CLK_EnablePLL(CLK_PLLCTL_PLLSRC_HXT, (s_ClockRateConversion[u32Fi].u32FS * 10));
        if(s_ClockRateConversion[u32Fi].u32FS <= 6000000)
        {
            CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_PLL, CLK_CLKDIV0_HCLK(1UL));
        }
        else if(s_ClockRateConversion[u32Fi].u32FS >= 15000000)
        {
            CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_PLL, CLK_CLKDIV0_HCLK(3UL));
        }
        else
        {
            CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_PLL, CLK_CLKDIV0_HCLK(2UL));
        }
        CLK_SetModuleClock(SC0_MODULE, CLK_CLKSEL3_SC0SEL_PLL, CLK_CLKDIV1_SC0(10));
    }
}


int32_t Card_ResponseATR(SC_INFO_T *sc_info)
{
    uint8_t i;
    uint8_t u32Fi, u32Di, u32Len;
    uint8_t u8PCK = 0;
    SC_T *sc;

    sc = sc_info->sc;

    if(sc == SC0)
        NVIC_EnableIRQ(SC0_IRQn);
    else if(sc == SC1)
        NVIC_EnableIRQ(SC1_IRQn);
    else if(sc == SC2)
        NVIC_EnableIRQ(SC2_IRQn);
    else
        while(1) {}

#if (SEL_PROTOCOL_T == 0UL)
    if(sc_info->u8Protocol == 0)
    {
        u32Len = sizeof(s_au8CardATR_T0);
        memcpy((void *)(uint32_t)s_au8CardATRBuf, s_au8CardATR_T0, u32Len);
    }
    else
    {
        printf("\nERROR. Unknow ptotocol.\n");
        while(1) {}
    }
#endif
#if (SEL_PROTOCOL_T == 1UL)
    if(sc_info->u8Protocol == 1)
    {
        u32Len = sizeof(g_au8CardATR_T1);
        memcpy((void *)s_au8CardATRBuf, g_au8CardATR_T1, u32Len);
    }
    else
    {
        printf("\nERROR. Unknow ptotocol.\n");
        while(1) {}
    }
#endif

    sc->CTL = SC_CTL_SCEN_Msk; /* Disable RX to receive data in initial state */
    sc->CTL |= (SC_CTL_RXOFF_Msk | SC_CTL_TXOFF_Msk);
    sc->ETUCTL = 0x173; // 372 * SC clocks
    sc->ALTCTL |= SC_ALTCTL_ACTEN_Msk;
    while((sc->INTSTS & SC_INTSTS_INITIF_Msk) != SC_INTSTS_INITIF_Msk);
    sc->INTSTS = SC_INTSTS_INITIF_Msk;

    sc->CTL &= ~SC_CTL_TXOFF_Msk;

    u32Fi = s_au8CardATRBuf[2] >> 4;
    u32Di = s_au8CardATRBuf[2] & 0xF;

    sc_info->OP_State = SC_OP_WRITE_ATR;
    sc_info->pu8TxBuf = (uint8_t *)(uint32_t)&s_au8CardATRBuf[0];
    sc_info->u32TxLen = u32Len;
    sc_info->u32TxPos = 0;
    sc_info->pu8RxBuf = (uint8_t *)(uint32_t)&s_au8SCRxBuf[0];
    sc_info->u32RxLen = 0;
    sc_info->u32RxPos = 0;
    /* Enable to send data in IRQ */
    SC_ENABLE_INT(sc, (SC_INTEN_TBEIEN_Msk | SC_INTEN_RDAIEN_Msk));
    while(sc_info->OP_State == SC_OP_WRITE_ATR) {}

    if(sc_info->OP_State == SC_OP_PPS_EXCHANGE)
    {
        /* Enable RX to receive data */
        sc->CTL &= ~SC_CTL_RXOFF_Msk;
        while(sc_info->OP_State == SC_OP_PPS_EXCHANGE) {}
        if(sc_info->OP_State == SC_OP_PPS_ERROR)
        {
            printf("ERROR. PPS EXCHANGE fail.\n");
            return -1;
        }
        sc->CTL |= SC_CTL_RXOFF_Msk; /* Disable RX to receive data */

        /* Calculate PCK of PPS */
        for(i = 0; i < (sc_info->u32RxLen - 1); i++)
            u8PCK ^= s_au8SCRxBuf[i];
        if(u8PCK != s_au8SCRxBuf[i])
        {
            printf("\nERROR. PPS checksum error.\n");
            return -1;
        }

        sc_info->OP_State = SC_OP_WRITE;
        sc_info->pu8TxBuf = (uint8_t *)(uint32_t)&s_au8SCRxBuf[0];
        sc_info->u32TxLen = sc_info->u32RxLen;
        sc_info->u32TxPos = 0;
        /* Enable to send data in IRQ */
        SC_ENABLE_INT(sc, SC_INTEN_TBEIEN_Msk);
        while(sc_info->OP_State == SC_OP_WRITE) {}

        /* Update parameters */
        sc->ETUCTL = (s_ClockRateConversion[u32Fi].u32F / s_BitRateAdjustment[u32Di].u32DNumerator) - 1;

        Card_SetFreqAfterPPSExchange(u32Fi, u32Di);
    }
    else
    {
        //; // No PPS exchange
    }
    if(sc_info->u8Protocol == 0)
        SC_SetBlockGuardTime(sc, 16); // for T=0
    else
        SC_SetBlockGuardTime(sc, 22); // for T=1

    /* Set card in READ state */
    memset((void *)(uint32_t)s_au8SCRxBuf, 0x0, sizeof(s_au8SCRxBuf));
    sc_info->OP_State = SC_OP_READ;
    sc_info->pu8TxBuf = (uint8_t *)(uint32_t)&s_au8SCRxBuf[0];
    sc_info->u32TxLen = sizeof(s_au8SCRxBuf);
    sc_info->u32TxPos = 0;
    sc_info->pu8RxBuf = (uint8_t *)(uint32_t)&s_au8SCRxBuf[0];
    sc_info->u32RxLen = 5;
    sc_info->u32RxPos = 0;
    sc_info->i32CmdStatus = -1;

    /* Enable RX to receive data */
    sc->CTL &= ~SC_CTL_RXOFF_Msk;

    return 0;
}


/* Return -1 means invalid command */
static int32_t Check_IsValidT0Cmd(SC_T *sc)
{
    //uint8_t u32CLA, u32INS, u32P1, u32P2, u32P3;
    uint8_t u32INS, u32P1, u32P2;

    //u32CLA  = s_sSCInfo.pu8RxBuf[0];
    u32INS  = s_sSCInfo.pu8RxBuf[1];
    u32P1   = s_sSCInfo.pu8RxBuf[2];
    u32P2   = s_sSCInfo.pu8RxBuf[3];
    //u32P3   = s_sSCInfo.pu8RxBuf[4];

    if((u32INS == SC_INS_SELECT) && (u32P1 == 0x0) && (u32P2 == 0x0))
    {
        /* Valid INS */
        SC_WRITE(sc, s_sSCInfo.pu8RxBuf[1]);
        s_sSCInfo.u32RxLen = 5 + s_sSCInfo.pu8RxBuf[4];
    }
    else if(u32INS == SC_INS_GET_RESPONSE)
    {
        /* Valid INS */
        SC_WRITE(sc, s_sSCInfo.pu8RxBuf[1]);
    }
    else
    {
        /* INS ... not supported */
        return -1;
    }

    return u32INS;
}


void ProcessIRQ_TxRx(SC_T *sc)
{
    uint32_t i;
    uint32_t u32Tmp;
    char c;

    /* Transmit buffer empty interrupt */
    if(sc->INTSTS & SC_INTSTS_TBEIF_Msk)
    {
        if((s_sSCInfo.OP_State == SC_OP_WRITE) || (s_sSCInfo.OP_State == SC_OP_WRITE_ATR))
        {
            /* We can push 4 bytes into FIFO at most due to FIFO depth limitation */
            for(i = 0; i < 4; i++)
            {
                /* Wait Tx empty */
                while((sc->STATUS & SC_STATUS_TXEMPTY_Msk) == 0ul) {}
                u32Tmp = s_sSCInfo.u32TxPos;
                SC_WRITE(sc, s_sSCInfo.pu8TxBuf[u32Tmp]);
                s_sSCInfo.u32TxPos++;
                if(s_sSCInfo.u8RestToATR == 1)
                {
                    /* Wait reader check "TS" //CLK_SysTickDelay(80000) */
                    if(s_sSCInfo.u32TxPos == 1)
                    {
                        CLK_SysTickDelay(1000);
                    }
                }
                u32Tmp = s_sSCInfo.u32TxPos;
                if(u32Tmp == s_sSCInfo.u32TxLen)
                {
                    /* Wait all Tx transfer finished */
                    while((sc->STATUS & SC_STATUS_TXACT_Msk) == SC_STATUS_TXACT_Msk) {}
                    SC_DISABLE_INT(sc, SC_INTEN_TBEIEN_Msk);
                    if(s_sSCInfo.OP_State == SC_OP_WRITE_ATR)
                    {
                        if(((s_au8CardATRBuf[2] >> 4) > 1) || ((s_au8CardATRBuf[2] & 0xF) > 1)) // Fi, Di not default value
                        {
                            s_sSCInfo.OP_State = SC_OP_PPS_EXCHANGE;
                        }
                        else
                        {
                            s_sSCInfo.OP_State = SC_OP_IDLE;
                        }
                    }
                    else
                    {
                        s_sSCInfo.OP_State = SC_OP_IDLE;
                    }
                    break;
                }
            }
        }
    }

    if(sc->INTSTS & SC_INTSTS_RDAIF_Msk)
    {
        if(s_sSCInfo.OP_State == SC_OP_PPS_EXCHANGE)
        {
            c = SC_READ(sc);
            /* Total PPSS, PPS0, [PPS1/2/3], PCK */
            u32Tmp = s_sSCInfo.u32RxPos;
            s_sSCInfo.pu8RxBuf[u32Tmp] = c;
            s_sSCInfo.u32RxPos++;

            if(s_sSCInfo.u32RxPos <= 2) // for PPSS, PPS0
                s_sSCInfo.u32RxLen++;

            if(s_sSCInfo.pu8RxBuf[0] != 0xFF) // check PPSS
            {
                //printf("\nERROR. Wrong PPSS, 0x%x.\n", s_sSCInfo.pu8RxBuf[0]);
                s_sSCInfo.OP_State = SC_OP_PPS_ERROR;
            }
            if(s_sSCInfo.u32RxPos == 2) // get PPS0
            {
                if((s_sSCInfo.pu8RxBuf[1]&BIT4) == BIT4)
                    s_sSCInfo.u32RxLen++; // support PPS1
                if((s_sSCInfo.pu8RxBuf[1]&BIT5) == BIT5)
                    s_sSCInfo.u32RxLen++; // support PPS2
                if((s_sSCInfo.pu8RxBuf[1]&BIT6) == BIT6)
                    s_sSCInfo.u32RxLen++; // support PPS3

                /* Get max RX counts */
                s_sSCInfo.u32RxLen++; // PCK
            }
            u32Tmp = s_sSCInfo.u32RxPos;
            if((u32Tmp >= 2) && (u32Tmp == s_sSCInfo.u32RxLen))
            {
                s_sSCInfo.OP_State = SC_OP_IDLE;
            }
        }
        else if(s_sSCInfo.OP_State == SC_OP_READ)
        {
            if(s_sSCInfo.u8Protocol == 0UL)
            {
                c = SC_READ(sc);
                /* For T=0 transmission protocol */
                u32Tmp = s_sSCInfo.u32RxPos;
                s_sSCInfo.pu8RxBuf[u32Tmp] = c;
                s_sSCInfo.u32RxPos++;
                if(s_sSCInfo.u32RxPos == 5)
                {
                    /* if s_sSCInfo.u32TxLen = ERR_INVALID_CMD, means invalid command */
                    s_sSCInfo.i32CmdStatus = Check_IsValidT0Cmd(sc);
                }
                u32Tmp = s_sSCInfo.u32RxPos;
                if(u32Tmp == s_sSCInfo.u32RxLen)
                    s_sSCInfo.OP_State = SC_OP_IDLE;
            }
            else if(s_sSCInfo.u8Protocol == 1UL)
            {
                c = SC_READ(sc);
                /* For T=1 transmission protocol */
                u32Tmp = s_sSCInfo.u32RxPos;
                s_sSCInfo.pu8RxBuf[u32Tmp] = c;
                s_sSCInfo.u32RxPos++;
                if(s_sSCInfo.u32RxPos == 3) // received LEN
                {
                    s_sSCInfo.u32RxLen = 1 + 1 + 1 + s_sSCInfo.pu8RxBuf[2] + 1; /* NAD, PCB, LEN, [APDU], EDC */
                }
                u32Tmp = s_sSCInfo.u32RxPos;
                if(u32Tmp == s_sSCInfo.u32RxLen)
                    s_sSCInfo.OP_State = SC_OP_IDLE;
            }
            else
            {
                printf("Unknown protocol (%d)\n", s_sSCInfo.u8Protocol);

                sc->CTL |= (SC_CTL_RXOFF_Msk | SC_CTL_TXOFF_Msk);
                NVIC_DisableIRQ(SC0_IRQn);
                s_sSCInfo.OP_State = SC_OP_IDLE;
                s_sSCInfo.u8RestToATR = 0;
                //while(1) {}
            }
        }
        else
        {
            /* Discard these data */
            c = SC_READ(sc);
            printf("SC: Unknown data %02x (STATE:%d)\n", c, s_sSCInfo.OP_State);
        }
    }
}


#if (SEL_PROTOCOL_T == 0UL)

static const uint8_t s_u8RespMF[] =
{
    0x00, 0x00, 0x00, 0x00, 0x7F,
    0x10, 0x02, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x0D, 0x13, 0x00,
    0x0A, 0x04, 0x00, 0x83, 0x8A,
    0x83, 0x8A, 0x00, 0x01, 0x00,
    0x00,
};

static const uint8_t s_u8RespDF[] =
{
    0xDF, 0x0A, 0x11, 0x22, 0x33,
    0x44, 0x55, 0x66, 0x77, 0x88,
};

static const uint8_t s_u8RespEF[] =
{
    0xEF, 0x12, 0x01, 0x02, 0x03,
    0x04, 0x05, 0x06, 0x07, 0x08,
    0x09, 0x0A, 0x0B, 0x0C, 0x0D,
    0x0E, 0x0F, 0x10,
};

int32_t Process_T0(SC_INFO_T *sc_info)
{
    SC_T *sc;
    uint8_t i;
    uint8_t u32SW1, u32SW2, u32RespLen;
    //uint8_t u8CLA, u8INS, u8P1, u8P2, u8P3, u8D[8];
    uint8_t u8INS, u8P1, u8P2, u8P3, u8D[8];

    sc = sc_info->sc;

    //u8CLA  = s_au8SCRxBuf[0];
    u8INS  = s_au8SCRxBuf[1];
    u8P1   = s_au8SCRxBuf[2];
    u8P2   = s_au8SCRxBuf[3];
    u8P3   = s_au8SCRxBuf[4];
    u8D[0] = s_au8SCRxBuf[5];
    u8D[1] = s_au8SCRxBuf[6];

    if(DEF_DEBUG) /* Enable for debug */
    {
        if(sc_info->u32TxLen == ERR_INVALID_CMD)
        {
            printf("\n---------> (%d) (Invalid)\n", (5));
            for(i = 0; i < (5); i++)
                printf("   0x%02x\n", s_au8SCRxBuf[i]);
            printf("---------\n");
        }
        else
        {
            printf("\n---------> (%d)\n", (5 + u8P3));
            for(i = 0; i < (5 + u8P3); i++)
                printf("   0x%02x\n", s_au8SCRxBuf[i]);
            printf("---------\n");
        }
    }

    do
    {
        /* Check if valid cmd exist or not */
        if(sc_info->i32CmdStatus == -1)
        {
            //// "Instruction not supported"
            //u32SW1 = 0x6D;
            //u32SW2 = 0x00;
            //// "Function not supported"
            //u32SW1 = 0x6A;
            //u32SW2 = 0x81;
            //// "File not found"
            //u32SW1 = 0x6A;
            //u32SW2 = 0x82;
            u32SW1 = 0x6D;
            u32SW2 = 0x00;

            sc_info->OP_State = SC_OP_WRITE;
            sc_info->pu8TxBuf = (uint8_t *)(uint32_t)&s_au8SCTxBuf[0];
            sc_info->u32TxLen = 2;
            sc_info->u32TxPos = 0;

            s_au8SCTxBuf[0] = u32SW1;
            s_au8SCTxBuf[1] = u32SW2;
            break;
        }

        /* Check INS */
        switch(u8INS)
        {
            case SC_INS_SELECT: // INS: SELECT FILE
                if((u8P1 != u8P2) || (u8P1 != 0x0) || (u8P3 != 0x2))
                {
                    sc_info->u32RxPos = sc_info->u32RxLen = 0;
                    printf("\nERROR. SELECT FILE, APDU data.\n");
                    return -1;
                }
                u32SW1 = 0x9F;
                /* Prepare response data and length */
                if((u8D[0] == 0x3F) && (u8D[1] == 0x00))        /* MF */
                {
                    sc_info->pu8RespBuf = (uint8_t *)(uint32_t)&s_u8RespMF[0];
                    sc_info->u32RespLen = sizeof(s_u8RespMF);
                }
                else if((u8D[0] == 0x7F) && (u8D[1] == 0x10))   /* DF */
                {
                    sc_info->pu8RespBuf = (uint8_t *)(uint32_t)&s_u8RespDF[0];
                    sc_info->u32RespLen = sizeof(s_u8RespDF);
                }
                else if((u8D[0] == 0x6F) && (u8D[1] == 0x3A))   /* EF */
                {
                    sc_info->pu8RespBuf = (uint8_t *)(uint32_t)&s_u8RespEF[0];
                    sc_info->u32RespLen = sizeof(s_u8RespEF);
                }
                else
                {
                    sc_info->u32RxPos = sc_info->u32RxLen = 0;
                    printf("\nERROR. SELECT FILE, directory.\n");
                    return -1;
                }
                u32SW2 = (uint8_t)sc_info->u32RespLen;

                sc_info->OP_State = SC_OP_WRITE;
                sc_info->pu8TxBuf = (uint8_t *)(uint32_t)&s_au8SCTxBuf[0];
                sc_info->u32TxLen = 2;
                sc_info->u32TxPos = 0;

                s_au8SCTxBuf[0] = u32SW1;
                s_au8SCTxBuf[1] = u32SW2;
                break;

            case SC_INS_GET_DATA: // INS: GET DATA
                u32SW1 = 0x90;
                u32SW2 = 0x00;

                sc_info->OP_State = SC_OP_WRITE;
                sc_info->pu8TxBuf = (uint8_t *)(uint32_t)&s_au8SCTxBuf[0];
                sc_info->u32TxLen = 2;
                sc_info->u32TxPos = 0;

                s_au8SCTxBuf[0] = u32SW1;
                s_au8SCTxBuf[1] = u32SW2;
                break;

            case SC_INS_GET_RESPONSE: // INS: GET RESPONSE
                if((u8P1 != u8P2) || (u8P1 != 0x0) || (u8P3 == 0x0))
                {
                    u32RespLen = sc_info->u32RespLen = 0;
                    sc_info->u32RxPos = sc_info->u32RxLen = 0;
                    printf("\nERROR. 0x%x cmd data.\n", u8INS);
                    return -1;
                }
                if(sc_info->u32RespLen == 0)
                {
                    u32RespLen = sc_info->u32RespLen = 0;
                    u32SW1 = 0x94; /* No "EF" selected */
                    u32SW2 = 0x00;
                }
                else if(sc_info->u32RespLen < u8P3)
                {
                    u32RespLen = sc_info->u32RespLen = 0;
                    u32SW1 = 0x94; /* Over the selected size */
                    u32SW2 = 0x02;
                }
                else
                {
                    u32RespLen = u8P3;
                    u32SW1 = 0x90; /* Success */
                    u32SW2 = 0x00;
                }

                /* Data: [Data] + SW1 + SW2 */
                memcpy((void *)(uint32_t)(s_au8SCTxBuf), sc_info->pu8RespBuf, u32RespLen);
                s_au8SCTxBuf[u32RespLen + 0] = u32SW1;
                s_au8SCTxBuf[u32RespLen + 1] = u32SW2;

                sc_info->OP_State = SC_OP_WRITE;
                sc_info->pu8TxBuf = (uint8_t *)(uint32_t)&s_au8SCTxBuf[0];
                sc_info->u32TxLen = u32RespLen + 2;
                sc_info->u32TxPos = 0;
                break;

            default:
                sc_info->u32RxPos = sc_info->u32RxLen = 0;
                printf("\nERROR. T=0. Wrong INS (0x%x).\n", u8INS);
                return -1;
        }
    }
    while(0);

    printf("Receive T=0 INS: 0x%02x, D0/1: 0x%02x/0x%02x. Response SW1/SW2: 0x%02x/0x%02x.\n", u8INS, u8D[0], u8D[1], u32SW1, u32SW2);

    /* Enable to send data in IRQ */
    SC_ENABLE_INT(sc, SC_INTEN_TBEIEN_Msk);
    while(sc_info->OP_State == SC_OP_WRITE) {}

    return 0;
}

#endif /* End of SEL_PROTOCOL_T == 0UL */


#if (SEL_PROTOCOL_T == 1UL)

#define SC_T1_BLOCK_S_IFS_REQ       (0xC1UL)
#define SC_T1_BLOCK_S_IFS_RES       (0xE1UL)
#define SC_T1_BLOCK_I               (0x00UL)
#define SC_T1_BLOCK_R               (0x80UL)
#define SC_T1_BLOCK_S               (0xC0UL)


/* Example:
    00A40000023F00      ... SELECT
    00B0000007          ... READ 7 bytes
    00D6000003112233    ... WRITE 3 bytes, 11 22 33
*/
static int32_t Check_IsValidT1Cmd(void)
{
    //uint8_t u8NAD, u8PCB, u8LEN;
    //uint8_t u8CLA, u8INS, u8P1, u8P2, u8P3;
    uint8_t u8INS, u8P1, u8P2;

    //u8NAD = s_au8SCRxBuf[0];
    //u8PCB = s_au8SCRxBuf[1];
    //u8LEN = s_au8SCRxBuf[2];
    //u8CLA = s_au8SCRxBuf[3];
    u8INS = s_au8SCRxBuf[4];
    u8P1  = s_au8SCRxBuf[5];
    u8P2  = s_au8SCRxBuf[6];
    //u8P3  = s_au8SCRxBuf[7];

    if((u8INS == SC_INS_SELECT) && (u8P1 == 0x0) && (u8P2 == 0x0))
    {
        return SC_INS_SELECT;
    }
    else if(u8INS == SC_INS_READ_BINARY)
    {
        return SC_INS_SELECT;
    }
    else if(u8INS == SC_INS_WRITE_BINARY)
    {
        return SC_INS_SELECT;
    }
    else
    {
        return -1;
    }
}


volatile uint32_t g_u32FileID = 0;
volatile uint32_t g_u32MaxFileSize = 0;
volatile uint8_t g_au8FileBuf[256];
int32_t Process_T1(SC_INFO_T *sc_info)
{
    SC_T *sc;
    uint32_t u32RxLen, u32TxLen;
    //uint8_t u8NAD, u8PCB, u8LEN, u8CLA, u8INS, u8EDC = 0;
    uint8_t u8PCB, u8INS, u8EDC = 0;
    uint8_t u8Data, u8RespIFLen, u8InDataLen;
    uint8_t *pu8RxBuf, *pu8TxBuf;
    uint8_t i;

    sc = sc_info->sc;
    u32RxLen = sc_info->u32RxLen;
    pu8RxBuf = (uint8_t *)&s_au8SCRxBuf[0];
    pu8TxBuf = (uint8_t *)&s_au8SCTxBuf[0];

    //u8NAD = s_au8SCRxBuf[0];
    u8PCB = s_au8SCRxBuf[1];
    //u8LEN = s_au8SCRxBuf[2];
    //u8CLA = s_au8SCRxBuf[3];
    u8INS = s_au8SCRxBuf[4];
    u8EDC = s_au8SCRxBuf[u32RxLen - 1];

    /* Calculate and check received EDC */
    u8Data = 0;
    for(i = 0; i < (u32RxLen - 1); i++)
        u8Data ^= s_au8SCRxBuf[i];
    if(u8EDC != u8Data)
    {
        sc_info->u32RxPos = sc_info->u32RxLen = 0;
        printf("\nERROR. Wrong EDC. (0x%x)\n", u8Data);
        return -1;
    }

    if(DEF_DEBUG) /* Enable for debug */
    {
        printf("\n---------> (%d)\n", u32RxLen);
        for(i = 0; i < u32RxLen; i++)
            printf("   0x%02x\n", s_au8SCRxBuf[i]);
        printf("---------<\n");
    }

    do
    {
        /* Process IFS_REQ */
        if(u8PCB == SC_T1_BLOCK_S_IFS_REQ)
        {
            u32TxLen = u32RxLen;

            g_u32MaxFileSize = s_au8SCRxBuf[3];

            /* Set default buffer data */
            for(i = 0; i < g_u32MaxFileSize; i++)
                g_au8FileBuf[i] = 0xA0 + i;

            /* Prepare response data to reader */
            memcpy((void *)(s_au8SCTxBuf), (void *)(s_au8SCRxBuf), u32TxLen);
            /* u8PCB */
            s_au8SCTxBuf[1] = SC_T1_BLOCK_S_IFS_RES;
            /* Calculate EDC */
            s_au8SCTxBuf[u32TxLen - 1] = 0x0;
            for(i = 0; i < (u32TxLen - 1); i++)
                s_au8SCTxBuf[u32TxLen - 1] ^= pu8TxBuf[i];
            printf("    [T=1 IFS_REQ. PCB: 0x%02x. Response EDC: 0x%02x.]\n", u8PCB, s_au8SCTxBuf[u32TxLen - 1]);
            break;
        }

        /* Check if valid cmd exist or not */
        s_sSCInfo.i32CmdStatus = Check_IsValidT1Cmd();
        if(s_sSCInfo.i32CmdStatus < 0)
        {
            u8RespIFLen = 2; /* "SW1-SW0" */

            u32TxLen = 1 + 1 + 1 + u8RespIFLen + 1; /* NAD, PCB, LEN, [IF], EDC */

            /* SW1-SW0 */
            s_au8SCTxBuf[u32TxLen - 3] = 0x6D;
            s_au8SCTxBuf[u32TxLen - 2] = 0x00;

            /* NAD, PCB */
            memcpy((void *)(s_au8SCTxBuf), (void *)(s_au8SCRxBuf), 2);
            /* LEN */
            s_au8SCTxBuf[2] = u8RespIFLen;
            /* INFO FIELD */
            memcpy((void *)&s_au8SCTxBuf[3], (void *)(g_au8FileBuf), u8RespIFLen - 2);
            /* Calculate EDC */
            s_au8SCTxBuf[u32TxLen - 1] = 0x0;
            for(i = 0; i < (u32TxLen - 1); i++)
                s_au8SCTxBuf[u32TxLen - 1] ^= pu8TxBuf[i];
            break;
        }

        /* Check [INS] to parse received TPDU */
        switch(u8INS)
        {
            case SC_INS_SELECT: // SELECT FILE
                /* Get FILE ID */
                g_u32FileID = (pu8RxBuf[8] << 8) | pu8RxBuf[9];

                u8RespIFLen = 2; /* "SW1-SW0" */

                u32TxLen = 1 + 1 + 1 + u8RespIFLen + 1; /* NAD, PCB, LEN, [IF], EDC */

                /* SW1-SW0 */
                s_au8SCTxBuf[u32TxLen - 3] = 0x90;
                s_au8SCTxBuf[u32TxLen - 2] = 0x00;
                break;

            case SC_INS_READ_BINARY: // READ BINARY
                if(g_u32MaxFileSize == 0)
                {
                    sc_info->u32RxPos = sc_info->u32RxLen = 0;
                    printf("Invalid File Size: 0x%x.\n", g_u32MaxFileSize);
                    return -1;
                }

                u8RespIFLen = s_au8SCRxBuf[7] + 2; /* Response data + "SW1-SW0" */

                u32TxLen = 1 + 1 + 1 + u8RespIFLen + 1; /* NAD, PCB, LEN, [IF], EDC */

                /* SW1-SW0 */
                s_au8SCTxBuf[u32TxLen - 3] = 0x90;
                s_au8SCTxBuf[u32TxLen - 2] = 0x00;
                break;

            case SC_INS_WRITE_BINARY: // WRITE BINARY
                if(g_u32MaxFileSize == 0)
                {
                    sc_info->u32RxPos = sc_info->u32RxLen = 0;
                    printf("Invalid File Size: 0x%x.\n", g_u32MaxFileSize);
                    return -1;
                }

                /* Update data to buffer */
                u8InDataLen = s_au8SCRxBuf[7]; /* Write-in data length */
                memcpy((void *)(g_au8FileBuf), (void *)(s_au8SCRxBuf + 8), u8InDataLen);

                u8RespIFLen = 2; /* "SW1-SW0" */

                u32TxLen = 1 + 1 + 1 + u8RespIFLen + 1; /* NAD, PCB, LEN, [IF], EDC */

                /* SW1-SW0 */
                s_au8SCTxBuf[u32TxLen - 3] = 0x90;
                s_au8SCTxBuf[u32TxLen - 2] = 0x00;
                break;
        }

        /* NAD, PCB */
        memcpy((void *)(s_au8SCTxBuf), (void *)(s_au8SCRxBuf), 2);
        /* LEN */
        s_au8SCTxBuf[2] = u8RespIFLen;
        /* INFO FIELD */
        memcpy((void *)&s_au8SCTxBuf[3], (void *)(g_au8FileBuf), u8RespIFLen - 2);
        /* Calculate EDC */
        s_au8SCTxBuf[u32TxLen - 1] = 0x0;
        for(i = 0; i < (u32TxLen - 1); i++)
            s_au8SCTxBuf[u32TxLen - 1] ^= pu8TxBuf[i];
    }
    while(0);

    if(DEF_DEBUG) /* Enable for debug */
    {
        printf("Tx data...\n");
        for(i = 0; i < u32TxLen; i++)
            printf("   %0d: 0x%02x\n", i, s_au8SCTxBuf[i]);
    }

    sc_info->OP_State = SC_OP_WRITE;
    sc_info->pu8TxBuf = (uint8_t *)&s_au8SCTxBuf[0];
    sc_info->u32TxLen = u32TxLen;
    sc_info->u32TxPos = 0;

    /* Enable to send data in IRQ */
    SC_ENABLE_INT(sc, SC_INTEN_TBEIEN_Msk);
    while(sc_info->OP_State == SC_OP_WRITE) {}

    printf("Receive T=1 INS: 0x%02x. Response EDC: 0x%02x.\n", u8INS, s_au8SCTxBuf[u32TxLen - 1]);
    return 0;
}

#endif /* End of SEL_PROTOCOL_T == 1UL */


void SC0_IRQHandler(void)
{
    ProcessIRQ_TxRx(SC0);
}


void GPB_IRQHandler(void)
{
    /* Get RST_PIN high */
    if(GPIO_GET_INT_FLAG(PB, BIT3))
    {
        GPIO_CLR_INT_FLAG(PB, BIT3);
        if(CARD_PWR == 1)
        {
            s_sSCInfo.u8RestToATR = 1;
        }
    }
}

void Card_SetFreqForATR(void)
{
#if defined(SC_CLK_MODE_AUTO)
    CLK_SetCoreClock(PLL_CLOCK);
    CLK_EnableModuleClock(SC0_MODULE);
    CLK_SetModuleClock(SC0_MODULE, CLK_CLKSEL3_SC0SEL_PCLK0, CLK_CLKDIV1_SC0(10));
#elif defined(SC_CLK_MODE_3P58M)
    CLK_SetCoreClock(SC_CLK_MODE_3P58M * 10);
    CLK_EnableModuleClock(SC0_MODULE);
    CLK_SetModuleClock(SC0_MODULE, CLK_CLKSEL3_SC0SEL_PCLK0, CLK_CLKDIV1_SC0(10));
#elif defined(SC_CLK_MODE_3P75M)
    CLK_SetCoreClock(SC_CLK_MODE_3P75M * 10);
    CLK_EnableModuleClock(SC0_MODULE);
    CLK_SetModuleClock(SC0_MODULE, CLK_CLKSEL3_SC0SEL_PCLK0, CLK_CLKDIV1_SC0(10));
#elif defined(SC_CLK_MODE_4M)
    CLK_SetCoreClock(SC_CLK_MODE_4M * 10);
    CLK_EnableModuleClock(SC0_MODULE);
    CLK_SetModuleClock(SC0_MODULE, CLK_CLKSEL3_SC0SEL_PCLK0, CLK_CLKDIV1_SC0(10));
#elif defined(SC_CLK_MODE_4P8M)
    CLK_SetCoreClock(SC_CLK_MODE_4P8M * 10);
    CLK_EnableModuleClock(SC0_MODULE);
    CLK_SetModuleClock(SC0_MODULE, CLK_CLKSEL3_SC0SEL_PCLK0, CLK_CLKDIV1_SC0(10));
#elif defined(SC_CLK_MODE_6M)
    CLK_SetCoreClock(SC_CLK_MODE_6M * 8);
    CLK_EnableModuleClock(SC0_MODULE);
    CLK_SetModuleClock(SC0_MODULE, CLK_CLKSEL3_SC0SEL_PCLK0, CLK_CLKDIV1_SC0(8));
#else
    -- - Without SC_CLK_SEL_xxx -- -
#endif
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Switch HCLK clock source to HIRC */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable HXT */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Enable PLL */
    CLK->PLLCTL = CLK_PLLCTL_128MHz_HIRC;

    /* Waiting for PLL stable */
    CLK_WaitClockReady(CLK_STATUS_PLLSTB_Msk);

    /* Select HCLK clock source as PLL and HCLK source divider as 2 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_PLL, CLK_CLKDIV0_HCLK(2));

    /* Set SysTick source to HCLK/2 */
    CLK_SetSysTickClockSrc(CLK_CLKSEL0_STCLKSEL_HCLK_DIV2);

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & (~(UART0_RXD_PB12_Msk | UART0_TXD_PB13_Msk))) | UART0_RXD_PB12 | UART0_TXD_PB13;
}

void UART_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART module */
    SYS_ResetModule(UART0_RST);

    /* Configure UART and set UART Baudrate */
    UART_Open(DEBUG_PORT, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    SC_T *sc;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

#if (DEF_DEBUG == 1)
    /* Debug I/O */
    PB->MODE = (PB->MODE & ~GPIO_MODE_MODE15_Msk) | (1 << GPIO_MODE_MODE15_Pos);
    PB15 = 1;
#endif

    /* Init UART for printf */
    UART_Init();

    /* Init card info and I/O pins */
    Init_CardInfo_And_CardPin((SC_INFO_T *)(uint32_t)&s_sSCInfo, SC0, SEL_PROTOCOL_T);

    printf("\n\nCPU @ %dHz \n", SystemCoreClock);
    printf("+--------------------------------------------+\n");
    printf("|    Emulate SC0 interface as a Smartcard    |\n");
    printf("+--------------------------------------------+\n");
    printf("# Smartcard I/O configuration:\n");
    printf("    SC0PWR (PB.2)  <--  input from READER_PWR pin\n");
    printf("    SC0RST (PB.3)  <--  input from READER_RST pin\n");
    printf("    SC0DAT (PB.4)  <--> connect with READER_DAT pin\n");
    printf("    SC0CD  (PC.12) -->  output to READER_CD pin\n");
#if defined(SC_CLK_MODE_AUTO)
    printf("    TM2    (PD.0) <--  input from READER_CLK pin to calculate clock frequency.\n");
#endif
    printf("\n\n");

    Card_SetFreqForATR();

    printf("Trigger CD_INSERT event.\n\n");
    CARD_CD = 0; /* Trigger reader active... */

    sc = s_sSCInfo.sc;
    sc->CTL = SC_CTL_SCEN_Msk; /* Disable RX to receive data in initial state */
    sc->CTL |= (SC_CTL_RXOFF_Msk | SC_CTL_TXOFF_Msk);
    sc->ETUCTL = 0x173; // 372 * SC clocks
    sc->ALTCTL |= SC_ALTCTL_ACTEN_Msk;
    while((sc->INTSTS & SC_INTSTS_INITIF_Msk) != SC_INTSTS_INITIF_Msk);
    sc->INTSTS = SC_INTSTS_INITIF_Msk;
#if (DEF_DEBUG == 1)
    PB15 = 0;
#endif

    /* Enable CARD_RST rising interrupt event */
    GPIO_EnableInt(PB, 3, GPIO_INT_RISING);
    NVIC_EnableIRQ(GPB_IRQn);

    while(1)
    {
        if(s_sSCInfo.u8RestToATR == 1)
        {
#if (DEF_DEBUG == 1)
            PB15 ^= 1;
#endif
#if defined(SC_CLK_MODE_AUTO)
            if(Card_AutoFreqCheck() > 0) // 1. Get valid reader clock. 2. Adjust card working frequency
            {
                if(Card_ResponseATR((SC_INFO_T *)&s_sSCInfo) < 0)
                {
                    s_sSCInfo.u8RestToATR = 0;
                    printf("[FAIL. Send ATR. (A)]\n\n");
                }
                else
                {
                    s_sSCInfo.u8RestToATR = 0x5A;
                    printf("[PASS. Send ATR. (A)]\n\n");
                }
            }
#else
            Card_SetFreqForATR();
            if(Card_ResponseATR((SC_INFO_T *)(uint32_t)&s_sSCInfo) < 0)
            {
                s_sSCInfo.u8RestToATR = 0;
                printf("[FAIL. Send ATR. (M)]\n\n");
            }
            else
            {
                s_sSCInfo.u8RestToATR = 0x5A;
                printf("[PASS. Send ATR. (M)]\n\n");
            }
#endif
        }

        if(s_sSCInfo.u8RestToATR == 0x5A)
        {
            /* Process received data */
            /* Process cmd&&data from reader, then change OP_State to SC_OP_IDLE */
            while((s_sSCInfo.OP_State == SC_OP_READ) && (s_sSCInfo.u8RestToATR == 0x5A)) {}

            if(s_sSCInfo.u8RestToATR == 1)
                continue; /* Got RST event again. Send ATR to reader */

#if (SEL_PROTOCOL_T == 0UL)
            if(s_sSCInfo.u8Protocol == 0)
            {
                if(Process_T0((SC_INFO_T *)(uint32_t)&s_sSCInfo) < 0)
                {
                    sc->CTL |= (SC_CTL_RXOFF_Msk | SC_CTL_TXOFF_Msk);
                    s_sSCInfo.OP_State = SC_OP_IDLE;
                    s_sSCInfo.u8RestToATR = 0;
                    printf("\nERROR. Process T0.\n");
                    continue;
                }
            }
#endif
#if (SEL_PROTOCOL_T == 1UL)
            if(s_sSCInfo.u8Protocol == 1)
            {
                if(Process_T1((SC_INFO_T *)&s_sSCInfo) < 0)
                {
                    sc->CTL |= (SC_CTL_RXOFF_Msk | SC_CTL_TXOFF_Msk);
                    s_sSCInfo.OP_State = SC_OP_IDLE;
                    s_sSCInfo.u8RestToATR = 0;
                    printf("\nERROR. Process T1.\n");
                    continue;
                }
            }
#endif

            /* Set in READ state */
            memset((void *)(uint32_t)s_au8SCRxBuf, 0x0, sizeof(s_au8SCRxBuf));
            s_sSCInfo.OP_State = SC_OP_READ;
            s_sSCInfo.pu8TxBuf = (uint8_t *)(uint32_t)&s_au8SCRxBuf[0];
            s_sSCInfo.u32TxLen = sizeof(s_au8SCRxBuf);
            s_sSCInfo.u32TxPos = 0;
            s_sSCInfo.pu8RxBuf = (uint8_t *)(uint32_t)&s_au8SCRxBuf[0];
            s_sSCInfo.u32RxLen = 5;
            s_sSCInfo.u32RxPos = 0;
            s_sSCInfo.i32CmdStatus = -1;
        }
    }
}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
