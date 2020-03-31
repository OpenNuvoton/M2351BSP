#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "NuMicro.h"

/* mbeTLS header files */
#include "ecp.h"
#include "ecdsa.h"

#define CRYPTO_MAX_KEY_LEN 1024

static unsigned char s_aucBuf[CRYPTO_MAX_KEY_LEN];
static unsigned char s_aucTmp[CRYPTO_MAX_KEY_LEN];

/* timer ticks - 100 ticks per second */
static volatile uint32_t  s_u32TickCnt;

void SysTick_Handler(void);
void enable_sys_tick(int ticks_per_second);
void start_timer0(void);
uint32_t  get_timer0_counter(void);
int ECDSATest(void);

void SysTick_Handler(void)
{
    s_u32TickCnt++;
}


void enable_sys_tick(int ticks_per_second)
{
    s_u32TickCnt = 0;
    SystemCoreClock = 64000000;         /* HCLK is 64 MHz */
    if(SysTick_Config(SystemCoreClock / (uint32_t)ticks_per_second))
    {
        /* Setup SysTick Timer for 1 second interrupts  */
        printf("Set system tick error!!\n");
        while(1);
    }
}

void start_timer0()
{
    /* Start TIMER0  */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & (~CLK_CLKSEL1_TMR0SEL_Msk)) | CLK_CLKSEL1_TMR0SEL_HXT;
    CLK->APBCLK0 |= CLK_APBCLK0_TMR0CKEN_Msk;    /* enable TIMER0 clock                  */
    TIMER0->CTL = 0;                   /* disable timer                                  */
    TIMER0->INTSTS = (TIMER_INTSTS_TWKF_Msk | TIMER_INTSTS_TIF_Msk);  /* clear interrupt status */
    TIMER0->CMP = 0xFFFFFE;            /* maximum time                                   */
    TIMER0->CNT = 0;                   /* clear timer counter                            */
    /* start timer */
    TIMER0->CTL = (11 << TIMER_CTL_PSC_Pos) | TIMER_ONESHOT_MODE | TIMER_CTL_CNTEN_Msk;
}

uint32_t  get_timer0_counter()
{
    return TIMER0->CNT;
}

static int myrand( void *rng_state, unsigned char *output, size_t len )
{
#if !defined(__OpenBSD__)
    size_t i;

    if( rng_state != NULL )
        rng_state  = NULL;

    for( i = 0; i < len; ++i )
        output[i] = (unsigned char)rand();
#else
    if( rng_state != NULL )
        rng_state = NULL;

    arc4random_buf( output, len );
#endif /* !OpenBSD */

    return( 0 );
}





int ECDSATest(void)
{
    int ret = 0;
    uint32_t u32Time;
    mbedtls_ecdsa_context ecdsa;
    size_t sig_len;
    const mbedtls_ecp_curve_info *curve_info = mbedtls_ecp_curve_info_from_grp_id(MBEDTLS_ECP_DP_SECP256K1);

    /* Initializes an ECDSA context. */
    mbedtls_ecdsa_init( &ecdsa );
    memset( s_aucBuf, 0x2A, sizeof( s_aucBuf ) );

    printf(" mbedtls ECDSA init done  \n\n");

    enable_sys_tick(1000);
    start_timer0();

    printf(" mbedtls ECDSA generate key       : ");
    ret = mbedtls_ecdsa_genkey( &ecdsa, curve_info->grp_id, myrand, NULL );
    if(ret == 0)
    {
        printf("passed");
        u32Time = get_timer0_counter();

        /* TIMER0->CNT is the elapsed us */
        printf("     takes %d.%d seconds,  %d ticks\n", u32Time / 1000000, u32Time / 1000, s_u32TickCnt);
    }
    else
    {
        printf("failed! ret[%d]\n", ret);
        mbedtls_ecdsa_free(&ecdsa);
        return ret;
    }

    enable_sys_tick(1000);
    start_timer0();

    printf(" mbedtls ECDSA key pair           : ");
    ret = mbedtls_ecdsa_from_keypair( &ecdsa, &ecdsa );
    if(ret == 0)
    {
        printf("passed");
        u32Time = get_timer0_counter();

        /* TIMER0->CNT is the elapsed us */
        printf("     takes %d.%d seconds,  %d ticks\n", u32Time / 1000000, u32Time / 1000, s_u32TickCnt);
    }
    else
    {
        printf("failed! ret[%d]\n", ret);
        mbedtls_ecdsa_free(&ecdsa);
        return ret;
    }

    enable_sys_tick(1000);
    start_timer0();

    /* Computes the ECDSA signature and writes it to a buffer */
    printf(" mbedtls ECDSA signature generate : ");
    ret = mbedtls_ecdsa_write_signature( &ecdsa, MBEDTLS_MD_SHA256, s_aucBuf, curve_info->bit_size, s_aucTmp, &sig_len, myrand, NULL);
    if(ret == 0)
    {
        printf("passed");
        u32Time = get_timer0_counter();

        /* TIMER0->CNT is the elapsed us */
        printf("     takes %d.%d seconds,  %d ticks\n", u32Time / 1000000, u32Time / 1000, s_u32TickCnt);
    }
    else
    {
        printf("failed! ret[%d]\n", ret);
        mbedtls_ecdsa_free(&ecdsa);
        return ret;
    }

    enable_sys_tick(1000);
    start_timer0();

    /* Reads and verifies an ECDSA signature */
    printf(" mbedtls ECDSA signature verify   : ");
    ret = mbedtls_ecdsa_read_signature(&ecdsa, s_aucBuf, curve_info->bit_size, s_aucTmp, sig_len);
    if(ret == 0)
    {
        printf("passed");
        u32Time = get_timer0_counter();

        /* TIMER0->CNT is the elapsed us */
        printf("     takes %d.%d seconds,  %d ticks\n", u32Time / 1000000, u32Time / 1000, s_u32TickCnt);
    }
    else
    {
        printf("failed! ret[%d]\n", ret);
        mbedtls_ecdsa_free(&ecdsa);
        return ret;
    }

    mbedtls_ecdsa_free(&ecdsa);

    printf("\n mbedtls ECDSA done  \n");
    return ret;
}


