#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "NuMicro.h"

/* mbeTLS header files */
#include "ecdh.h"


#define CRYPTO_DH_MAX_KEY_LEN 1024

static unsigned char s_aucBuf[CRYPTO_DH_MAX_KEY_LEN];

/* timer ticks - 100 ticks per second */
static volatile uint32_t  s_u32TickCnt;

void SysTick_Handler(void);
void enable_sys_tick(int ticks_per_second);
void start_timer0(void);
uint32_t  get_timer0_counter(void);
int ECDHTest(void);

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





int ECDHTest(void)
{
    int ret = 0;
    uint32_t u32Time;
    size_t size;
    mbedtls_ecp_group_id curve;
    mbedtls_ecdh_context ecdh;


    /* Initializes an ECDH context. */
    mbedtls_ecdh_init(&ecdh);
    printf(" mbedtls ECDH init done  \n\n");

    /* Set up the ECDH context with the information given. */
    /* Adapts P256 curve                                   */
    curve = MBEDTLS_ECP_DP_SECP256K1;

    enable_sys_tick(1000);
    start_timer0();

    printf(" mbedtls ECDH setup        : ");
    ret = mbedtls_ecdh_setup (&ecdh, curve);

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
        return ret;
    }

    enable_sys_tick(1000);
    start_timer0();

    /* Generates an EC key pair and exports its in the format used in a TLS ServerKeyExchange handshake message. */
    printf(" mbedtls ECDH make params  : ");
    ret = mbedtls_ecdh_make_params(&ecdh, &size, s_aucBuf, CRYPTO_DH_MAX_KEY_LEN, myrand, NULL);

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
        return ret;
    }

    enable_sys_tick(1000);
    start_timer0();

    /* Generate a public key and exports it as a TLS ClientKeyExchange payload. */
    printf(" mbedtls ECDH make public  : ");
    ret = mbedtls_ecdh_make_public(&ecdh, &size, s_aucBuf, CRYPTO_DH_MAX_KEY_LEN, myrand, NULL);

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
        return ret;
    }

    enable_sys_tick(1000);
    start_timer0();

    /* Parse and process the ECDHE payload of a TLS ClientKeyExchange message*/
    printf(" mbedtls ECDH read public  : ");
    ret = mbedtls_ecdh_read_public(&ecdh, s_aucBuf, size);

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
        return ret;
    }

    enable_sys_tick(1000);
    start_timer0();

    /* Derives and exports the shared secret.                         */
    /* This is the last function used by both TLS client and servers. */
    printf(" mbedtls ECDH calc secret  : ");
    ret = mbedtls_ecdh_calc_secret(&ecdh, &size, s_aucBuf, CRYPTO_DH_MAX_KEY_LEN, myrand, NULL);

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
        return ret;
    }

    mbedtls_ecdh_free(&ecdh);
    return ret;
}


