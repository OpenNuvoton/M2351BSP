#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "NuMicro.h"

/* mbeTLS header files */
#include "sha256.h"
#include "platform_util.h"
#include "platform.h"

/* timer ticks - 100 ticks per second */
static volatile uint32_t  s_u32TickCnt;

void SysTick_Handler(void);
void enable_sys_tick(int ticks_per_second);
void start_timer0(void);
uint32_t  get_timer0_counter(void);
int SHA256Test(void);

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

/*
 * FIPS-180-2 test vectors
 */
static const unsigned char sha256_test_buf[3][57] =
{
    { "abc" },
    { "abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq" },
    { "" }
};

static const size_t sha256_test_buflen[3] =
{
    3, 56, 1000
};

static const unsigned char sha256_test_sum[6][32] =
{
    /*
     * SHA-224 test vectors
     */
    {   0x23, 0x09, 0x7D, 0x22, 0x34, 0x05, 0xD8, 0x22,
        0x86, 0x42, 0xA4, 0x77, 0xBD, 0xA2, 0x55, 0xB3,
        0x2A, 0xAD, 0xBC, 0xE4, 0xBD, 0xA0, 0xB3, 0xF7,
        0xE3, 0x6C, 0x9D, 0xA7
    },
    {   0x75, 0x38, 0x8B, 0x16, 0x51, 0x27, 0x76, 0xCC,
        0x5D, 0xBA, 0x5D, 0xA1, 0xFD, 0x89, 0x01, 0x50,
        0xB0, 0xC6, 0x45, 0x5C, 0xB4, 0xF5, 0x8B, 0x19,
        0x52, 0x52, 0x25, 0x25
    },
    {   0x20, 0x79, 0x46, 0x55, 0x98, 0x0C, 0x91, 0xD8,
        0xBB, 0xB4, 0xC1, 0xEA, 0x97, 0x61, 0x8A, 0x4B,
        0xF0, 0x3F, 0x42, 0x58, 0x19, 0x48, 0xB2, 0xEE,
        0x4E, 0xE7, 0xAD, 0x67
    },

    /*
     * SHA-256 test vectors
     */
    {   0xBA, 0x78, 0x16, 0xBF, 0x8F, 0x01, 0xCF, 0xEA,
        0x41, 0x41, 0x40, 0xDE, 0x5D, 0xAE, 0x22, 0x23,
        0xB0, 0x03, 0x61, 0xA3, 0x96, 0x17, 0x7A, 0x9C,
        0xB4, 0x10, 0xFF, 0x61, 0xF2, 0x00, 0x15, 0xAD
    },
    {   0x24, 0x8D, 0x6A, 0x61, 0xD2, 0x06, 0x38, 0xB8,
        0xE5, 0xC0, 0x26, 0x93, 0x0C, 0x3E, 0x60, 0x39,
        0xA3, 0x3C, 0xE4, 0x59, 0x64, 0xFF, 0x21, 0x67,
        0xF6, 0xEC, 0xED, 0xD4, 0x19, 0xDB, 0x06, 0xC1
    },
    {   0xCD, 0xC7, 0x6E, 0x5C, 0x99, 0x14, 0xFB, 0x92,
        0x81, 0xA1, 0xC7, 0xE2, 0x84, 0xD7, 0x3E, 0x67,
        0xF1, 0x80, 0x9A, 0x48, 0xA4, 0x97, 0x20, 0x0E,
        0x04, 0x6D, 0x39, 0xCC, 0xC7, 0x11, 0x2C, 0xD0
    }
};


int SHA256Test(void)
{
    int i, j, k, buflen, ret = 0;
    unsigned char *buf;
    unsigned char sha256sum[32];
    mbedtls_sha256_context ctx;
    uint32_t u32Time;


    buf = mbedtls_calloc( 1024, sizeof(unsigned char) );
    if( NULL == buf )
    {
        printf( "Buffer allocation failed\n" );
        return( 1 );
    }

    /* Initialize a SHA-256 contex */
    mbedtls_sha256_init( &ctx );

    for( i = 0; i < 6; i++ )
    {
        j = i % 3;
        k = i < 3;

        printf( "  SHA-%d test #%d: ", 256 - k * 32, j + 1 );

        enable_sys_tick(100000);
        start_timer0();

        /* Start a SHA-224 or SHA-256 checksum calculation*/
        if( ( ret = mbedtls_sha256_starts_ret( &ctx, k ) ) != 0 )
            goto fail;


        /* Feed an input buffer into an ongoing SHA-256 checksum calculation. */
        if( j == 2 )
        {
            memset( buf, 'a', buflen = 1000 );

            for( j = 0; j < 1000; j++ )
            {
                ret = mbedtls_sha256_update_ret( &ctx, buf, (size_t)buflen );
                if( ret != 0 )
                    goto fail;
            }

        }
        else
        {
            ret = mbedtls_sha256_update_ret( &ctx, sha256_test_buf[j],
                                             sha256_test_buflen[j] );
            if( ret != 0 )
                goto fail;
        }

        /* Finish the SHA-256 operation, and writes the result to the output buffer */
        if( ( ret = mbedtls_sha256_finish_ret( &ctx, sha256sum ) ) != 0 )
            goto fail;


        if( memcmp( sha256sum, sha256_test_sum[i], (size_t)(32 - k * 4 )) != 0 )
        {
            ret = 1;
            goto fail;
        }

        printf("passed");
        u32Time = get_timer0_counter();

        /* TIMER0->CNT is the elapsed us */
        printf("     takes %d us,  %d ticks\n", u32Time, s_u32TickCnt);
    }

    printf( "\n" );

    goto exit;

fail:
    printf( "failed\n" );

exit:
    mbedtls_sha256_free( &ctx );
    mbedtls_free( buf );

    return( ret );
}



