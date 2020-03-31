#include <stdio.h>
#include "NuMicro.h"

/* BEGIN_HEADER */
#include "rsa.h"
#include "md2.h"
#include "md4.h"
#include "md5.h"
#include "sha1.h"
#include "sha256.h"
#include "sha512.h"
#include "entropy.h"
#include "ctr_drbg.h"
#include "dhm.h"
#include "aes.h"
#include "entropy.h"
#include "ctr_drbg.h"
#include "bignum.h"
#include "x509.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

static const char SSL_USER_PRIV_KEY_PEM[] =
    "-----BEGIN RSA PRIVATE KEY-----\n"
    "MIIEowIBAAKCAQEAqwuUQKLDi53mVp6MmYtsGFp8ot0JHHq2RyMFjIRSgALdCzai\n"
    "CoaeBCCNThAmhakQE79TUz57VC2Vnaj8KIsxWIcDeti8/zo39MfykUDdo8pp4Sn5\n"
    "Yi3m/g5d18+RU0pOi2JG5EKnNHOtjjEqnXWi4qRUp/cVw24ogy3+4SGN73Qqh8Fs\n"
    "UdBjccVh7X2oiDWb8gFX6emY2UAoSUY1IHvEccvPNS657fyz5x4nLOxa4+MWJVih\n"
    "7I8cXyANVZ/bxcngpQWzzQN7Jyp715+ui51xBj/posNYMlce+F6fCgAhsepSxhGh\n"
    "ahqeuDkKreJ3n6nNWRp01Xf7H1sbBE6GltAz0QIDAQABAoIBAAzl7KILJA/NMmdp\n"
    "wVR6zQXxHODzJhK9ti0bGPoFqGr6zExiLEn66MOK6NzwHteJbirvDIuEdKxeW5/t\n"
    "9EXiaTAxzjNfULE2ZK3Svhnx+ES3qNBP5/xdVcPmtXDmuCC9w7qDCLGBzTYJWxcT\n"
    "4hDJpCTPG4sm+L8p+Wga+dNkQl3CFyHHINDZ0pKcP0kDDt6inKfiU7uU4lFYbCZy\n"
    "PceUgIOTQiNVoPQYtkHgZAtmD9rcwdq2/0GZEbzTkZuSE9S8+WlGxJP5xMGzeVsv\n"
    "zZ/scx0LM7fz5Zq0lsvAwSB1mcs04DaaNpU7Z0tXDIS249RTqdtpPkJzmevpAGhF\n"
    "VNe30/kCgYEA4rflfqyw/YHWKRxCGJRO+q0gPvlBIes30noz5Hxl0knb/J5Ng4Nx\n"
    "xMaIMZgCbwHbw5i01JOPvVKICROKb8wkli4Y2eVzxMPKk2CSpji16RQZ4eOl3YXL\n"
    "1Vnn07Ei+GpsGgDNF0HWf/Ur7es/KdAPCWbKJyoSR90+WN29gP2+Zp8CgYEAwSLv\n"
    "Kt/vdd6XKnR9xR3IajsW/X2GR/x/m2JffJPOP6VpDTKAbv86hVHDV0oBEDMDc7qy\n"
    "023ognyFCPb9Gzol2lq8egjMsisA2bgoB9HqldrSYlaZ0wPe0QJBf1gZ29jPyVJ0\n"
    "ciaBbNbSRhwTrwet7Bae9EbpJsyvBxVh00v0f48CgYEAvKQKviXudmCL01UB4fW0\n"
    "6XsXs44tlY1juyuW9exTxG9ULZOCJ4U9Kl+OfsVecQL42ny7KY1GMl7zdanerDsN\n"
    "zi+42cTDWNsYORxHqSrSoYbqKjwCjJmBCppt/IQM9umF3PUBsPJFCd7zmFj/C0lk\n"
    "2Yu/dGrbHxSFheeqgCOhQz0CgYBfZxdHUYji64o2cYay+QxH1Vp86yWKp6KNKeHL\n"
    "EuP9soKa/0hMDA1nT8UzeB3gV6Kr5xxwrkj9M+8vR3otmeKa4tlZWsFqfS2VXo9/\n"
    "lWTQk1/7LZYckzvceMXL1sQnQgkaBH366SRjlBYYhcP/YMa76Uypk+GVxePrltdU\n"
    "3Z8v5wKBgEXL38yc9LqTIWe1U40ZZKvp2A8c86jtstorEEFqXharE8kxcEpL8ZLL\n"
    "wjgPKdfNMIuApHSrhG7a7gU1rgJyDy1sOIwSvgTYrWfITPTVu5owvSZEblx4KYOm\n"
    "g8hke3Oego4v9cwctkQss3/HZ6rs3PR942oAetuxLy3KPF83IeFm\n"
    "-----END RSA PRIVATE KEY-----\n";



/*
 * Example RSA-1024 keypair, for test purposes
 */
#define KEY_LEN 128

#define RSA_N   "9292758453063D803DD603D5E777D788" \
                "8ED1D5BF35786190FA2F23EBC0848AEA" \
                "DDA92CA6C3D80B32C4D109BE0F36D6AE" \
                "7130B9CED7ACDF54CFC7555AC14EEBAB" \
                "93A89813FBF3C4F8066D2D800F7C38A8" \
                "1AE31942917403FF4946B0A83D3D3E05" \
                "EE57C6F5F5606FB5D4BC6CD34EE0801A" \
                "5E94BB77B07507233A0BC7BAC8F90F79"

#define RSA_E   "10001"

#define RSA_D   "24BF6185468786FDD303083D25E64EFC" \
                "66CA472BC44D253102F8B4A9D3BFA750" \
                "91386C0077937FE33FA3252D28855837" \
                "AE1B484A8A9A45F7EE8C0C634F99E8CD" \
                "DF79C5CE07EE72C7F123142198164234" \
                "CABB724CF78B8173B9F880FC86322407" \
                "AF1FEDFDDE2BEB674CA15F3E81A1521E" \
                "071513A1E85B5DFA031F21ECAE91A34D"

#define RSA_P   "C36D0EB7FCD285223CFB5AABA5BDA3D8" \
                "2C01CAD19EA484A87EA4377637E75500" \
                "FCB2005C5C7DD6EC4AC023CDA285D796" \
                "C3D9E75E1EFC42488BB4F1D13AC30A57"

#define RSA_Q   "C000DF51A7C77AE8D7C7370C1FF55B69" \
                "E211C2B9E5DB1ED0BF61D0D9899620F4" \
                "910E4168387E3C30AA1E00C339A79508" \
                "8452DD96A9A5EA5D9DCA68DA636032AF"

#define PT_LEN  24
#define RSA_PT  "\xAA\xBB\xCC\x03\x02\x01\x00\xFF\xFF\xFF\xFF\xFF" \
                "\x11\x22\x33\x0A\x0B\x0C\xCC\xDD\xDD\xDD\xDD\xDD"

#define USE_PEM_FILE 0

static uint32_t s_u32Time, s_u32Ratio, s_u32Cnt;
/* timer ticks - 100 ticks per second */
static volatile uint32_t  s_u32TickCnt;


void SysTick_Handler(void);
void enable_sys_tick(int ticks_per_second);
void start_timer0(void);
uint32_t  get_timer0_counter(void);
int PEMtoRSA(void);
int RSAEncryptWithHashTest( int verbose );

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
    return TIMER0->CNT ? TIMER0->CNT : s_u32Ratio*s_u32TickCnt;
}


#if defined(MBEDTLS_PKCS1_V15)
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
#endif /* MBEDTLS_PKCS1_V15 */





int PEMtoRSA(void)
{
    int ret = 0;
    mbedtls_pk_context pk;


    enable_sys_tick(1000);
    start_timer0();
    for(s_u32Cnt = 0; s_u32Cnt<100000; s_u32Cnt++)
    {
        __NOP();
    }

    s_u32Time = get_timer0_counter();
    s_u32Ratio = s_u32Time/s_u32TickCnt;


    mbedtls_pk_init(&pk);

    enable_sys_tick(1000);
    start_timer0();

    printf("\n  mbedtls_pk_parse_key      :");
#if USE_PEM_FILE
    /*
     * Read the RSA private key
     */
    if( ( ret = mbedtls_pk_parse_keyfile( &pk, "./private.pem", "" ) ) != 0 )
    {
        printf( " failed\n  ! mbedtls_pk_parse_keyfile returned -0x%04x\n", ret );
        return -1;
    }
#else
    if((ret = mbedtls_pk_parse_key(
                  &pk,
                  (const unsigned char *)SSL_USER_PRIV_KEY_PEM,
                  sizeof(SSL_USER_PRIV_KEY_PEM),
                  NULL,  0)) != 0)
    {
        printf("  failed\n ! mbedtls_pk_parse_key returned %d\n", ret);
        return -1;
    }
#endif


    printf(" passed");
    s_u32Time = get_timer0_counter();

    /* TIMER0->CNT is the elapsed us */
    printf("     takes %d.%d seconds,  %d ticks\n", s_u32Time / 1000000, s_u32Time / 1000, s_u32TickCnt);

    enable_sys_tick(1000);
    start_timer0();

    printf("  mbedtls_rsa_check_privkey :");
    /* check RSA key */
    mbedtls_rsa_context *rsa = mbedtls_pk_rsa(pk);
    if (mbedtls_rsa_check_privkey( rsa ) != 0 )
    {
        printf("  failed\n !mbedtls_rsa_check_privkey\n");
        return -1;
    }

    printf(" passed");
    s_u32Time = get_timer0_counter();

    /* TIMER0->CNT is the elapsed us */
    printf("     takes %d.%d seconds,  %d ticks\n", s_u32Time / 1000000, s_u32Time / 1000, s_u32TickCnt);

    mbedtls_pk_free(&pk);

    return 0;
}



/*
 * Checkup routine
 */
int RSAEncryptWithHashTest( int verbose )
{
    int ret = 0;
#if defined(MBEDTLS_PKCS1_V15)
    size_t len;
    mbedtls_rsa_context rsa;
    unsigned char rsa_plaintext[PT_LEN];
    unsigned char rsa_decrypted[PT_LEN];
    unsigned char rsa_ciphertext[KEY_LEN];
#if defined(MBEDTLS_SHA1_C)
    unsigned char sha1sum[20];
#endif

    mbedtls_mpi K;

    mbedtls_mpi_init( &K );
    mbedtls_rsa_init( &rsa, MBEDTLS_RSA_PKCS_V15, 0 );

    MBEDTLS_MPI_CHK( mbedtls_mpi_read_string( &K, 16, RSA_N  ) );
    MBEDTLS_MPI_CHK( mbedtls_rsa_import( &rsa, &K, NULL, NULL, NULL, NULL ) );
    MBEDTLS_MPI_CHK( mbedtls_mpi_read_string( &K, 16, RSA_P  ) );
    MBEDTLS_MPI_CHK( mbedtls_rsa_import( &rsa, NULL, &K, NULL, NULL, NULL ) );
    MBEDTLS_MPI_CHK( mbedtls_mpi_read_string( &K, 16, RSA_Q  ) );
    MBEDTLS_MPI_CHK( mbedtls_rsa_import( &rsa, NULL, NULL, &K, NULL, NULL ) );
    MBEDTLS_MPI_CHK( mbedtls_mpi_read_string( &K, 16, RSA_D  ) );
    MBEDTLS_MPI_CHK( mbedtls_rsa_import( &rsa, NULL, NULL, NULL, &K, NULL ) );
    MBEDTLS_MPI_CHK( mbedtls_mpi_read_string( &K, 16, RSA_E  ) );
    MBEDTLS_MPI_CHK( mbedtls_rsa_import( &rsa, NULL, NULL, NULL, NULL, &K ) );

    MBEDTLS_MPI_CHK( mbedtls_rsa_complete( &rsa ) );

    if( verbose != 0 )
        printf( "\n  RSA key validation     : " );


    enable_sys_tick(1000);
    start_timer0();

    if( mbedtls_rsa_check_pubkey(  &rsa ) != 0 ||
            mbedtls_rsa_check_privkey( &rsa ) != 0 )
    {
        if( verbose != 0 )
            printf( "failed\n" );

        ret = 1;
        goto cleanup;
    }

    if( verbose != 0 )
    {
        printf("passed");
        s_u32Time = get_timer0_counter();

        /* TIMER0->CNT is the elapsed us */
        printf("     takes %d.%d seconds,  %d ticks\n", s_u32Time / 1000000, s_u32Time / 1000, s_u32TickCnt);
    }

    enable_sys_tick(1000);
    start_timer0();

    printf( "  PKCS#1 encryption      : " );

    memcpy( rsa_plaintext, RSA_PT, PT_LEN );

    if( mbedtls_rsa_pkcs1_encrypt( &rsa, myrand, NULL, MBEDTLS_RSA_PUBLIC,
                                   PT_LEN, rsa_plaintext,
                                   rsa_ciphertext ) != 0 )
    {
        if( verbose != 0 )
            printf( "failed\n" );

        ret = 1;
        goto cleanup;
    }

    if( verbose != 0 )
    {
        printf("passed");
        s_u32Time = get_timer0_counter();

        /* TIMER0->CNT is the elapsed us */
        printf("     takes %d.%d seconds,  %d ticks\n", s_u32Time / 1000000, s_u32Time / 1000, s_u32TickCnt);
    }

    enable_sys_tick(1000);
    start_timer0();

    printf( "  PKCS#1 decryption      : " );

    if( mbedtls_rsa_pkcs1_decrypt( &rsa, myrand, NULL, MBEDTLS_RSA_PRIVATE,
                                   &len, rsa_ciphertext, rsa_decrypted,
                                   sizeof(rsa_decrypted) ) != 0 )
    {
        if( verbose != 0 )
            printf( "failed\n" );

        ret = 1;
        goto cleanup;
    }

    if( memcmp( rsa_decrypted, rsa_plaintext, len ) != 0 )
    {
        if( verbose != 0 )
            printf( "failed\n" );

        ret = 1;
        goto cleanup;
    }

    if( verbose != 0 )
    {
        printf("passed");
        s_u32Time = get_timer0_counter();

        /* TIMER0->CNT is the elapsed us */
        printf("     takes %d.%d seconds,  %d ticks\n", s_u32Time / 1000000, s_u32Time / 1000, s_u32TickCnt);
    }
    enable_sys_tick(1000);
    start_timer0();


#if defined(MBEDTLS_SHA1_C)
    if( verbose != 0 )
        printf( "  PKCS#1 data sign       : " );

    if( mbedtls_sha1_ret( rsa_plaintext, PT_LEN, sha1sum ) != 0 )
    {
        if( verbose != 0 )
            printf( "failed\n" );

        return( 1 );
    }

    if( mbedtls_rsa_pkcs1_sign( &rsa, myrand, NULL,
                                MBEDTLS_RSA_PRIVATE, MBEDTLS_MD_SHA1, 0,
                                sha1sum, rsa_ciphertext ) != 0 )
    {
        if( verbose != 0 )
            printf( "failed\n" );

        ret = 1;
        goto cleanup;
    }


    if( verbose != 0 )
    {
        printf("passed");
        s_u32Time = get_timer0_counter();

        /* TIMER0->CNT is the elapsed us */
        printf("     takes %d.%d seconds,  %d ticks\n", s_u32Time / 1000000, s_u32Time / 1000, s_u32TickCnt);
    }
    enable_sys_tick(1000);
    start_timer0();
    printf( "  PKCS#1 sig. verify     : " );

    if( mbedtls_rsa_pkcs1_verify( &rsa, NULL, NULL,
                                  MBEDTLS_RSA_PUBLIC, MBEDTLS_MD_SHA1, 0,
                                  sha1sum, rsa_ciphertext ) != 0 )
    {
        if( verbose != 0 )
            printf( "failed\n" );

        ret = 1;
        goto cleanup;
    }

    if( verbose != 0 )
    {
        printf("passed");
        s_u32Time = get_timer0_counter();

        /* TIMER0->CNT is the elapsed us */
        printf("     takes %d.%d seconds,  %d ticks\n", s_u32Time / 1000000, s_u32Time / 1000, s_u32TickCnt);
    }
    enable_sys_tick(1000);
    start_timer0();
#endif /* MBEDTLS_SHA1_C */

    if( verbose != 0 )
        printf( "\n" );

cleanup:
    mbedtls_mpi_free( &K );
    mbedtls_rsa_free( &rsa );
#else /* MBEDTLS_PKCS1_V15 */
    ((void) verbose);
#endif /* MBEDTLS_PKCS1_V15 */
    return( ret );
}
