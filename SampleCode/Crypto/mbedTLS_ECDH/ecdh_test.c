#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "NuMicro.h"

/* mbeTLS header files */
#include "ecdh.h"


#define CRYPTO_DH_MAX_KEY_LEN 1024

unsigned char buf[CRYPTO_DH_MAX_KEY_LEN];


static int myrand( void *rng_state, unsigned char *output, size_t len )
{
#if !defined(__OpenBSD__)
    size_t i;

    if( rng_state != NULL )
        rng_state  = NULL;

    for( i = 0; i < len; ++i )
        output[i] = rand();
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
    uint32_t size, curve;
    mbedtls_ecdh_context ecdh;


    /* Initializes an ECDH context. */
    mbedtls_ecdh_init(&ecdh);
    printf(" mbedtls ECDH init done  \n\n");

    /* Set up the ECDH context with the information given. */
    /* Adapts P256 curve                                   */
    curve = MBEDTLS_ECP_DP_SECP256K1;

    printf(" mbedtls ECDH setup        : ");
    ret = mbedtls_ecdh_setup (&ecdh, curve);

    if(ret == 0)
    {
        printf("passed\n");
    }
    else
    {
        printf("failed! ret[%d]\n", ret);
        return ret;
    }

    /* Generates an EC key pair and exports its in the format used in a TLS ServerKeyExchange handshake message. */
    printf(" mbedtls ECDH make params  : ");
    ret = mbedtls_ecdh_make_params(&ecdh, &size, buf, CRYPTO_DH_MAX_KEY_LEN, myrand, NULL);

    if(ret == 0)
    {
        printf("passed\n");
    }
    else
    {
        printf("failed! ret[%d]\n", ret);
        return ret;
    }

    /* Generate a public key and exports it as a TLS ClientKeyExchange payload. */
    printf(" mbedtls ECDH make public  : ");
    ret = mbedtls_ecdh_make_public(&ecdh, &size, buf, CRYPTO_DH_MAX_KEY_LEN, myrand, NULL);

    if(ret == 0)
    {
        printf("passed\n");
    }
    else
    {
        printf("failed! ret[%d]\n", ret);
        return ret;
    }

    /* Parse and process the ECDHE payload of a TLS ClientKeyExchange message*/
    printf(" mbedtls ECDH read public  : ");
    ret = mbedtls_ecdh_read_public(&ecdh, buf, size);

    if(ret == 0)
    {
        printf("passed\n");
    }
    else
    {
        printf("failed! ret[%d]\n", ret);
        return ret;
    }

    /* Derives and exports the shared secret.                         */
    /* This is the last function used by both TLS client and servers. */
    printf(" mbedtls ECDH calc secret  : ");
    ret = mbedtls_ecdh_calc_secret(&ecdh, &size, buf, CRYPTO_DH_MAX_KEY_LEN, myrand, NULL);

    if(ret == 0)
    {
        printf("passed\n");
    }
    else
    {
        printf("failed! ret[%d]\n", ret);
        return ret;
    }

    mbedtls_ecdh_free(&ecdh);
    return ret;
}


