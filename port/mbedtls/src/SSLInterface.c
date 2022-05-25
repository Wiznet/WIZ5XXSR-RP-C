/*
 * file: SSLInterface.c
 * description: mbedtls callback functions
 * author: peter
 * company: wiznet
 * data: 2015.11.26
 */
#include <stdio.h>
#include <string.h>
//#include "stm32l5xx.h"

#include "mbedtls/x509_crt.h"
#include "mbedtls/error.h"
#include "port_common.h"

#include "SSLInterface.h"
#include "SSL_Random.h"
#include "socket.h"
#include "ConfigData.h"
#include "timerHandler.h"
#include "deviceHandler.h"
#include "storageHandler.h"
#include "common.h"

//unsigned char tempBuf[DEBUG_BUFFER_SIZE] = {0,};

extern uint8_t *g_rootca_buf;
extern uint8_t *g_clica_buf;
extern uint8_t *g_pkey_buf;

int WIZnetRecvTimeOut(void *ctx, unsigned char *buf, size_t len, uint32_t timeout)
{
    int ret;
    uint32_t start_ms = millis();
    do
    {
        if(getSn_RX_RSR((uint8_t)ctx)){
            return recv((uint8_t)ctx, (uint8_t *)buf, (uint16_t)len);
        }
    }while((millis() - start_ms) < timeout);

    return MBEDTLS_ERR_SSL_TIMEOUT;
}

/*Shell for mbedtls recv function*/
int WIZnetRecv(void *ctx, unsigned char *buf, unsigned int len )
{
    return (recv((uint8_t)ctx, (uint8_t *)buf, (uint16_t)len));
}

/*Shell for mbedtls recv non-block function*/
int WIZnetRecvNB(void *ctx, unsigned char *buf, unsigned int len )
{
    uint32_t recv_len = 0;

    getsockopt((uint8_t)(ctx), SO_RECVBUF, &recv_len);
    if (recv_len > 0)
        return recv((uint8_t)ctx, (uint8_t *)buf, (uint16_t)len);
    else
        return 0;
}


/*Shell for mbedtls send function*/
int WIZnetSend(void *ctx, const unsigned char *buf, unsigned int len )
{
    return (send((uint8_t)ctx, (uint8_t *)buf, (uint16_t)len));
}

/*Shell for mbedtls debug function.
 *DEBUG_LEBEL can be changed from 0 to 3*/
#ifdef MBEDTLS_DEBUG_C
void WIZnetDebugCB(void *ctx, int level, const char *file, int line, const char *str)
{
    if(level <= DEBUG_LEVEL)
    {
       printf("%s\r\n",str);
    }
}
#endif


/* SSL context initialization
 * */
int wiz_tls_init(wiz_tls_context* tlsContext, int* socket_fd, const char * host)
{
    struct __ssl_option *ssl_option = (struct __ssl_option *)&(get_DevConfig_pointer()->ssl_option);
    int ret = 1;
    const char *pers = "ssl_client1";
    const char *alpnProtocols[] = { "x-amzn-mqtt-ca", NULL };
#if defined (MBEDTLS_ERROR_C)
    char error_buf[100];
#endif

#if defined (MBEDTLS_DEBUG_C)
    debug_set_threshold(DEBUG_LEVEL);
#endif

  /*
    Initialize session data
  */
#if defined (MBEDTLS_ENTROPY_C) 
    tlsContext->entropy = malloc(sizeof(mbedtls_entropy_context));
#endif
    tlsContext->ctr_drbg = malloc(sizeof(mbedtls_ctr_drbg_context));
    tlsContext->ssl = malloc(sizeof(mbedtls_ssl_context));
    tlsContext->conf = malloc(sizeof(mbedtls_ssl_config));
    tlsContext->cacert = malloc(sizeof(mbedtls_x509_crt));    
    tlsContext->clicert = malloc(sizeof(mbedtls_x509_crt));
    tlsContext->pkey = malloc(sizeof(mbedtls_pk_context));

#if defined (MBEDTLS_ENTROPY_C)
    mbedtls_entropy_init( tlsContext->entropy);
#endif

    mbedtls_ctr_drbg_init(tlsContext->ctr_drbg);
    mbedtls_ssl_init(tlsContext->ssl);
    mbedtls_ssl_config_init(tlsContext->conf);
    mbedtls_x509_crt_init(tlsContext->cacert);
    mbedtls_x509_crt_init(tlsContext->clicert);
    mbedtls_pk_init(tlsContext->pkey);

  /*
    Initialize certificates
  */
#if defined (MBEDTLS_ENTROPY_C) 
    if((ret = mbedtls_ctr_drbg_seed(tlsContext->ctr_drbg, mbedtls_entropy_func, tlsContext->entropy,    \
                    (const unsigned char *) pers, strlen(pers))) != 0) {
      PRT_SSL(" failed\r\n  ! mbedtls_ctr_drbg_seed returned -0x%x\r\n", -ret);
      return -1;
    }
#endif

#if defined (MBEDTLS_DEBUG_C)
    mbedtls_ssl_conf_dbg(tlsContext->conf, WIZnetDebugCB, stdout);
#endif
    
  /*
    Parse certificate
  */
    if (ssl_option->root_ca_option != MBEDTLS_SSL_VERIFY_NONE)
    {
        PRT_SSL(" Loading the CA root certificate len = %d\r\n", ssl_option->rootca_len);
        g_rootca_buf = FLASH_ROOTCA_ADDR + XIP_BASE;
        ret = mbedtls_x509_crt_parse(tlsContext->cacert, (const char *)g_rootca_buf, ssl_option->rootca_len + 1);
        if(ret < 0) 
        {
          PRT_SSL(" failed\r\n  !  mbedtls_x509_crt_parse returned -0x%x while parsing root cert\r\n", -ret);
          return -1;
        }
        PRT_SSL("ok! mbedtls_x509_crt_parse returned -0x%x while parsing root cert\r\n", -ret);
    }

    if (ssl_option->client_cert_enable == ENABLE)
    {
        g_clica_buf = FLASH_CLICA_ADDR + XIP_BASE;
        g_pkey_buf = FLASH_PRIKEY_ADDR + XIP_BASE;
       
        ret = mbedtls_x509_crt_parse((tlsContext->clicert), (const char *)g_clica_buf, ssl_option->clica_len + 1);
        if(ret != 0) {
            PRT_SSL(" failed\r\n  !  mbedtls_x509_crt_parse returned -0x%x while parsing device cert\r\n", -ret);
            return -1;
        }
        PRT_SSL("ok! mbedtls_x509_crt_parse returned -0x%x while parsing device cert\r\n", -ret);

        ret = mbedtls_pk_parse_key(tlsContext->pkey, (const char *)g_pkey_buf, ssl_option->pkey_len + 1, NULL, 0);
        if(ret != 0) {
            PRT_SSL(" failed\r\n  !  mbedtls_pk_parse_key returned -0x%x while parsing private key\r\n", -ret);
            return -1;
        }
        PRT_SSL("ok! mbedtls_pk_parse_key returned -0x%x while parsing private key\r\n", -ret);
    }

    PRT_SSL("host = %s\r\n", host);
    if((ret = mbedtls_ssl_set_hostname(tlsContext->ssl, host)) != 0)
    {
        PRT_SSL(" failed mbedtls_ssl_set_hostname returned %d\r\n", ret);
        return -1;
    }
    PRT_SSL("ok! mbedtls_ssl_set_hostname returned %d\r\n", ret);
    
    if((ret = mbedtls_ssl_config_defaults(tlsContext->conf,
                        MBEDTLS_SSL_IS_CLIENT,
                        MBEDTLS_SSL_TRANSPORT_STREAM,
                        MBEDTLS_SSL_PRESET_DEFAULT)) != 0)
    {
      PRT_SSL(" failed mbedtls_ssl_config_defaults returned %d\r\n", ret);
      return -1;
    }

    PRT_SSL("ssl_option->root_ca_option = %d\r\n", ssl_option->root_ca_option);
    PRT_SSL("socket_fd = %d\r\n", socket_fd);
    mbedtls_ssl_conf_authmode(tlsContext->conf, ssl_option->root_ca_option);
    mbedtls_ssl_conf_ca_chain(tlsContext->conf, tlsContext->cacert, NULL);
    mbedtls_ssl_conf_rng(tlsContext->conf, SSLRandomCB, tlsContext->ctr_drbg);
    
    if (ssl_option->client_cert_enable == ENABLE)
    {
        if((ret = mbedtls_ssl_conf_own_cert(tlsContext->conf, tlsContext->clicert, tlsContext->pkey)) != 0) 
        {
          PRT_SSL("failed! mbedtls_ssl_conf_own_cert returned %d\r\n", ret);
          return -1;
        }
        PRT_SSL("ok! mbedtls_ssl_conf_own_cert returned %d\r\n", ret);
    }
    
    mbedtls_ssl_conf_endpoint(tlsContext->conf, MBEDTLS_SSL_IS_CLIENT);
    mbedtls_ssl_conf_read_timeout(tlsContext->conf, ssl_option->recv_timeout);

    if((ret = mbedtls_ssl_setup(tlsContext->ssl, tlsContext->conf)) != 0)
    {
      PRT_SSL(" failed mbedtls_ssl_setup returned -0x%x\r\n", -ret);
      return -1;
    }
    mbedtls_ssl_set_bio(tlsContext->ssl, socket_fd, SSLSendCB, SSLRecvCB, SSLRecvTimeOutCB);

    PRT_SSL("return 1\r\n");
    return 1;
}

/*Free the memory for ssl context*/
void wiz_tls_deinit(wiz_tls_context* tlsContext)
{
  /*  free SSL context memory  */

    PRT_SSL("SSL Free\r\n");
    mbedtls_ssl_free( tlsContext->ssl );
    mbedtls_ssl_config_free( tlsContext->conf );
    mbedtls_ctr_drbg_free( tlsContext->ctr_drbg );
#if defined (MBEDTLS_ENTROPY_C)
    mbedtls_entropy_free( tlsContext->entropy );
#endif
    mbedtls_x509_crt_free( tlsContext->cacert );
    mbedtls_x509_crt_free(tlsContext->clicert);
    mbedtls_pk_free(tlsContext->pkey);

#if defined (MBEDTLS_ENTROPY_C)
    free(tlsContext->entropy);
#endif
    free(tlsContext->ctr_drbg);
    free(tlsContext->ssl);
    free(tlsContext->conf);
    free(tlsContext->cacert);
    free(tlsContext->clicert);
    free(tlsContext->pkey);
}

int wiz_tls_socket(wiz_tls_context* tlsContext, unsigned int port)
{
    /*socket open*/
    return socket((uint8_t)(tlsContext->ssl->p_bio), Sn_MR_TCP, (uint16_t)port, (SF_TCP_NODELAY));
}

/* SSL handshake */
int wiz_tls_connect(wiz_tls_context* tlsContext, char * addr, unsigned int port)
{
    int ret;

    /*Connect to the target*/
   ret = connect((uint8_t)(tlsContext->ssl->p_bio), (uint8_t *)addr, (uint16_t)port);
   if(ret != SOCK_OK)
       return ret;

#if defined(MBEDTLS_DEBUG_C)
    printf(" Performing the SSL/TLS handshake...\r\n");
#endif

    while( ( ret = mbedtls_ssl_handshake( tlsContext->ssl ) ) != 0 )
    {
        if( ret != MBEDTLS_ERR_SSL_WANT_READ && ret != MBEDTLS_ERR_SSL_WANT_WRITE )
        {
#if defined(MBEDTLS_ERROR_C)
            mbedtls_strerror(ret, (char *) tempBuf, DEBUG_BUFFER_SIZE );
            printf( " failed\n\r  ! mbedtls_ssl_handshake returned %d: %s\n\r", ret, tempBuf );
#endif
            return( -1 );
        }
    }

#if defined(MBEDTLS_DEBUG_C)
    printf( " ok\n\r    [ Ciphersuite is %s ]\n\r",
    mbedtls_ssl_get_ciphersuite( tlsContext->ssl ) );
#endif
    return( 0 );
}

int wiz_tls_connect_timeout(wiz_tls_context* tlsContext, char * addr, unsigned int port, uint32_t timeout)
{
    int ret;
    uint32_t start_ms = millis(), flags;
    uint8_t sock = (uint8_t)(tlsContext->ssl->p_bio);
    struct __ssl_option *ssl_option = (struct __ssl_option *)&(get_DevConfig_pointer()->ssl_option);

    /*Connect to the target*/
    do {
        ret = connect(sock, (uint8_t *)addr, (uint16_t)port);
        PRT_SSL("connect ret = %d\r\n", ret);
        if((ret == SOCK_OK) || (ret == SOCKERR_TIMEOUT)) break;
    } while((millis() - start_ms) < timeout);

    if(ret == SOCK_BUSY) return -1;
    if(ret != SOCK_OK) return ret;

    PRT_SSL(" Performing the SSL/TLS handshake...\r\n");

    while( ( ret = mbedtls_ssl_handshake( tlsContext->ssl ) ) != 0 )
    {
        if( ret != MBEDTLS_ERR_SSL_WANT_READ && ret != MBEDTLS_ERR_SSL_WANT_WRITE )
        {
            //mbedtls_strerror(ret, (char *) tempBuf, DEBUG_BUFFER_SIZE );
            //PRT_SSL( " failed\n\r  ! mbedtls_ssl_handshake returned %d: %s\n\r", ret, tempBuf );
          PRT_SSL( " failed\n\r  ! mbedtls_ssl_handshake returned -0x%x\n\r", -ret);
            return( -1 );
        }
    }

    if (ssl_option->root_ca_option == MBEDTLS_SSL_VERIFY_REQUIRED)
    {
        PRT_SSL("  . Verifying peer X.509 certificate...\r\n");
        
        /* In real life, we probably want to bail out when ret != 0 */
        if((flags = mbedtls_ssl_get_verify_result(tlsContext->ssl)) != 0)
        {
            char vrfy_buf[512];
            PRT_SSL("failed\r\n");
            mbedtls_x509_crt_verify_info(vrfy_buf, sizeof(vrfy_buf), "  ! ", flags);
            PRT_SSL("%s\r\n", vrfy_buf);
            return -1;
        }
        else
        {
            PRT_SSL("ok\r\n");
        }
    }
    PRT_SSL( " ok\n\r    [ Ciphersuite is %s ]\n\r",
    mbedtls_ssl_get_ciphersuite( tlsContext->ssl ) );
    return( 0 );
}



/* SSL handshake */
int wiz_tls_socket_connect(wiz_tls_context* tlsContext, char * addr, unsigned int port)
{
    int ret;
    uint8_t sock = (uint8_t)(tlsContext->ssl->p_bio);

    /*socket open*/
    ret = socket(sock, Sn_MR_TCP, 0, 0x00);
    if(ret != sock)
        return ret;

    /*Connect to the target*/
    ret = connect(sock, addr, port);
    if(ret != SOCK_OK)
        return ret;

#if defined(MBEDTLS_DEBUG_C)
    printf(" Performing the SSL/TLS handshake...\r\n");
#endif

    while( ( ret = mbedtls_ssl_handshake( tlsContext->ssl ) ) != 0 )
    {
        if( ret != MBEDTLS_ERR_SSL_WANT_READ && ret != MBEDTLS_ERR_SSL_WANT_WRITE )
        {
#if defined(MBEDTLS_ERROR_C)
            memset(tempBuf, 0, 1024);
            mbedtls_strerror(ret, (char *) tempBuf, DEBUG_BUFFER_SIZE );
            printf( " failed\n\r  ! mbedtls_ssl_handshake returned %d: %s\n\r", ret, tempBuf );
#endif
            return( -1 );
        }
    }

#if defined(MBEDTLS_DEBUG_C)
    printf( " ok\n\r    [ Ciphersuite is %s ]\n\r",
            mbedtls_ssl_get_ciphersuite( tlsContext->ssl ) );
#endif

    return( 0 );
}

int wiz_tls_socket_connect_timeout(wiz_tls_context* tlsContext, char * addr, unsigned int port, unsigned int local_port, uint32_t timeout)
{
    int ret;
    uint32_t start_ms = millis();

    uint8_t sock = (uint8_t)(tlsContext->ssl->p_bio);

    PRT_SSL("tlsContext = %d\r\n", tlsContext);
    /*socket open*/
    ret = socket(sock, Sn_MR_TCP, local_port, 0x00);
    if(ret != sock)
        return ret;

    /*Connect to the target*/
    ret = connect(sock, addr, port);
    if(ret != SOCK_OK)
        return ret;
    
    PRT_SSL("connect ret = %d\r\n", ret);
    PRT_SSL(" Performing the SSL/TLS handshake...\r\n");

    while( ( ret = mbedtls_ssl_handshake( tlsContext->ssl ) ) != 0 )
    {
        if( ret != MBEDTLS_ERR_SSL_WANT_READ && ret != MBEDTLS_ERR_SSL_WANT_WRITE )
        {
#if defined(MBEDTLS_ERROR_C)
            memset(tempBuf, 0, 1024);
            mbedtls_strerror(ret, (char *) tempBuf, DEBUG_BUFFER_SIZE );
            printf( " failed\r\n  ! mbedtls_ssl_handshake returned %d: %s\n\r", ret, tempBuf );
#endif
            PRT_SSL( " failed\r\n  ! mbedtls_ssl_handshake returned -0x%x\n\r", -ret);
            return( -1 );
        }

        if((millis() - start_ms) > timeout) // timeout
        {
#if defined(MBEDTLS_ERROR_C)
            memset(tempBuf, 0, 1024);
            mbedtls_strerror(ret, (char *) tempBuf, DEBUG_BUFFER_SIZE );
            printf( " timeout\r\n  ! mbedtls_ssl_handshake returned %d: %s\n\r", ret, tempBuf );
#endif
            PRT_SSL( " timeout\r\n  ! mbedtls_ssl_handshake returned -0x%x\n\r", ret);
            return( -1 );
        }
    }

#if defined(MBEDTLS_DEBUG_C)
    printf( " ok\n\r    [ Ciphersuite is %s ]\n\r",
            mbedtls_ssl_get_ciphersuite( tlsContext->ssl ) );
#endif
    PRT_SSL( " ok\n\r    [ Ciphersuite is %s ]\n\r",
        mbedtls_ssl_get_ciphersuite( tlsContext->ssl ) );
    return( 0 );
}

unsigned int wiz_tls_read(wiz_tls_context* tlsContext, unsigned char* readbuf, unsigned int len)
{
    return mbedtls_ssl_read( tlsContext->ssl, readbuf, len );
}

unsigned int wiz_tls_write(wiz_tls_context* tlsContext, unsigned char* writebuf, unsigned int len)
{
    return mbedtls_ssl_write( tlsContext->ssl, writebuf, len );
}

int wiz_tls_disconnect(wiz_tls_context* tlsContext, uint32_t timeout)
{
    int ret = 0;
    uint8_t sock = (uint8_t)(tlsContext->ssl->p_bio);
    uint32_t tickStart = millis();

    do {
        ret = disconnect(sock);
        if((ret == SOCK_OK) || (ret == SOCKERR_TIMEOUT)) break;
    } while ((millis() - tickStart) < timeout);

    if(ret == SOCK_OK)
        ret = sock; // socket number

    return ret;
}


/* ssl Close notify */
unsigned int wiz_tls_close_notify(wiz_tls_context* tlsContext)
{
    uint32_t rc;
    do rc = mbedtls_ssl_close_notify( tlsContext->ssl );
    while( rc == MBEDTLS_ERR_SSL_WANT_WRITE );
    return rc;
}


/* ssl session reset */
int wiz_tls_session_reset(wiz_tls_context* tlsContext)
{
    return mbedtls_ssl_session_reset( tlsContext->ssl );
}


int check_ca(uint8_t *ca_data, uint32_t ca_len)
{
    int ret;

    mbedtls_x509_crt ca_cert;
    mbedtls_x509_crt_init(&ca_cert);


    PRT_SSL("ca_len = %d\r\n", ca_len);
    ret = mbedtls_x509_crt_parse(&ca_cert, (const char *)ca_data, ca_len + 1);
    if(ret < 0) 
    {
        PRT_SSL(" failed\r\n  !  mbedtls_x509_crt_parse returned -0x%x while parsing root cert\r\n", -ret);
    }
    else
        PRT_SSL("ok! mbedtls_x509_crt_parse returned -0x%x while parsing root cert\r\n", -ret);

    mbedtls_x509_crt_free(&ca_cert);
    return ret;
}

int check_pkey(uint8_t *pkey_data, uint32_t pkey_len)
{
    int ret;

    mbedtls_pk_context pk_cert;
    mbedtls_pk_init(&pk_cert);

    PRT_SSL("pkey_len = %d\r\n", pkey_len);
    
    ret = mbedtls_pk_parse_key(&pk_cert, (const char *)pkey_data, pkey_len + 1, NULL, 0);
    if(ret != 0) {
        PRT_SSL(" failed\r\n  !  mbedtls_pk_parse_key returned -0x%x while parsing private key\r\n", -ret);
    }
    else
    {
        PRT_SSL(" ok !  mbedtls_pk_parse_key returned -0x%x while parsing private key\r\n", -ret);
    }

    mbedtls_pk_free(&pk_cert);
    return ret;
}


