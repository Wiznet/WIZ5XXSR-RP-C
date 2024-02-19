#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "common.h"
#include "port_common.h"

#include "wizchip_conf.h"
#include "socket.h"

// MQTT
#include "mqtt_interface.h"
#include "wizchip_conf.h"
#include "MQTTClient.h"
#include "mqttsHandler.h"

// TLS support
#include "SSLInterface.h"
//#include "gcp_ciotc_config.h"

/* Private define ------------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

// TLS support
static wiz_tls_context mqttTlsContext;


/*
 * @brief New TLS network setting
 * @param  n : pointer to a Network structure
 *         that contains the configuration information for the Network.
 *         sn : socket number where x can be (0..7).
 *         host : host name
 * @retval None
 */
int NewNetwork_mqtt_tls(Network* n, int sn, const char * host)
{

    n->my_socket = sn;
    n->mqttread = mqtt_tls_read;
    n->mqttwrite = mqtt_tls_write;
    n->disconnect = mqtt_tls_disconnect;

    return wiz_tls_init(&mqttTlsContext, n->my_socket, host);
}

int mqtt_tls_connect(uint8_t *remote_ip, uint16_t remote_port, uint32_t timeout)
{  
  return wiz_tls_connect_timeout(&mqttTlsContext,
                                remote_ip,
                                remote_port,
                                timeout);

}


/*
 * @brief read function
 * @param  n : pointer to a Network structure
 *         that contains the configuration information for the Network.
 *         buffer : pointer to a read buffer.
 *         len : buffer length.
 */
int mqtt_tls_read(Network* n, unsigned char* buffer, unsigned int len, int timeout_ms)
{
    int i;
    int size = 0;
    //if((getSn_SR(n->my_socket) == SOCK_ESTABLISHED) && (getSn_RX_RSR(n->my_socket) > 0))
    if(getSn_SR(n->my_socket) == SOCK_ESTABLISHED)
        size = wiz_tls_read(&mqttTlsContext, buffer, len);

    return size;
}

/*
 * @brief write function
 * @param  n : pointer to a Network structure
 *         that contains the configuration information for the Network.
 *         buffer : pointer to a read buffer.
 *         len : buffer length.
 */
int mqtt_tls_write(Network* n, unsigned char* buffer, unsigned int len, int timeout_ms)
{
    int size = 0;
    if(getSn_SR(n->my_socket) == SOCK_ESTABLISHED)
        size = wiz_tls_write(&mqttTlsContext, buffer, len);

    return size;
}

/*
 * @brief deinit function
 * @param  n : pointer to a Network structure
 *         that contains the configuration information for the Network.
 */
void mqtt_tls_deinit(Network * n)
{
    wiz_tls_close_notify(&mqttTlsContext);
    wiz_tls_session_reset(&mqttTlsContext);
    wiz_tls_deinit(&mqttTlsContext);
}

/*
 * @brief disconnect function
 * @param  n : pointer to a Network structure
 *         that contains the configuration information for the Network.
 */
void mqtt_tls_disconnect(Network * n)
{
    mqtt_tls_deinit(n);
    close(n->my_socket);
}


