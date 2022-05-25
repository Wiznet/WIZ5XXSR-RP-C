/*
 * dnsHandler.c
 *
 *  Created on: Jan 19, 2021
 *      Author: Hoon-Mac
 */

#include "ConfigData.h"
#include "wizchip_conf.h"
#include "socket.h"

#include "dns.h"
#include "dnsHandler.h"
#include "common.h"
#include "port_common.h"
#include "timerHandler.h"

/* Header for all domain messages */
struct dhdr_handler
{
	uint16_t id;   /* Identification */
	uint8_t	qr;      /* Query/Response */
#define	QUERY    0
#define	RESPONSE 1
	uint8_t	opcode;
#define	IQUERY   1
	uint8_t	aa;      /* Authoratative answer */
	uint8_t	tc;      /* Truncation */
	uint8_t	rd;      /* Recursion desired */
	uint8_t	ra;      /* Recursion available */
	uint8_t	rcode;   /* Response code */
#define	NO_ERROR       0
#define	FORMAT_ERROR   1
#define	SERVER_FAIL    2
#define	NAME_ERROR     3
#define	NOT_IMPL       4
#define	REFUSED        5
	uint16_t qdcount;	/* Question count */
	uint16_t ancount;	/* Answer count */
	uint16_t nscount;	/* Authority (name server) count */
	uint16_t arcount;	/* Additional record count */
};


uint32_t (*getTick)(void);
static uint8_t dns_state = STATE_DNS_STOP;

extern uint8_t  DNS_SOCKET;
extern uint8_t* pDNSMSG;       // DNS message buffer
extern uint8_t  DNS_SOCKET;    // SOCKET number for DNS
extern uint16_t DNS_MSGID;     // DNS message ID
extern uint32_t dns_1s_tick;   // for timout of DNS processing


/* DNS CLIENT INIT */
void DNS_init_handler(uint8_t s, uint8_t * buf, uint32_t (*tickFunc)(void))
{
    
    DNS_init(s, buf);

    getTick = tickFunc;
    dns_state = STATE_DNS_INIT;
}


int8_t DNS_run_handler(uint8_t * dns_ip, uint8_t * name, uint8_t * ip_from_dns, uint32_t timeout)
{
    static uint32_t tickStart;
    struct dhdr_handler dhp;

    int8_t ret = DNS_RET_RUNNING;
    uint8_t ip[4];
    uint16_t len, port;

    if(dns_state == STATE_DNS_STOP) return DNS_RET_STOPPED;

    switch(dns_state)
    {
        case STATE_DNS_INIT:
            socket(DNS_SOCKET, Sn_MR_UDP, 0, 0x00);
            dns_state = STATE_DNS_SEND_QUERY;
            break;

        case STATE_DNS_SEND_QUERY:
            len = dns_makequery(0, (char *)name, pDNSMSG, MAX_DNS_BUF_SIZE);
            sendto(DNS_SOCKET, pDNSMSG, len, dns_ip, IPPORT_DOMAIN);
#ifdef _DNS_DEBUG_
            printf("> DNS Query to DNS Server : %d.%d.%d.%d\r\n", dns_ip[0], dns_ip[1], dns_ip[2], dns_ip[3]);
#endif

            tickStart = getTick();
            dns_state = STATE_DNS_RECV_RESPONSE;
            break;

        case STATE_DNS_RECV_RESPONSE:
            if ((len = getSn_RX_RSR(DNS_SOCKET)) > 0) {
                if (len > MAX_DNS_BUF_SIZE) len = MAX_DNS_BUF_SIZE;
                len = recvfrom(DNS_SOCKET, pDNSMSG, len, ip, &port);
#ifdef _DNS_DEBUG_
                printf("> Receive DNS message from %d.%d.%d.%d(%d). len = %d\r\n", ip[0], ip[1], ip[2], ip[3],port,len);
#endif
                ret = parseDNSMSG(&dhp, pDNSMSG, ip_from_dns);
                if(ret) {
                    ret = DNS_RET_SUCCESS;
                } else {
                    if(ret == -1) {
#ifdef _DNS_DEBUG_
                        printf("> Please increase defined MAX_DOMAIN_NAME size [%d]\r\n", MAX_DOMAIN_NAME);
#endif
                    }
                    ret = DNS_RET_FAILED;
                }
                dns_state = STATE_DNS_DONE;
            }

            // timeout checker
            if((getTick() - tickStart) >= timeout) {
                dns_state = STATE_DNS_DONE;
                ret = DNS_RET_TIMEOUT;
            }
            break;

        case STATE_DNS_DONE:
            tickStart = 0;
            close(DNS_SOCKET);
            dns_state = STATE_DNS_STOP;
            break;

        default:
            break;
    }

    return ret;
}


int8_t process_dns(void)
{
    DevConfig *dev_config = get_DevConfig_pointer();
    int8_t ret = 0;
    uint8_t dns_retry = 0;

#ifdef _MAIN_DEBUG_
    printf(" - DNS Client running\r\n");
#endif

    set_device_status(ST_UPGRADE);

    do {
        ret = get_ipaddr_from_dns((uint8_t *)dev_config->network_connection.dns_domain_name,
                                             dev_config->network_connection.remote_ip,
                                             (DNS_WAIT_TIME * 300));
        dns_retry++;

        if(dns_retry > 2) {
#ifndef __USE_DNS_INFINITE_LOOP__
            PRT_ERR(" - DNS Failed\r\n\r\n");
            break;
#else // If DNS query failed, process_dns() function will try to DNS steps again.
            PRT_ERR(" - DNS Failed, Try again...\r\n\r\n");
            dns_retry = 0;
#endif
        }

        if(dev_config->network_option.dhcp_use) DHCP_run();
    } while(ret != TRUE);

    set_device_status(0, ST_OPEN);
    return ret;
}


int8_t get_ipaddr_from_dns(uint8_t * domain, uint8_t * ip_from_dns, uint32_t timeout)
{
    int8_t ret;
    uint8_t msgbuf[1024];

    DevConfig *dev_config = get_DevConfig_pointer();

    DNS_init_handler(SOCK_DNS, msgbuf, millis);

    do {
        ret = DNS_run_handler(dev_config->network_option.dns_server_ip, domain, ip_from_dns, timeout);

        // Process the requests of configuration tool during the DNS client run
        do_segcp();
    } while(ret == DNS_RET_RUNNING);

    if(ret == DNS_RET_SUCCESS)
    {
        PRT_INFO(" - DNS: [%s] Get Server IP - %d.%d.%d.%d\r\n", domain, ip_from_dns[0], ip_from_dns[1], ip_from_dns[2], ip_from_dns[3]);
    }
    else
    {
        PRT_ERR(" - DNS: [%s] DNS failed (%d)\r\n", domain, ret);
        return FALSE; // DNS failed
    }
    return ret;
}



