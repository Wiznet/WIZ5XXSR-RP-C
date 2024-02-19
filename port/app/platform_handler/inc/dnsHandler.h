/*
 * dnsHandler.h
 *
 *  Created on: Jan 19, 2021
 *      Author: Hoon-Mac
 */

#ifndef PLATFORMHANDLER_DNSHANDLER_H_
#define PLATFORMHANDLER_DNSHANDLER_H_


// DNS non-block mode: DNS op states
#define STATE_DNS_INIT            0
#define STATE_DNS_SEND_QUERY      1
#define STATE_DNS_RECV_RESPONSE   2
#define STATE_DNS_DONE            3
#define STATE_DNS_STOP            4

// DNS non-block mode: DNS return value
enum
{
    DNS_RET_TIMEOUT = -2,
    DNS_RET_FAILED = -1,
    DNS_RET_RUNNING = 0,
    DNS_RET_SUCCESS = 1,
    DNS_RET_STOPPED = 2
};

void DNS_init_handler(uint8_t s, uint8_t * buf, uint32_t (*tickFunc)(void));
int8_t DNS_run_handler(uint8_t * dns_ip, uint8_t * name, uint8_t * ip_from_dns, uint32_t timeout);
int8_t process_dns(void);
int8_t get_ipaddr_from_dns(uint8_t * domain, uint8_t * ip_from_dns, uint32_t timeout);


#endif /* PLATFORMHANDLER_DNSHANDLER_H_ */
