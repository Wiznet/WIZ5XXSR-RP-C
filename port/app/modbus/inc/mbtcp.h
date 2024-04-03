#ifndef _MBTCP_H_
#define _MBTCP_H_

#include <stdbool.h>
#include <stdint.h>

extern uint8_t mbTCPtid1;
extern uint8_t mbTCPtid2;
	
extern volatile uint8_t *pucRTUBufferCur;
extern volatile uint16_t usRTUBufferPos;

bool MBtcp2rtuFrame(uint8_t sock);

#endif


