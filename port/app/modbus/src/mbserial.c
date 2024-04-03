#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "port_common.h"
#include "WIZ5XXSR-RP_board.h"

#include "uartHandler.h"
#include "common.h"
#include "mbserial.h"
#include "mbrtu.h"
#include "mbascii.h"

BUFFER_DECLARATION(data0_rx);

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/**
 * @brief	UART interrupt handler using ring buffers
 * @return	Nothing
 */

int UART_read(void *data, int bytes)
{
  uint32_t i;
  uint8_t *data_ptr = data;
  if(IS_BUFFER_EMPTY(data0_rx)) return RET_NOK;
  
  for(i=0; i<bytes; i++)
    data_ptr[i] = (uint8_t)BUFFER_OUT(data0_rx);
  BUFFER_OUT_MOVE(data0_rx, i);
  return i;
}

uint32_t UART_write(void *data, int bytes)
{
  uint32_t i;
  uint8_t *data_ptr = data;

  for(i=0; i<bytes; i++)
    uart_putc(UART_ID, data_ptr[i]);
  return bytes;
}
