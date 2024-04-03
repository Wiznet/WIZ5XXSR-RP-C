#ifndef __MODBUS_SERIAL_H__
#define __MODBUS_SERIAL_H__

int UART_read(void *data, int bytes);
uint32_t UART_write(void *data, int bytes);

#endif
