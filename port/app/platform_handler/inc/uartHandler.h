#ifndef UARTHANDLER_H_
#define UARTHANDLER_H_

#include <stdint.h>
#include "port_common.h"
#include "common.h"
#include "ConfigData.h"
//#include "seg.h"

//#define _UART_DEBUG_

#ifndef DATA_BUF_SIZE
    #define DATA_BUF_SIZE 2048
#endif

// XON/XOFF: Transmitter On / Off, Software flow control
#define UART_XON                        0x11 // 17
#define UART_XOFF                       0x13 // 19
#define UART_ON_THRESHOLD               (uint16_t)(SEG_DATA_BUF_SIZE / 10)
#define UART_OFF_THRESHOLD              (uint16_t)(SEG_DATA_BUF_SIZE - UART_ON_THRESHOLD)

// UART interface selector, RS-232/TTL or RS-422/485
#define UART_IF_RS232_TTL               0
#define UART_IF_TTL                     0
#define UART_IF_RS232                   1
#define UART_IF_RS422                   2
#define UART_IF_RS485                   3
#define UART_IF_RS485_REVERSE           4

#define UART_IF_STR_TTL                 "TTL"
#define UART_IF_STR_RS232               "RS-232"
#define UART_IF_STR_RS422               "RS-422"
#define UART_IF_STR_RS485               "RS-485"

// If the define '__USE_UART_IF_SELECTOR__' disabled, default UART interface is selected to be 'UART_IF_DEFAULT'
//#define UART_IF_DEFAULT                 UART_IF_RS485
#define UART_IF_DEFAULT               UART_IF_RS232

// UART RTS/CTS pins
// RTS: output, CTS: input
#define UART_RTS_HIGH                   1
#define UART_RTS_LOW                    0

#define UART_CTS_HIGH                   1
#define UART_CTS_LOW                    0

#define UART_ID uart1

#define DEBUG_UART_DEFAULT_BAUDRATE     (115200)

enum baud {
    baud_300 = 0,
    baud_600 = 1,
    baud_1200 = 2,
    baud_1800 = 3,
    baud_2400 = 4,
    baud_4800 = 5,
    baud_9600 = 6,
    baud_14400 = 7,
    baud_19200 = 8,
    baud_28800 = 9,
    baud_38400 = 10,
    baud_57600 = 11,
    baud_115200 = 12,
    baud_230400 = 13,
    baud_460800 = 14
};

enum word_len {
    word_len7 = 0,
    word_len8 = 1,
    word_len9 = 2
};

enum stop_bit {
    stop_bit1 = 0,
    stop_bit2 = 1
};

enum parity {
    parity_none = 0,
    parity_odd = 1,
    parity_even = 2
};

enum flow_ctrl {
    flow_none = 0,
    flow_xon_xoff = 1,
    flow_rts_cts = 2,
    flow_rtsonly = 3,  // RTS_ONLY
    flow_reverserts = 4 // Reverse RTS
};

enum protocol {
    protocol_none = 0,
    modbus_rtu = 1,
    modbus_ascii = 2
};

extern uint8_t flag_ringbuf_full;

extern uint32_t baud_table[];
extern uint8_t word_len_table[];
extern uint8_t stop_bit_table[];
extern uint8_t * parity_table[];
extern uint8_t * flow_ctrl_table[];
extern uint8_t * uart_if_table[];

void DEBUG_UART_Configuration(void);
void DATA0_UART_Configuration(void);
void DATA0_UART_Interrupt_Enable(void);
void DATA1_UART_Configuration(void);

// XON/XOFF Software flow control: Check the Buffer usage and Send the start/stop commands
void check_uart_flow_control(uint8_t flow_ctrl);

// Hardware flow control by GPIOs (RTS/CTS)
#ifdef __USE_GPIO_HARDWARE_FLOWCONTROL__
    uint8_t get_uart_cts_pin(void);
    void set_uart_rts_pin_high(void);
    void set_uart_rts_pin_low(void);
#endif

int32_t platform_uart_putc(uint16_t ch);                    // User Buffer -> UART
int32_t platform_uart_getc(void);                                 // Ring Buffer -> User
int32_t platform_uart_getc_nonblk(void);
int32_t platform_uart_puts(uint8_t* buf, uint16_t bytes);
int32_t platform_uart_gets(uint8_t* buf, uint16_t bytes);

uint8_t get_byte_from_uart(void);                        // UART Port -> User
void get_byte_from_uart_it(void);                        // UART Port -> User (global variable for IRQ handler)
void put_byte_to_uart_buffer(uint8_t ch);          // User -> Ring Buffer

uint16_t get_uart_buffer_usedsize(void);
uint16_t get_uart_buffer_freesize(void);
int8_t is_uart_buffer_empty(void);
int8_t is_uart_buffer_full(void);

void uart_rx_flush(void);                                // UART buffer flush

uint8_t get_uart_rs485_sel(void);
void uart_rs485_rs422_init(void);
void uart_rs485_disable(void);
void uart_rs485_enable(void);

//#define MIN(_a, _b) (_a < _b) ? _a : _b
#define MEM_FREE(mem_p) do{ if(mem_p) { free(mem_p); mem_p = NULL; } }while(0)	//
//#define BITSET(var_v, bit_v) SET_BIT(var_v, bit_v)	//(var_v |= bit_v)
//#define BITCLR(var_v, bit_v) CLEAR_BIT(var_v, bit_v)//(var_v &= ~(bit_v))

#define BUFFER_DEFINITION(_name, _size) \
    uint8_t _name##_buf[_size]; \
    volatile uint16_t _name##_wr=0; \
    volatile uint16_t _name##_rd=0; \
    volatile uint16_t _name##_sz=_size;
#define BUFFER_DECLARATION(_name) \
    extern uint8_t _name##_buf[]; \
    extern uint16_t _name##_wr, _name##_rd, _name##_sz;
#define BUFFER_CLEAR(_name) \
    _name##_wr=0;\
    _name##_rd=0;

#define BUFFER_USED_SIZE(_name) ((_name##_sz + _name##_wr - _name##_rd) % _name##_sz)
#define BUFFER_FREE_SIZE(_name) ((_name##_sz + _name##_rd - _name##_wr - 1) % _name##_sz)
#define IS_BUFFER_EMPTY(_name) ( (_name##_rd) == (_name##_wr))
#define IS_BUFFER_FULL(_name) (BUFFER_FREE_SIZE(_name) == 0)	// I guess % calc takes time a lot, so...
//#define IS_BUFFER_FULL(_name) ((_name##_rd!=0 && _name##_wr==_name##_rd-1)||(_name##_rd==0 && _name##_wr==_name##_sz-1))

#define BUFFER_IN(_name) _name##_buf[_name##_wr]
#define BUFFER_IN_OFFSET(_name, _offset) _name##_buf[_name##_wr + _offset]
#define BUFFER_IN_MOVE(_name, _num) _name##_wr = (_name##_wr + _num) % _name##_sz
#define BUFFER_IN_1ST_SIZE(_name) (_name##_sz - _name##_wr - ((_name##_rd==0)?1:0))
#define BUFFER_IN_2ND_SIZE(_name) ((_name##_rd==0) ? 0 : _name##_rd-1)
#define IS_BUFFER_IN_SEPARATED(_name) (_name##_rd <= _name##_wr)

#define BUFFER_OUT(_name) _name##_buf[_name##_rd]
#define BUFFER_OUT_OFFSET(_name, _offset) _name##_buf[_name##_rd + _offset]
#define BUFFER_OUT_MOVE(_name, _num) _name##_rd = (_name##_rd + _num) % _name##_sz
#define BUFFER_OUT_1ST_SIZE(_name) (_name##_sz - _name##_rd)
#define BUFFER_OUT_2ND_SIZE(_name) (_name##_wr)
#define IS_BUFFER_OUT_SEPARATED(_name) (_name##_rd > _name##_wr)

#endif /* UARTHANDLER_H_ */
