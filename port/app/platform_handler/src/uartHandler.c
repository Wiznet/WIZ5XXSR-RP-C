#include <string.h>
#include "common.h"
#include "ConfigData.h"
#include "uartHandler.h"
#include "gpioHandler.h"
#include "seg.h"
#include "port_common.h"
#include "WIZ5XXSR-RP_board.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/


/* Private functions prototypes ----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

// UART Ring buffer declaration
BUFFER_DEFINITION(data0_rx, SEG_DATA_BUF_SIZE);

uint8_t flag_ringbuf_full = 0;

uint32_t baud_table[] = {300, 600, 1200, 1800, 2400, 4800, 9600, 14400, 19200, 28800, 38400, 57600, 115200, 230400, 460800};
uint8_t word_len_table[] = {7, 8, 9};
uint8_t * parity_table[] = {(uint8_t *)"N", (uint8_t *)"ODD", (uint8_t *)"EVEN"};
uint8_t stop_bit_table[] = {1, 2};
uint8_t * flow_ctrl_table[] = {(uint8_t *)"NONE", (uint8_t *)"XON/XOFF", (uint8_t *)"RTS/CTS", (uint8_t *)"RTS Only", (uint8_t *)"RTS Only Reverse"};
uint8_t * uart_if_table[] = {(uint8_t *)UART_IF_STR_TTL, (uint8_t *)UART_IF_STR_RS232, (uint8_t *)UART_IF_STR_RS422, (uint8_t *)UART_IF_STR_RS485, (uint8_t *)UART_IF_STR_RS485};
//uint8_t * uart_if_table[] = {(uint8_t *)UART_IF_STR_RS232_TTL, (uint8_t *)UART_IF_STR_RS422_485};


// XON/XOFF Status; 
static uint8_t xonoff_status = UART_XON;

// RTS Status; __USE_GPIO_HARDWARE_FLOWCONTROL__ defined
#ifdef __USE_GPIO_HARDWARE_FLOWCONTROL__
    static uint8_t rts_status = UART_RTS_LOW;
#endif

// UART Interface selector; RS-422 or RS-485 use only
static uint8_t uart_if_mode = UART_IF_RS422;

/* Public functions ----------------------------------------------------------*/

////////////////////////////////////////////////////////////////////////////////
// Data UART Configuration
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
// Data UART Configuration & IRQ handler
////////////////////////////////////////////////////////////////////////////////

// RX interrupt handler
void on_uart_rx(void)
{
    //uartRxByte: // 1-byte character variable for UART Interrupt request handler

    uint8_t ch;

    while (uart_is_readable(UART_ID))
        ch = uart_getc(UART_ID);

//#ifdef _UART_DEBUG_
#if 0
    printf("%c", ch);// ## UART echo; for debugging
#endif

    if(!(check_modeswitch_trigger(ch))) // ret: [0] data / [!0] trigger code
    {
        if(is_uart_buffer_full() == FALSE)
        {
            if(check_serial_store_permitted(ch)) // ret: [0] not permitted / [1] permitted
            {
                put_byte_to_uart_buffer(ch);
            }
        }
        else
        {
            // buffer full
            flag_ringbuf_full = ON;
        }
        init_time_delimiter_timer();
    }
}

void DATA0_UART_Configuration(void)
{
    struct __serial_option *serial_option = (struct __serial_option *)&(get_DevConfig_pointer()->serial_option);
    uint8_t valid_arg = 0;
    uint8_t temp_data_bits, temp_stop_bits, temp_parity;

    uart_deinit(UART_ID);

    // Set up our UART with a basic baud rate.
    uart_init(UART_ID, 2400);

    // Set the TX and RX pins by using the function select on the GPIO
    // Set datasheet for more information on function select
    gpio_set_function(DATA0_UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(DATA0_UART_RX_PIN, GPIO_FUNC_UART);
    gpio_set_function(DATA0_UART_CTS_PIN, GPIO_FUNC_UART);
    gpio_set_function(DATA0_UART_RTS_PIN, GPIO_FUNC_UART);

    /* Set Baud Rate */
    if(serial_option->baud_rate < (sizeof(baud_table) / sizeof(baud_table[0])))
    {
        uart_set_baudrate(UART_ID, baud_table[serial_option->baud_rate]);
        valid_arg = 1;
    }
    
    if(!valid_arg)
        uart_set_baudrate(UART_ID, baud_table[baud_115200]);

    /* Set Data Bits */
    switch(serial_option->data_bits) {
        case word_len7:
            temp_data_bits = 7;
            break;
        case word_len8:
            temp_data_bits = 8;
            break;
        case word_len9:
            temp_data_bits = 9;
            break;
        default:
            temp_data_bits = 8;
            serial_option->data_bits = word_len8;
            break;
    }

    /* Set Stop Bits */
    switch(serial_option->stop_bits) {
        case stop_bit1:
            temp_stop_bits = 1;
            break;
        case stop_bit2:
            temp_stop_bits = 2;
            break;
        default:
            temp_stop_bits = 1;
            serial_option->stop_bits = stop_bit1;
            break;
    }

    /* Set Parity Bits */
    switch(serial_option->parity) {
        case parity_none:
            temp_parity = UART_PARITY_NONE;
            break;
        case parity_odd:
            temp_parity = UART_PARITY_ODD;
            break;
        case parity_even:
            temp_parity = UART_PARITY_EVEN;
            break;
        default:
            temp_parity = UART_PARITY_NONE;
            serial_option->parity = parity_none;
            break;
    }
    
    /* Flow Control */
    if((serial_option->uart_interface == UART_IF_TTL) || (serial_option->uart_interface == UART_IF_RS232))
    {
        // RS232 Hardware Flow Control
        //7     RTS     Request To Send     Output
        //8     CTS     Clear To Send       Input
        switch(serial_option->flow_control) {
            case flow_none:
                uart_set_hw_flow(UART_ID, false, false);
                break;
            case flow_rts_cts:
#ifdef __USE_GPIO_HARDWARE_FLOWCONTROL__
                uart_set_hw_flow(UART_ID, false, false);
                set_uart_rts_pin_low(uartNum);
#else
                uart_set_hw_flow(UART_ID, true, true);
#endif
                break;
            case flow_xon_xoff:
                uart_set_hw_flow(UART_ID, false, false);
                break;
            default:
                uart_set_hw_flow(UART_ID, false, false);
                serial_option->flow_control = flow_none;
                break;
        }
    }

#ifdef __USE_UART_485_422__
    else // UART_IF_RS422 || UART_IF_RS485
    {
        uart_set_hw_flow(UART_ID, false, false);
        
        // GPIO configuration (RTS pin -> GPIO: 485SEL)
        if((serial_option->flow_control != flow_rtsonly) && (serial_option->flow_control != flow_reverserts))
            uart_if_mode = get_uart_rs485_sel();
        else
        {
            if(serial_option->flow_control == flow_rtsonly)
                uart_if_mode = UART_IF_RS485;
            else
                uart_if_mode = UART_IF_RS485_REVERSE;            
        }
        uart_rs485_rs422_init();
        serial_option->uart_interface = uart_if_mode;
    }
#endif

    // Set our data format
    uart_set_format(UART_ID, temp_data_bits, temp_stop_bits, temp_parity);

    // Turn off FIFO's - we want to do this character by character
    uart_set_fifo_enabled(UART_ID, false);

    PRT_INFO("serial_option->flow_control = %d\r\n", serial_option->flow_control);
    PRT_INFO("data_bits = %d, stop_bits = %d, parity = %d\r\n", temp_data_bits, temp_stop_bits, temp_parity);
    PRT_INFO("baud = %d\r\n", baud_table[baud_115200]);
}

void DATA0_UART_Interrupt_Enable(void)
{
    // And set up and enable the interrupt handlers
    int UART_IRQ = UART_ID == uart0 ? UART0_IRQ : UART1_IRQ;
    irq_set_exclusive_handler(UART_IRQ, on_uart_rx);
    irq_set_enabled(UART_IRQ, true);

    // Set up a RX interrupt
    // We need to set up the handler first
    // Select correct interrupt for the UART we are using

    // Now enable the UART to send interrupts - RX only
    uart_set_irq_enables(UART_ID, true, false);


}

void check_uart_flow_control(uint8_t flow_ctrl)
{
    if(flow_ctrl == flow_xon_xoff)
    {
        if((xonoff_status == UART_XON) && (get_uart_buffer_usedsize() > UART_OFF_THRESHOLD)) // Send the transmit stop command to peer - go XOFF
        {
            platform_uart_putc(UART_XOFF);
            xonoff_status = UART_XOFF;
#ifdef _UART_DEBUG_
            printf(" >> SEND XOFF [%d / %d]\r\n", get_uart_buffer_usedsize(), SEG_DATA_BUF_SIZE);
#endif
        }
        else if((xonoff_status == UART_XOFF) && (get_uart_buffer_usedsize() < UART_ON_THRESHOLD)) // Send the transmit start command to peer. -go XON
        {
            platform_uart_putc(UART_XON);
            xonoff_status = UART_XON;
#ifdef _UART_DEBUG_
            printf(" >> SEND XON [%d / %d]\r\n", get_uart_buffer_usedsize(uartNum), SEG_DATA_BUF_SIZE);
#endif
        }
    }
#ifdef __USE_GPIO_HARDWARE_FLOWCONTROL__
    else if(flow_ctrl == flow_rts_cts) // RTS pin control
    {
        // Buffer full occurred
        if((rts_status == UART_RTS_LOW) && (get_uart_buffer_usedsize(uartNum) > UART_OFF_THRESHOLD))
        {
            set_uart_rts_pin_high(uartNum);
            rts_status = UART_RTS_HIGH;
#ifdef _UART_DEBUG_
            printf(" >> UART_RTS_HIGH [%d / %d]\r\n", get_uart_buffer_usedsize(uartNum), SEG_DATA_BUF_SIZE);
#endif
        }
        
        // Clear the buffer full event
        if((rts_status == UART_RTS_HIGH) && (get_uart_buffer_usedsize(uartNum) <= UART_OFF_THRESHOLD))
        {
            set_uart_rts_pin_low(uartNum);
            rts_status = UART_RTS_LOW;
#ifdef _UART_DEBUG_
            printf(" >> UART_RTS_LOW [%d / %d]\r\n", get_uart_buffer_usedsize(uartNum), SEG_DATA_BUF_SIZE);
#endif
        }
    }
#endif
}


int32_t platform_uart_putc(uint16_t ch)
{
    struct __serial_option *serial_option = (struct __serial_option *)&(get_DevConfig_pointer()->serial_option);
    uint8_t c[1];

    if(serial_option->data_bits == word_len8)
    {
        c[0] = ch & 0x00FF;
    }
    else if (serial_option->data_bits == word_len7)
    {
        c[0] = ch & 0x007F; // word_len7
    }
    uart_putc(UART_ID, c[0]);

    return RET_OK;
}

int32_t platform_uart_puts(uint8_t* buf, uint16_t bytes)
{
    uint32_t i;

    for(i=0; i<bytes; i++)
        platform_uart_putc(buf[i]);

#if 0
    else if(uartNum == DEBUG_UART_PORTNUM)
    {
        HAL_UART_Transmit(&huart2, (uint8_t *)&buf, bytes, 1000);
    }
#endif
    return bytes;
}

int32_t platform_uart_getc(void)
{
    int32_t ch;

    while(IS_BUFFER_EMPTY(data0_rx));
    ch = (int32_t)BUFFER_OUT(data0_rx);
    BUFFER_OUT_MOVE(data0_rx, 1);

    return ch;
}

int32_t platform_uart_getc_nonblk(void)
{
    int32_t ch;

    if(IS_BUFFER_EMPTY(data0_rx)) return RET_NOK;
    ch = (int32_t)BUFFER_OUT(data0_rx);
    BUFFER_OUT_MOVE(data0_rx, 1);

    return ch;
}

int32_t platform_uart_gets(uint8_t* buf, uint16_t bytes)
{
    uint16_t lentot = 0, len1st = 0;

    lentot = bytes = MIN(BUFFER_USED_SIZE(data0_rx), bytes);
    if(IS_BUFFER_OUT_SEPARATED(data0_rx) && (len1st = BUFFER_OUT_1ST_SIZE(data0_rx)) < bytes) {
        memcpy(buf, &BUFFER_OUT(data0_rx), len1st);
        BUFFER_OUT_MOVE(data0_rx, len1st);
        bytes -= len1st;
    }
    memcpy(buf+len1st, &BUFFER_OUT(data0_rx), bytes);
    BUFFER_OUT_MOVE(data0_rx, bytes);

    return lentot;
}

void uart_rx_flush(void)
{
    BUFFER_CLEAR(data0_rx);
}

void put_byte_to_uart_buffer(uint8_t ch)
{
    BUFFER_IN(data0_rx) = ch;
    BUFFER_IN_MOVE(data0_rx, 1);
}


uint16_t get_uart_buffer_usedsize(void)
{
    uint16_t len = 0;

    len = BUFFER_USED_SIZE(data0_rx);
    return len;
}

uint16_t get_uart_buffer_freesize(void)
{
    uint16_t len = 0;

    len = BUFFER_FREE_SIZE(data0_rx);
    return len;
}

int8_t is_uart_buffer_empty(void)
{
    int8_t ret = 0;

    ret = IS_BUFFER_EMPTY(data0_rx);
    return ret;
}

int8_t is_uart_buffer_full(void)
{
    int8_t ret = 0;

    ret = IS_BUFFER_FULL(data0_rx);
    return ret;
}

#ifdef __USE_UART_485_422__
uint8_t get_uart_rs485_sel(void)
{
    struct __serial_option *serial_option = (struct __serial_option *)&(get_DevConfig_pointer()->serial_option);

    GPIO_Configuration(DATA0_UART_RTS_PIN, GPIO_IN, IO_NOPULL); // UART0 RTS pin: GPIO / Input
    GPIO_Output_Set(DATA0_UART_RTS_PIN);
    
    if(GPIO_Input_Read(DATA0_UART_RTS_PIN) == IO_LOW)
        uart_if_mode = UART_IF_RS422;
    else
        uart_if_mode = UART_IF_RS485;
    return uart_if_mode;
}


void uart_rs485_rs422_init(void)
{
    GPIO_Configuration(DATA0_UART_RTS_PIN, GPIO_OUT, IO_NOPULL); // UART0 RTS pin: GPIO / Output
}


void uart_rs485_enable(void)
{
    if(uart_if_mode == UART_IF_RS485)
    {
      GPIO_Output_Set(DATA0_UART_RTS_PIN);
      delay_ms(1);
    }
    else if(uart_if_mode == UART_IF_RS485_REVERSE)
    {
        // RTS pin -> High
        GPIO_Output_Reset(DATA0_UART_RTS_PIN);
        delay_ms(1);
    }    //UART_IF_RS422: None
}


void uart_rs485_disable(void)
{
    if(uart_if_mode == UART_IF_RS485)
    {
        // RTS pin -> Low
        GPIO_Output_Reset(DATA0_UART_RTS_PIN);
        delay_ms(1);
    }
    else if(uart_if_mode == UART_IF_RS485_REVERSE)
    {
        // RTS pin -> High
        GPIO_Output_Set(DATA0_UART_RTS_PIN);
        delay_ms(1);
    }
    
    //UART_IF_RS422: None
}
#endif

#ifdef __USE_GPIO_HARDWARE_FLOWCONTROL__
    
uint8_t get_uart_cts_pin(void)
{
    uint8_t cts_pin = UART_CTS_HIGH;

#ifdef _UART_DEBUG_
    static uint8_t prev_cts_pin;
#endif
    cts_pin = GPIO_Input_Read(DATA0_UART_CTS_PIN);


#ifdef _UART_DEBUG_
    if(cts_pin != prev_cts_pin)
    {
        printf(" >> UART_CTS_%s\r\n", cts_pin?"HIGH":"LOW");
        prev_cts_pin = cts_pin;
    }
#endif
    
    return cts_pin;
}

void set_uart_rts_pin_high(void)
{
    GPIO_Output_Set(DATA0_UART_RTS_PIN);
}

void set_uart_rts_pin_low(void)
{
    GPIO_Output_Reset(DATA0_UART_RTS_PIN);
}

#endif
