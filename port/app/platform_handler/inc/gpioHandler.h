#ifndef __GPIOHANDLER_H__
#define __GPIOHANDLER_H__

#include <stdint.h>

#define _GPIO_DEBUG_

typedef enum
{
	IO_DIGITAL_INPUT  = 0,
	IO_DIGITAL_OUTPUT = 1,
	IO_ANALOG_INPUT   = 2,
	IO_ANALOG_OUTPUT  = 3,
	IO_UNKNOWN        = 0xff
} USER_IO_Define;

typedef enum
{
	IO_INPUT  = 0,
	IO_OUTPUT = 1
} USER_IO_Direction;

typedef enum
{
	IO_DIGITAL   = 0,
	IO_ANALOG_IN = 1
} USER_IO_Type;

typedef enum
{
	IO_LOW  = 0,
	IO_HIGH = 1
} USER_IO_Status;

typedef enum
{
	IO_DISABLE = 0,
	IO_ENABLE  = 1
} USER_IO_Enable;

typedef enum
{
	IO_NOPULL = 0,
	IO_PULLUP  = 1,
	IO_PULLDOWN  = 2,
} USER_IO_PULL;

extern uint8_t        USER_IO_SEL[];
extern const char*    USER_IO_STR[];
extern const char*    USER_IO_PIN_STR[];
extern const char*    USER_IO_TYPE_STR[];
extern const char*    USER_IO_DIR_STR[];


// Device I/O Initialize
void Device_IO_Init(void);

// Status IO
void init_connection_status_io(void);

void init_phylink_status_pin(void);
void init_tcpconnection_status_pin(void);
void init_flowcontrol_dtr_pin(void);
void init_flowcontrol_dsr_pin(void);
void init_status_pin(void);
void set_connection_status_io(uint16_t pin, uint8_t set);
uint8_t get_connection_status_io(uint16_t pin);


// todo:
void set_flowcontrol_dtr_pin(uint8_t set);
uint8_t get_flowcontrol_dsr_pin(void);

uint8_t get_user_io_enabled(uint8_t io_sel);
uint8_t get_user_io_type(uint8_t io_sel);
uint8_t get_user_io_direction(uint8_t io_sel);
uint8_t set_user_io_enable(uint8_t io_sel, uint8_t enable);
uint8_t set_user_io_type(uint8_t io_sel, uint8_t type);
uint8_t set_user_io_direction(uint8_t io_sel, uint8_t dir);
uint8_t get_user_io_val(uint16_t io_sel, uint16_t * val); // get the I/O status or value
uint8_t set_user_io_val(uint16_t io_sel, uint16_t * val); // set the I/O status, digital output only
uint8_t get_user_io_bitorder(uint16_t io_sel);
uint16_t read_ADC(uint8_t ch);

// Check the PHY link status 
uint8_t check_phylink_status(void);
void gpio_handler_timer_msec(void); // This function have to call every 1 millisecond by Timer IRQ handler routine.

void GPIO_Configuration(uint16_t GPIO_Pin, USER_IO_Direction GPIO_Mode, USER_IO_PULL GPIO_Pull);
void GPIO_Output_Set(uint16_t GPIO_Pin);
void GPIO_Output_Reset(uint16_t GPIO_Pin);
uint8_t GPIO_Output_Toggle(uint16_t GPIO_Pin);
uint8_t GPIO_Input_Read(uint16_t GPIO_Pin);

void factory_reset_pin_callback(uint32_t gpio, uint32_t events);

#endif

