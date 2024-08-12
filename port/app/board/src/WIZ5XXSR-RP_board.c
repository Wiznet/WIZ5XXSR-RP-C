
#include "port_common.h"
#include "common.h"
#include "WIZ5XXSR-RP_board.h"

#include "timerHandler.h"
#include "uartHandler.h"
#include "gpioHandler.h"

volatile uint16_t phylink_check_time_msec = 0;
uint8_t flag_check_phylink = 0;
uint8_t flag_hw_trig_enable = 0;

// LEDs on board
const uint16_t LED_PIN[LEDn] = {LED1_PIN, LED2_PIN, LED3_PIN};
uint8_t GPIO_INIT[LEDn] = {DISABLE, DISABLE, DISABLE};

// Serial interface mode selector 0/1
void init_serial_mode_select_pin(void);
uint8_t get_serial_mode_select_pin(uint8_t sel);

/* RP2040 Board Initialization */
void RP2040_Board_Init(void)
{
#ifdef __USE_SERIAL_FLASH__
    /* On-board Serial Flash Initialize */
    SFlash_Init();
#endif

#ifdef __USE_HW_FACTORY_RESET__
    // Factory reset pin initialize
    init_factory_reset_pin();
#endif

#ifdef __USE_HW_APPBOOT_ENTRY__
    // AppBoot entry pin initialize
    init_appboot_entry_pin();
#endif

#ifdef __USE_UART_IF_SELECTOR__
    // UART interface selector pin initialize
    init_uart_if_sel_pin(); // UART interface selector: RS-232 / RS-422 / RS-485
#endif
    init_connection_status_io();

    /* GPIOs Initialize */
    Device_IO_Init();

    /* HW_TRIG input pin - Check this Pin only once at boot (switch) */
    init_hw_trig_pin();

    // STATUS #1 : PHY link status (LED_0)
    // STATUS #2 : TCP connection status (LED_1)
    LED_Init(LED3);
}

uint8_t get_phylink(void)
{
    return wizphy_getphylink();
}

// Hardware mode switch pin, active low
void init_hw_trig_pin(void)
{
    GPIO_Configuration(HW_TRIG_PIN, IO_INPUT, IO_PULLUP);
}

uint8_t get_hw_trig_pin(void)
{
    // HW_TRIG input; Active low
    uint8_t hw_trig, i;
    for(i = 0; i < 5; i++)
    {
        hw_trig = GPIO_Input_Read(HW_TRIG_PIN);
        if(hw_trig != 0) return 1; // High
        wiz_delay_ms(5);
    }
    return 0; // Low
}


void init_uart_if_sel_pin(void)
{
    init_serial_mode_select_pin();

    // for WIZ750SR series
    //GPIO_Configuration(UART_IF_SEL_PORT, UART_IF_SEL_PIN, GPIO_Mode_IN);
}


uint8_t get_uart_if_sel_pin(void)
{  // Status of UART interface selector pin input; [0] RS-232/TTL mode, [1] RS-422/485 mode
  #ifdef __USE_UART_IF_SELECTOR__
    return GPIO_ReadInputDataBit(UART_IF_SEL_PORT, UART_IF_SEL_PIN);
  #else
    //return UART_IF_DEFAULT;
    return UART_IF_RS485;
  #endif
}

// TCP connection status pin
void init_tcpconnection_status_pin(void)
{
  GPIO_Configuration(STATUS_TCPCONNECT_PIN, IO_OUTPUT, IO_NOPULL);
  
  // Pin initial state; Low
  GPIO_Output_Reset(STATUS_TCPCONNECT_PIN);
}


#ifdef __USE_HW_FACTORY_RESET__
void init_factory_reset_pin(void)
{
    //GPIO_Configuration(FAC_RSTn_PORT, FAC_RSTn_PIN, GPIO_MODE_INPUT, GPIO_PULLUP);
    //NVIC_EnableIRQ(FAC_RESET_EXTI_IRQn);
    GPIO_Configuration(FAC_RSTn_PIN, IO_INPUT, IO_PULLUP);
    gpio_set_irq_enabled_with_callback(FAC_RSTn_PIN, GPIO_IRQ_EDGE_FALL, true, &factory_reset_pin_callback);
}

uint8_t get_factory_reset_pin(void)
{
    return GPIO_Input_Read(FAC_RSTn_PIN);
}
#endif

/**
  * @brief  Configures LED GPIO.
  * @param  Led: Specifies the Led to be configured.
  *   This parameter can be one of following parameters:
  *     @arg LED1
  *     @arg LED2
  * @retval None
  */
void LED_Init(Led_TypeDef Led)
{
  if(Led >= LEDn) return;
  
  /* Configure the GPIO_LED pin */
  gpio_init(LED_PIN[Led]);
  gpio_set_dir(LED_PIN[Led], GPIO_OUT);  
  
  /* LED off */
  LED_Off(Led);
}

/**
  * @brief  Turns selected LED On.
  * @param  Led: Specifies the Led to be set on.
  *   This parameter can be one of following parameters:
  *     @arg LED1
  *     @arg LED2
  * @retval None
  */
void LED_On(Led_TypeDef Led)
{
  if(Led >= LEDn) return;
  gpio_put(LED_PIN[Led], 1);
}

/**
  * @brief  Turns selected LED Off.
  * @param  Led: Specifies the Led to be set off.
  *   This parameter can be one of following parameters:
  *     @arg LED1
  *     @arg LED2
  * @retval None
  */
void LED_Off(Led_TypeDef Led)
{
  if(Led >= LEDn) return;
  gpio_put(LED_PIN[Led], 0);
}

/**
  * @brief  Toggles the selected LED.
  * @param  Led: Specifies the Led to be toggled.
  *   This parameter can be one of following parameters:
  *     @arg LED1
  *     @arg LED2
  * @retval None
  */
void LED_Toggle(Led_TypeDef Led)
{
  uint32_t pin_mask = (1ul << LED_PIN[Led]);
  
  if(Led >= LEDn) return;
  gpio_xor_mask(pin_mask);
}

uint8_t get_LED_Status(Led_TypeDef Led)
{
    if(GPIO_INIT[Led] != ENABLE) return 0;
    return gpio_get(LED_PIN[Led]);
}
