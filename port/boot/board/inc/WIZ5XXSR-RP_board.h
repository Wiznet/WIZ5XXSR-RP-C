/*
*
@file   WIZ5XXSR-RP_board.h
@brief
*/

#ifndef __WIZ5XXSR-RP_BOARD_H__ 
#define __WIZ5XXSR-RP_BOARD_H__ 

#include <stdint.h>
#include "common.h"

////////////////////////////////
// Product Configurations     //
////////////////////////////////

/* Target Board Selector */
//#define DEVICE_BOARD_NAME WIZwiki_W7500ECO
//#define DEVICE_BOARD_NAME W7500P_S2E
//#define DEVICE_BOARD_NAME WIZ750SR
//#define DEVICE_BOARD_NAME WIZ750SR_1xx
//#define DEVICE_BOARD_NAME W7500_S2E

#define DEVICE_BOARD_NAME WIZ5XXSR_RP

    
typedef enum {RESET = 0, SET = !RESET} FlagStatus, ITStatus;

#if (DEVICE_BOARD_NAME == WIZ5XXSR_RP) // Chip product
    #define __USE_DHCP_INFINITE_LOOP__          // When this option is enabled, if DHCP IP allocation failed, process_dhcp() function will try to DHCP steps again.
    #define __USE_DNS_INFINITE_LOOP__           // When this option is enabled, if DNS query failed, process_dns() function will try to DNS steps again.
    #define __USE_HW_FACTORY_RESET__            // Use Factory reset pin
    //#define __USE_UART_IF_SELECTOR__            // Use Serial interface port selector pin
    #define __USE_SAFE_SAVE__                   // When this option is enabled, data verify is additionally performed in the flash save of config-data.
    #define __USE_WATCHDOG__                  // WDT timeout 30 Second
        #define __USE_UART_485_422__
    //#define __USE_USERS_GPIO__
    #define DEVICE_ID_DEFAULT                   "WIZ5XXSR-RP"//"S2E_SSL-MB" // Device name
    #define DEVICE_CLOCK_SELECT                 CLOCK_SOURCE_EXTERNAL // or CLOCK_SOURCE_INTERNAL
    #define DEVICE_UART_CNT                     (1)
    #define DEVICE_SETTING_PASSWORD_DEFAULT     "00000000"
    #define DEVICE_GROUP_DEFAULT                "WORKGROUP" // Device group
    #define DEVICE_TARGET_SYSTEM_CLOCK   PLL_SYS_KHZ
#endif

/* PHY Link check  */
#define PHYLINK_CHECK_CYCLE_MSEC  1000

/* Factory Reset period  */
#define FACTORY_RESET_TIME_MS   5000

////////////////////////////////
// Pin definitions        //
////////////////////////////////

#if (DEVICE_BOARD_NAME == WIZ5XXSR_RP) // ##20161031 WIZ750SR-1xx      
  #define DTR_PIN                 8
  #define DSR_PIN                 9
    
  #define STATUS_PHYLINK_PIN      10
  #define STATUS_TCPCONNECT_PIN   11

  // UART0
  #define UART0_TX_PIN      0
  #define UART0_RX_PIN      1
  #define UART0_CTS_PIN     2
  #define UART0_RTS_PIN     3

  // UART1
  #define DATA0_UART_TX_PIN      4
  #define DATA0_UART_RX_PIN      5
  #define DATA0_UART_CTS_PIN     6
  #define DATA0_UART_RTS_PIN     7

  #define BOOT_MODE_PIN          13
  #define FAC_RSTn_PIN           28
  #define HW_TRIG_PIN            29
  #define DATA0_UART_PORTNUM          (1)
  
#ifdef __USE_USERS_GPIO__
  #define USER_IO_NO_ADC        0xFF
  #define USER_IOn       2

  #define USER_IO_A      (uint16_t)(0x01 <<  0)     // USER's I/O A
  #define USER_IO_B      (uint16_t)(0x01 <<  1)     // USER's I/O B

  #define USER_IO_A_PIN       26
  #define USER_IO_A_ADC_CH    3

  #define USER_IO_B_PIN       27
  #define USER_IO_B_ADC_CH    0xFF
#endif

  #define LED1_PIN      STATUS_PHYLINK_PIN        //STATUS_PHYLINK
  #define LED2_PIN      STATUS_TCPCONNECT_PIN    //STATUS_TCP_PIN
  #define LED3_PIN      12    //Blink
  #define LEDn    3
#endif

  typedef enum
  {
    LED1 = 0, // PHY link status
    LED2 = 1, // TCP connection status
    LED3 = 2  // blink
  } Led_TypeDef;

  extern volatile uint16_t phylink_check_time_msec;
  extern uint8_t flag_check_phylink;
  extern uint8_t flag_hw_trig_enable;
  
  void RP2040_Board_Init(void);
  void init_hw_trig_pin(void);
  uint8_t get_hw_trig_pin(void);

#if ((DEVICE_BOARD_NAME == WIZ750SR) || (DEVICE_BOARD_NAME == W7500P_S2E) || (DEVICE_BOARD_NAME == WIZ750SR_1xx))
  void init_phylink_in_pin(void);
  uint8_t get_phylink_in_pin(void);
#endif
  
  void init_uart_if_sel_pin(void);
  uint8_t get_uart_if_sel_pin(void);
  void init_factory_reset_pin(void);
  
#ifdef __USE_BOOT_ENTRY__
  void init_boot_entry_pin(void);
  uint8_t get_boot_entry_pin(void);
#endif
  
  void LED_Init(Led_TypeDef Led);
  void LED_On(Led_TypeDef Led);
  void LED_Off(Led_TypeDef Led);
  void LED_Toggle(Led_TypeDef Led);
  uint8_t get_LED_Status(Led_TypeDef Led);
  
#endif

