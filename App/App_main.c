/**
  ******************************************************************************
  * @file    RP2040 Serial to Ethernet Project - WIZ5XXSR-RP App
  * @author  Mason Lee, PaaS Team
  * @version v1.0.0
  * @date    April-2022
  * @brief   Main program body
  ******************************************************************************
  * @attention
  * @par Revision history
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, WIZnet SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2022 WIZnet Co., Ltd.</center></h2>
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "common.h"
#include "WIZ5XXSR-RP_board.h"

#include "port_common.h"

#include "dhcp.h"
#include "dhcp_cb.h"
#include "dns.h"

#include "seg.h"
#include "segcp.h"
#include "ConfigData.h"

#include "timerHandler.h"
#include "uartHandler.h"
#include "deviceHandler.h"
#include "dnsHandler.h"
#include "ConfigData.h"
#include "flashHandler.h"
#include "gpioHandler.h"
#include "storageHandler.h"
#include "wizchip_conf.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
static void RP2040_Init(void);
static void RP2040_W5X00_Init(void);
static void set_W5X00_NetTimeout(void);

/* Private variables ---------------------------------------------------------*/
extern uint8_t flag_process_dhcp_success;

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
    DevConfig *dev_config = get_DevConfig_pointer();
    uint8_t serial_mode;

    RP2040_Init();
    RP2040_W5X00_Init();
    RP2040_Board_Init();
    load_DevConfig_from_storage();

    DATA0_UART_Configuration();
    check_mac_address();
    set_W5X00_NetTimeout();
    DATA0_UART_Interrupt_Enable();
    Timer_Configuration();
    init_connection_status_io();
    sleep_ms(100);
    
    flag_hw_trig_enable = !get_hw_trig_pin();
    PRT_INFO("check_phylink_status\r\n");
    while (check_phylink_status() == PHY_LINK_OFF){
      sleep_ms(100);
    }

    // Debug UART: Device information print out
    display_Dev_Info_header();
    display_Dev_Info_main();
    
    Net_Conf();
    
    if(dev_config->network_option.dhcp_use)
    {
        if(process_dhcp() == DHCP_IP_LEASED) // DHCP success
        {
            flag_process_dhcp_success = ON;
        }
        else // DHCP failed
        {
            Net_Conf(); // Set default static IP settings
        }
    }

    display_Net_Info();
    display_Dev_Info_dhcp();

    if(dev_config->network_connection.working_mode != TCP_SERVER_MODE)
    {
        if(dev_config->network_connection.dns_use)
        {

            if(process_dns()) // DNS success
            {
                flag_process_dns_success = ON;
                PRT_INFO("flag_process_dns_success == ON\r\n");
            }
            display_Dev_Info_dns();
        }
    }
  
    serial_mode = get_serial_communation_protocol();
    if(serial_mode == SEG_SERIAL_PROTOCOL_NONE)
    {
        PRT_INFO(" > Serial to Ethernet Gateway Mode\r\n");
    }

    if(flag_hw_trig_enable)
    {
        init_trigger_modeswitch(DEVICE_AT_MODE);
        flag_hw_trig_enable = 0;
    }

#ifdef __USE_WATCHDOG__
    watchdog_enable(DEVICE_WDT_TIMEOUT, 0);
#endif
    while (1)
    {
        if (flag_check_phylink)
        {
            flag_check_phylink = 0;
            if (check_phylink_status() == PHY_LINK_OFF)
                device_reboot();
        }
        do_segcp();
        do_seg(SEG_DATA0_SOCK);

        if(flag_process_dhcp_success == ON) DHCP_run(); // DHCP client handler for IP renewal
#ifdef __USE_WATCHDOG__
        watchdog_update();
#endif
    }   
    return 0;
}



/*****************************************************************************
 * Private functions
 ****************************************************************************/
static void RP2040_Init(void)
{
    set_sys_clock_khz(PLL_SYS_KHZ, true);
    
    clock_configure(
        clk_peri,
        0,                                                // No glitchless mux
        CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS, // System PLL on AUX mux
        PLL_SYS_KHZ * 1000,                               // Input frequency
        PLL_SYS_KHZ * 1000                                // Output (must be same as no divider)
    );
    SystemCoreClockUpdate();
    flash_critical_section_init();
    sleep_ms(10);
}

static void RP2040_W5X00_Init(void)
{
    wizchip_spi_initialize((PLL_SYS_KHZ * 1000 / 4)); //33.25Mhz
    wizchip_cris_initialize();

    wizchip_reset();
    wizchip_initialize();
    wizchip_check();
}

static void set_W5X00_NetTimeout(void)
{
    DevConfig *dev_config = get_DevConfig_pointer();
    wiz_NetTimeout net_timeout;
    
    net_timeout.retry_cnt = dev_config->network_option.tcp_rcr_val;
    net_timeout.time_100us = 2000;
    wizchip_settimeout(&net_timeout);
    
    wizchip_gettimeout(&net_timeout); // TCP timeout settings
    PRT_INFO(" - Network Timeout Settings - RCR: %d, RTR: %d\r\n", net_timeout.retry_cnt, net_timeout.time_100us);
}


