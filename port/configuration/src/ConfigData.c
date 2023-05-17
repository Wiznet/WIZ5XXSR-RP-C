/*
 * ConfigData.c 
 */

#include <stdio.h>
#include <string.h>
#include "port_common.h"
#include "WIZ5XXSR-RP_board.h"
#include "ConfigData.h"

#include "timerHandler.h"
#include "flashHandler.h"
#include "storageHandler.h"
#include "deviceHandler.h"
#include "gpioHandler.h"
#include "uartHandler.h"

#include "wizchip_conf.h"

#include "seg.h"
#include "segcp.h"

#include "mbedtls/ssl.h"

static DevConfig dev_config;
uint8_t mac[] = {0x00, 0x08, 0xdc, 0xAA, 0xBB, 0xCC};

DevConfig* get_DevConfig_pointer(void)
{
    return &dev_config;
}

void set_DevConfig_to_factory_value(void)
{

    dev_config.device_common.fw_ver[0] = MAJOR_VER;
    dev_config.device_common.fw_ver[1] = MINOR_VER;
    dev_config.device_common.fw_ver[2] = MAINTENANCE_VER;

    /* Product code */
    // WIZ550S2E  : 000
    // WIZ550web  : 120
    // W7500S2E   : 010 (temporary)
    // WIZ2000-MB : 201
    // WIZ510SSL : 301
    // WIZ5XXSR-RP : 401
    dev_config.device_common.device_type[0] = 0x04;
    dev_config.device_common.device_type[1] = 0x00;
    dev_config.device_common.device_type[2] = 0x01;

    memset(dev_config.device_common.device_name, 0x00, sizeof(DEVICE_ID_DEFAULT));
    memcpy(dev_config.device_common.device_name, DEVICE_ID_DEFAULT, sizeof(DEVICE_ID_DEFAULT));

    dev_config.device_common.device_mode = DEVICE_APP_MODE;    // Reserved field for App / Boot identification

    dev_config.config_common.app_protocol = 0;      // Reserved field for device support protocols
    dev_config.config_common.packet_size = sizeof(DevConfig);
    memset(dev_config.config_common.pw_search, 0x00, sizeof(dev_config.config_common.pw_search));

    dev_config.network_common.local_ip[0] = 192;
    dev_config.network_common.local_ip[1] = 168;
    dev_config.network_common.local_ip[2] = 11;
    dev_config.network_common.local_ip[3] = 2;

    dev_config.network_common.gateway[0] = 192;
    dev_config.network_common.gateway[1] = 168;
    dev_config.network_common.gateway[2] = 11;
    dev_config.network_common.gateway[3] = 1;

    dev_config.network_common.subnet[0] = 255;
    dev_config.network_common.subnet[1] = 255;
    dev_config.network_common.subnet[2] = 255;
    dev_config.network_common.subnet[3] = 0;

    dev_config.network_connection.working_mode = TCP_SERVER_MODE; //UDP_MODE; //TCP_MIXED_MODE;
    dev_config.network_connection.working_state = ST_OPEN;

    dev_config.network_connection.local_port = 5000;

    dev_config.network_connection.remote_port = 5000;
    dev_config.network_connection.remote_ip[0] = 192;
    dev_config.network_connection.remote_ip[1] = 168;
    dev_config.network_connection.remote_ip[2] = 11;
    dev_config.network_connection.remote_ip[3] = 3;

    dev_config.network_connection.fixed_local_port = DISABLE;
    dev_config.network_connection.dns_use = DISABLE;

    memset(dev_config.network_connection.dns_domain_name, 0x00, sizeof(dev_config.network_connection.dns_domain_name));
    memcpy(dev_config.network_connection.dns_domain_name, "192.168.11.3", 12);
    dev_config.serial_data_packing.packing_time = 0;

    dev_config.serial_data_packing.packing_size = 0;
    dev_config.serial_data_packing.packing_delimiter[0] = 0; // packing_delimiter used only one-byte (for WIZ107SR compatibility)
    dev_config.serial_data_packing.packing_delimiter[1] = 0;
    dev_config.serial_data_packing.packing_delimiter[2] = 0;
    dev_config.serial_data_packing.packing_delimiter[3] = 0;
    dev_config.serial_data_packing.packing_delimiter_length = 0;
    dev_config.serial_data_packing.packing_data_appendix = 0;

    dev_config.tcp_option.inactivity = 0;        // sec, default: NONE
    dev_config.tcp_option.reconnection = 3000;   // msec, default: 3 sec
    dev_config.tcp_option.keepalive_en = ENABLE;
    dev_config.tcp_option.keepalive_wait_time = 7000;
    dev_config.tcp_option.keepalive_retry_time = 5000;

    memset(dev_config.tcp_option.pw_connect, 0x00, sizeof(dev_config.tcp_option.pw_connect));
    dev_config.tcp_option.pw_connect_en = DISABLE;

    // Default Settings for Data UART: 115200-8-N-1, No flowctrl
    dev_config.serial_option.uart_interface = UART_IF_RS232;
    dev_config.serial_option.protocol = SEG_SERIAL_PROTOCOL_NONE;
    dev_config.serial_option.baud_rate = baud_115200;
    dev_config.serial_option.data_bits = word_len8;
    dev_config.serial_option.parity = parity_none;
    dev_config.serial_option.stop_bits = stop_bit1;
    dev_config.serial_option.flow_control = flow_none;

#ifdef __USE_DSR_DTR_DEFAULT__
        dev_config.serial_option.dtr_en = ENABLE;
        dev_config.serial_option.dsr_en = ENABLE;
#else
        dev_config.serial_option.dtr_en = DISABLE;
        dev_config.serial_option.dsr_en = DISABLE;
#endif

    //dev_config.serial_info[0].serial_debug_en = DISABLE;
    dev_config.serial_common.serial_debug_en = ENABLE;
    dev_config.serial_common.uart_interface_cnt = DEVICE_UART_CNT;

    dev_config.network_option.dhcp_use = DISABLE;

    dev_config.network_option.dns_server_ip[0] = 8; // Default DNS server IP: Google Public DNS (8.8.8.8)
    dev_config.network_option.dns_server_ip[1] = 8;
    dev_config.network_option.dns_server_ip[2] = 8;
    dev_config.network_option.dns_server_ip[3] = 8;

    dev_config.network_option.tcp_rcr_val = 8; // Default RCR(TCP retransmission retry count) value: 8

    dev_config.serial_command.serial_command = ENABLE;
    dev_config.serial_command.serial_command_echo = DISABLE;
    dev_config.serial_command.serial_trigger[0] = 0x2b; // Default serial command mode trigger code: '+++' (0x2b, 0x2b, 0x2b)
    dev_config.serial_command.serial_trigger[1] = 0x2b;
    dev_config.serial_command.serial_trigger[2] = 0x2b;

#ifdef __USE_USERS_GPIO__
    dev_config.user_io_info.user_io_enable = USER_IO_A | USER_IO_B; // [Enabled] / Disabled
    dev_config.user_io_info.user_io_type = 0;
    dev_config.user_io_info.user_io_direction = 0;
    dev_config.user_io_info.user_io_status = 0;
#else
    dev_config.user_io_info.user_io_enable = 0;
    dev_config.user_io_info.user_io_type = 0;
    dev_config.user_io_info.user_io_direction = 0;
    dev_config.user_io_info.user_io_status = 0;
#endif

    // SSL Option
    dev_config.ssl_option.root_ca_option = 0; //MBEDTLS_SSL_VERIFY_NONE;
    dev_config.ssl_option.client_cert_enable = DISABLE;
    dev_config.ssl_option.recv_timeout = 2000;
    
    // MQTT Option

    memset(dev_config.mqtt_option.user_name, 0x00, sizeof(dev_config.mqtt_option.user_name));
    memset(dev_config.mqtt_option.password, 0x00, sizeof(dev_config.mqtt_option.password));
    memset(dev_config.mqtt_option.client_id, 0x00, sizeof(dev_config.mqtt_option.client_id));
    memset(dev_config.mqtt_option.pub_topic, 0x00, sizeof(dev_config.mqtt_option.pub_topic));
    memset(dev_config.mqtt_option.sub_topic_0, 0x00, sizeof(dev_config.mqtt_option.sub_topic_0));
    memset(dev_config.mqtt_option.sub_topic_1, 0x00, sizeof(dev_config.mqtt_option.sub_topic_1));
    memset(dev_config.mqtt_option.sub_topic_2, 0x00, sizeof(dev_config.mqtt_option.sub_topic_2));
    dev_config.mqtt_option.qos = QOS0;
    dev_config.mqtt_option.keepalive = 0;

    // fixed local port enable / disable

    dev_config.device_option.pw_setting_en = ENABLE;
    memset(dev_config.device_option.pw_setting, 0x00, sizeof(dev_config.device_option.pw_setting));
    memset(dev_config.device_option.device_group, 0x00, sizeof(dev_config.device_option.device_group));
    memset(dev_config.device_option.device_alias, 0x00, sizeof(dev_config.device_option.device_alias));

    memcpy(dev_config.device_option.pw_setting, DEVICE_SETTING_PASSWORD_DEFAULT, sizeof(DEVICE_SETTING_PASSWORD_DEFAULT));
    memcpy(dev_config.device_option.device_group, DEVICE_GROUP_DEFAULT, sizeof(DEVICE_GROUP_DEFAULT));

    sprintf((char *)dev_config.device_option.device_alias, "%s-%02X%02X%02X%02X%02X%02X",
                                                       dev_config.device_common.device_name,
                                                       dev_config.network_common.mac[0],
                                                       dev_config.network_common.mac[1],
                                                       dev_config.network_common.mac[2],
                                                       dev_config.network_common.mac[3],
                                                       dev_config.network_common.mac[4],
                                                       dev_config.network_common.mac[5]);

    dev_config.devConfigVer = DEV_CONFIG_VER;//DEV_CONFIG_VER;
}

void load_DevConfig_from_storage(void)
{
    int ret = -1;

    read_storage(STORAGE_CONFIG, &dev_config, sizeof(DevConfig));
    read_storage(STORAGE_MAC, dev_config.network_common.mac, 6);

    if(dev_config.serial_common.serial_debug_en)
      stdio_init_all();
    
    PRT_INFO("MAC = %02X%02X%02X%02X%02X%02X\r\n", dev_config.network_common.mac[0], dev_config.network_common.mac[1], dev_config.network_common.mac[2], \
                                                   dev_config.network_common.mac[3], dev_config.network_common.mac[4], dev_config.network_common.mac[5]);

    PRT_INFO("dev_config.devConfigVer = %d, DEV_CONFIG_VER = %d\r\n", dev_config.devConfigVer, DEV_CONFIG_VER);
    if((dev_config.config_common.packet_size == 0x0000) ||
       (dev_config.config_common.packet_size == 0xFFFF) ||
       (dev_config.config_common.packet_size != sizeof(DevConfig)) ||
        dev_config.devConfigVer != DEV_CONFIG_VER)
    { 
        PRT_INFO(" Config Data size: %d / %d\r\n", dev_config.config_common.packet_size, sizeof(DevConfig));        
        set_DevConfig_to_factory_value();

        //erase_storage(STORAGE_CONFIG);
        write_storage(STORAGE_CONFIG, 0, (uint8_t *)&dev_config, sizeof(DevConfig));
        read_storage(STORAGE_CONFIG, &dev_config, sizeof(DevConfig));

        PRT_INFO("After Config Data size: %d / %d\r\n", dev_config.config_common.packet_size, sizeof(DevConfig));
        device_raw_reboot();
    }
    
    if((dev_config.serial_option.flow_control == flow_rtsonly) || (dev_config.serial_option.flow_control == flow_reverserts))   // Edit for supporting RTS only in 17/3/28 , recommend adapting to WIZ750SR 
    {
      dev_config.serial_option.uart_interface = UART_IF_RS422;    //temporarily set RS422, Actual setting is done in DATA0_UART_Configuration.
    }
    else
    {
      dev_config.serial_option.uart_interface = get_uart_if_sel_pin();
    }

    dev_config.device_common.fw_ver[0] = MAJOR_VER;
    dev_config.device_common.fw_ver[1] = MINOR_VER;
    dev_config.device_common.fw_ver[2] = MAINTENANCE_VER;
}

void load_boot_DevConfig_from_storage(void)
{
    int ret = -1;

    read_storage(STORAGE_CONFIG, &dev_config, sizeof(DevConfig));
    read_storage(STORAGE_MAC, dev_config.network_common.mac, 6);

    if(dev_config.serial_common.serial_debug_en)
      stdio_init_all();

    PRT_INFO("MAC = %02X%02X%02X%02X%02X%02X\r\n", dev_config.network_common.mac[0], dev_config.network_common.mac[1], dev_config.network_common.mac[2], \
                                                   dev_config.network_common.mac[3], dev_config.network_common.mac[4], dev_config.network_common.mac[5]);

    if((dev_config.config_common.packet_size == 0x0000) ||
       (dev_config.config_common.packet_size == 0xFFFF))
    {
        set_DevConfig_to_factory_value();
        //write_storage(STORAGE_CONFIG, 0, (uint8_t *)&dev_config, sizeof(DevConfig));
        //read_storage(STORAGE_CONFIG, &dev_config, sizeof(DevConfig));
        //device_raw_reboot();
    }
    
    if((dev_config.serial_option.flow_control == flow_rtsonly) || (dev_config.serial_option.flow_control == flow_reverserts))   // Edit for supporting RTS only in 17/3/28 , recommend adapting to WIZ750SR 
    {
      dev_config.serial_option.uart_interface = UART_IF_RS422;    //temporarily set RS422, Actual setting is done in DATA0_UART_Configuration.
    }
    else
    {
      dev_config.serial_option.uart_interface = get_uart_if_sel_pin();
    }

    if ((dev_config.network_common.mac[0] == 0xFF))
    {
      dev_config.network_common.mac[0] = 0x00;
      dev_config.network_common.mac[1] = 0x08;
      dev_config.network_common.mac[2] = 0xDC;
      dev_config.network_common.mac[3] = 0x12;
      dev_config.network_common.mac[4] = 0x34;
      dev_config.network_common.mac[5] = 0x56;
    }

    dev_config.device_common.fw_ver[0] = BOOT_MAJOR_VER;
    dev_config.device_common.fw_ver[1] = BOOT_MINOR_VER;
    dev_config.device_common.fw_ver[2] = BOOT_MAINTENANCE_VER;
}


void save_DevConfig_to_storage(void)
{
    erase_storage(STORAGE_CONFIG);
#ifndef __USE_SAFE_SAVE__
    write_storage(STORAGE_CONFIG, 0, (uint8_t *)&dev_config, sizeof(DevConfig));
#else
    DevConfig dev_config_tmp;
    uint8_t update_success = SEGCP_DISABLE;
    uint8_t retry_cnt = 0;
    int ret;
    
    do {
        write_storage(STORAGE_CONFIG, 0, (uint8_t *)&dev_config, sizeof(DevConfig));
        read_storage(STORAGE_CONFIG, &dev_config_tmp, sizeof(DevConfig));
        
#endif        
        if(memcmp(&dev_config, &dev_config_tmp, sizeof(DevConfig)) == 0) { // Config-data set is successfully updated.
            update_success = SEGCP_ENABLE;
        } else {
            retry_cnt++;
            PRT_SEGCP(" > DevConfig update failed, Retry: %d\r\n", retry_cnt);
        }
        
        if(retry_cnt >= MAX_SAVE_RETRY) {
            break;
        }
    } while(update_success != SEGCP_ENABLE);
}

void get_DevConfig_value(void *dest, const void *src, uint16_t size)
{
    memcpy(dest, src, size);
}

void set_DevConfig_value(void *dest, const void *value, const uint16_t size)
{
    memcpy(dest, value, size);
}

void set_DevConfig(wiz_NetInfo *net)
{
    set_DevConfig_value(dev_config.network_common.mac, net->mac, sizeof(dev_config.network_common.mac));
    set_DevConfig_value(dev_config.network_common.local_ip, net->ip, sizeof(dev_config.network_common.local_ip));
    set_DevConfig_value(dev_config.network_common.gateway, net->gw, sizeof(dev_config.network_common.gateway));
    set_DevConfig_value(dev_config.network_common.subnet, net->sn, sizeof(dev_config.network_common.subnet));
    set_DevConfig_value(dev_config.network_option.dns_server_ip, net->dns, sizeof(dev_config.network_option.dns_server_ip));
    if(net->dhcp == NETINFO_STATIC)
        dev_config.network_option.dhcp_use = DISABLE;
    else
        dev_config.network_option.dhcp_use = ENABLE;
}

void get_DevConfig(wiz_NetInfo *net)
{
    get_DevConfig_value(net->mac, dev_config.network_common.mac, sizeof(net->mac));
    get_DevConfig_value(net->ip, dev_config.network_common.local_ip, sizeof(net->ip));
    get_DevConfig_value(net->gw, dev_config.network_common.gateway, sizeof(net->gw));
    get_DevConfig_value(net->sn, dev_config.network_common.subnet, sizeof(net->sn));
    get_DevConfig_value(net->dns, dev_config.network_option.dns_server_ip, sizeof(net->dns));
    if(dev_config.network_option.dhcp_use)
        net->dhcp = NETINFO_DHCP;
    else
        net->dhcp = NETINFO_STATIC;
}

void display_Net_Info(void)
{
    DevConfig *dev_config = get_DevConfig_pointer();
    wiz_NetInfo gWIZNETINFO;

    ctlnetwork(CN_GET_NETINFO, (void*) &gWIZNETINFO);
    PRT_INFO(" # MAC: %02X:%02X:%02X:%02X:%02X:%02X\r\n", gWIZNETINFO.mac[0], gWIZNETINFO.mac[1], gWIZNETINFO.mac[2], gWIZNETINFO.mac[3], gWIZNETINFO.mac[4], gWIZNETINFO.mac[5]);
    PRT_INFO(" # IP : %d.%d.%d.%d / Port : \r\n", gWIZNETINFO.ip[0], gWIZNETINFO.ip[1], gWIZNETINFO.ip[2], gWIZNETINFO.ip[3]);
    PRT_INFO("%d ", dev_config->network_connection.local_port);
    PRT_INFO("\r\n");
    PRT_INFO(" # GW : %d.%d.%d.%d\r\n", gWIZNETINFO.gw[0], gWIZNETINFO.gw[1], gWIZNETINFO.gw[2], gWIZNETINFO.gw[3]);
    PRT_INFO(" # SN : %d.%d.%d.%d\r\n", gWIZNETINFO.sn[0], gWIZNETINFO.sn[1], gWIZNETINFO.sn[2], gWIZNETINFO.sn[3]);
    PRT_INFO(" # DNS: %d.%d.%d.%d\r\n", gWIZNETINFO.dns[0], gWIZNETINFO.dns[1], gWIZNETINFO.dns[2], gWIZNETINFO.dns[3]);
    
    if(dev_config->network_connection.working_mode != TCP_SERVER_MODE)
    {
        if(dev_config->network_connection.dns_use == SEGCP_ENABLE)
        {
            PRT_INFO(" # Destination Domain: %s / Port: %d\r\n",
                    dev_config->network_connection.dns_domain_name,
                    dev_config->network_connection.remote_port);
        }
        else
        {
            PRT_INFO(" # Destination IP: %d.%d.%d.%d / Port: %d\r\n",
                    dev_config->network_connection.remote_ip[0],
                    dev_config->network_connection.remote_ip[1],
                    dev_config->network_connection.remote_ip[2],
                    dev_config->network_connection.remote_ip[3],
                    dev_config->network_connection.remote_port);

            if(dev_config->network_connection.working_mode == UDP_MODE)
            {
                if((dev_config->network_connection.remote_ip[0] == 0) &&
                    (dev_config->network_connection.remote_ip[1] == 0) &&
                    (dev_config->network_connection.remote_ip[2] == 0) &&
                    (dev_config->network_connection.remote_ip[3] == 0))
                {
                    PRT_INFO(" ## UDP 1:N Mode\r\n");
                }
                else
                {
                    PRT_INFO(" ## UDP 1:1 Mode\r\n");
                }
            }
        }
    }
    
    printf("\r\n");
}

void Mac_Conf(void)
{
    DevConfig *dev_config = get_DevConfig_pointer();
    setSHAR(dev_config->network_common.mac);
}

void Net_Conf(void)
{
    DevConfig *dev_config = get_DevConfig_pointer();
    wiz_NetInfo gWIZNETINFO;

    /* wizchip netconf */
    get_DevConfig_value(gWIZNETINFO.mac, dev_config->network_common.mac, sizeof(gWIZNETINFO.mac[0]) * 6);
    get_DevConfig_value(gWIZNETINFO.ip, dev_config->network_common.local_ip, sizeof(gWIZNETINFO.ip[0]) * 4);
    get_DevConfig_value(gWIZNETINFO.gw, dev_config->network_common.gateway, sizeof(gWIZNETINFO.gw[0]) * 4);
    get_DevConfig_value(gWIZNETINFO.sn, dev_config->network_common.subnet, sizeof(gWIZNETINFO.sn[0]) * 4);
    get_DevConfig_value(gWIZNETINFO.dns, dev_config->network_option.dns_server_ip, sizeof(gWIZNETINFO.dns));
    if(dev_config->network_option.dhcp_use)
        gWIZNETINFO.dhcp = NETINFO_DHCP;
    else
        gWIZNETINFO.dhcp = NETINFO_STATIC;

    ctlnetwork(CN_SET_NETINFO, (void*) &gWIZNETINFO);
}

void set_dhcp_mode(void)
{
    DevConfig *dev_config = get_DevConfig_pointer();
    dev_config->network_option.dhcp_use = 1;
}

void check_mac_address(void)
{
    DevConfig *dev_config = get_DevConfig_pointer();
    int ret;
    uint8_t buf[12], vt, temp;
    uint32_t vi, vj;
    uint8_t temp_buf[] = "INPUT MAC ? ";

    if (dev_config->network_common.mac[0] != 0x00 || dev_config->network_common.mac[1] != 0x08 || dev_config->network_common.mac[2] != 0xDC)
    {
        PRT_INFO("%s\r\n", temp_buf);
        //platform_uart_puts(temp_buf, strlen(temp_buf));
        uart_puts(UART_ID, temp_buf);
        
        while(1){
          vt = uart_getc(UART_ID);
          if(vt == 'S') {
            temp = 'R';
            uart_putc(UART_ID, temp);
            break;
          }
        }
        for(vi = 0; vi < 12; vi++){
          buf[vi] = uart_getc(UART_ID);
          uart_putc(UART_ID, buf[vi]);
        }
        platform_uart_puts("\r\n",2);
        for(vi = 0, vj = 0 ; vi < 6 ; vi++, vj += 2){
          dev_config->network_common.mac[vi] = get_hex(buf[vj], buf[vj+1]);
          mac[vi] = get_hex(buf[vj], buf[vj+1]);
        }

        ret = erase_flash_sector(FLASH_MAC_ADDR);
        PRT_INFO("erase_storage ret = %d\r\n", ret);

        ret = write_flash(FLASH_MAC_ADDR, mac, 6);
        PRT_INFO("write_flash ret = %d\r\n", ret);

        sprintf((char *)dev_config->device_option.device_alias, "%s-%02X%02X%02X%02X%02X%02X",
                                                       dev_config->device_common.device_name,
                                                       mac[0],
                                                       mac[1],
                                                       mac[2],
                                                       mac[3],
                                                       mac[4],
                                                       mac[5]);
        write_storage(STORAGE_CONFIG, 0, (uint8_t *)dev_config, sizeof(DevConfig));
        device_raw_reboot();
    }

}

uint8_t get_hex(uint8_t b0, uint8_t b1)
{
  uint8_t buf[2];
  buf[0]   = b0;
  buf[1]   = b1;
  buf[0]   = atonum(buf[0]);
  buf[0] <<= 4;
  buf[0]  += atonum(buf[1]);
  return(buf[0]);
}

char atonum(char ch)
{
  ch -= '0';
  if (ch > 9) ch -= 7;
  if (ch > 15) ch -= 0x20;
  return(ch);
}


