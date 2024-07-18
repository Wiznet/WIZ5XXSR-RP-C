
#include <string.h>
#include <stdlib.h>
#include "common.h"
#include "ConfigData.h"
#include "wizchip_conf.h"

#include "socket.h"
#include "seg.h"
#include "segcp.h"
#include "flashHandler.h"
#include "storageHandler.h"
#include "gpioHandler.h"
#include "deviceHandler.h"
#include "uartHandler.h"
#include "timerHandler.h"
#include "util.h"

#include "dns.h"
#include "dhcp.h"
#include "dhcp_cb.h"

#include "dns.h"

uint16_t get_firmware_from_network(uint8_t sock, uint8_t * buf);
uint16_t get_firmware_from_server(uint8_t sock, uint8_t * server_ip, uint8_t * buf);
int8_t process_dns_fw_server(uint8_t * domain_ip, uint8_t * buf);

void reset_fw_update_timer(void);
uint16_t get_any_port(void);

uint8_t enable_fw_update_timer = SEGCP_DISABLE;
volatile uint16_t fw_update_time = 0;
uint8_t flag_fw_update_timeout = SEGCP_DISABLE;
    
uint8_t enable_fw_from_network_timer = SEGCP_DISABLE;
volatile uint16_t fw_from_network_time = 0;
uint8_t flag_fw_from_network_timeout = SEGCP_DISABLE;

uint8_t flag_fw_from_server_failed = SEGCP_DISABLE;
static uint16_t any_port = 0;

uint8_t g_send_buf[DATA_BUF_SIZE];
uint8_t g_send_mqtt_buf[DATA_BUF_SIZE];
uint8_t g_recv_mqtt_buf[DATA_BUF_SIZE];
uint8_t g_recv_buf[DATA_BUF_SIZE];

uint8_t *g_rootca_buf;
uint8_t *g_clica_buf;
uint8_t *g_clica_buf;
uint8_t *g_pkey_buf;
uint8_t g_temp_buf[FLASH_SECTOR_SIZE];

void device_set_factory_default(void)
{
    set_DevConfig_to_factory_value();
    save_DevConfig_to_storage();
}


void device_socket_termination(void)
{
    uint8_t i;
    for(i = 0; i < _WIZCHIP_SOCK_NUM_; i++)
    {
        process_socket_termination(i, 100);
    }
}

void device_reboot(void)
{
    device_socket_termination();
    
    clear_data_transfer_bytecount(SEG_ALL);
    clear_data_transfer_megacount(SEG_ALL);
    
    device_raw_reboot();
    while(1);
}

void device_raw_reboot(void)
{
    NVIC_SystemReset();
    while(1);
}

uint8_t device_bank_update(void)
{
    struct __firmware_update *fwupdate = (struct __firmware_update *)&(get_DevConfig_pointer()->firmware_update);
    struct __serial_common *serial_common = (struct __serial_common *)&(get_DevConfig_pointer()->serial_common);

    uint8_t ret = DEVICE_FWUP_RET_PROGRESS;
    uint16_t recv_len = 0;
    uint16_t write_len = 0;
    static uint32_t write_fw_len;
    uint32_t f_addr;
    uint32_t remain_len = 0, buf_len = 0;
    int err;
    
    if((fwupdate->fwup_size == 0) || (fwupdate->fwup_size > FLASH_APP_BANK_SIZE))
    {
        if(serial_common->serial_debug_en)
            PRT_INFO(" > SEGCP:BU_UPDATE:FAILED - Invalid firmware size: %ld bytes (Firmware size must be within %d bytes)\r\n",
                    fwupdate->fwup_size,
                    FLASH_APP_BANK_SIZE);

        return DEVICE_FWUP_RET_FAILED;
    }

    if(serial_common->serial_debug_en)
        PRT_INFO(" > SEGCP:BU_UPDATE:NETWORK - Firmware size: [%ld] bytes\r\n", fwupdate->fwup_size);

    write_fw_len = 0;
    f_addr = FLASH_START_ADDR_BANK1_OFFSET;
    
    // init firmware update timer
    enable_fw_update_timer = SEGCP_ENABLE;
    close(SOCK_FWUPDATE);
    
    do
    {

#ifdef __USE_WATCHDOG__
        watchdog_update();
#endif
        recv_len = get_firmware_from_network(SOCK_FWUPDATE, g_recv_buf);
        if(recv_len > 0)
        {
            if (buf_len + recv_len < FLASH_SECTOR_SIZE)
            {
                memcpy(g_temp_buf + buf_len, g_recv_buf, recv_len);
                buf_len += recv_len;
            }
            else
            {
                //printf("f_addr = 0x%x\r\n", f_addr);
                remain_len = (buf_len + recv_len) - FLASH_SECTOR_SIZE;
                memcpy(g_temp_buf + buf_len, g_recv_buf, recv_len - remain_len);

                PRT_INFO("Write_addr = 0x%08X\r\n", f_addr);
                err = write_flash(f_addr, (uint8_t *)g_temp_buf, FLASH_SECTOR_SIZE);
                if (err < 0) {
                    PRT_ERR("write to flash fail(%d)!\n", err);
                    ret = DEVICE_FWUP_RET_FAILED;
                    break;
                }
                f_addr += FLASH_SECTOR_SIZE;

                memset(g_temp_buf, 0xFF, FLASH_SECTOR_SIZE);
                memcpy(g_temp_buf, g_recv_buf + (recv_len - remain_len), remain_len);
                buf_len = remain_len;                
            }
            write_fw_len += recv_len;
            
            fw_update_time = 0; // Reset fw update timeout counter
        }
        
        // Firmware update failed: Timeout occurred
        if(flag_fw_update_timeout == SEGCP_ENABLE)
        {
            if(serial_common->serial_debug_en)
                printf(" > SEGCP:BU_UPDATE:FAILED - Firmware update timeout\r\n");

            ret = DEVICE_FWUP_RET_FAILED;
            break;
        }
        
        // Firmware update failed: timeout occurred at get_firmware_from_network() function
        if(flag_fw_from_network_timeout == SEGCP_ENABLE)
        {
            if(serial_common->serial_debug_en)
                printf(" > SEGCP:BU_UPDATE:FAILED - Network download timeout\r\n");

            ret = DEVICE_FWUP_RET_FAILED;
            break;
        }
    } while(write_fw_len < fwupdate->fwup_size);

    PRT_INFO("write_fw_len = %ld, fwup_size = %ld bytes\r\n", write_fw_len, fwupdate->fwup_size);
    if(write_fw_len == fwupdate->fwup_size)
    {
        if (buf_len > 0)
        {
            PRT_INFO("buf_len > 0, Write_addr = 0x%08X\r\n", f_addr);
            delay_ms(10);
            err = write_flash(f_addr, (uint8_t *)g_temp_buf, FLASH_SECTOR_SIZE);
            if (err < 0) {
                PRT_ERR("write to flash fail(%d)!\n", err);
                ret = DEVICE_FWUP_RET_FAILED;
                return ret;
            }
        }   
    
        PRT_INFO(" > SEGCP:BU_UPDATE:SUCCESS\r\n");

        fwupdate->fwup_copy_flag = 1;
        ret = DEVICE_FWUP_RET_SUCCESS;
    }

    reset_fw_update_timer();

    return ret;
}

int device_bank_copy(void)
{
    struct __firmware_update *fwupdate = (struct __firmware_update *)&(get_DevConfig_pointer()->firmware_update);
    struct __serial_common *serial_common = (struct __serial_common *)&(get_DevConfig_pointer()->serial_common);

    uint32_t write_fw_len;
    uint32_t f_addr_src, f_addr_dst;
    int err;
    
    if((fwupdate->fwup_size == 0) || (fwupdate->fwup_size > FLASH_APP_BANK_SIZE))
    {
        if(serial_common->serial_debug_en)
            PRT_INFO(" > SEGCP:BU_COPY:FAILED - Invalid firmware size: %ld bytes (Firmware size must be within %d bytes)\r\n",
                    fwupdate->fwup_size,
                    FLASH_APP_BANK_SIZE);
    }

    if(serial_common->serial_debug_en)
        PRT_INFO(" > SEGCP:BU_COPY:NETWORK - Firmware size: [%ld] bytes\r\n", fwupdate->fwup_size);

    f_addr_src = FLASH_START_ADDR_BANK1;
    f_addr_dst = FLASH_START_ADDR_BANK0_OFFSET;
    
    for(write_fw_len = 0; write_fw_len < (fwupdate->fwup_size + FLASH_SECTOR_SIZE); write_fw_len+=FLASH_SECTOR_SIZE)
    {
        err = write_flash(f_addr_dst, (uint8_t *)f_addr_src, FLASH_SECTOR_SIZE);
        PRT_INFO(" > COPY:ADDR_RANGE - [0x%x > 0x%x]\r\n", f_addr_src, f_addr_dst);
        f_addr_dst+=FLASH_SECTOR_SIZE;
        f_addr_src+=FLASH_SECTOR_SIZE;
        if (err < 0) {
            PRT_ERR("write to flash fail(%d)!\n", err);
            return err;
        }
    }
    PRT_INFO("write_fw_len = %d, fwupdate->fwup_size = %d\r\n", write_fw_len, fwupdate->fwup_size);
    
    return 0;
}


int device_bank_check(uint8_t bank_num)
{
    uint32_t fw_data;
    
    if (bank_num == 0)
      fw_data = *(uint32_t *)(FLASH_START_ADDR_BANK0);
    else if (bank_num == 1)
      fw_data = *(uint32_t *)(FLASH_START_ADDR_BANK1);
    else
      return -1;
   PRT_INFO("fw_data = 0x%08X\r\n", fw_data);

   if ((fw_data == 0xFFFFFFFF) || (fw_data == 0x00000000))
   {
      return -1;
    }
   return 0;
}
void reset_fw_update_timer(void)
{
    // Enables
    enable_fw_update_timer = SEGCP_DISABLE;
    enable_fw_from_network_timer = SEGCP_DISABLE;
    
    // Counters
    fw_update_time = 0;
    fw_from_network_time = 0;
    
    // Flags
    flag_fw_update_timeout = SEGCP_DISABLE;
    flag_fw_from_network_timeout = SEGCP_DISABLE;
}

uint16_t get_any_port(void)
{
    if(any_port)
    {
        if(any_port < 0xffff) 	any_port++;
        else					any_port = 0;
    }
    
    if(any_port == 0) any_port = 50001;
    
    return any_port;
}

uint16_t get_firmware_from_network(uint8_t sock, uint8_t * buf)
{
    struct __firmware_update *fwupdate = (struct __firmware_update *)&(get_DevConfig_pointer()->firmware_update);
    uint8_t len_buf[2] = {0, };
    uint16_t len = 0;
    uint8_t state = getSn_SR(sock);
    
    static uint32_t recv_fwsize;

    switch(state)
    {
        case SOCK_INIT:
            //listen(sock);
            break;
        
        case SOCK_LISTEN:
            break;
        
        case SOCK_ESTABLISHED:
            if(getSn_IR(sock) & Sn_IR_CON)
            {
                // init network firmware update timer
                enable_fw_from_network_timer = SEGCP_ENABLE;
                setSn_IR(sock, Sn_IR_CON);
            }
            
            // Timeout occurred
            if(flag_fw_from_network_timeout == SEGCP_ENABLE)
            {
#ifdef _FWUP_DEBUG_
                printf(" > SEGCP:UPDATE:NET_TIMEOUT\r\n");
#endif
                //disconnect(sock);
                close(sock);
                return 0;
            }
            
            // DATA_BUF_SIZE
            if((len = getSn_RX_RSR(sock)) > 0)
            {
                if(len > DATA_BUF_SIZE) len = DATA_BUF_SIZE;
                if(recv_fwsize + len > fwupdate->fwup_size) len = fwupdate->fwup_size - recv_fwsize; // remain
                
                len = recv(sock, buf, len);
                recv_fwsize += len;
#ifdef _FWUP_DEBUG_
                printf(" > SEGCP:UPDATE:RECV_LEN - %d bytes | [%d] bytes\r\n", len, recv_fwsize);
#endif
                // Send ACK - receviced length - to configuration tool
                len_buf[0] = (uint8_t)((0xff00 & len) >> 8); // endian-independent code: Datatype translation, byte order regardless
                len_buf[1] = (uint8_t)(0x00ff & len);
                
                send(sock, len_buf, 2);
                
                fw_from_network_time = 0;
                
                if(recv_fwsize >= fwupdate->fwup_size)
                {
#ifdef _FWUP_DEBUG_
                    printf(" > SEGCP:UPDATE:NETWORK - UPDATE END | [%d] bytes\r\n", recv_fwsize);
#endif
                    // timer disable: network timeout
                    reset_fw_update_timer();
                    
                    // socket close
                    disconnect(sock);
                }
            }
            break;
            
        case SOCK_CLOSE_WAIT:
            disconnect(sock);
            break;
        
        case SOCK_FIN_WAIT:
        case SOCK_CLOSED:
            if(socket(sock, Sn_MR_TCP, DEVICE_FWUP_PORT, SF_TCP_NODELAY) == sock)
            {
                recv_fwsize = 0;
                listen(sock);
                
#ifdef _FWUP_DEBUG_
                printf(" > SEGCP:UPDATE:SOCKOPEN\r\n");
#endif
            }
            break;
            
        default:     
            break;
    }
    
    return len;
}


void display_Dev_Info_header(void)
{
    DevConfig *dev_config = get_DevConfig_pointer();

    printf("\r\n");
    PRT_INFO("%s\r\n", STR_BAR);

    PRT_INFO(" %s \r\n", DEVICE_ID_DEFAULT); //PRT_INFO(" %s \r\n", dev_config->device_common.device_name);
    PRT_INFO(" >> WIZnet Device Server\r\n");

    PRT_INFO(" >> Firmware version: %d.%d.%d %s\r\n", dev_config->device_common.fw_ver[0],
                                                    dev_config->device_common.fw_ver[1],
                                                    dev_config->device_common.fw_ver[2],
                                                    STR_VERSION_STATUS);
    PRT_INFO("%s\r\n", STR_BAR);
}

// Only for Serial 1-channel device
void display_Dev_Info_main(void)
{
    uint8_t serial_mode;
    DevConfig *dev_config = get_DevConfig_pointer();

    PRT_INFO(" - Device type: %s\r\n", dev_config->device_common.device_name);
    PRT_INFO(" - Device name: %s\r\n", dev_config->device_option.device_alias);
    PRT_INFO(" - Device group: %s\r\n", dev_config->device_option.device_group);

    PRT_INFO(" - Device mode: %s\r\n", str_working[dev_config->network_connection.working_mode]);

    PRT_INFO(" - Data channel: [UART Port %d] %s mode\r\n", ((dev_config->serial_option.uart_interface == UART_IF_TTL) ||
                                                      (dev_config->serial_option.uart_interface == UART_IF_RS232))?0:1,
                                                       uart_if_table[dev_config->serial_option.uart_interface]);
    PRT_INFO(" - Network settings: \r\n");
    PRT_INFO("\t- Obtaining IP settings: [%s]\r\n", (dev_config->network_option.dhcp_use == 1)?"Automatic - DHCP":"Static");
    PRT_INFO("\t- TCP/UDP ports\r\n");
    PRT_INFO("\t   + S2E data port: [%d]\r\n", dev_config->network_connection.local_port);
    PRT_INFO("\t   + TCP/UDP setting port: [%d]\r\n", DEVICE_SEGCP_PORT);
    PRT_INFO("\t   + Firmware update port: [%d]\r\n", DEVICE_FWUP_PORT);
    PRT_INFO("\t- TCP Retransmission retry: [%d]\r\n", getRCR());

    PRT_INFO(" - Search ID code: \r\n");
    PRT_INFO("\t- %s: [%s]\r\n", (dev_config->config_common.pw_search != 0)?"Enabled":"Disabled", (dev_config->config_common.pw_search != 0)?dev_config->config_common.pw_search:"None");

    PRT_INFO(" - Ethernet connection password: \r\n");
    PRT_INFO("\t- %s %s\r\n", (dev_config->tcp_option.pw_connect_en == 1)?"Enabled":"Disabled", "(TCP server / mixed mode only)");

    PRT_INFO(" - Connection timer settings: \r\n");
    PRT_INFO("\t- Inactivity timer: ");
    if(dev_config->tcp_option.inactivity) PRT_INFO("[%d] (sec)\r\n", dev_config->tcp_option.inactivity);
        else PRT_INFO("%s\r\n", STR_DISABLED);
    PRT_INFO("\t- Reconnect interval: ");
    if(dev_config->tcp_option.reconnection) PRT_INFO("[%d] (msec)\r\n", dev_config->tcp_option.reconnection);
        else PRT_INFO("%s\r\n", STR_DISABLED);

    PRT_INFO(" - Serial settings: \r\n");

      //todo:
      PRT_INFO("\t- Communication Protocol: ");
      serial_mode = get_serial_communation_protocol();
      if(serial_mode)
          PRT_INFO("[%s]\r\n", (serial_mode == SEG_SERIAL_MODBUS_RTU) ? STR_MODBUS_RTU:STR_MODBUS_ASCII);
      else
          PRT_INFO("[%s]\r\n", STR_DISABLED);

      PRT_INFO("\t- Data %s port:\r\n", STR_UART);
      PRT_INFO("\t   + UART IF: [%s]\r\n", uart_if_table[dev_config->serial_option.uart_interface]);
      printf("\t   + %ld-", baud_table[dev_config->serial_option.baud_rate]);
      printf("%d-", word_len_table[dev_config->serial_option.data_bits]);
      printf("%s-", parity_table[dev_config->serial_option.parity]);
      printf("%d / ", stop_bit_table[dev_config->serial_option.stop_bits]);
      if((dev_config->serial_option.uart_interface == UART_IF_TTL) || (dev_config->serial_option.uart_interface == UART_IF_RS232))
      {
          printf("Flow control: %s", flow_ctrl_table[dev_config->serial_option.flow_control]);
      }
      else if((dev_config->serial_option.uart_interface == UART_IF_RS422) || (dev_config->serial_option.uart_interface == UART_IF_RS485))
      {
          if((dev_config->serial_option.flow_control == flow_rtsonly) || (dev_config->serial_option.flow_control == flow_reverserts))
          {
              printf("Flow control: %s", flow_ctrl_table[dev_config->serial_option.flow_control]);
          }
          else
          {
              printf("Flow control: %s", flow_ctrl_table[0]); // RS-422/485; flow control - NONE only
          }
      }
      PRT_INFO("\r\n");

      PRT_INFO("\t- Debug %s port:\r\n", STR_UART);
      PRT_INFO("\t   + %s / %s %s\r\n", "115200-8-N-1", "NONE", "(fixed)");

      PRT_INFO(" - Serial data packing options:\r\n");
      PRT_INFO("\t- Time: ");
      if(dev_config->serial_data_packing.packing_time) PRT_INFO("[%d] (msec)\r\n", dev_config->serial_data_packing.packing_time);
      else PRT_INFO("%s\r\n", STR_DISABLED);
      PRT_INFO("\t- Size: ");
      if(dev_config->serial_data_packing.packing_size) PRT_INFO("[%d] (bytes)\r\n", dev_config->serial_data_packing.packing_size);
      else PRT_INFO("%s\r\n", STR_DISABLED);
      PRT_INFO("\t- Char: ");
      if(dev_config->serial_data_packing.packing_delimiter_length == 1) PRT_INFO("[%.2X] (hex only)\r\n", dev_config->serial_data_packing.packing_delimiter[0]);
      else PRT_INFO("%s\r\n", STR_DISABLED);

      PRT_INFO(" - Serial command mode switch code:\r\n");
      PRT_INFO("\t- %s\r\n", (dev_config->serial_command.serial_command == 1)?STR_ENABLED:STR_DISABLED);
      PRT_INFO("\t- [%.2X][%.2X][%.2X] (Hex only)\r\n",
              dev_config->serial_command.serial_trigger[0],
              dev_config->serial_command.serial_trigger[1],
              dev_config->serial_command.serial_trigger[2]);

#ifdef __USE_USERS_GPIO__ // not used
    PRT_INFO(" - Hardware information: User I/O pins\r\n");
    PRT_INFO("\t- UserIO A: [%s] - %s / %s\r\n", "%s", USER_IO_TYPE_STR[get_user_io_type(USER_IO_SEL[0])], USER_IO_DIR_STR[get_user_io_direction(USER_IO_SEL[0])], USER_IO_PIN_STR[0]);
    PRT_INFO("\t- UserIO B: [%s] - %s / %s\r\n", "%s", USER_IO_TYPE_STR[get_user_io_type(USER_IO_SEL[1])], USER_IO_DIR_STR[get_user_io_direction(USER_IO_SEL[1])], USER_IO_PIN_STR[1]);
#endif

    PRT_INFO("%s\r\n", STR_BAR);
}


void display_Dev_Info_dhcp(void)
{
    DevConfig *dev_config = get_DevConfig_pointer();

    if(dev_config->network_option.dhcp_use)
    {
        if(flag_process_dhcp_success == ON) PRT_INFO(" # DHCP IP Leased time : %ld seconds\r\n", getDHCPLeasetime());
        else PRT_INFO(" # DHCP Failed\r\n");
    }
}


void display_Dev_Info_dns(void)
{
    DevConfig *dev_config = get_DevConfig_pointer();

    if(dev_config->network_connection.dns_use)
    {
        if(flag_process_dns_success == ON)
        {
            PRT_INFO(" # DNS: %s => %d.%d.%d.%d : %d\r\n", dev_config->network_connection.dns_domain_name,
                                                            dev_config->network_connection.remote_ip[0],
                                                            dev_config->network_connection.remote_ip[1],
                                                            dev_config->network_connection.remote_ip[2],
                                                            dev_config->network_connection.remote_ip[3],
                                                            dev_config->network_connection.remote_port);
        }
        else
            PRT_INFO(" # DNS Failed\r\n");
    }
    PRT_INFO("\r\n");
}

int8_t process_dhcp(void)
{
    uint8_t ret = 0;
    uint8_t dhcp_retry = 0;

    PRT_DHCP(" - DHCP Client running\r\n");
    
    DHCP_init(SOCK_DHCP, g_send_buf);
    reg_dhcp_cbfunc(w5x00_dhcp_assign, w5x00_dhcp_assign, w5x00_dhcp_conflict);
    while(1)
    {
        ret = DHCP_run();
        sleep_ms(1000);

        if(ret == DHCP_IP_LEASED)
        {
            PRT_DHCP(" - DHCP Success\r\n");
            break;
        }
        else if(ret == DHCP_FAILED)
        {
            dhcp_retry++;
            if(dhcp_retry <= 3) PRT_DHCP(" - DHCP Timeout occurred and retry [%d]\r\n", dhcp_retry);
        }

        if(dhcp_retry > 3)
        {
#ifndef __USE_DHCP_INFINITE_LOOP__
            PRT_DHCP(" - DHCP Failed\r\n\r\n");
            DHCP_stop();
            break;
#else // If DHCP allocation failed, process_dhcp() function will try to DHCP steps again.
            PRT_DHCP(" - DHCP Failed, Try again...\r\n\r\n");
            DHCP_init(SOCK_DHCP, g_send_buf);
            dhcp_retry = 0;
#endif
        }

        do_segcp(); // Process the requests of configuration tool during the DHCP client run.
    }

    set_device_status_all(ST_OPEN);

    return ret;
}

#if 0
time_t get_ntp_time(uint8_t * NTPServer, uint32_t timeout)
{
    int8_t ret = 0;
    uint8_t ntp_server_ip[4] = {0, };

    uint32_t tickStart;
    time_t currentTime;

#ifdef _MAIN_DEBUG_
    printf(" - NTP: DNS client running\r\n");
#endif

    ret = get_ipaddr_from_dns(NTPServer, ntp_server_ip, (DNS_WAIT_TIME * 300));
    if(ret <= 0) return 0; // DNS failed

    SNTP_init(SOCK_NTP, ntp_server_ip, 40, g_send_buf);
    tickStart = HAL_GetTick();
    do{
        ret = SNTP_run(&currentTime);
        if(ret == 1)
        {
            return currentTime;
        }

        // Process the requests of configuration tool during the NTP client run
        do_segcp();
    }while((HAL_GetTick() - tickStart) < timeout);

#ifdef _MAIN_DEBUG_
    printf(" - NTP: Get time failed\r\n");
#endif

    return 0;
}
#endif

#ifdef __USE_WATCHDOG__
void wdt_reset(void)
{
    //Reload the Watchdog time counter
    //__HAL_IWDG_RELOAD_COUNTER(&hiwdg);
}
#endif

