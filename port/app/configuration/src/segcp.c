#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <stdlib.h>

#include "port_common.h"
#include "common.h"
#include "WIZ5XXSR-RP_board.h"
#include "w5x00_spi.h"
#include "socket.h"
#include "ConfigData.h"
#include "storageHandler.h"
#include "deviceHandler.h"
#include "SSLInterface.h"

#include "seg.h"
#include "segcp.h"
#include "util.h"
#include "uartHandler.h"
#include "gpioHandler.h"
#include "timerHandler.h"

/* Private define ------------------------------------------------------------*/

#define END_CERT "-----END CERTIFICATE-----"
#define END_PKEY "-----END RSA PRIVATE KEY-----"

// Ring Buffer declaration
BUFFER_DECLARATION(data0_rx);

/* Private functions ---------------------------------------------------------*/
uint16_t uart_get_commandline(uint8_t* buf, uint16_t maxSize);

/* Private variables ---------------------------------------------------------*/
static uint8_t gSEGCPREQ[CONFIG_BUF_SIZE];
static uint8_t gSEGCPREP[CONFIG_BUF_SIZE];

static uint8_t SEGCP_UART = SEG_DATA0_UART; // default: SEG_DATA0_UART

uint8_t * strDEVSTATUS[]  = {"BOOT", "OPEN", "CONNECT", "UPGRADE", "ATMODE", "UDP", 0};

// [K!]: Hidden command, Erase the MAC address and configuration data
#if (DEVICE_UART_CNT > 1)
uint8_t * tbSEGCPCMD[] = {"MC", "VR", "MN", "IM", "OP", "DD", "CP", "PO", "DG", "KA",
                          "KI", "KE", "RI", "LI", "SM", "GW", "DS", "PI", "PP", "DX",
                          "DP", "DI", "DW", "DH", "LP", "RP", "RH", "BR", "DB", "PR",
                          "SB", "FL", "IT", "PT", "PS", "PD", "TE", "SS", "NP", "SP",
                          "LG", "ER", "FW", "MA", "PW", "SV", "EX", "RT", "UN", "ST",
                          "FR", "EC", "K!", "UE", "GA", "GB", "GC", "GD", "CA", "CB",
                          "CC", "CD", "SC", "S0", "S1", "RX", "FS", "FC", "FP", "FD",
                          "FH", "UI", "QS", "QO", "QH", "QP", "QL", "RV", "RA", "RS",
                          "RE", "RR", "EN", "EI", "EB", "ED", "EP", "ES", "EF", "E0",
                          "E1", "NT", "NS", "ND", "CR", "NR", "AB", "TR", "BU", "LF",
                          "AE", "AP", "MB", "SE", "CE", "CT", "N0", "N1", "N2", "AL",
                          "GR", "AM", "QF", "MM", "CS", "CM", "C0", "C1", "C2", "C3",
                          0};
#else

uint8_t * tbSEGCPCMD[] = {"MC", "VR", "MN", "IM", "OP", "CP", "DG", "KA", "KI", "KE",
                          "RI", "LI", "SM", "GW", "DS", "DH", "LP", "RP", "RH", "BR",
                          "DB", "PR", "SB", "FL", "PO", "IT", "PT", "PS", "PD", "TE",
                          "SS", "NP", "SP", "MA", "PW", "SV", "EX", "RT", "UN", "ST",
                          "FR", "EC", "GA", "GB", "GC", "GD", "CA", "CB", "CC", "CD",
                          "SC", "S0", "S1", "RX", "UI", "TR", "QU", "QP", "QC", "QK",
                          "PU", "U0", "U1", "U2", "QO", "RC", "CE", "OC", "LC", "PK",
                          "UF", "FW", "SO", 0};
 
#endif
uint8_t * tbSEGCPERR[] = {"ERNULL", "ERNOTAVAIL", "ERNOPARAM", "ERIGNORED", "ERNOCOMMAND", "ERINVALIDPARAM", "ERNOPRIVILEGE"};

uint8_t gSEGCPPRIVILEGE = SEGCP_PRIVILEGE_CLR;

// Keep-alive timer values for TCP unicast search function
uint8_t enable_configtool_keepalive_timer = SEGCP_DISABLE;
volatile uint16_t configtool_keepalive_time = 0;
uint8_t flag_send_configtool_keepalive = SEGCP_DISABLE;

extern uint8_t *g_rootca_buf;
extern uint8_t *g_clica_buf;
extern uint8_t *g_pkey_buf;
extern uint8_t g_temp_buf[];

void do_segcp(void)
{
    DevConfig *dev_config = get_DevConfig_pointer();
    uint16_t segcp_ret = 0;

    // Process the serial AT command mode
    if(opmode == DEVICE_AT_MODE)
    {
        segcp_ret = proc_SEGCP_uart(gSEGCPREQ, gSEGCPREP);
        if(segcp_ret & SEGCP_RET_ERR)
            if(dev_config->serial_common.serial_debug_en) PRT_ERR(" > SEGCP:ERROR:%04X\r\n", segcp_ret);
    }
    
    segcp_ret |= proc_SEGCP_udp(gSEGCPREQ, gSEGCPREP);
    segcp_ret |= proc_SEGCP_tcp(gSEGCPREQ, gSEGCPREP);

    segcp_ret_handler(segcp_ret);

}

void segcp_ret_handler(uint16_t segcp_ret)
{
    DevConfig *dev_config = get_DevConfig_pointer();

    uint8_t ret = 0;
    uint8_t i;
    teDEVSTATUS status_bak;

    if(segcp_ret && ((segcp_ret & SEGCP_RET_ERR) != SEGCP_RET_ERR)) // Command parsing success
    {
        if(segcp_ret & SEGCP_RET_SWITCH)
        {
            if(opmode == DEVICE_GW_MODE)
                init_trigger_modeswitch(DEVICE_AT_MODE); // DEVICE_GW_MODE -> DEVICE_AT_MODE
            else
                init_trigger_modeswitch(DEVICE_GW_MODE); // DEVICE_AT_MODE -> DEVICE_GW_MODE
        }
    
        if(segcp_ret & SEGCP_RET_FACTORY)
            device_set_factory_default();
        else if(segcp_ret & SEGCP_RET_SAVE)
        {
            PRT_SEGCP("segcp_ret & SEGCP_RET_SAVE\r\n");
            save_DevConfig_to_storage();
        }
        else if(segcp_ret & SEGCP_RET_ERASE_EEPROM)
        {
            PRT_SEGCP("segcp_ret & SEGCP_RET_ERASE_EEPROM\r\n");
            erase_storage(STORAGE_MAC);
            erase_storage(STORAGE_CONFIG);        
        }
        if(segcp_ret & SEGCP_RET_FWUP)
        {
            status_bak = (teDEVSTATUS)get_device_status();
            set_device_status(ST_UPGRADE);

            if((segcp_ret & SEGCP_RET_FWUP_BANK) == segcp_ret)
            {
                ret = device_bank_update(); // BANK Firmware update by Configuration tool
            }
            else
            {
                ret = DEVICE_FWUP_RET_FAILED;
            }
            
            if(ret == DEVICE_FWUP_RET_SUCCESS)
            {
                status_bak = (teDEVSTATUS)get_device_status();
                set_device_status(ST_OPEN);
                

                save_DevConfig_to_storage();
                
                device_reboot();
            }
            else
            {
                // Clear the firmware update flags and size
                dev_config->firmware_update.fwup_size = 0;
                dev_config->firmware_update.fwup_flag = SEGCP_DISABLE;
                dev_config->firmware_update.fwup_server_flag = SEGCP_DISABLE;
                set_device_status(status_bak);
                close(SOCK_FWUPDATE);

                if(dev_config->serial_common.serial_debug_en) printf(" > SEGCP:UPDATE:FAILED\r\n");
            }
        }

        if (segcp_ret & SEGCP_RET_REBOOT)
        {
            PRT_SEGCP("segcp_ret & SEGCP_RET_REBOOT\r\n");
            if(opmode == DEVICE_AT_MODE) 
                if(dev_config->serial_common.serial_debug_en) platform_uart_puts((uint8_t *)"REBOOT\r\n", 8);
            device_reboot();
        }
    }
}

void set_segcp_uart(uint8_t uartNum)
{
    SEGCP_UART = uartNum;
}


uint8_t get_segcp_uart(void)
{
    return SEGCP_UART;
}


uint8_t parse_SEGCP(uint8_t * pmsg, uint8_t * param)
{
    uint8_t** pcmd;
    uint8_t cmdnum = 0;
    uint8_t i;
    uint32_t len;
    
    *param = 0;

    for(pcmd = tbSEGCPCMD; *pcmd != 0; pcmd++)
    {
        if(!strncmp((char *)pmsg, *pcmd, strlen(*pcmd))) break;
    }

    if(*pcmd == 0) 
    {
        PRT_SEGCP("[UNKNOWN]:%s\r\n", pmsg);
        return SEGCP_UNKNOWN;
    }
    
    cmdnum = (uint8_t)(pcmd - tbSEGCPCMD);
    
    if(cmdnum == (uint8_t)SEGCP_MA) 
    {
        if((pmsg[8] == '\r') && (pmsg[9] == '\n'))
            memcpy(param, (uint8_t*)&pmsg[2], 6);
        else
            return SEGCP_UNKNOWN;
    }
    else if(cmdnum == (uint8_t)SEGCP_PW)
    {
        for(i = 0; pmsg[2+i] != '\r'; i++)
            param[i] = pmsg[2+i];
        
        if(pmsg[2+i+1] == '\n')
        {
            param[i] = 0; param[i+1] = 0;
        }
        else
            return SEGCP_UNKNOWN;
    }

    else if((cmdnum == (uint8_t)SEGCP_OC) || (cmdnum == (uint8_t)SEGCP_LC) || \
            (cmdnum == (uint8_t)SEGCP_PK)) //|| (cmdnum == (uint8_t)SEGCP_UP))
    {
        PRT_SEGCP("cmd == %s\r\n", tbSEGCPCMD[cmdnum]);
        len = strlen(pmsg);
        
        if (*(pmsg + len) == NULL)
        {
            if ((*(pmsg + len + 1) == '\n') || (*(pmsg + len + 1) == NULL))
            {
                *(pmsg + len) = '\r';
                *(pmsg + len + 1) = '\n';
            }
            else
                *(pmsg + len) = '\n';
        }
        *param = 1;
        return cmdnum;
    }
    else
    {
        strcpy(param, (uint8_t*)&pmsg[2]);
    }

#if 0
    PRT_SEGCP("[%d:%s:", cmdnum, *pcmd);

    if(cmdnum == SEGCP_MA)
    {
        for(i = 0; i < 6; i++) PRT_SEGCP("%.2x", param[i]);
    }
    else
    {
        PRT_SEGCP("%s",param);
    }
    PRT_SEGCP("]\r\n");
#endif
    
    return cmdnum;
}

uint16_t proc_SEGCP(uint8_t* segcp_req, uint8_t* segcp_rep)
{
    DevConfig *dev_config = get_DevConfig_pointer();
    
    //uint8_t  i = 0;
    uint16_t ret = 0;
    int ret_2;
    uint8_t  cmdnum = 0;
    uint8_t* treq;
    
    char * trep = segcp_rep;
    uint16_t param_len = 0;

#ifdef __USE_USERS_GPIO__
    uint8_t  io_num = 0;
    uint8_t  io_type = 0;
    uint8_t  io_dir = 0;
#endif

    uint8_t  tmp_byte = 0;
    uint16_t tmp_int = 0;
    uint32_t tmp_long = 0;

    uint16_t tmp_port;
    uint8_t tmp_ip[4];
    uint8_t param[SEGCP_PARAM_MAX*2];

    uint32_t len;
    uint8_t *tmp_ptr;
    
    PRT_SEGCP("SEGCP_REQ : %s\r\n",segcp_req);

    memset(trep, 0, sizeof(trep));
    treq = strtok(segcp_req, SEGCP_DELIMETER);
    
    while(treq)
    {
        //PRT_SEGCP("SEGCP_REQ_TOK : %s\r\n",treq);
        if((cmdnum = parse_SEGCP(treq, param)) != SEGCP_UNKNOWN)
        {
            param_len = strlen((const char *)param);
            
            if(*param == 0)
            {
                memcpy(trep, tbSEGCPCMD[cmdnum], SEGCP_CMD_MAX);
                trep+=SEGCP_CMD_MAX;
                
                switch((teSEGCPCMDNUM)cmdnum)
                {
                    case SEGCP_MC: sprintf(trep,"%02X:%02X:%02X:%02X:%02X:%02X", 
                                            dev_config->network_common.mac[0], dev_config->network_common.mac[1], dev_config->network_common.mac[2],
                                            dev_config->network_common.mac[3], dev_config->network_common.mac[4], dev_config->network_common.mac[5]);
                        break;
                    case SEGCP_VR: 
                        if(strcmp(STR_VERSION_STATUS, "Stable") == 0)
                        {
                            sprintf(trep,"%d.%d.%d", dev_config->device_common.fw_ver[0],
                                                     dev_config->device_common.fw_ver[1],
                                                     dev_config->device_common.fw_ver[2]); // Standard stable version
                        }
                        else if(strcmp(STR_VERSION_STATUS, "Develop") == 0)
                        {
                            // Develop version 
                            sprintf(trep,"%d.%d.%ddev", dev_config->device_common.fw_ver[0],
                                                        dev_config->device_common.fw_ver[1],
                                                        dev_config->device_common.fw_ver[2]);
                        }
                        else
                        {
                            // Custom version
                            sprintf(trep,"%d.%d.%d%s", dev_config->device_common.fw_ver[0],
                                                       dev_config->device_common.fw_ver[1],
                                                       dev_config->device_common.fw_ver[2], STR_VERSION_STATUS);
                        }
                        break;
                    case SEGCP_MN: sprintf(trep,"%s", dev_config->device_common.device_name);
                        break;
                    case SEGCP_IM: sprintf(trep,"%d", dev_config->network_option.dhcp_use);	// 0:STATIC, 1:DHCP (PPPoE X)
                        break;
                    case SEGCP_OP: sprintf(trep,"%d", dev_config->network_connection.working_mode); // opmode
                        break;
                    case SEGCP_CP: sprintf(trep,"%d", dev_config->tcp_option.pw_connect_en);
                        break;
                    case SEGCP_DG: sprintf(trep,"%d", dev_config->serial_common.serial_debug_en);
                        PRT_SEGCP("SEGCP_DG = %s\r\n", trep);
                        break;
                    case SEGCP_KA: sprintf(trep,"%d", dev_config->tcp_option.keepalive_en);
                        break;
                    case SEGCP_KI: sprintf(trep,"%d", dev_config->tcp_option.keepalive_wait_time);
                        break;
                    case SEGCP_KE: sprintf(trep,"%d", dev_config->tcp_option.keepalive_retry_time);
                        break;
                    case SEGCP_RI: sprintf(trep,"%d", dev_config->tcp_option.reconnection);
                        break;
                    case SEGCP_LI:
                        sprintf(trep,"%d.%d.%d.%d", dev_config->network_common.local_ip[0], dev_config->network_common.local_ip[1],
                                                    dev_config->network_common.local_ip[2], dev_config->network_common.local_ip[3]);
                        break;
                    case SEGCP_SM: 
                        sprintf(trep,"%d.%d.%d.%d", dev_config->network_common.subnet[0], dev_config->network_common.subnet[1],
                                                    dev_config->network_common.subnet[2], dev_config->network_common.subnet[3]);
                        break;
                    case SEGCP_GW: 
                        sprintf(trep,"%d.%d.%d.%d", dev_config->network_common.gateway[0], dev_config->network_common.gateway[1],
                                                    dev_config->network_common.gateway[2], dev_config->network_common.gateway[3]);
                        break;
                    case SEGCP_DS:
                        sprintf(trep,"%d.%d.%d.%d", dev_config->network_option.dns_server_ip[0], dev_config->network_option.dns_server_ip[1],
                                                    dev_config->network_option.dns_server_ip[2], dev_config->network_option.dns_server_ip[3]);
                        break;

                    case SEGCP_DH:
                        if(dev_config->device_common.device_name[0] == 0)
                            sprintf(trep,"%c",SEGCP_NULL);
                        else
                            //sprintf(trep, "%s-%02X%02X%02X", dev_config->module_name, dev_config->network_info_common.mac[3], dev_config->network_info_common.mac[4], dev_config->network_info_common.mac[5]);
                            sprintf(trep, "%s-%02X%02X%02X%02X%02X%02X", dev_config->device_common.device_name,
                                            dev_config->network_common.mac[0],
                                            dev_config->network_common.mac[1],
                                            dev_config->network_common.mac[2],
                                            dev_config->network_common.mac[3],
                                            dev_config->network_common.mac[4],
                                            dev_config->network_common.mac[5]);
                        break;
                    case SEGCP_LP: sprintf(trep, "%d", dev_config->network_connection.local_port);
                        break;
                    case SEGCP_RP: sprintf(trep, "%d", dev_config->network_connection.remote_port);
                        break;
                    case SEGCP_RH: 
                        if(dev_config->network_connection.dns_use == SEGCP_DISABLE)
                        {
                            sprintf(trep, "%d.%d.%d.%d", dev_config->network_connection.remote_ip[0],
                                                         dev_config->network_connection.remote_ip[1],
                                                         dev_config->network_connection.remote_ip[2],
                                                         dev_config->network_connection.remote_ip[3]);
                        }
                        else
                        {
                            if(dev_config->network_connection.dns_domain_name[0] == 0)
                                sprintf(trep, "%c", SEGCP_NULL);
                            else
                                sprintf(trep, "%s", dev_config->network_connection.dns_domain_name);
                        }
                        break;
                    case SEGCP_BR: sprintf(trep, "%d", dev_config->serial_option.baud_rate);
                        break;
                    case SEGCP_DB: sprintf(trep, "%d", dev_config->serial_option.data_bits);
                        break;
                    case SEGCP_PR: sprintf(trep, "%d", dev_config->serial_option.parity);
                        break;
                    case SEGCP_SB: sprintf(trep, "%d", dev_config->serial_option.stop_bits);
                        break;
                    case SEGCP_FL: sprintf(trep, "%d", dev_config->serial_option.flow_control);
                        break;
                    case SEGCP_PO: sprintf(trep, "%d", dev_config->serial_option.protocol);
                        break;
                    case SEGCP_IT: sprintf(trep, "%d", dev_config->tcp_option.inactivity);
                        break;
                    case SEGCP_PT: sprintf(trep, "%d", dev_config->serial_data_packing.packing_time);
                        break;
                    case SEGCP_PS: sprintf(trep, "%d", dev_config->serial_data_packing.packing_size);
                        break;
                    case SEGCP_PD: sprintf(trep, "%02X", dev_config->serial_data_packing.packing_delimiter[0]);
                        break;
                    case SEGCP_TE: sprintf(trep, "%d", dev_config->serial_command.serial_command);
                        break;
                    case SEGCP_SS: sprintf(trep, "%02X%02X%02X", dev_config->serial_command.serial_trigger[0],
                                                                 dev_config->serial_command.serial_trigger[1],
                                                                 dev_config->serial_command.serial_trigger[2]);
                        break;
                    case SEGCP_NP:
                        if(dev_config->tcp_option.pw_connect[0] == 0) sprintf(trep,"%c",SEGCP_NULL);
                        else sprintf(trep, "%s", dev_config->tcp_option.pw_connect);
                        break;
                    case SEGCP_SP:
                        if(dev_config->config_common.pw_search[0] == 0) sprintf(trep,"%c",SEGCP_NULL);
                        else sprintf(trep, "%s", dev_config->config_common.pw_search);
                        break;
                    case SEGCP_MA:
                    case SEGCP_PW: ret |= SEGCP_RET_ERR_NOTAVAIL;
                        break;
#ifdef __USE_USERS_GPIO__
                    // GET GPIOs Status / Value
                    case SEGCP_GA:
                    case SEGCP_GB:
                        io_num = (teSEGCPCMDNUM)cmdnum - SEGCP_GA;
                        if(get_user_io_val(USER_IO_SEL[io_num], &tmp_int) != 0) sprintf(trep, "%d", tmp_int);
                        else ret |= SEGCP_RET_ERR_NOTAVAIL;
                        break;
                    
                    // GET GPIOs settings; Type and Direction
                    case SEGCP_CA:
                    case SEGCP_CB:
                        io_num = (teSEGCPCMDNUM)cmdnum - SEGCP_CA;
                        io_type = get_user_io_type(USER_IO_SEL[io_num]);
                        io_dir = get_user_io_direction(USER_IO_SEL[io_num]);
                        sprintf(trep, "%d", (((io_type & 0x01) << 1) | io_dir));
                        break;
#endif

                    // GET Status pin's setting and status
                    case SEGCP_SC: // mode select
                        sprintf(trep, "%d%d", dev_config->serial_option.dtr_en, dev_config->serial_option.dsr_en);
                        break;
                    case SEGCP_S0:
                        sprintf(trep, "%d", get_connection_status_io(STATUS_PHYLINK_PIN)); // STATUS_PHYLINK_PIN (in) == DTR_PIN (out)
                        break;
                    case SEGCP_S1:
                        sprintf(trep, "%d", get_connection_status_io(STATUS_TCPCONNECT_PIN)); // STATUS_TCPCONNECT_PIN (in) == DSR_PIN (in)
                        break;
                    case SEGCP_RX:
                        uart_rx_flush();
                        sprintf(trep, "%s", "FLUSH");
                        break;
                    case SEGCP_SV:
                        if(gSEGCPPRIVILEGE & (SEGCP_PRIVILEGE_SET|SEGCP_PRIVILEGE_WRITE)) ret |= SEGCP_RET_SAVE;
                        else ret |= SEGCP_RET_ERR_NOPRIVILEGE;
                        break;
                    case SEGCP_EX:
                        if(gSEGCPPRIVILEGE & (SEGCP_PRIVILEGE_SET|SEGCP_PRIVILEGE_WRITE)) ret |= SEGCP_RET_SWITCH;
                        else ret |= SEGCP_RET_ERR_NOPRIVILEGE;
                        break;
                    case SEGCP_RT:
                        if(gSEGCPPRIVILEGE & (SEGCP_PRIVILEGE_SET|SEGCP_PRIVILEGE_WRITE)) ret |= SEGCP_RET_REBOOT;
                        else ret |= SEGCP_RET_ERR_NOPRIVILEGE;
                        break;
                    case SEGCP_UN:
                        sprintf(trep, "%s", uart_if_table[dev_config->serial_option.uart_interface]);
                        PRT_SEGCP("dev_config->serial_option.uart_interface = %d\r\n", dev_config->serial_option.uart_interface);
                        PRT_SEGCP("uart_if_table[dev_config->serial_option.uart_interface] = %s\r\n", uart_if_table[dev_config->serial_option.uart_interface]);
                        break;
                    case SEGCP_UI:
                        sprintf(trep, "%d", dev_config->serial_option.uart_interface);
                        break;
                    case SEGCP_ST:
                        sprintf(trep, "%s", strDEVSTATUS[dev_config->network_connection.working_state]);
                        break;
                    case SEGCP_FR: 
                        if(gSEGCPPRIVILEGE & (SEGCP_PRIVILEGE_SET|SEGCP_PRIVILEGE_WRITE)) 
                        {
                            // #20161110 Hidden option, Local port number [1] + FR cmd => K! (EEPROM Erase)
                            if(dev_config->network_connection.local_port == 1)
                                ret |= SEGCP_RET_ERASE_EEPROM | SEGCP_RET_REBOOT; // EEPROM Erase
                            else
                                ret |= SEGCP_RET_FACTORY | SEGCP_RET_REBOOT; // Factory Reset
                        }
                        else
                            ret |= SEGCP_RET_ERR_NOPRIVILEGE;
                        break;
                    case SEGCP_EC:
                        sprintf(trep,"%d",dev_config->serial_command.serial_command_echo);
                        break;
                    case SEGCP_TR:
                        sprintf(trep, "%d", dev_config->network_option.tcp_rcr_val);
                        break;
                    case SEGCP_RC: // root ca option
                        sprintf(trep, "%d", dev_config->ssl_option.root_ca_option);
                        break;
                    
                    case SEGCP_CE: // client cert en/dis
                        sprintf(trep, "%d", dev_config->ssl_option.client_cert_enable);
                        break;

                    case SEGCP_SO: // SSL Recv Timeout
                        sprintf(trep, "%d", dev_config->ssl_option.recv_timeout);
                        break;


                    case SEGCP_QU: // mqtt username
                        if(dev_config->mqtt_option.user_name[0] == 0) sprintf(trep,"%c",SEGCP_NULL);
                        else sprintf(trep, "%s", dev_config->mqtt_option.user_name);
                        break;

                    case SEGCP_QP: // mqtt password
                        if(dev_config->mqtt_option.password[0] == 0) sprintf(trep,"%c",SEGCP_NULL);
                        else sprintf(trep, "%s", dev_config->mqtt_option.password);
                        break;

                    case SEGCP_QC: // mqtt client id
                        if(dev_config->mqtt_option.client_id[0] == 0) sprintf(trep,"%c",SEGCP_NULL);
                        else sprintf(trep, "%s", dev_config->mqtt_option.client_id);
                        break;

                    case SEGCP_QK: // mqtt keepalive
                        sprintf(trep, "%d", dev_config->mqtt_option.keepalive);
                        break;
                    
                    case SEGCP_PU: // mqtt publish topic
                        if(dev_config->mqtt_option.pub_topic[0] == 0) sprintf(trep,"%c",SEGCP_NULL);
                        else sprintf(trep, "%s", dev_config->mqtt_option.pub_topic);
                        break;

                    case SEGCP_U0: // mqtt subscribe topic
                        if(dev_config->mqtt_option.sub_topic_0[0] == 0) sprintf(trep,"%c",SEGCP_NULL);
                        else sprintf(trep, "%s", dev_config->mqtt_option.sub_topic_0);
                        //sprintf(trep, "%s", dev_config->mqtt_option.sub_topic);
                        break;

                    case SEGCP_U1: // mqtt subscribe topic
                        if(dev_config->mqtt_option.sub_topic_1[0] == 0) sprintf(trep,"%c",SEGCP_NULL);
                        else sprintf(trep, "%s", dev_config->mqtt_option.sub_topic_1);
                        //sprintf(trep, "%s", dev_config->mqtt_option.sub_topic);
                        break;

                    case SEGCP_U2: // mqtt subscribe topic
                        if(dev_config->mqtt_option.sub_topic_2[0] == 0) sprintf(trep,"%c",SEGCP_NULL);
                        else sprintf(trep, "%s", dev_config->mqtt_option.sub_topic_2);
                        //sprintf(trep, "%s", dev_config->mqtt_option.sub_topic);
                        break;
                    
                    case SEGCP_QO: // mqtt qos level
                        sprintf(trep, "%d", dev_config->mqtt_option.qos);
                        break;

                    case SEGCP_UF: // fw bank copy flag
                        sprintf(trep, "%d", dev_config->firmware_update.fwup_copy_flag);
                        break;
                        
                    default:
                        //ret |= SEGCP_RET_ERR_NOCOMMAND;
                        //sprintf(trep,"%s", strDEVSTATUS[dev_config->network_connection[0].working_state]);
                        sprintf(trep,"%c", SEGCP_NULL);
                        break;
                }
                
                if(ret & (SEGCP_RET_ERR | SEGCP_RET_REBOOT | SEGCP_RET_SWITCH | SEGCP_RET_SAVE))
                //if(ret & (SEGCP_RET_REBOOT | SEGCP_RET_SWITCH | SEGCP_RET_SAVE))
                {
                    trep -= SEGCP_CMD_MAX;
                    *trep = 0;
                }
                else
                {
                    strcat(trep, SEGCP_DELIMETER);
                    trep += strlen(trep);
                }
            }
            else if(gSEGCPPRIVILEGE & (SEGCP_PRIVILEGE_SET|SEGCP_PRIVILEGE_WRITE))
            {
                switch((teSEGCPCMDNUM)cmdnum)
                {
                    case SEGCP_MC:
                        if((dev_config->network_common.mac[0] == 0x00) && (dev_config->network_common.mac[1] == 0x08) && (dev_config->network_common.mac[2] == 0xDC)) ret |= SEGCP_RET_ERR_IGNORED;
                        else if(!is_macaddr(param, ".:-", dev_config->network_common.mac)) ret |= SEGCP_RET_ERR_INVALIDPARAM;
                        break;
                    case SEGCP_VR: 
                    case SEGCP_MN:
                        ret |= SEGCP_RET_ERR_IGNORED;
                        break;
                    case SEGCP_IM:
                        tmp_byte = is_hex(*param);
                        if(param_len != 1 || tmp_byte > SEGCP_DHCP) ret |= SEGCP_RET_ERR_INVALIDPARAM;
                        else dev_config->network_option.dhcp_use = tmp_byte;
                        break;
                    case SEGCP_OP: 
                        tmp_byte = is_hex(*param);
                        if(param_len != 1 || tmp_byte > MQTTS_CLIENT_MODE)
                            ret |= SEGCP_RET_ERR_INVALIDPARAM;
                        else
                        {
                            process_socket_termination(SEG_DATA0_SOCK, 100);
                            dev_config->network_connection.working_mode = tmp_byte;
                        }
                        break;

                    case SEGCP_CP:
                        tmp_byte = is_hex(*param);
                        if(param_len != 1 || tmp_byte > SEGCP_ENABLE) ret |= SEGCP_RET_ERR_INVALIDPARAM;
                        else dev_config->tcp_option.pw_connect_en = tmp_byte;
                        break;
                    case SEGCP_DG:
                        tmp_byte = is_hex(*param);
                        if(param_len != 1 || tmp_byte > SEG_DEBUG_ALL) ret |= SEGCP_RET_ERR_INVALIDPARAM;
                        else dev_config->serial_common.serial_debug_en = tmp_byte;
                        break;
                    case SEGCP_KA:
                        tmp_byte = is_hex(*param);
                        if(param_len != 1 || tmp_byte > SEGCP_ENABLE) ret |= SEGCP_RET_ERR_INVALIDPARAM;
                        else dev_config->tcp_option.keepalive_en = tmp_byte;
                        break;
                    case SEGCP_KI:
                        tmp_long = atol(param);
                        if(tmp_long > 0xFFFF) ret |= SEGCP_RET_ERR_INVALIDPARAM;
                        else dev_config->tcp_option.keepalive_wait_time = (uint16_t) tmp_long;
                        break;
                    case SEGCP_KE:
                        tmp_long = atol(param);
                        if(tmp_long > 0xFFFF) ret |= SEGCP_RET_ERR_INVALIDPARAM;
                        else dev_config->tcp_option.keepalive_retry_time = (uint16_t) tmp_long;
                        break;
                    case SEGCP_RI:
                        tmp_long = atol(param);
                        if(tmp_long > 0xFFFF) ret |= SEGCP_RET_ERR_INVALIDPARAM;
                        else dev_config->tcp_option.reconnection = (uint16_t) tmp_long;
                        break;
                    case SEGCP_LI:
                        if(is_ipaddr(param, tmp_ip))
                        {
                            dev_config->network_common.local_ip[0] = tmp_ip[0];
                            dev_config->network_common.local_ip[1] = tmp_ip[1];
                            dev_config->network_common.local_ip[2] = tmp_ip[2];
                            dev_config->network_common.local_ip[3] = tmp_ip[3];
                        }
                        else
                        {
                            ret |= SEGCP_RET_ERR_INVALIDPARAM;
                        }
                        break;
                    case SEGCP_SM:
                        if(is_ipaddr(param, tmp_ip))
                        {
                            dev_config->network_common.subnet[0] = tmp_ip[0];
                            dev_config->network_common.subnet[1] = tmp_ip[1];
                            dev_config->network_common.subnet[2] = tmp_ip[2];
                            dev_config->network_common.subnet[3] = tmp_ip[3];
                        }
                        else ret |= SEGCP_RET_ERR_INVALIDPARAM;
                        break;
                    case SEGCP_GW:
                        if(is_ipaddr(param, tmp_ip))
                        {
                            dev_config->network_common.gateway[0] = tmp_ip[0];
                            dev_config->network_common.gateway[1] = tmp_ip[1];
                            dev_config->network_common.gateway[2] = tmp_ip[2];
                            dev_config->network_common.gateway[3] = tmp_ip[3];
                        }
                        else ret |= SEGCP_RET_ERR_INVALIDPARAM;
                        break;
                    case SEGCP_DS:
                        if(is_ipaddr(param, tmp_ip))
                        {
                            dev_config->network_option.dns_server_ip[0] = tmp_ip[0];
                            dev_config->network_option.dns_server_ip[1] = tmp_ip[1];
                            dev_config->network_option.dns_server_ip[2] = tmp_ip[2];
                            dev_config->network_option.dns_server_ip[3] = tmp_ip[3];
                        }
                        else ret |= SEGCP_RET_ERR_INVALIDPARAM;
                        break;

                    case SEGCP_DH:
                        if(param_len > sizeof(dev_config->device_common.device_name)-1) ret |= SEGCP_RET_ERR_INVALIDPARAM;
                        else 
                        {
                            if(param[0] == SEGCP_NULL)
                                dev_config->device_common.device_name[0] = 0;
                            else
                                sprintf(dev_config->device_common.device_name, "%s", param);
                        }
                        break;
                    case SEGCP_LP:
                        tmp_long = atol(param);
                        if(tmp_long > 0xFFFF) ret |= SEGCP_RET_ERR_INVALIDPARAM;
                        else dev_config->network_connection.local_port = (uint16_t)tmp_long;
                        break;
                    case SEGCP_RP:
                        tmp_long = atol(param);
                        if(tmp_long > 0xFFFF) ret |= SEGCP_RET_ERR_INVALIDPARAM;
                        else dev_config->network_connection.remote_port = (uint16_t)tmp_long;
                        break;
                    case SEGCP_RH:
                        if(is_ipaddr(param, tmp_ip))
                        {
                            dev_config->network_connection.dns_use = SEGCP_DISABLE;
                            dev_config->network_connection.remote_ip[0] = tmp_ip[0];
                            dev_config->network_connection.remote_ip[1] = tmp_ip[1];
                            dev_config->network_connection.remote_ip[2] = tmp_ip[2];
                            dev_config->network_connection.remote_ip[3] = tmp_ip[3];
                            strcpy(dev_config->network_connection.dns_domain_name, param);
                        }
                        else
                        {
                            dev_config->network_connection.dns_use = SEGCP_ENABLE;
                            if(param[0] == SEGCP_NULL)
                                dev_config->network_connection.dns_domain_name[0] = 0;
                            else
                                strcpy(dev_config->network_connection.dns_domain_name, param);
                        }
                        
                        break;
                    case SEGCP_BR:
                        tmp_int = atoi(param);
                        if(param_len > 2 || tmp_int > baud_460800) ret |= SEGCP_RET_ERR_INVALIDPARAM; // ## 20180208 Added by Eric, Supports baudrate up to 460.8kbps 
                        else dev_config->serial_option.baud_rate = (uint8_t)tmp_int;
                        break;
                    case SEGCP_DB:
                        tmp_byte = is_hex(*param);
                        if(param_len != 1 || tmp_byte > word_len8) ret |= SEGCP_RET_ERR_INVALIDPARAM;
                        else dev_config->serial_option.data_bits = tmp_byte;
                        break;
                    case SEGCP_PR:
                        tmp_byte = is_hex(*param);
                        if(param_len != 1 || tmp_byte > parity_even) ret |= SEGCP_RET_ERR_INVALIDPARAM;
                        else dev_config->serial_option.parity = tmp_byte;
                        break;
                    case SEGCP_SB:
                        tmp_byte = is_hex(*param);
                        if(param_len != 1 || tmp_byte > stop_bit2) ret |= SEGCP_RET_ERR_INVALIDPARAM;
                        else dev_config->serial_option.stop_bits = tmp_byte;
                        break;
                    case SEGCP_FL:
                        tmp_byte = is_hex(*param);
                        if(param_len != 1 || tmp_byte > flow_reverserts)
                        {
                            ret |= SEGCP_RET_ERR_INVALIDPARAM;
                        }
                        else
                        {
                            if((dev_config->serial_option.uart_interface == UART_IF_RS422) ||
                               (dev_config->serial_option.uart_interface == UART_IF_RS485))
                            {
                                if((tmp_byte != flow_rtsonly) && (tmp_byte != flow_reverserts))
                                    dev_config->serial_option.flow_control = flow_none;
                                else
                                    dev_config->serial_option.flow_control = tmp_byte;
                            }
                            else
                                dev_config->serial_option.flow_control = tmp_byte;
                        }
                        break;
                    case SEGCP_PO:
                        tmp_int = atoi(param);
                        if(param_len > 2 || tmp_int > modbus_ascii) ret |= SEGCP_RET_ERR_INVALIDPARAM;
                        else dev_config->serial_option.protocol = tmp_int;
                        break;
                    case SEGCP_IT:
                        tmp_long = atol(param);
                        if(tmp_long > 0xFFFF) ret |= SEGCP_RET_ERR_INVALIDPARAM;
                        else dev_config->tcp_option.inactivity = (uint16_t)tmp_long;
                        break;
                    case SEGCP_PT:
                        tmp_long = atol(param);
                        if(tmp_long > 0xFFFF) ret |= SEGCP_RET_ERR_INVALIDPARAM;
                        else dev_config->serial_data_packing.packing_time = (uint16_t)tmp_long;
                        break;
                    case SEGCP_PS:
                        tmp_int = atoi(param);
                        if(param_len > 4 || tmp_int > (SEG_DATA_BUF_SIZE / 2)) ret |= SEGCP_RET_ERR_INVALIDPARAM;
                        else dev_config->serial_data_packing.packing_size = (uint16_t)tmp_int;
                        break;
                    case SEGCP_PD:
                        if(param_len != 2 || !is_hexstr(param))
                            ret |= SEGCP_RET_ERR_INVALIDPARAM;
                        else
                        {
                            str_to_hex(param, &tmp_byte);
                            dev_config->serial_data_packing.packing_delimiter[0] = tmp_byte;
                            
                            if(dev_config->serial_data_packing.packing_delimiter[0] == 0x00)
                                dev_config->serial_data_packing.packing_delimiter_length = 0;
                            else 
                                dev_config->serial_data_packing.packing_delimiter_length = 1;
                        }
                        break;
                    case SEGCP_TE:
                        tmp_byte = is_hex(*param);
                        if(param_len != 1 || tmp_byte > SEGCP_ENABLE) ret |= SEGCP_RET_ERR_INVALIDPARAM;
                        else dev_config->serial_command.serial_command = tmp_byte;
                        break;
                    case SEGCP_SS:
                        if(param_len != 6 || !is_hexstr(param) || !str_to_hex(param, dev_config->serial_command.serial_trigger))
                            ret |= SEGCP_RET_ERR_INVALIDPARAM;
                        break;
                    case SEGCP_NP:
                        if(param_len > sizeof(dev_config->tcp_option.pw_connect)-1)
                            ret |= SEGCP_RET_ERR_INVALIDPARAM;
                        else
                        {
                            if(param[0] == SEGCP_NULL) dev_config->tcp_option.pw_connect[0] = 0;
                            else sprintf(dev_config->tcp_option.pw_connect, "%s", param);
                        }
                        break;
                    case SEGCP_SP:
                        if(param_len > sizeof(dev_config->config_common.pw_search)-1)
                            ret |= SEGCP_RET_ERR_INVALIDPARAM;
                        else
                        {
                            if(param[0] == SEGCP_NULL) dev_config->config_common.pw_search[0] = 0;
                            else sprintf(dev_config->config_common.pw_search, "%s", param);
                        }
                        break;

                    // SET status pin mode selector
                    case SEGCP_SC:
                        str_to_hex(param, &tmp_byte);
                        
                        tmp_int = (tmp_byte & 0xF0) >> 4;   // [0] PHY link / [1] DTR
                        tmp_byte = (tmp_byte & 0x0F);       // [0] TCP connection / [1] DSR
                        
                        if((param_len > 2) || (tmp_byte > IO_HIGH) || (tmp_int > IO_HIGH)) // Invalid parameters
                            ret |= SEGCP_RET_ERR_INVALIDPARAM;
                        else
                        {
                            dev_config->serial_option.dtr_en = (uint8_t)tmp_int;
                            dev_config->serial_option.dsr_en = tmp_byte;
                            
                            // Set the DTR pin to high when the DTR signal enabled
                            if(dev_config->serial_option.dtr_en == SEGCP_ENABLE) set_flowcontrol_dtr_pin(ON);
                        }
                        break;
                    case SEGCP_S0:
                    case SEGCP_S1:
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                        break;

                    case SEGCP_RX:
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                        break;

                    case SEGCP_EC:
                        tmp_byte = is_hex(*param);
                        if(param_len != 1 || tmp_byte > SEGCP_ENABLE) ret |= SEGCP_RET_ERR_INVALIDPARAM;
                        else dev_config->serial_command.serial_command_echo = tmp_byte;
                        break;
                    
                    case SEGCP_TR: // TCP Retransmission retry count
                        tmp_int = atoi(param);
                        if((param_len > 3) || (tmp_int < 1) || (tmp_int > 0xFF)) ret |= SEGCP_RET_ERR_INVALIDPARAM;
                        else dev_config->network_option.tcp_rcr_val = (uint8_t)tmp_int;
                        break;
                                            
                    case SEGCP_RC: // root ca option
                        tmp_byte = atoi(param);
                        if (tmp_byte > 2) //0: Verify_none / 1: Verify_option / 2: Verify_require
                        {
                            ret |= SEGCP_RET_ERR_INVALIDPARAM;
                            break;
                        }
                        dev_config->ssl_option.root_ca_option = tmp_byte;
                        break;
                    
                    case SEGCP_CE: // client cert en/dis
                        tmp_byte = atoi(param);
                        if (tmp_byte > 1)
                        {
                            ret |= SEGCP_RET_ERR_INVALIDPARAM;
                            break;
                        }
                        dev_config->ssl_option.client_cert_enable = tmp_byte;
                        break;
                        
                    case SEGCP_SO: // SSL Recv Timeout
                        tmp_int = atoi(param);
                        if (tmp_int > SSL_RECV_MAX_TIMEOUT) ret |= SEGCP_RET_ERR_INVALIDPARAM;
                        else dev_config->ssl_option.recv_timeout = tmp_int;
                        break;

                    case SEGCP_QU: // mqtt username
                        if(param[0] == SEGCP_NULL)
                            dev_config->mqtt_option.user_name[0] = 0;
                        else
                            sprintf(dev_config->mqtt_option.user_name, "%s", param);
                        break;

                    case SEGCP_QP: // mqtt password
                        if(param[0] == SEGCP_NULL)
                            dev_config->mqtt_option.password[0] = 0;
                        else
                            sprintf(dev_config->mqtt_option.password, "%s", param);
                        break;


                    case SEGCP_QC: // mqtt client id
                        if(param[0] == SEGCP_NULL)
                            dev_config->mqtt_option.client_id[0] = 0;
                        else
                            sprintf(dev_config->mqtt_option.client_id, "%s", param);
                        break;

                    case SEGCP_QK: // mqtt keepalive
                        dev_config->mqtt_option.keepalive = atoi(param);
                        break;
                    
                    case SEGCP_PU: // mqtt publish topic
                        if(param[0] == SEGCP_NULL)
                            dev_config->mqtt_option.pub_topic[0] = 0;
                        else
                            sprintf(dev_config->mqtt_option.pub_topic, "%s", param);
                        break;

                    case SEGCP_U0: // mqtt subscribe topic
                        if(param[0] == SEGCP_NULL)
                            dev_config->mqtt_option.sub_topic_0[0] = 0;
                        else
                            sprintf(dev_config->mqtt_option.sub_topic_0, "%s", param);
                        break;

                    case SEGCP_U1: // mqtt subscribe topic
                        if(param[0] == SEGCP_NULL)
                            dev_config->mqtt_option.sub_topic_1[0] = 0;
                        else
                            sprintf(dev_config->mqtt_option.sub_topic_1, "%s", param);
                        break;

                    case SEGCP_U2: // mqtt subscribe topic
                        if(param[0] == SEGCP_NULL)
                            dev_config->mqtt_option.sub_topic_2[0] = 0;
                        else
                            sprintf(dev_config->mqtt_option.sub_topic_2, "%s", param);
                        break;

                    case SEGCP_QO: // mqtt qos level
                        tmp_byte = atoi(param);
                        if (tmp_byte > 2)
                        {
                            ret |= SEGCP_RET_ERR_INVALIDPARAM;
                            break;
                        }
                        dev_config->mqtt_option.qos = tmp_byte;
                        break;
                    case SEGCP_UF: // Current Bank
                        tmp_byte = atoi(param);
                        if (tmp_byte > 1)
                        {
                            ret |= SEGCP_RET_ERR_INVALIDPARAM;
                            break;
                        }
                        if (0 == device_bank_check(tmp_byte))
                          dev_config->firmware_update.fwup_copy_flag = tmp_byte;
                        break;
                    case SEGCP_OC: // rootca
                        {
                            tmp_ptr = g_temp_buf;
                            memset(tmp_ptr, NULL, ROOTCA_BUF_SIZE);
                            sprintf(tmp_ptr, "%s", treq+SEGCP_CMD_MAX);

                            tmp_ptr += strlen(tmp_ptr);
                            while((len = getSn_RX_RSR(SEGCP_UDP_SOCK)) > 0)
                            {
                                PRT_SEGCP("while((len = getSn_RX_RSR(SEGCP_UDP_SOCK)) > 0)\r\n");
                                len = recvfrom(SEGCP_UDP_SOCK, tmp_ptr, len, tmp_ip, &tmp_port);
                                tmp_ptr += len;
                                if((tmp_ptr - g_temp_buf) > ROOTCA_BUF_SIZE)
                                {
                                    ret |= SEGCP_RET_ERR_INVALIDPARAM;
                                    break;
                                }
                            }
                            if(!(ret & SEGCP_RET_ERR))
                            {
                                tmp_ptr = strstr(g_temp_buf, END_CERT);
                                if (tmp_ptr == NULL)
                                {                                    
                                    ret |= SEGCP_RET_ERR_INVALIDPARAM;
                                    break;
                                }
                                tmp_ptr += strlen(END_CERT);

                                if (*tmp_ptr == '\n')
                                    tmp_ptr++;
                                else if (*tmp_ptr == '\r')
                                    tmp_ptr+=2;
                                else
                                {
                                    *(tmp_ptr) = '\r';
                                    *(tmp_ptr+1) = '\n';
                                    tmp_ptr+=2;
                                }
                                
                                dev_config->ssl_option.rootca_len = tmp_ptr - g_temp_buf;
                                g_temp_buf[dev_config->ssl_option.rootca_len] = NULL;

                                PRT_SEGCP("g_rootca_buf = \r\n%s\r\n", g_temp_buf);
                                
                                ret_2 = check_ca(g_temp_buf, dev_config->ssl_option.rootca_len);
                                if (ret_2 < 0)
                                    ret |= SEGCP_RET_ERR_INVALIDPARAM;
                                else
                                {
                                    save_DevConfig_to_storage();
                                    erase_storage(STORAGE_ROOTCA);
                                    write_storage(STORAGE_ROOTCA, 0, (uint8_t *)g_temp_buf, dev_config->ssl_option.rootca_len + 1);
                                    
                                    memcpy(trep, tbSEGCPCMD[cmdnum], SEGCP_CMD_MAX);
                                    trep+=SEGCP_CMD_MAX;
                                    strcat(trep, SEGCP_DELIMETER);
                                }
                            }
                            return ret;
                        }
                        break;

                    case SEGCP_LC: // client_cert
                        {
                            tmp_ptr = g_temp_buf;
                            memset(tmp_ptr, NULL, CLICA_BUF_SIZE);
                            sprintf(tmp_ptr, "%s", treq+SEGCP_CMD_MAX);

                            tmp_ptr += strlen(tmp_ptr);
                            while((len = getSn_RX_RSR(SEGCP_UDP_SOCK)) > 0)
                            {
                                PRT_SEGCP("while((len = getSn_RX_RSR(SEGCP_UDP_SOCK)) > 0)\r\n");
                                len = recvfrom(SEGCP_UDP_SOCK, tmp_ptr, len, tmp_ip, &tmp_port);
                                tmp_ptr += len;
                                if((tmp_ptr - g_temp_buf) > CLICA_BUF_SIZE)
                                {
                                    ret |= SEGCP_RET_ERR_INVALIDPARAM;
                                    break;
                                }
                            }
                            if(!(ret & SEGCP_RET_ERR))
                            {
                                tmp_ptr = strstr(g_temp_buf, END_CERT);
                                if (tmp_ptr == NULL)
                                {                                    
                                    ret |= SEGCP_RET_ERR_INVALIDPARAM;
                                    break;
                                }
                                tmp_ptr += strlen(END_CERT);

                                if (*tmp_ptr == '\n')
                                    tmp_ptr++;
                                else if (*tmp_ptr == '\r')
                                    tmp_ptr+=2;
                                else
                                {
                                    *(tmp_ptr) = '\r';
                                    *(tmp_ptr+1) = '\n';
                                    tmp_ptr+=2;
                                }
                            
                                dev_config->ssl_option.clica_len = tmp_ptr - g_temp_buf;
                                g_temp_buf[dev_config->ssl_option.clica_len] = NULL;
                                ret_2 = check_ca(g_temp_buf, dev_config->ssl_option.clica_len);
                                if (ret_2 < 0)
                                    ret |= SEGCP_RET_ERR_INVALIDPARAM;
                                else
                                {
                                    save_DevConfig_to_storage();
                                    erase_storage(STORAGE_CLICA);
                                    write_storage(STORAGE_CLICA, 0, (uint8_t *)g_temp_buf, dev_config->ssl_option.clica_len + 1);

                                    memcpy(trep, tbSEGCPCMD[cmdnum], SEGCP_CMD_MAX);
                                    trep+=SEGCP_CMD_MAX;
                                    strcat(trep, SEGCP_DELIMETER);
                                }
                            }
                            PRT_SEGCP("g_clica_buf = \r\n%s\r\n", g_temp_buf);
                            return ret;
                        }
                        break;

                    case SEGCP_PK: // pkey
                        {
                            tmp_ptr = g_temp_buf;
                            memset(tmp_ptr, NULL, PKEY_BUF_SIZE);
                            sprintf(tmp_ptr, "%s", treq+SEGCP_CMD_MAX);

                            tmp_ptr += strlen(tmp_ptr);
                            while((len = getSn_RX_RSR(SEGCP_UDP_SOCK)) > 0)
                            {
                                PRT_SEGCP("while((len = getSn_RX_RSR(SEGCP_UDP_SOCK)) > 0)\r\n");
                                len = recvfrom(SEGCP_UDP_SOCK, tmp_ptr, len, tmp_ip, &tmp_port);
                                tmp_ptr += len;
                                if((tmp_ptr - g_temp_buf) > PKEY_BUF_SIZE)
                                {
                                    ret |= SEGCP_RET_ERR_INVALIDPARAM;
                                    break;
                                }
                            }
                            if(!(ret & SEGCP_RET_ERR))
                            {
                                tmp_ptr = strstr(g_temp_buf, END_PKEY);
                                if (tmp_ptr == NULL)
                                {                                    
                                    ret |= SEGCP_RET_ERR_INVALIDPARAM;
                                    break;
                                }
                                tmp_ptr += strlen(END_PKEY);

                                if (*tmp_ptr == '\n')
                                    tmp_ptr++;
                                else if (*tmp_ptr == '\r')
                                    tmp_ptr+=2;
                                else
                                {
                                    *(tmp_ptr) = '\r';
                                    *(tmp_ptr+1) = '\n';
                                    tmp_ptr+=2;
                                }
                                    
                                dev_config->ssl_option.pkey_len = tmp_ptr - g_temp_buf;
                                g_temp_buf[dev_config->ssl_option.pkey_len] = NULL;
                                ret_2 = check_pkey(g_temp_buf, dev_config->ssl_option.pkey_len);
                                if (ret_2 < 0)
                                    ret |= SEGCP_RET_ERR_INVALIDPARAM;
                                else
                                {
                                    save_DevConfig_to_storage();
                                    erase_storage(STORAGE_PKEY);
                                    write_storage(STORAGE_PKEY, 0, (uint8_t *)g_temp_buf, dev_config->ssl_option.pkey_len + 1);
                                    
                                    memcpy(trep, tbSEGCPCMD[cmdnum], SEGCP_CMD_MAX);
                                    trep+=SEGCP_CMD_MAX;
                                    strcat(trep, SEGCP_DELIMETER);
                                }
                            }
                            PRT_SEGCP("g_pkey_buf = \r\n%s\r\n", g_temp_buf);
                            return ret;
                        }
                        break;


                    case SEGCP_FW: // f/w update
                        tmp_long = atol(param);
                    
                        if(tmp_long > (uint32_t)FLASH_APP_BANK_SIZE)
                        {
                            dev_config->firmware_update.fwup_size = 0;
                            ret |= SEGCP_RET_ERR_INVALIDPARAM;
                            PRT_SEGCP("SEGCP_FW:ERROR:TOOBIG\r\n");
                        }
                        else
                        {
                            dev_config->firmware_update.fwup_size = tmp_long;
                            dev_config->firmware_update.fwup_flag = SEGCP_ENABLE;
                            ret |= SEGCP_RET_FWUP_BANK;

                            sprintf(trep,"FW%d.%d.%d.%d:%d\r\n", dev_config->network_common.local_ip[0],
                                                                 dev_config->network_common.local_ip[1],
                                                                 dev_config->network_common.local_ip[2],
                                                                 dev_config->network_common.local_ip[3],
                                                                 (uint16_t)DEVICE_FWUP_PORT);
                            
                            process_socket_termination(SEG_DATA0_SOCK, 100);
                            PRT_SEGCP("SEGCP_FW:OK\r\n");
                        }
                        break;
#ifdef __USE_USERS_GPIO__
                    // SET GPIOs Status / Value (Digital output only)
                    case SEGCP_GA:
                    case SEGCP_GB:
                        io_num = (teSEGCPCMDNUM)cmdnum - SEGCP_GA;
                        tmp_int = is_hex(*param);
                        if(param_len != 1 || tmp_int > IO_HIGH) ret |= SEGCP_RET_ERR_INVALIDPARAM;
                        else
                            if(set_user_io_val(USER_IO_SEL[io_num], &tmp_int) == 0) ret |= SEGCP_RET_ERR_INVALIDPARAM;
                        break;
                    
                    // SET GPIOs settings; Type and Direction ('Analog output' mode is not allowed)
                    case SEGCP_CA:
                    case SEGCP_CB:
                        io_num = (teSEGCPCMDNUM)cmdnum - SEGCP_CA;
                        tmp_int = atoi(param);
                        
                        io_type = (uint8_t)(tmp_int >> 1);
                        io_dir = (uint8_t)(tmp_int & 0x01);
                        
                        if((param_len > 2) || (io_type > IO_ANALOG_IN) || (io_dir > IO_OUTPUT)) // Invalid parameters
                            ret |= SEGCP_RET_ERR_INVALIDPARAM;
                        else
                        {
                            if((io_type == IO_ANALOG_IN) && (io_dir == IO_OUTPUT)) // This case not allowed. (Analog output)
                                ret |= SEGCP_RET_ERR_INVALIDPARAM;
                            else
                            {
                                // IO type and Direction settings
                                set_user_io_type(USER_IO_SEL[io_num], io_type);
                                set_user_io_direction(USER_IO_SEL[io_num], io_dir);
                                init_user_io(USER_IO_SEL[io_num]);
                            }
                        }
                        break;
#endif




                    case SEGCP_UN:
                    case SEGCP_ST:
                    case SEGCP_MA:
                    case SEGCP_EX: 
                    case SEGCP_SV:
                    case SEGCP_RT:
                    case SEGCP_FR:
                    case SEGCP_PW:
                    case SEGCP_UI:
                        ret |= SEGCP_RET_ERR_NOTAVAIL;
                        break;
                    default:
                        //ret |= SEGCP_RET_ERR_NOCOMMAND;
                        break;
                }
            }
            else
                ret |= SEGCP_RET_ERR_NOPRIVILEGE;
        }
        else
            ret |= SEGCP_RET_ERR_NOCOMMAND;
        
        // Process the serial command mode
        if(opmode == DEVICE_AT_MODE)
        {
            if(ret & SEGCP_RET_ERR)
            {
                treq[2] = 0;
                sprintf(trep,"%s:%s\r\n",tbSEGCPERR[((ret-SEGCP_RET_ERR) >> 8)],(cmdnum!=SEGCP_UNKNOWN)? tbSEGCPCMD[cmdnum] : treq);
#ifdef DBG_LEVEL_SEGCP
                PRT_SEGCP("ERROR : %s\r\n",trep);
#endif
                uart_rx_flush();
                return ret;
            }
        }
        treq = strtok(NULL, SEGCP_DELIMETER);
    }
    
#ifdef DBG_LEVEL_SEGCP
    PRT_SEGCP("END of [proc_SEGCP] function - RET[0x%.4x]\r\n\r\n", ret);
#endif
    
    return ret;
}

uint16_t proc_SEGCP_udp(uint8_t* segcp_req, uint8_t* segcp_rep)
{
    DevConfig *dev_config = get_DevConfig_pointer();
    
    uint16_t ret = 0;
    uint16_t len = 0;
    
    uint8_t destip[4];
    uint16_t destport;
    
    uint8_t tpar[SEGCP_PARAM_MAX*2];
    uint8_t* treq;
    uint8_t* trep;
    
    gSEGCPPRIVILEGE = SEGCP_PRIVILEGE_CLR;
    switch(getSn_SR(SEGCP_UDP_SOCK))
    {
        case SOCK_UDP:
            if((len = getSn_RX_RSR(SEGCP_UDP_SOCK)) > 0)
            {
                PRT_SEGCP("len = getSn_RX_RSR = %d\r\n", len);
                treq = segcp_req;
                trep = segcp_rep;
                len = recvfrom(SEGCP_UDP_SOCK, treq, len, destip, &destport);
                PRT_SEGCP("len = recvfrom = %d\r\n", len);
                
                //treq[len-1] = 0;
                treq[len] = 0;
                
                if(SEGCP_MA == parse_SEGCP(treq, tpar))
                {
                    if(!memcmp(tpar,"\xFF\xFF\xFF\xFF\xFF\xFF", 6))
                        gSEGCPPRIVILEGE |= (SEGCP_PRIVILEGE_SET | SEGCP_PRIVILEGE_READ);
                    else if(!memcmp(tpar, dev_config->network_common.mac, sizeof(dev_config->network_common.mac)))
                        gSEGCPPRIVILEGE |= (SEGCP_PRIVILEGE_SET | SEGCP_PRIVILEGE_WRITE);
                    else break;
                    
                    if(gSEGCPPRIVILEGE & SEGCP_PRIVILEGE_SET)
                    {
                        sprintf(trep,"%s%c%c%c%c%c%c\r\n",tbSEGCPCMD[SEGCP_MA],
                            dev_config->network_common.mac[0],
                            dev_config->network_common.mac[1],
                            dev_config->network_common.mac[2],
                            dev_config->network_common.mac[3],
                            dev_config->network_common.mac[4],
                            dev_config->network_common.mac[5]);
                        
                        treq += 10;
                        trep += 10;
                        
                        if(SEGCP_PW == parse_SEGCP(treq, tpar))
                        {
                            if((tpar[0] == SEGCP_NULL && dev_config->config_common.pw_search[0] == 0) || !strcmp(tpar, dev_config->config_common.pw_search))
                            {
                                memcpy(trep,treq, strlen(tpar)+4);  // "PWxxxx\r\n"
                                treq += (strlen(tpar) + 4);
                                trep += (strlen(tpar) + 4);
                                ret = proc_SEGCP(treq,trep);
                                
                                sendto(SEGCP_UDP_SOCK, segcp_rep, 14+strlen(tpar)+strlen(trep), "\xFF\xFF\xFF\xFF", destport);

                                PRT_SEGCP("tpar_len = %d, trep_len = %d\r\n", strlen(tpar), strlen(trep));

                                PRT_SEGCP("tpar = %s, trep = %s\r\n", tpar, trep);
                                PRT_SEGCP("send to len = %d\r\n", 14+strlen(tpar)+strlen(trep));
                                PRT_SEGCP(">> strtok: %s\r\n", segcp_rep);
                            }
                        }
                    }
                    else
                        return 0;
                }
                else
                    return 0;
            }
            break;
        case SOCK_CLOSED:
            socket(SEGCP_UDP_SOCK, Sn_MR_UDP, DEVICE_SEGCP_PORT, 0x00);
            break;
    }
    return ret;
}


uint16_t proc_SEGCP_tcp(uint8_t* segcp_req, uint8_t* segcp_rep)
{
    DevConfig *dev_config = get_DevConfig_pointer();
    
    uint16_t ret = 0;
    uint16_t len = 0;
    
    uint8_t tpar[SEGCP_PARAM_MAX+1];
    uint8_t * treq;
    uint8_t * trep;
    
    uint8_t state = getSn_SR(SEGCP_TCP_SOCK);
    gSEGCPPRIVILEGE = SEGCP_PRIVILEGE_CLR;
    
    switch(state)
    {
        case SOCK_INIT:
            break;
        
        case SOCK_LISTEN:
            break;
        
        case SOCK_ESTABLISHED:
            if(getSn_IR(SEGCP_TCP_SOCK) & Sn_IR_CON)
            {
                // TCP unicast search: Keep-alive timer enable 
                enable_configtool_keepalive_timer = ENABLE;
                configtool_keepalive_time = 0;
                setSn_IR(SEGCP_TCP_SOCK, Sn_IR_CON); // TCP connection interrupt clear
            }
            
            if(flag_send_configtool_keepalive == SEGCP_ENABLE) // default: 15sec
                flag_send_configtool_keepalive = SEGCP_DISABLE; // flag clear

            if((len = getSn_RX_RSR(SEGCP_TCP_SOCK)) > 0)
            {
                treq = segcp_req;
                trep = segcp_rep;
                len = recv(SEGCP_TCP_SOCK, treq, len);
                treq[len-1] = 0x00;

                if(SEGCP_MA == parse_SEGCP(treq, tpar))
                {
                    if(!memcmp(tpar, "\xFF\xFF\xFF\xFF\xFF\xFF", 6))
                        gSEGCPPRIVILEGE |= (SEGCP_PRIVILEGE_SET | SEGCP_PRIVILEGE_READ);
                    else if(!memcmp(tpar, dev_config->network_common.mac, sizeof(dev_config->network_common.mac)))
                        gSEGCPPRIVILEGE |= (SEGCP_PRIVILEGE_SET | SEGCP_PRIVILEGE_WRITE);
                    else break;
                    
                    if(gSEGCPPRIVILEGE & SEGCP_PRIVILEGE_SET)
                    {
                        sprintf(trep,"%s%c%c%c%c%c%c\r\n",tbSEGCPCMD[SEGCP_MA],
                            dev_config->network_common.mac[0],
                            dev_config->network_common.mac[1],
                            dev_config->network_common.mac[2],
                            dev_config->network_common.mac[3],
                            dev_config->network_common.mac[4],
                            dev_config->network_common.mac[5]);
                        
                        treq += 10;
                        trep += 10;
                        
                        if(SEGCP_PW == parse_SEGCP(treq,tpar))
                        {
                            if((tpar[0] == SEGCP_NULL && dev_config->config_common.pw_search[0] == 0) || !strcmp(tpar, dev_config->config_common.pw_search))
                            {
                                memcpy(trep,treq, strlen(tpar)+4);  // "PWxxxx\r\n"
                                treq += (strlen(tpar) + 4);
                                trep += (strlen(tpar) + 4);
                                ret = proc_SEGCP(treq,trep);
                                send(SEGCP_TCP_SOCK, segcp_rep, 14+strlen(tpar)+strlen(trep));
                            }
                        }
                    }
                    else
                        return 0;
                }
                else
                    return 0;
            }
            break;
            
        case SOCK_CLOSE_WAIT:
            disconnect(SEGCP_TCP_SOCK);
        
        case SOCK_CLOSED:
        case SOCK_FIN_WAIT:
            close(SEGCP_TCP_SOCK);
            
            if(socket(SEGCP_TCP_SOCK, Sn_MR_TCP, DEVICE_SEGCP_PORT, SF_TCP_NODELAY) == SEGCP_TCP_SOCK)
            {
                //Keep-alive timer keep disabled until TCP connection established.
                enable_configtool_keepalive_timer = DISABLE;
                listen(SEGCP_TCP_SOCK);
            }
            break;
    }
    return ret;
}

uint16_t proc_SEGCP_uart(uint8_t * segcp_req, uint8_t * segcp_rep)
{
    DevConfig *dev_config = get_DevConfig_pointer();
    
    uint16_t len = 0;
    uint16_t ret = 0;

    if(get_uart_buffer_usedsize())
    {
        len = uart_get_commandline(segcp_req, CONFIG_BUF_SIZE);
        
        if(len != 0)
        {
            gSEGCPPRIVILEGE = SEGCP_PRIVILEGE_SET | SEGCP_PRIVILEGE_WRITE;
            ret = proc_SEGCP(segcp_req, segcp_rep);
            if(segcp_rep[0])
            {
                if(dev_config->serial_common.serial_debug_en)
                {
                    printf("%s",segcp_rep);
                }
                platform_uart_puts(segcp_rep, strlen((char *)segcp_rep));
            }
        }
    }
    return ret;
}

void set_SEGCP_privilege(uint8_t ptype)
{
    if(ptype == SEGCP_PRIVILEGE_READ)
    {
        gSEGCPPRIVILEGE |= (SEGCP_PRIVILEGE_SET | SEGCP_PRIVILEGE_READ);
    }
    else if(ptype == SEGCP_PRIVILEGE_WRITE)
    {
        gSEGCPPRIVILEGE |= (SEGCP_PRIVILEGE_SET | SEGCP_PRIVILEGE_WRITE);
    }
    else if(ptype == SEGCP_PRIVILEGE_CLR)
    {
        gSEGCPPRIVILEGE = SEGCP_PRIVILEGE_CLR;
    }
}

uint16_t uart_get_commandline(uint8_t* buf, uint16_t maxSize)
{
    DevConfig *dev_config = get_DevConfig_pointer();
    
    uint16_t i;
    uint16_t len = get_uart_buffer_usedsize();

    if(len >= 4) // Minimum of command: 4-bytes, e.g., MC\r\n (MC$0d$0a)
    {
        memset(buf, NULL, CONFIG_BUF_SIZE);
        for(i = 0; i < maxSize; i++)
        {
            buf[i] = platform_uart_getc();
            if(buf[i] == 0x0a) break;	// [0x0a]: end of command (Line feed)
        }

        if ((!(memcmp(buf, "OC", SEGCP_CMD_MAX))) || (!(memcmp(buf, "LC", SEGCP_CMD_MAX))))
        {
            for(i++; i < maxSize; i++)
            {
                buf[i] = platform_uart_getc();
                if (strstr(buf, END_CERT))
                {
                    sleep_ms(10);
                    uart_rx_flush();
                    break;
                }
            }
        }
        else if (!(memcmp(buf, "PK", SEGCP_CMD_MAX)))
        {
            for(i++; i < maxSize; i++)
            {
                buf[i] = platform_uart_getc();
                if (strstr(buf, END_PKEY))
                {
                    delay_ms(10);
                    uart_rx_flush();
                    break;
                }
            }
        }
        buf[i+1] = 0x00; // end of string
        PRT_SEGCP("buf = %s\r\n", buf);
        if(dev_config->serial_command.serial_command_echo == SEGCP_ENABLE)
            platform_uart_puts(buf, i);
    }
    else
        return 0;    
    return (uint16_t)strlen(buf);
}

// Function for Timer
void segcp_timer_msec(void)
{
    if(enable_configtool_keepalive_timer)
    {
        if(configtool_keepalive_time < 0xFFFF) 	configtool_keepalive_time++;
        else									configtool_keepalive_time = 0;
        
        if(configtool_keepalive_time >= CONFIGTOOL_KEEPALIVE_TIME_MS)
        {
            flag_send_configtool_keepalive = SEGCP_ENABLE;
            configtool_keepalive_time = 0;
        }
    }
}
