/*
 * ConfigData.h
 */

#ifndef __CONFIGDATA_H__
#define __CONFIGDATA_H__

#include <stdint.h>

#define DEVICE_NAME_SIZE        20
#define DEVICE_ALIAS_SIZE       40
#define DEVICE_GROUP_SIZE       40

#define DNS_DOMAIN_SIZE         128


#define MQTT_TOPIC_SIZE		    128
#define MQTT_USER_NAME_SIZE		128
#define MQTT_PASSWORD_SIZE		128
#define MQTT_CLIENT_ID_SIZE		128
// Safe Save
#define MAX_SAVE_RETRY              5
#define SAVE_INTERVAL_MS            1000

enum bank_num{
    APP_BANK0 = 0,
    APP_BANK1
};

struct __device_common {
    uint8_t fw_ver[3];          // Major Version . Minor Version . Maintenance Version
    uint8_t device_type[3];
    uint8_t device_name[DEVICE_NAME_SIZE];
    uint8_t device_mode;
} __attribute__((packed));

// todo: group settings
struct __config_common {
    uint16_t app_protocol;
    uint16_t packet_size;
    char pw_search[10];
} __attribute__((packed));

struct __network_common {
    uint8_t mac[6];
    uint8_t local_ip[4];
    uint8_t gateway[4];
    uint8_t subnet[4];
} __attribute__((packed));

struct __network_connection {
    uint8_t working_mode;           // TCP_CLIENT_MODE (0), TCP_SERVER_MODE (1), TCP_MIXED_MODE (2), UDP_MODE (3), SSL_TCP_CLIENT_MODE (4), MQTT_CLIENT_MODE (5), MQTTS_CLIENT_MODE (6)
    uint8_t working_state;          // Network operation state: BOOT(0), OPEN (1), CONNECT (2), UPGRADE (3), ATMODE (4)
    uint16_t local_port;
    uint16_t remote_port;
    uint8_t remote_ip[4];
    uint8_t fixed_local_port;
    uint8_t dns_use;
    char dns_domain_name[DNS_DOMAIN_SIZE];
} __attribute__((packed));

struct __network_option {
    uint8_t dhcp_use;
    uint8_t dns_server_ip[4];
    uint8_t tcp_rcr_val;
} __attribute__((packed));

struct __tcp_option {
    uint16_t inactivity;
    uint16_t reconnection;
    uint8_t keepalive_en;
    uint16_t keepalive_wait_time;
    uint16_t keepalive_retry_time;
    char pw_connect[10];
    uint8_t pw_connect_en;
} __attribute__((packed));

struct __serial_common {
    uint8_t uart_interface_cnt;
    uint8_t serial_debug_en;
} __attribute__((packed));

struct __serial_option {        // todo: [0] RS-232/TTL, [1] RS-422 [2] RS-485
    uint8_t uart_interface;     // UART interface; [0] TTL [1] RS-232 [2] RS-422 [3] RS-485, This value is determined at the initial routine of device.
    uint8_t protocol;           // Serial communication protocol; [0] None, [1] Modbus RTU, [2] Modbus ASCII
    uint8_t baud_rate;          // 0 ~ (enum)
    uint8_t data_bits;          // 7, 8, 9
    uint8_t parity;             // None, Odd, Even
    uint8_t stop_bits;          // 1, 1.5, 2
    uint8_t flow_control;       // None, RTS/CTS, XON/XOFF, RTS Only for RS422/485
    uint8_t dtr_en;             // DTR/DSR Enable, Pins for these signals are shared with [Connection status pins]
    uint8_t dsr_en;             // DTR/DSR Enable, Pins for these signals are shared with [Connection status pins]
} __attribute__((packed));

struct __serial_data_packing {
    uint16_t packing_time;              // 0~2048
    uint16_t packing_size;              // 0~2048
    uint8_t packing_delimiter[4];
    uint8_t packing_delimiter_length;   // 0~4
    uint8_t packing_data_appendix;      // 0~2
} __attribute__((packed));

struct __serial_command {
    uint8_t serial_command;         // Serial Command Mode enable
    uint8_t serial_trigger[3];      // Serial Command mode entry trigger code (default: +++)
    uint8_t serial_command_echo;    // Serial Command echoback enable
} __attribute__((packed));

struct __user_io_info {
    uint16_t user_io_enable;        // 0: Disable / 1: Enable
    uint16_t user_io_type;          // 0: Digital / 1: Analog
    uint16_t user_io_direction;     // 0: Input / 1: Output
    uint16_t user_io_status;        // Digital Output only`
} __attribute__((packed));

struct __firmware_update {
    uint8_t fwup_flag;
    uint16_t fwup_port;
    uint32_t fwup_size;
    uint8_t fwup_server_flag;
    uint16_t fwup_server_port;
    uint8_t fwup_copy_flag;
} __attribute__((packed));

#if 1
struct __device_option {
    uint8_t pw_setting_en;
    uint8_t pw_setting[10];
    uint8_t device_alias[DEVICE_ALIAS_SIZE];
    uint8_t device_group[DEVICE_GROUP_SIZE];
//    uint8_t ntp_domain_name[NTP_SERVER_DOMAIN_CNT][NTP_SERVER_DOMAIN_SIZE];
} __attribute__((packed));
#endif

struct __ssl_option {
    uint8_t root_ca_option;     //0: Verify_none / 1: Verify_option / 2: Verify_require
    uint8_t client_cert_enable;
    uint32_t rootca_len;
    uint32_t clica_len;
    uint32_t pkey_len;
    uint32_t recv_timeout;
} __attribute__((packed));

struct __mqtt_option {
    uint8_t pub_topic[MQTT_TOPIC_SIZE];
    uint8_t sub_topic_0[MQTT_TOPIC_SIZE];
    uint8_t sub_topic_1[MQTT_TOPIC_SIZE];
    uint8_t sub_topic_2[MQTT_TOPIC_SIZE];
    uint8_t user_name[MQTT_USER_NAME_SIZE];
    uint8_t client_id[MQTT_CLIENT_ID_SIZE];
    uint8_t password[MQTT_PASSWORD_SIZE];
    uint32_t keepalive;
    uint8_t qos;
} __attribute__((packed));


typedef struct __DevConfig {
    struct __device_common device_common;
    struct __config_common config_common;
    struct __network_common network_common;
    struct __network_connection network_connection;
    struct __network_option network_option;
    struct __tcp_option tcp_option;
    struct __serial_common serial_common;
    struct __serial_command serial_command;
    struct __serial_option serial_option;
    struct __serial_data_packing serial_data_packing;
    struct __user_io_info user_io_info;
    struct __firmware_update firmware_update;
    struct __ssl_option ssl_option;
    struct __mqtt_option mqtt_option;
    struct __device_option device_option;
    uint32_t devConfigVer;
} __attribute__((packed)) DevConfig;

DevConfig* get_DevConfig_pointer(void);
void set_DevConfig_to_factory_value(void);
void load_DevConfig_from_storage(void);
void save_DevConfig_to_storage(void);
void get_DevConfig_value(void *dest, const void *src, uint16_t size);
void set_DevConfig_value(void *dest, const void *value, const uint16_t size);

void display_Net_Info(void);
void Mac_Conf(void);
void Net_Conf(void);
void set_dhcp_mode(void);

uint8_t get_hex(uint8_t b0, uint8_t b1);
char atonum(char ch);

#endif /* __CONFIGDATA_H__ */
