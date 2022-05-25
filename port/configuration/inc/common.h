#ifndef _COMMON_H
#define _COMMON_H

#include <stdint.h>

//////////////////////////////////
// Product Version              //
//////////////////////////////////
/* Application Firmware Version */
#define MAJOR_VER               1
#define MINOR_VER               0
#define MAINTENANCE_VER         5

#define DEV_CONFIG_VER          103

//#define STR_VERSION_STATUS      "Develop" // or "Stable"
#define STR_VERSION_STATUS      "Stable"

//////////////////////////////////
// W5100S HW Socket Definition  //
//////////////////////////////////
// 0 ~ 3
#define SOCK_MAX_USED           4

#define SOCK_DATA               0
#define SOCK_CONFIG_UDP         1
#define SOCK_CONFIG_TCP         2
#define SOCK_DHCP               3
#define SOCK_DNS                3
#define SOCK_FWUPDATE           3
#define SOCK_NETBIOS            3
#define SOCK_NTP                3
//#define SOCK_MQTT               3

#define SEG_DATA0_SOCK          SOCK_DATA
#define SEG_DATA1_SOCK          SOCK_DATA // The 2-port application must be set the SEG_DATA1_SOCK socket number with the differently SEG_DATA0_SOCK
#define SEGCP_UDP_SOCK          SOCK_CONFIG_UDP
#define SEGCP_TCP_SOCK          SOCK_CONFIG_TCP

//////////////////////////////////
// Ethernet                     //
//////////////////////////////////
/* Buffer size */
#define DATA_BUF_SIZE           2048
#define CONFIG_BUF_SIZE         2048 //512
#define MQTT_BUF_SIZE                   2048

#define ROOTCA_BUF_SIZE         2048
#define CLICA_BUF_SIZE          2048
#define PKEY_BUF_SIZE           2048

//////////////////////////////////
// Available board list         //
//////////////////////////////////
//#define WIZ2000_MB              1
#define S2E_SSL                   1
#define WIZ5XXSR-RP               2
#define UNKNOWN_DEVICE          0xff

//////////////////////////////////
//        Clock Setting         //
//////////////////////////////////

#define PLL_SYS_KHZ             (133000UL)

//////////////////////////////////
// Defines                      //
//////////////////////////////////
// Defines for S2E Status
typedef enum{ST_BOOT, ST_OPEN, ST_CONNECT, ST_UPGRADE, ST_ATMODE, ST_UDP} teDEVSTATUS;   // for Device status

#define DEVICE_APPBOOT_MODE     ST_BOOT
#define DEVICE_APP_MODE         ST_OPEN

// Gateway / command mode
#define DEVICE_AT_MODE          0
#define DEVICE_GW_MODE          1

// Network operation mode
#define TCP_CLIENT_MODE         0
#define TCP_SERVER_MODE         1
#define TCP_MIXED_MODE          2
#define UDP_MODE                3
#define SSL_TCP_CLIENT_MODE  4
#define MQTT_CLIENT_MODE  5
#define MQTTS_CLIENT_MODE  6

#define MQTT_TIMEOUT_MS                 400     // unit: ms

//#define MODBUS_TCP_CLIENT_MODE  4   // TCP client (Master)
//#define MODBUS_TCP_SERVER_MODE  5   // TCP server (Slave)

#define MIXED_SERVER            0
#define MIXED_CLIENT            1

// On/Off Status
typedef enum
{
    OFF = 0,
    ON  = 1
} OnOff_State_Type;

// True/False Status
typedef enum
{
    FALSE = 0,
    TRUE  = 1
} TrueFalse_State_Type;

typedef enum
{
  DISABLE = 0,
  ENABLE = !DISABLE
} FunctionalState;

#define OP_COMMAND              0
#define OP_DATA                 1

#define RET_OK                  0
#define RET_NOK                 -1
#define RET_TIMEOUT             -2

#define STR_UART                "UART"
#define STR_ENABLED             "Enabled"
#define STR_DISABLED            "Disabled"
#define STR_BAR                 "=================================================="
#define STR_MODBUS_RTU          "ModbusRTU"
#define STR_MODBUS_ASCII        "ModbusASCII"
#define STR_MODBUS_TCP          "ModbusTCP"


#endif //_COMMON_H


