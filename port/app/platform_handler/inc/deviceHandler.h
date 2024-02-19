#ifndef DEVICEHANDLER_H_
#define DEVICEHANDLER_H_

#include <stdint.h>
#include "WIZ5XXSR-RP_board.h"
#include "storageHandler.h"
#include "port_common.h"

/* Debug message enable */
#define _FWUP_DEBUG_

/* Application Port */
#define DEVICE_SEGCP_PORT     50001 // Search / Setting Port (UDP Broadcast / TCP unicast)
#define DEVICE_FWUP_PORT      50002 // Firmware Update Port
#define DEVICE_DDNS_PORT      3030  // Not  used

#define FLASH_SIZE                0x00200000
#define FLASH_BOOTLOADER_SIZE     0x80000
#define FLASH_PARAMETER_SIZE      0x10000
#define FLASH_APP_BANK_SIZE       0x80000


#define FLASH_START_ADDR_BANK0_OFFSET    FLASH_BOOTLOADER_SIZE
#define FLASH_START_ADDR_BANK1_OFFSET    FLASH_START_ADDR_BANK0_OFFSET + FLASH_APP_BANK_SIZE

#define FLASH_START_ADDR_BANK0    XIP_BASE + FLASH_BOOTLOADER_SIZE
#define FLASH_START_ADDR_BANK1    FLASH_START_ADDR_BANK0 + FLASH_APP_BANK_SIZE

#define FLASH_END_ADDR (XIP_BASE + FLASH_SIZE - 1)

#define FLASH_DEV_INFO_ADDR FLASH_START_ADDR_BANK1_OFFSET + FLASH_APP_BANK_SIZE
#define FLASH_ROOTCA_ADDR   FLASH_DEV_INFO_ADDR + 0x1000
#define FLASH_CLICA_ADDR    FLASH_ROOTCA_ADDR + 0x1000
#define FLASH_PRIKEY_ADDR   FLASH_CLICA_ADDR + 0x1000
#define FLASH_MAC_ADDR      FLASH_PRIKEY_ADDR + 0x1000

#define DEVICE_APP_SIZE         FLASH_APP_BANK_SIZE

#define DEVICE_BOOT_ADDR          (0)
#define DEVICE_APP_MAIN_ADDR      (DEVICE_BOOT_ADDR + DEVICE_BOOT_SIZE)
#define DEVICE_APP_BACKUP_ADDR    (DEVICE_APP_MAIN_ADDR + DEVICE_APP_SIZE)
#define DEVICE_CONFIG_ADDR        (FLASH_DEV_INFO_ADDR)
#define DEVICE_MAC_ADDR           (FLASH_MAC_ADDR)


/* Defines for firmware update */
#define DEVICE_FWUP_SIZE        DEVICE_APP_SIZE // Firmware size - 50kB MAX
#define DEVICE_FWUP_TIMEOUT     20000 // 20 secs.

#define DEVICE_FWUP_RET_SUCCESS   0x80
#define DEVICE_FWUP_RET_FAILED    0x40
#define DEVICE_FWUP_RET_PROGRESS  0x20
#define DEVICE_FWUP_RET_NONE      0x00

#define DEVICE_WDT_TIMEOUT    30000 // 30 secs

void device_set_factory_default(void);
void device_socket_termination(void);
void device_reboot(void);
void device_raw_reboot(void);
int device_bank_check(uint8_t bank_num);
int device_bank_copy(void);

uint8_t device_firmware_update(teDATASTORAGE stype); // Firmware update by Configuration tool / Flash to Flash

// function for timer
void device_timer_msec(void);

#endif /* DEVICEHANDLER_H_ */
