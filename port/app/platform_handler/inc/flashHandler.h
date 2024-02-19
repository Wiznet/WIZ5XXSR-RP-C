#ifndef __FLASHHANDLER_H__
#define __FLASHHANDLER_H__

#include <stdint.h>
//#define _FLASH_DEBUG_




/* WIZ5XXSR-RP Application memory map */
// RP2040
/*
 * Flash
 *  - Main flash size: 2Mbytes
 *  
 *
 Top Flash Memory address /-------------------------------------------\  0x10200000
                          |                                           |
                          |                 Parameters (64KB)         |
                          |-------------------------------------------|  0x101F0000
                          |                                           |
                          |                 Reserved (448KB)          |
                          |                                           |
                          |-------------------------------------------|  0x10180000
                          |                                           |
                          |                                           |
                          |               FW Bin Bank (512KB)         |
                          |                                           |
                          |                                           |
                          |-------------------------------------------|  0x10100000
                          |                                           |
                          |                                           |
                          |           Application Bank (512KB)        |
                          |                                           |
                          |                                           |
                          |-------------------------------------------|  0x10080000
    Page   1 (256B)       |                                           |
                          |                                           |
                          |              Bootloader (512KB)           |  
    Page   0 (256B)       |                                           |
                          |                                           |
                          \-------------------------------------------/  0x10000000
*/


static void flash_critical_section_lock(void);
static void flash_critical_section_unlock(void);
void flash_critical_section_init(void);
uint32_t write_flash(uint32_t addr, uint8_t * data, uint32_t data_len);
uint32_t read_flash(uint32_t addr, uint8_t *data, uint32_t data_len);
int8_t erase_flash_sector(uint32_t addr);
void Copy_Interrupt_VectorTable(uint32_t vtor);


#endif
