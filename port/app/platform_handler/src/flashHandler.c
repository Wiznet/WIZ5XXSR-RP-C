#include <string.h>
#include "port_common.h"
#include "common.h"
#include "flashHandler.h"

#ifdef _FLASH_DEBUG_
	#include <stdio.h>
#endif

static uint8_t *flash_buf = NULL;
static critical_section_t g_flash_cri_sec;

static void flash_critical_section_lock(void)
{
    critical_section_enter_blocking(&g_flash_cri_sec);
}

static void flash_critical_section_unlock(void)
{
    critical_section_exit(&g_flash_cri_sec);
}

void flash_critical_section_init(void)
{
    critical_section_init(&g_flash_cri_sec);
}

uint32_t write_flash(uint32_t addr, uint8_t * data, uint32_t data_len)
{
    uint32_t i, access_len;

    flash_buf = malloc(FLASH_SECTOR_SIZE);

    for (i=0; i<data_len; i+=FLASH_SECTOR_SIZE)
    {
        memset(flash_buf, 0xFF, FLASH_SECTOR_SIZE);
        read_flash(addr + i, flash_buf, FLASH_SECTOR_SIZE);
        erase_flash_sector(addr + i);
        access_len = MIN(data_len - i, FLASH_SECTOR_SIZE);
        memcpy(flash_buf, data + i, access_len);
        flash_critical_section_lock();
        flash_range_program(addr + i, flash_buf, FLASH_SECTOR_SIZE);
        flash_critical_section_unlock();
    }

    free(flash_buf);
    return data_len;
}

uint32_t read_flash(uint32_t addr, uint8_t *data, uint32_t data_len)
{
    addr += XIP_BASE;
    memcpy(data, addr, data_len);

    return data_len;
}

int8_t erase_flash_sector(uint32_t addr)
{
    
    //PRT_FLASH("addr = 0x%X\r\n", addr);
    flash_critical_section_lock();
    flash_range_erase(addr, FLASH_SECTOR_SIZE);
    flash_critical_section_unlock();

    //PRT_FLASH("erase done addr = 0x%X\r\n", addr);
    
    return 1;
}

void Copy_Interrupt_VectorTable(uint32_t vtor)
{
	// Derived from the Lesaf Labs Cortex-M3 bootloader.
	// Copyright (c) 2010 LeafLabs LLC.
	// Modified 2021 Brian Starkey <stark3y@gmail.com>
	// Originally under The MIT License
	uint32_t reset_vector = *(volatile uint32_t *)(vtor + 0x04);
    __disable_irq();
	SCB->VTOR = (volatile uint32_t)(vtor);

	asm volatile("msr msp, %0"::"g"
			(*(volatile uint32_t *)vtor));
	asm volatile("bx %0"::"r" (reset_vector));

    __enable_irq(); 
}