
#include <string.h>
#include "common.h"
#include "WIZ5XXSR-RP_board.h"
#include "flashHandler.h"
#include "deviceHandler.h"
#include "storageHandler.h"

uint32_t read_storage(teDATASTORAGE stype, void *data, uint16_t size)
{
    uint32_t ret_len;
    
    switch(stype)
    {
        case STORAGE_MAC:
            ret_len = read_flash(FLASH_MAC_ADDR, data, size);
            break;
        
        case STORAGE_CONFIG:
            ret_len = read_flash(FLASH_DEV_INFO_ADDR, data, size);
            break;
        default:
            break;
    }
    
    return ret_len;
}


uint32_t write_storage(teDATASTORAGE stype, uint32_t addr, void *data, uint16_t size)
{
    uint32_t ret_len;

    switch(stype)
    {
        case STORAGE_MAC:
            write_flash(FLASH_MAC_ADDR, data, size);
            break;
        
        case STORAGE_CONFIG:
            write_flash(FLASH_DEV_INFO_ADDR, data, size);
            break;
        
        case STORAGE_APPBOOT:
            ret_len = write_flash(addr, data, size);
            break;
            
        case STORAGE_ROOTCA:
            write_flash(FLASH_ROOTCA_ADDR, data, size);            
            break;
        
        case STORAGE_CLICA:
            write_flash(FLASH_CLICA_ADDR, data, size);
            break;
            
        case STORAGE_PKEY:
            write_flash(FLASH_PRIKEY_ADDR, data, size);
            break;            
        default:
            break;
    }
    
    return ret_len;
}

void erase_storage(teDATASTORAGE stype)
{
    uint16_t i;
    uint32_t address, working_address;
    
    uint8_t blocks = 0;
    uint32_t sectors = 0, remainder = 0;
    int ret;
    
    switch(stype)
    {
        case STORAGE_MAC:
            printf("can't erase MAC in f/w\r\n");
            break;
        
        case STORAGE_CONFIG:
            erase_flash_sector(FLASH_DEV_INFO_ADDR);
            break;
        
        case STORAGE_APPBOOT:
            address = DEVICE_BOOT_ADDR;
            break;

        case STORAGE_APPBANK:
            address = FLASH_START_ADDR_BANK0_OFFSET;
            break;
            
        case STORAGE_BINBANK:
            address = FLASH_START_ADDR_BANK1_OFFSET;
            break;

        case STORAGE_ROOTCA:
            erase_flash_sector(FLASH_ROOTCA_ADDR);
            break;
        case STORAGE_CLICA:
            erase_flash_sector(FLASH_CLICA_ADDR);
        default:
            break;
    }

    if((stype == STORAGE_APPBANK) || (stype == STORAGE_BINBANK))
    {
        working_address = address;
        sectors = FLASH_APP_BANK_SIZE / FLASH_SECTOR_SIZE;

        for(i = 0; i < sectors; i++)
        {
            ret = erase_flash_sector(working_address);
            working_address += FLASH_SECTOR_SIZE;
        }
        //working_address += (sectors * SECT_SIZE);
        PRT_INFO(" > STORAGE:ERASE_END:ADDR_RANGE - [0x%x ~ 0x%x]\r\n", address, working_address-1);
    }
}
