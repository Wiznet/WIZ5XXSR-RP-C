#ifndef STORAGEHANDLER_H_
#define STORAGEHANDLER_H_

#include <stdint.h>

typedef enum {
    STORAGE_MAC,
    STORAGE_CONFIG, 
    STORAGE_APPBOOT,
    STORAGE_APPBANK,
    STORAGE_BINBANK,
    STORAGE_ROOTCA, 
    STORAGE_CLICA,
    STORAGE_PKEY
} teDATASTORAGE;



uint32_t read_storage(teDATASTORAGE stype, void *data, uint16_t size);
uint32_t write_storage(teDATASTORAGE stype, uint32_t addr, void *data, uint16_t size);
void erase_storage(teDATASTORAGE stype);

#endif /* STORAGEHANDLER_H_ */
