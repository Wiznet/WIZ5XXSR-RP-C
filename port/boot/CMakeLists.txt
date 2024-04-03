# ioLibrary_Driver
add_library(BOOT_IOLIBRARY_FILES STATIC)

target_sources(BOOT_IOLIBRARY_FILES PUBLIC
        ${BOOT_PORT_DIR}/ioLibrary_Driver/src/w5x00_spi.c
        ${BOOT_PORT_DIR}/ioLibrary_Driver/src/w5x00_gpio_irq.c
        )

target_include_directories(BOOT_IOLIBRARY_FILES PUBLIC
        ${WIZNET_DIR}/Ethernet
        ${BOOT_PORT_DIR}/ioLibrary_Driver/inc
        ${BOOT_PORT_DIR}
        )

target_link_libraries(BOOT_IOLIBRARY_FILES PRIVATE
        cmsis_core
        pico_stdlib
        hardware_adc
        hardware_flash
        hardware_resets
        hardware_spi
        hardware_dma
        hardware_clocks
        DHCP_FILES
        ETHERNET_FILES
        DNS_FILES
        )

# configuration        
add_library(BOOT_S2E_FILES STATIC)

target_sources(BOOT_S2E_FILES PUBLIC
        ${BOOT_PORT_DIR}/board/src/WIZ5XXSR-RP_board.c
        
        ${BOOT_PORT_DIR}/configuration/src/ConfigData.c
        ${BOOT_PORT_DIR}/configuration/src/segcp.c
        ${BOOT_PORT_DIR}/configuration/src/util.c

        ${BOOT_PORT_DIR}/platform_handler/src/dhcp_cb.c
        ${BOOT_PORT_DIR}/platform_handler/src/deviceHandler.c
        ${BOOT_PORT_DIR}/platform_handler/src/flashHandler.c
        ${BOOT_PORT_DIR}/platform_handler/src/gpioHandler.c
        ${BOOT_PORT_DIR}/platform_handler/src/storageHandler.c
        ${BOOT_PORT_DIR}/platform_handler/src/timerHandler.c
        ${BOOT_PORT_DIR}/platform_handler/src/uartHandler.c
        ${BOOT_PORT_DIR}/serial_to_ethernet/src/seg.c
        )

target_include_directories(BOOT_S2E_FILES PUBLIC
        ${BOOT_PORT_DIR}/board/inc
        ${BOOT_PORT_DIR}/configuration/inc
        ${BOOT_PORT_DIR}/platform_handler/inc
        ${BOOT_PORT_DIR}/serial_to_ethernet/inc
        ${BOOT_PORT_DIR}
        )

target_link_libraries(BOOT_S2E_FILES PRIVATE
        cmsis_core
        pico_stdlib
        hardware_adc
        hardware_clocks
        hardware_dma
        hardware_flash
        hardware_resets
        hardware_spi
        DHCP_FILES
        ETHERNET_FILES
        BOOT_IOLIBRARY_FILES
        mbedcrypto
        mbedx509
        )