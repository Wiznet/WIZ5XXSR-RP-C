# ioLibrary_Driver
add_library(IOLIBRARY_FILES STATIC)

target_sources(IOLIBRARY_FILES PUBLIC
        ${PORT_DIR}/ioLibrary_Driver/src/w5x00_spi.c
        ${PORT_DIR}/ioLibrary_Driver/src/w5x00_gpio_irq.c
        )

target_include_directories(IOLIBRARY_FILES PUBLIC
        ${WIZNET_DIR}/Ethernet
        ${PORT_DIR}/ioLibrary_Driver/inc
        ${PORT_DIR}
        )

target_link_libraries(IOLIBRARY_FILES PRIVATE
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

# timer
add_library(TIMER_FILES STATIC)

target_sources(TIMER_FILES PUBLIC
        ${PORT_DIR}/timer/src/timer.c
        )

target_include_directories(TIMER_FILES PUBLIC
        ${PORT_DIR}/timer/inc
        )

target_link_libraries(TIMER_FILES PRIVATE
        pico_stdlib      
        )

# configuration        
add_library(MBEDTLS_FILES STATIC)

target_sources(MBEDTLS_FILES PUBLIC
        ${PORT_DIR}/mbedtls/src/SSLInterface.c
        ${PORT_DIR}/mbedtls/src/SSL_Random.c
        )

target_include_directories(MBEDTLS_FILES PUBLIC
        ${MBEDTLS_DIR}/include
        ${PORT_DIR}/mbedtls/inc
        ${PORT_DIR}
        )

target_link_libraries(MBEDTLS_FILES PRIVATE
        cmsis_core
        pico_stdlib
        hardware_adc
        hardware_clocks
        hardware_dma
        hardware_flash
        hardware_resets
        hardware_spi
        DNS_FILES
        DHCP_FILES
        ETHERNET_FILES
        IOLIBRARY_FILES
        S2E_FILES
        MQTT_FILES
        mbedcrypto
        mbedx509
        mbedtls
        )

# configuration        
add_library(S2E_FILES STATIC)

target_sources(S2E_FILES PUBLIC
        ${PORT_DIR}/board/src/WIZ5XXSR-RP_board.c
        
        ${PORT_DIR}/configuration/src/ConfigData.c
        ${PORT_DIR}/configuration/src/segcp.c
        ${PORT_DIR}/configuration/src/util.c

        ${PORT_DIR}/platform_handler/src/dhcp_cb.c
        ${PORT_DIR}/platform_handler/src/deviceHandler.c
        ${PORT_DIR}/platform_handler/src/dnsHandler.c
        ${PORT_DIR}/platform_handler/src/flashHandler.c
        ${PORT_DIR}/platform_handler/src/gpioHandler.c
        ${PORT_DIR}/platform_handler/src/storageHandler.c
        ${PORT_DIR}/platform_handler/src/timerHandler.c
        ${PORT_DIR}/platform_handler/src/uartHandler.c
        ${PORT_DIR}/platform_handler/src/mqttsHandler.c
        ${PORT_DIR}/serial_to_ethernet/src/seg.c
        )

target_include_directories(S2E_FILES PUBLIC
        ${PORT_DIR}/board/inc
        ${PORT_DIR}/configuration/inc
        ${PORT_DIR}/platform_handler/inc
        ${PORT_DIR}/serial_to_ethernet/inc
        ${PORT_DIR}
        )

target_link_libraries(S2E_FILES PRIVATE
        cmsis_core
        pico_stdlib
        hardware_adc
        hardware_clocks
        hardware_dma
        hardware_flash
        hardware_resets
        hardware_spi
        DNS_FILES
        DHCP_FILES
        ETHERNET_FILES
        IOLIBRARY_FILES
        MBEDTLS_FILES
        MQTT_FILES
        mbedcrypto
        mbedx509
        mbedtls
        )
