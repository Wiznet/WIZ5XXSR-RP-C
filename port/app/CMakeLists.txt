# ioLibrary_Driver
add_library(APP_IOLIBRARY_FILES STATIC)

target_sources(APP_IOLIBRARY_FILES PUBLIC
        ${APP_PORT_DIR}/ioLibrary_Driver/src/w5x00_spi.c
        ${APP_PORT_DIR}/ioLibrary_Driver/src/w5x00_gpio_irq.c
        )

target_include_directories(APP_IOLIBRARY_FILES PUBLIC
        ${WIZNET_DIR}/Ethernet
        ${APP_PORT_DIR}/ioLibrary_Driver/inc
        ${APP_PORT_DIR}
        )

target_link_libraries(APP_IOLIBRARY_FILES PRIVATE
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
        ${APP_PORT_DIR}/timer/src/timer.c
        )

target_include_directories(TIMER_FILES PUBLIC
        ${APP_PORT_DIR}/timer/inc
        )

target_link_libraries(TIMER_FILES PRIVATE
        pico_stdlib      
        )

# configuration        
add_library(MBEDTLS_FILES STATIC)

target_sources(MBEDTLS_FILES PUBLIC
        ${APP_PORT_DIR}/mbedtls/src/SSLInterface.c
        ${APP_PORT_DIR}/mbedtls/src/SSL_Random.c
        )

target_include_directories(MBEDTLS_FILES PUBLIC
        ${MBEDTLS_DIR}/include
        ${APP_PORT_DIR}/mbedtls/inc
        ${APP_PORT_DIR}
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
        APP_IOLIBRARY_FILES
        APP_S2E_FILES
        MQTT_FILES
        MODBUS_FILES
        mbedcrypto
        mbedx509
        mbedtls
        )

add_library(MODBUS_FILES STATIC)

target_sources(MODBUS_FILES PUBLIC
        ${APP_PORT_DIR}/modbus/src/mb.c
        ${APP_PORT_DIR}/modbus/src/mbascii.c
        ${APP_PORT_DIR}/modbus/src/mbcrc.c
        ${APP_PORT_DIR}/modbus/src/mbrtu.c
        ${APP_PORT_DIR}/modbus/src/mbtcp.c
        ${APP_PORT_DIR}/modbus/src/mbtimer.c
        ${APP_PORT_DIR}/modbus/src/mbserial.c
        )
        
target_include_directories(MODBUS_FILES PUBLIC
        ${APP_PORT_DIR}/modbus/inc
        ${APP_PORT_DIR}
        )

target_link_libraries(MODBUS_FILES PRIVATE
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
        APP_IOLIBRARY_FILES
        APP_S2E_FILES
        MBEDTLS_FILES
        MQTT_FILES
        mbedcrypto
        mbedx509
        mbedtls
        )

# configuration        
add_library(APP_S2E_FILES STATIC)

target_sources(APP_S2E_FILES PUBLIC
        ${APP_PORT_DIR}/board/src/WIZ5XXSR-RP_board.c
        
        ${APP_PORT_DIR}/configuration/src/ConfigData.c
        ${APP_PORT_DIR}/configuration/src/segcp.c
        ${APP_PORT_DIR}/configuration/src/util.c

        ${APP_PORT_DIR}/platform_handler/src/dhcp_cb.c
        ${APP_PORT_DIR}/platform_handler/src/deviceHandler.c
        ${APP_PORT_DIR}/platform_handler/src/dnsHandler.c
        ${APP_PORT_DIR}/platform_handler/src/flashHandler.c
        ${APP_PORT_DIR}/platform_handler/src/gpioHandler.c
        ${APP_PORT_DIR}/platform_handler/src/storageHandler.c
        ${APP_PORT_DIR}/platform_handler/src/timerHandler.c
        ${APP_PORT_DIR}/platform_handler/src/uartHandler.c
        ${APP_PORT_DIR}/platform_handler/src/mqttsHandler.c
        ${APP_PORT_DIR}/serial_to_ethernet/src/seg.c
        )

target_include_directories(APP_S2E_FILES PUBLIC
        ${APP_PORT_DIR}/board/inc
        ${APP_PORT_DIR}/configuration/inc
        ${APP_PORT_DIR}/platform_handler/inc
        ${APP_PORT_DIR}/serial_to_ethernet/inc
        ${APP_PORT_DIR}
        )

target_link_libraries(APP_S2E_FILES PRIVATE
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
        APP_IOLIBRARY_FILES
        MBEDTLS_FILES
        MQTT_FILES
        MODBUS_FILES
        mbedcrypto
        mbedx509
        mbedtls
        )
