set(TARGET_NAME App_main)

add_executable(${TARGET_NAME}
        ${TARGET_NAME}.c
        )

target_link_libraries(${TARGET_NAME} PRIVATE
        cmsis_core
        pico_stdlib
        pico_stdlib_headers
        hardware_adc
        hardware_dma
        hardware_flash
        hardware_structs
        hardware_spi
        ETHERNET_FILES
        IOLIBRARY_FILES
        MQTT_FILES
        MBEDTLS_FILES
        DHCP_FILES
        DNS_FILES
        TIMER_FILES
        APP_S2E_FILES
        )
set_target_properties(${TARGET_NAME} PROPERTIES COMPILE_FLAGS "-Wall")
pico_enable_stdio_usb(${TARGET_NAME} 0)
pico_enable_stdio_uart(${TARGET_NAME} 1)
pico_add_extra_outputs(${TARGET_NAME})



set(TARGET_NAME_LINKER ${TARGET_NAME}_linker)
add_executable(${TARGET_NAME_LINKER}
        ${TARGET_NAME}.c
        )
target_link_libraries(${TARGET_NAME_LINKER} PRIVATE
        cmsis_core
        pico_stdlib
        pico_stdlib_headers
        hardware_adc
        hardware_dma
        hardware_flash
        hardware_structs
        hardware_spi
        ETHERNET_FILES
        IOLIBRARY_FILES
        MQTT_FILES
        MBEDTLS_FILES
        DHCP_FILES
        DNS_FILES
        TIMER_FILES
        APP_S2E_FILES
        )
set_target_properties(${TARGET_NAME_LINKER} PROPERTIES COMPILE_FLAGS "-Wall")
pico_set_linker_script(${TARGET_NAME_LINKER} ${CMAKE_CURRENT_SOURCE_DIR}/${TARGET_NAME_LINKER}.ld)
pico_enable_stdio_usb(${TARGET_NAME_LINKER} 0)
pico_enable_stdio_uart(${TARGET_NAME_LINKER} 1)
pico_add_extra_outputs(${TARGET_NAME_LINKER})
