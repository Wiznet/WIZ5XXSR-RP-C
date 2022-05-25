set(TARGET_NAME Boot_main)

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
        S2E_FILES
)

function(target_cl_options option)
    target_compile_options(${TARGET_NAME} PRIVATE ${option})
    target_link_options(${TARGET_NAME} PRIVATE ${option})
endfunction()


target_cl_options("-ffunction-sections")
target_cl_options("-fdata-sections")
target_link_options(${TARGET_NAME} PRIVATE "LINKER:--gc-sections")


pico_enable_stdio_usb(${TARGET_NAME} 0)
pico_enable_stdio_uart(${TARGET_NAME} 1)

pico_add_extra_outputs(${TARGET_NAME})
pico_set_binary_type(${TARGET_NAME} copy_to_ram)
set_target_properties(${TARGET_NAME} PROPERTIES COMPILE_FLAGS "-Wall")

pico_set_linker_script(${TARGET_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/${TARGET_NAME}.ld)
