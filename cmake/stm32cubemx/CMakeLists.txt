cmake_minimum_required(VERSION 3.22)
# Enable CMake support for ASM and C languages
enable_language(C ASM)
# STM32CubeMX generated symbols (macros)
set(MX_Defines_Syms 
	UX_INCLUDE_USER_DEFINE_FILE 
	USE_HAL_DRIVER 
	STM32H562xx
    $<$<CONFIG:Debug>:DEBUG>
)

# STM32CubeMX generated include paths
set(MX_Include_Dirs
    ${CMAKE_CURRENT_SOURCE_DIR}/../../USBX/App
    ${CMAKE_CURRENT_SOURCE_DIR}/../../USBX/Target
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32H5xx_HAL_Driver/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32H5xx_HAL_Driver/Inc/Legacy
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/CMSIS/Device/ST/STM32H5xx/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/ST/usbx/common/core/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/ST/usbx/ports/generic/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/CMSIS/Include
)

# STM32CubeMX generated application sources
set(MX_Application_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/main.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/stm32h5xx_it.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/stm32h5xx_hal_msp.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/sysmem.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/syscalls.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../startup_stm32h562xx.s
)

# STM32 HAL/LL Drivers
set(STM32_Drivers_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/system_stm32h5xx.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_adc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_adc_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_dma.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_dma_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_rcc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_rcc_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_flash.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_flash_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_gpio.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_pwr.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_pwr_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_cortex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_exti.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_i2c.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_i2c_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_icache.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_tim.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_tim_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_uart.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_uart_ex.c
)

# Drivers Midllewares


set(USBX_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/ST/usbx/common/core/src/ux_system_tasks_run.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/ST/usbx/common/core/src/ux_utility_debug_callback_register.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/ST/usbx/common/core/src/ux_utility_debug_log.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/ST/usbx/common/core/src/ux_utility_delay_ms.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/ST/usbx/common/core/src/ux_utility_descriptor_pack.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/ST/usbx/common/core/src/ux_utility_descriptor_parse.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/ST/usbx/common/core/src/ux_utility_error_callback_register.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/ST/usbx/common/core/src/ux_utility_long_get.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/ST/usbx/common/core/src/ux_utility_long_get_big_endian.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/ST/usbx/common/core/src/ux_utility_long_put.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/ST/usbx/common/core/src/ux_utility_long_put_big_endian.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/ST/usbx/common/core/src/ux_utility_memory_allocate.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/ST/usbx/common/core/src/ux_utility_memory_allocate_add_safe.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/ST/usbx/common/core/src/ux_utility_memory_allocate_mulc_safe.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/ST/usbx/common/core/src/ux_utility_memory_allocate_mulv_safe.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/ST/usbx/common/core/src/ux_utility_memory_compare.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/ST/usbx/common/core/src/ux_utility_memory_copy.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/ST/usbx/common/core/src/ux_utility_memory_free.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/ST/usbx/common/core/src/ux_utility_memory_byte_pool_create.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/ST/usbx/common/core/src/ux_utility_memory_byte_pool_search.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/ST/usbx/common/core/src/ux_utility_memory_set.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/ST/usbx/common/core/src/ux_utility_pci_class_scan.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/ST/usbx/common/core/src/ux_utility_pci_read.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/ST/usbx/common/core/src/ux_utility_pci_write.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/ST/usbx/common/core/src/ux_utility_physical_address.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/ST/usbx/common/core/src/ux_utility_set_interrupt_handler.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/ST/usbx/common/core/src/ux_utility_short_get.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/ST/usbx/common/core/src/ux_utility_short_get_big_endian.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/ST/usbx/common/core/src/ux_utility_short_put.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/ST/usbx/common/core/src/ux_utility_short_put_big_endian.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/ST/usbx/common/core/src/ux_utility_string_length_check.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/ST/usbx/common/core/src/ux_utility_string_length_get.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/ST/usbx/common/core/src/ux_utility_string_to_unicode.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/ST/usbx/common/core/src/ux_utility_unicode_to_string.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/ST/usbx/common/core/src/ux_utility_virtual_address.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/ST/usbx/common/core/src/ux_system_error_handler.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/ST/usbx/common/core/src/ux_system_initialize.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/ST/usbx/common/core/src/ux_system_uninitialize.c
)

# Link directories setup
set(MX_LINK_DIRS

)
# Project static libraries
set(MX_LINK_LIBS 
    STM32_Drivers
    ${TOOLCHAIN_LINK_LIBRARIES}
    USBX	
)
# Interface library for includes and symbols
add_library(stm32cubemx INTERFACE)
target_include_directories(stm32cubemx INTERFACE ${MX_Include_Dirs})
target_compile_definitions(stm32cubemx INTERFACE ${MX_Defines_Syms})

# Create STM32_Drivers static library
add_library(STM32_Drivers OBJECT)
target_sources(STM32_Drivers PRIVATE ${STM32_Drivers_Src})
target_link_libraries(STM32_Drivers PUBLIC stm32cubemx)


# Create USBX static library
add_library(USBX OBJECT)
target_sources(USBX PRIVATE ${USBX_Src})
target_link_libraries(USBX PUBLIC stm32cubemx)

# Add STM32CubeMX generated application sources to the project
target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${MX_Application_Src})

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE ${MX_LINK_DIRS})

# Add libraries to the project
target_link_libraries(${CMAKE_PROJECT_NAME} ${MX_LINK_LIBS})

# Add the map file to the list of files to be removed with 'clean' target
set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES ADDITIONAL_CLEAN_FILES ${CMAKE_PROJECT_NAME}.map)

# Validate that STM32CubeMX code is compatible with C standard
if((CMAKE_C_STANDARD EQUAL 90) OR (CMAKE_C_STANDARD EQUAL 99))
    message(ERROR "Generated code requires C11 or higher")
endif()
