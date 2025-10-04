cmake_minimum_required(VERSION 3.22)

project(stm32cubemx)
add_library(stm32cubemx INTERFACE)

# Enable CMake support for ASM and C languages
enable_language(C ASM)

target_compile_definitions(stm32cubemx INTERFACE 
	USE_HAL_DRIVER 
	STM32H563xx
    $<$<CONFIG:Debug>:DEBUG>
)

target_include_directories(stm32cubemx INTERFACE
    ../../Core/Inc
    ../../Drivers/STM32H5xx_HAL_Driver/Inc
    ../../Drivers/STM32H5xx_HAL_Driver/Inc/Legacy
    ../../Drivers/CMSIS/Device/ST/STM32H5xx/Include
    ../../Drivers/CMSIS/Include
    ../../Middlewares/Third_Party/FreeRTOS/Source/include/
    ../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM33_NTZ/non_secure/
    ../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/
    ../../Middlewares/Third_Party/CMSIS/RTOS2/Include/
)

target_sources(stm32cubemx INTERFACE
    ../../Core/Src/main.cpp
    ../../Core/Src/app_freertos.c
    ../../Core/Src/stm32h5xx_it.c
    ../../Core/Src/stm32h5xx_hal_msp.c
    ../../Core/Src/stm32h5xx_hal_timebase_tim.c
    ../../Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_tim.c
    ../../Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_tim_ex.c
    ../../Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_cortex.c
    ../../Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_dcache.c
    ../../Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_rcc.c
    ../../Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_rcc_ex.c
    ../../Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_flash.c
    ../../Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_flash_ex.c
    ../../Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_gpio.c
    ../../Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_dma.c
    ../../Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_dma_ex.c
    ../../Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_pwr.c
    ../../Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_pwr_ex.c
    ../../Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal.c
    ../../Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_exti.c
    ../../Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_fdcan.c
    ../../Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_hash.c
    ../../Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_icache.c
    ../../Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_xspi.c
    ../../Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_ll_dlyb.c
    ../../Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_rng.c
    ../../Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_rng_ex.c
    ../../Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_ll_rng.c
    ../../Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_uart.c
    ../../Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_uart_ex.c
    ../../Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_pcd.c
    ../../Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_pcd_ex.c
    ../../Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_ll_usb.c
    ../../Core/Src/system_stm32h5xx.c
    ../../Middlewares/Third_Party/FreeRTOS/Source/croutine.c
    ../../Middlewares/Third_Party/FreeRTOS/Source/event_groups.c
    ../../Middlewares/Third_Party/FreeRTOS/Source/list.c
    ../../Middlewares/Third_Party/FreeRTOS/Source/queue.c
    ../../Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.c
    ../../Middlewares/Third_Party/FreeRTOS/Source/tasks.c
    ../../Middlewares/Third_Party/FreeRTOS/Source/timers.c
    ../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM33_NTZ/non_secure/port.c
    ../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM33_NTZ/non_secure/portasm.c
    ../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/cmsis_os2.c
    ../../Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c
    ../../Core/Src/sysmem.c
    ../../startup_stm32h563xx.s
)

target_link_directories(stm32cubemx INTERFACE
)

target_link_libraries(stm32cubemx INTERFACE
)

# Validate that STM32CubeMX code is compatible with C standard
if(CMAKE_C_STANDARD LESS 11)
    message(ERROR "Generated code requires C11 or higher")
endif()


