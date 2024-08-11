cmake_minimum_required(VERSION 3.10)
set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)

project(sm-1420-mfdd
	VERSION 0.1.0
	DESCRIPTION "Smart PDU"
	LANGUAGES ASM C CXX
)

add_library(stm32cubemxh7
    libs/STM32CubeH7/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_adc.c
    libs/STM32CubeH7/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_adc_ex.c
    libs/STM32CubeH7/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc.c
    libs/STM32CubeH7/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc_ex.c
    libs/STM32CubeH7/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_flash.c
    libs/STM32CubeH7/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_flash_ex.c
    libs/STM32CubeH7/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_gpio.c
    libs/STM32CubeH7/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_hsem.c
    libs/STM32CubeH7/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma.c
    libs/STM32CubeH7/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma_ex.c
    libs/STM32CubeH7/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_mdma.c
    libs/STM32CubeH7/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr.c
    libs/STM32CubeH7/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr_ex.c
    libs/STM32CubeH7/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_cortex.c
    libs/STM32CubeH7/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal.c
    libs/STM32CubeH7/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_i2c.c
    libs/STM32CubeH7/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_i2c_ex.c
    libs/STM32CubeH7/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_exti.c
    libs/STM32CubeH7/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dac.c
    libs/STM32CubeH7/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dac_ex.c
    libs/STM32CubeH7/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_tim.c
    libs/STM32CubeH7/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_tim_ex.c
    libs/STM32CubeH7/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_spi.c
    libs/STM32CubeH7/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_spi_ex.c
    libs/STM32CubeH7/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_uart.c
    libs/STM32CubeH7/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_uart_ex.c
    libs/STM32CubeH7/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pcd.c
    libs/STM32CubeH7/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pcd_ex.c
    libs/STM32CubeH7/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_usb.c
)

target_compile_definitions(stm32cubemxh7
INTERFACE 
	USE_HAL_DRIVER 
	STM32H753xx 
	STM32_THREAD_SAFE_STRATEGY=5
)

target_include_directories(stm32cubemxh7
PUBLIC
    libs/STM32CubeH7/Drivers/STM32H7xx_HAL_Driver/Inc
    libs/STM32CubeH7/Drivers/STM32H7xx_HAL_Driver/Inc/Legacy
    libs/STM32CubeH7/Middlewares/Third_Party/FreeRTOS/Source/include
    libs/STM32CubeH7/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2
    libs/STM32CubeH7/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F
    libs/STM32CubeH7/Drivers/CMSIS/Device/ST/STM32H7xx/Include
    libs/STM32CubeH7/Drivers/CMSIS/Include
)

add_executable(sm-1420-mfdd
	main.cpp
)
target_link_libraries(sm-1420-mfdd
	stm32cubemxh7
)
