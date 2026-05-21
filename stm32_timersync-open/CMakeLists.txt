cmake_minimum_required(VERSION 3.16)

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

project(PWM C ASM)

set(CPU_FLAGS "-mcpu=cortex-m3 -mthumb")
set(CMAKE_C_FLAGS "${CPU_FLAGS} -O0 -g -Wall -fdata-sections -ffunction-sections")
set(CMAKE_ASM_FLAGS "${CPU_FLAGS} -x assembler-with-cpp")
set(CMAKE_EXE_LINKER_FLAGS "${CPU_FLAGS} -specs=nano.specs -specs=nosys.specs -Wl,--gc-sections -Wl,-Map=PWM.map")
set(LINKER_SCRIPT "${CMAKE_SOURCE_DIR}/stm32f103c8.ld")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -T${LINKER_SCRIPT}")

add_definitions(-DSTM32F10X_MD -DUSE_STDPERIPH_DRIVER)

include_directories(
    USER
    CORE
    SYSTEM/sys
    SYSTEM/delay
    SYSTEM/usart
    HARDWARE/LED
    HARDWARE/KEY
    HARDWARE/TIMER
    STM32F10x_FWLib/inc
)


set(SOURCES
    CORE/startup_stm32f103_gcc.s
    USER/main.c
    USER/stm32f10x_it.c
    USER/system_stm32f10x.c

    SYSTEM/sys/sys.c
    SYSTEM/delay/delay.c
    SYSTEM/usart/usart.c

    HARDWARE/LED/led.c
    HARDWARE/KEY/key.c
    HARDWARE/TIMER/timer.c

    STM32F10x_FWLib/src/stm32f10x_gpio.c
    STM32F10x_FWLib/src/stm32f10x_rcc.c
    STM32F10x_FWLib/src/stm32f10x_tim.c
    STM32F10x_FWLib/src/stm32f10x_usart.c
    STM32F10x_FWLib/src/misc.c
)

add_executable(${PROJECT_NAME}.elf ${SOURCES})

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary ${PROJECT_NAME}.elf ${PROJECT_NAME}.bin
    COMMAND ${CMAKE_OBJCOPY} -O ihex ${PROJECT_NAME}.elf ${PROJECT_NAME}.hex
    COMMENT "Generating ${PROJECT_NAME}.bin and ${PROJECT_NAME}.hex"
)
