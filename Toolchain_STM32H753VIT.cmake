set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)
set(CMAKE_SYSTEM_PROCESSOR arm-none-eabi)

set(TOOLCHAIN_PREFIX                arm-none-eabi-)

set(CMAKE_C_COMPILER                ${TOOLCHAIN_PREFIX}gcc)
set(CMAKE_ASM_COMPILER              ${CMAKE_C_COMPILER})
set(CMAKE_CXX_COMPILER              ${TOOLCHAIN_PREFIX}g++)
set(CMAKE_LINKER                    ${TOOLCHAIN_PREFIX}g++)
set(CMAKE_OBJCOPY                   ${TOOLCHAIN_PREFIX}objcopy)
set(CMAKE_SIZE                      ${TOOLCHAIN_PREFIX}size)

set(CXX_STANDARD 11)
set(C_STANDARD 11)

set(CMAKE_EXECUTABLE_SUFFIX_ASM     ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_C       ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_CXX     ".elf")

set( ARCH_STM32 "-mcpu=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard")
set( FLAGS_STM32 "${ARCH_STM32} -ffunction-sections -fdata-sections -fstrict-volatile-bitfields" )

set( WARN_STM32 "-Wall -Werror=return-type" )

set(CMAKE_ASM_FLAGS 				"${FLAGS_STM32} ${WARN_STM32} -Wa,--warn -x assembler-with-cpp" CACHE STRING "" FORCE)
set(CMAKE_ASM_FLAGS_DEBUG 			"-Og -g3" CACHE STRING "" FORCE)
set(CMAKE_ASM_FLAGS_RELEASE 		"-Os" CACHE STRING "" FORCE)
set(CMAKE_ASM_FLAGS_RELWITHDEBINFO 	"-Os -g3" CACHE STRING "" FORCE)
set(CMAKE_ASM_FLAGS_MINSIZEREL 		"-Os" CACHE STRING "" FORCE)

set(CMAKE_C_FLAGS 					"${FLAGS_STM32} ${WARN_STM32} -fstack-usage" CACHE STRING "" FORCE)
set(CMAKE_C_FLAGS_DEBUG 			"-Og -g3" CACHE STRING "" FORCE)
set(CMAKE_C_FLAGS_RELEASE 			"-Os" CACHE STRING "" FORCE)
set(CMAKE_C_FLAGS_RELWITHDEBINFO 	"-Os -g3" CACHE STRING "" FORCE)
set(CMAKE_C_FLAGS_MINSIZEREL 		"-Os" CACHE STRING "" FORCE)

set(CMAKE_CXX_FLAGS 				"${FLAGS_STM32} ${WARN_STM32} -fstack-usage -fno-threadsafe-statics -fno-exceptions -fno-rtti -fno-use-cxa-atexit" CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS_DEBUG 			"-Og -g3" CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS_RELEASE 		"-Os" CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO 	"-Os -g3" CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS_MINSIZEREL 		"-Os" CACHE STRING "" FORCE)

set(CMAKE_C_LINK_FLAGS "${ARCH_STM32} -T\"${PROJECT_SOURCE_DIR}/STM32H753VITx_FLASH.ld\" --specs=nano.specs -Wl,-Map=${CMAKE_PROJECT_NAME}.map -Wl,--print-memory-usage" CACHE STRING "" FORCE)

# cmake does not know how to find the syscalls ST has in syscalls.c, so cannot check the compiler
set(CMAKE_C_COMPILER_WORKS   1)
set(CMAKE_CXX_COMPILER_WORKS 1)
