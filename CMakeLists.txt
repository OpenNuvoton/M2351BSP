cmake_minimum_required(VERSION 3.21)
cmake_policy(SET CMP0123 NEW)

# Create a symbolic link from ${base_name} in the binary directory
# to the corresponding path in the source directory.
function(link_to_source base_name)
    # Get OS dependent path to use in `execute_process`
    if(CMAKE_HOST_WIN32)
        # mklink is an internal command of cmd.exe it can only work with \
        string(REPLACE "/" "\\" link "${CMAKE_CURRENT_BINARY_DIR}/${base_name}")
        string(REPLACE "/" "\\" target "${CMAKE_CURRENT_SOURCE_DIR}/${base_name}")
    else()
        set(link "${CMAKE_CURRENT_BINARY_DIR}/${base_name}")
        set(target "${CMAKE_CURRENT_SOURCE_DIR}/${base_name}")
    endif()

    if(NOT EXISTS ${link})
        if(CMAKE_HOST_UNIX)
            set(command ln -s ${target} ${link})
        else()
            if(IS_DIRECTORY ${target})
                set(command cmd.exe /c mklink /j ${link} ${target})
            else()
                set(command cmd.exe /c mklink /h ${link} ${target})
            endif()
        endif()

        execute_process(COMMAND ${command}
            RESULT_VARIABLE result
            ERROR_VARIABLE output)

        if(NOT ${result} EQUAL 0)
            message(FATAL_ERROR "Could not create symbolic link for: ${target} --> ${output}")
        endif()
    endif()
endfunction(link_to_source)

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION "5")
set(CMAKE_SYSTEM_PROCESSOR ARM)

set(TOOLCHAIN_PATH C:/Keil_v5/ARM/ARMCLANG)
find_program(ARM_CC NAMES armclang PATHS ${TOOLCHAIN_PATH}/bin REQUIRED NO_DEFAULT_PATH)
find_program(ARM_CXX NAMES armclang PATHS ${TOOLCHAIN_PATH}/bin REQUIRED NO_DEFAULT_PATH)
find_program(ARM_ASM NAMES armasm PATHS ${TOOLCHAIN_PATH}/bin REQUIRED NO_DEFAULT_PATH)
find_program(ARM_AR NAMES armar PATHS ${TOOLCHAIN_PATH}/bin REQUIRED NO_DEFAULT_PATH)
find_program(ARM_LINK NAMES armlink PATHS ${TOOLCHAIN_PATH}/bin REQUIRED NO_DEFAULT_PATH)
find_program(ARM_FROMELF NAMES fromelf PATHS ${TOOLCHAIN_PATH}/bin REQUIRED NO_DEFAULT_PATH)

set(CMAKE_C_COMPILER ${ARM_CC} CACHE STRING "")
set(CMAKE_CXX_COMPILER ${ARM_CXX} CACHE STRING "")
set(CMAKE_ASM_COMPILER ${ARM_ASM} CACHE STRING "")
set(CMAKE_LINKER ${ARM_LINK} CACHE STRING "")

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
set(CMAKE_FIND_ROOT_PATH ${TOOLCHAIN_PATH})

set(LINK_MCU_CORE "Cortex-M23")

set(C_FLAGS "--target=arm-arm-none-eabi -mcpu=${LINK_MCU_CORE} -c -fno-rtti -funsigned-char -D__MICROLIB -gdwarf-3 -O2 -fno-function-sections -Wno-packed -Wno-missing-variable-declarations -Wno-missing-prototypes -Wno-missing-noreturn -Wno-sign-conversion -Wno-nonportable-include-path -Wno-reserved-id-macro -Wno-unused-macros -Wno-documentation-unknown-command -Wno-documentation -Wno-license-management -Wno-parentheses-equality -D__MICROLIB -DMBEDTLS_CONFIG_FILE=mbedtls_config.h -Ic:/Keil_v5/ARM/ARMCLANG/include")
set(CMAKE_C_FLAGS "-xc -std=c99  ${C_FLAGS}")
set(CMAKE_CXX_FLAGS ${C_FLAGS})
set(CMAKE_ASM_FLAGS "--cpu Cortex-M23 -g --16 --apcs=interwork --pd \"__MICROLIB SETA 1\"")
set(CMAKE_EXECUTABLE_SUFFIX ".axf")

project(m2351bsp)

function(create_project PRJ)
    add_executable(${PRJ} ${ARGN})
    target_link_libraries(${PRJ} system_startup standard_driver)

    target_link_options(${PRJ}
        PUBLIC
        --cpu=Cortex-M23
        --library_type=microlib
        --ro-base 0x00000000
        --rw-base 0x20000000
        --entry Reset_Handler
        --first __Vectors
        --strict
        --map
        --datacompressor=off
        --info=inline
        --entry Reset_Handler
        --summary_stderr
        --info totals
        --map
        --load_addr_map_info
        --xref
        --callgraph
        --symbols
    )

    add_custom_command(TARGET ${PRJ} POST_BUILD
        COMMAND ${ARM_FROMELF} --bin --output ${PRJ}.bin ${PRJ}.elf
        VERBATIM
    )
endfunction()

set(BSP_DIR ${CMAKE_CURRENT_LIST_DIR})

include_directories(${TOOLCHAIN_PATH}/include)
include_directories(${BSP_DIR}/Library/CMSIS/Include)
include_directories(${BSP_DIR}/Library/Device/Nuvoton/M2351/Include)
include_directories(${BSP_DIR}/Library/StdDriver/inc)
include_directories(${BSP_DIR}/Library/SmartcardLib/Include)

# Build library
add_subdirectory(${BSP_DIR}/Library/StdDriver/src)
add_subdirectory(${BSP_DIR}/Library/Device/Nuvoton/M2351/Source)

# Build sample code
add_subdirectory(${BSP_DIR}/SampleCode/StdDriver/CRYPTO_ECC_Demo)
