set(PRJ freertos_blinky_ns_secure)
add_executable(${PRJ} main.c)

target_link_libraries(${PRJ} system_startup_secure standard_driver_trustzone)

target_compile_options(${PRJ}
    PUBLIC
    -mcmse
    -gdwarf-3
)

target_compile_options(system_startup_secure
    PUBLIC
    -gdwarf-3
)
target_compile_options(standard_driver_trustzone
    PUBLIC
    -gdwarf-3
)

target_include_directories(system_startup_secure
    BEFORE
    PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/..
)
target_include_directories(standard_driver_trustzone
    BEFORE
    PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/..
)

target_include_directories(${PRJ}
    BEFORE
    PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/..
)

target_link_options(${PRJ}
    PUBLIC
    -T${CMAKE_CURRENT_SOURCE_DIR}/GCC/secure.ld
    -Wl,--cmse-implib,--out-implib=nsclib.o
    -Wl,-Map,${PRJ}.map
)

if(C_COMPILER_ID STREQUAL ARMCLANG)
    add_custom_command(TARGET ${PRJ} POST_BUILD
        COMMAND ${ARM_FROMELF} --bin --output ${PRJ}.bin ${PRJ}.elf
        VERBATIM
    )
endif()

if(C_COMPILER_ID STREQUAL GCC)
    add_custom_command(TARGET ${PRJ} POST_BUILD
        COMMAND ${ARM_OBJCOPY} -O binary ${PRJ}.elf ${PRJ}.bin
        VERBATIM
    )
endif()
