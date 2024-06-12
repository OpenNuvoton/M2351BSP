set(PRJ freertos_blinky_ns_nonsecure)
enable_language(C ASM)
add_executable(${PRJ} main.c main-blinky.c main-full.c ParTest_M2351.c RegTest_gcc.S)

target_link_libraries(${PRJ} system_startup standard_driver_trustzone libfreertos)
target_compile_options(system_startup
    PUBLIC
    -gdwarf-3
)
target_compile_options(standard_driver
    PUBLIC
    -gdwarf-3
)
target_include_directories(system_startup
    BEFORE
    PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
)
target_include_directories(standard_driver_trustzone
    BEFORE
    PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
)
target_include_directories(libfreertos
    BEFORE
    PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
)
target_compile_options(${PRJ}
    PUBLIC
    -gdwarf-3
)
target_compile_definitions(${PRJ}
    PUBLIC
    -DDEBUG_PORT=UART1_NS
    -DHeap_Size=0x3800
)
target_include_directories(${PRJ}
    BEFORE
    PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/..
)

target_link_options(${PRJ}
    PUBLIC
    -T${CMAKE_CURRENT_SOURCE_DIR}/GCC/nonsecure.ld
    -Wl,-Map,${PRJ}.map
    nsclib.o
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
