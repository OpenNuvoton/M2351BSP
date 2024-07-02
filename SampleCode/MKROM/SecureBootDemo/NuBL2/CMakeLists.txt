enable_language(C ASM)
set(PRJ secure_boot_bl2)
add_executable(${PRJ}
    main.c
    VerifyNuBL3x.c
    ${CMAKE_CURRENT_SOURCE_DIR}/FwInfo/FwInfo.c
    $<$<STREQUAL:${C_COMPILER_ID},ARMCLANG>:${CMAKE_CURRENT_SOURCE_DIR}/keyinfo/NuBL3xKeyStorage.S>
    $<$<STREQUAL:${C_COMPILER_ID},GCC>:${CMAKE_CURRENT_SOURCE_DIR}/keyinfo/NuBL3xKeyStorage_gcc.S>
)

target_link_libraries(${PRJ} system_startup_secure standard_driver_trustzone mkrom)

target_compile_options(${PRJ}
    PUBLIC
    -Wno-address-of-packed-member
    -mcmse
    -gdwarf-3
)

target_include_directories(${PRJ}
    BEFORE
    PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
)

target_compile_definitions(${PRJ}
    PUBLIC
    -DBIN_NUBL32="${CMAKE_CURRENT_LIST_DIR}/KeyInfo/NuBL32PubKeyEncrypted.bin"
    -DBIN_NUBL32_HASH="${CMAKE_CURRENT_LIST_DIR}/KeyInfo/NuBL32PubKeyEncrypted.bin"
    -DBIN_NUBL33="${CMAKE_CURRENT_LIST_DIR}/KeyInfo/NuBL33PubKeyEncrypted.bin"
    -DBIN_NUBL33_HASH="${CMAKE_CURRENT_LIST_DIR}/KeyInfo/NuBL33PubKeyEncrypted.bin"
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
    ${CMAKE_CURRENT_LIST_DIR}
)
target_include_directories(standard_driver_trustzone
    BEFORE
    PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_options(${PRJ}
    PUBLIC
    -march=armv8-m.base
    -ffunction-sections
    -fdata-sections
    -Xlinker
    --gc-sections
    -T${CMAKE_CURRENT_SOURCE_DIR}/GCC/nubl2.ld
    -Wl,--cmse-implib,--out-implib=nsclib_NuBL2.a
    -Wl,-Map,${PRJ}.map
    -L${BSP_DIR}/Library/StdDriver/src
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
