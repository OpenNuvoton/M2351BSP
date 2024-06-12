enable_language(C ASM)
add_library(system_startup
    system_m2351.c
    $<$<STREQUAL:${C_COMPILER_ID},ARMCLANG>:ARM/startup_m2351.s>
    $<$<STREQUAL:${C_COMPILER_ID},GCC>:GCC/_syscalls.c>
    $<$<STREQUAL:${C_COMPILER_ID},GCC>:GCC/startup_M2351.S>
)

add_library(system_startup_secure
    system_m2351.c
    $<$<STREQUAL:${C_COMPILER_ID},ARMCLANG>:ARM/startup_m2351.s>
    $<$<STREQUAL:${C_COMPILER_ID},GCC>:GCC/_syscalls.c>
    $<$<STREQUAL:${C_COMPILER_ID},GCC>:GCC/startup_M2351.S>
)
target_compile_options(system_startup_secure
    PUBLIC
    -mcmse
)
