

set(PRJ CRYPTO_ECC_Demo)
add_executable(${PRJ} main.c)
add_library(libmkrom STATIC IMPORTED)
set_target_properties(libmkrom PROPERTIES IMPORTED_LOCATION  ${BSP_DIR}/Library/StdDriver/Src/MKROMLib_Keil.lib)
target_link_libraries(${PRJ} system_startup standard_driver libmkrom)


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
 
# Convert to binary and copy to target device
message("Convert elf to binary file and download to target board .... ")
add_custom_command(TARGET ${PRJ} POST_BUILD
    COMMAND ${ARM_FROMELF} --bin --output ${PRJ}.bin ${PRJ}.elf
    VERBATIM
)

