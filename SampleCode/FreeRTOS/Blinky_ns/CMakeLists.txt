# set Nonsecure project is dependent on Secure project
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/Secure)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/Nonsecure)
add_dependencies(freertos_blinky_ns_nonsecure freertos_blinky_ns_secure)
