set(FREERTOS_SRC
    ./Source/timers.c
    ./Source/list.c
    ./Source/queue.c
    ./Source/tasks.c
    ./Source/portable/MemMang/heap_4.c
    ./Source/portable/GCC/ARM_CM0/port.c

    ./Demo/Common/Minimal/dynamic.c
    ./Demo/Common/Minimal/recmutex.c
    ./Demo/Common/Minimal/blocktim.c
    ./Demo/Common/Minimal/countsem.c
    ./Demo/Common/Minimal/QueueSet.c
    ./Demo/Common/Minimal/QueueOverwrite.c
)

add_library(libfreertos ${FREERTOS_SRC})
target_include_directories(libfreertos
    PUBLIC
    ./Source/include
    ./Source/portable/GCC/ARM_CM0
    ./Demo/Common/include
)
