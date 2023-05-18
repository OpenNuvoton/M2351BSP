FreeRTOS's tickless idle mode works by halting the periodic tick interrupt when the system is idle, i.e., when there are no tasks available to execute. The tick interrupt is then resumed with a corrective adjustment to the RTOS tick count value when needed. By disabling the tick interrupt, the microcontroller can conserve power by entering a deep sleep state until an interrupt is triggered, or a task needs to be transitioned to the Ready state by the RTOS kernel.

To activate the built-in tickless idle feature in FreeRTOS, you need to set the value of configUSE_TICKLESS_IDLE to 1 in the FreeRTOSConfig.h file.

Tickless Idle sample code demonostrate set configUSE_TICKLESS_IDLE to 1 in FreeRTOSConfig.h to enable tickless idle.
User can monitor HCLK by PD12 to check if the system enter deeper sleep (clock off).
In the sample code, a thread is created to print the tick count and delay ever 500ms.
PB1~5 are supported to wake up the system in this sample code.