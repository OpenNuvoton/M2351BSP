/*
    FreeRTOS V9.0.0 - Copyright (C) 2016 Real Time Engineers Ltd.
    All rights reserved

    VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation >>>> AND MODIFIED BY <<<< the FreeRTOS exception.

    ***************************************************************************
    >>!   NOTE: The modification to the GPL is included to allow you to     !<<
    >>!   distribute a combined work that includes FreeRTOS without being   !<<
    >>!   obliged to provide the source code for proprietary components     !<<
    >>!   outside of the FreeRTOS kernel.                                   !<<
    ***************************************************************************

    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE.  Full license text is available on the following
    link: http://www.freertos.org/a00114.html

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS provides completely free yet professionally developed,    *
     *    robust, strictly quality controlled, supported, and cross          *
     *    platform software that is more than just the market leader, it     *
     *    is the industry's de facto standard.                               *
     *                                                                       *
     *    Help yourself get started quickly while simultaneously helping     *
     *    to support the FreeRTOS project by purchasing a FreeRTOS           *
     *    tutorial book, reference manual, or both:                          *
     *    http://www.FreeRTOS.org/Documentation                              *
     *                                                                       *
    ***************************************************************************

    http://www.FreeRTOS.org/FAQHelp.html - Having a problem?  Start by reading
    the FAQ page "My application does not run, what could be wrong?".  Have you
    defined configASSERT()?

    http://www.FreeRTOS.org/support - In return for receiving this top quality
    embedded software for free we request you assist our global community by
    participating in the support forum.

    http://www.FreeRTOS.org/training - Investing in training allows your team to
    be as productive as possible as early as possible.  Now you can receive
    FreeRTOS training directly from Richard Barry, CEO of Real Time Engineers
    Ltd, and the world's leading authority on the world's leading RTOS.

    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool, a DOS
    compatible FAT file system, and our tiny thread aware UDP/IP stack.

    http://www.FreeRTOS.org/labs - Where new FreeRTOS products go to incubate.
    Come and try FreeRTOS+TCP, our new open source TCP/IP stack for FreeRTOS.

    http://www.OpenRTOS.com - Real Time Engineers ltd. license FreeRTOS to High
    Integrity Systems ltd. to sell under the OpenRTOS brand.  Low cost OpenRTOS
    licenses offer ticketed support, indemnification and commercial middleware.

    http://www.SafeRTOS.com - High Integrity Systems also provide a safety
    engineered and independently SIL3 certified version for use in safety and
    mission critical applications that require provable dependability.

    1 tab == 4 spaces!
*/

/**
 * Copyright (C) 2015-2017 Alibaba Group Holding Limited
 */
#include <string.h>
#include "FreeRTOS.h"
#include "semphr.h"
#include "queue.h"

void *osa_malloc(size_t size)
{
    return pvPortMalloc(size);
}

void osa_free(const void *ptr)
{
    vPortFree((void *)ptr);
}

void *osa_sem_init(uint32_t init_val)
{
	(void)init_val;
	return (void *)xSemaphoreCreateMutex();
}

void osa_sem_destroy(void *mutex)
{
	vSemaphoreDelete((QueueHandle_t)mutex);
}

uint32_t osa_sem_wait(void *mutex, int32_t msec)
{
	BaseType_t ret;
	ret = xSemaphoreTake((QueueHandle_t)mutex, msec/portTICK_PERIOD_MS);
	if (pdPASS == ret) {
		return 0;
	} else {
		return 0xFFFF0000;
	}
}

void osa_sem_release(void *mutex)
{
	xSemaphoreGive((QueueHandle_t)mutex);
}

char *osa_strcpy(char *dest, const char *src)
{
    return strcpy(dest, src);
}

char *osa_strncpy(char *dest, const char *src, size_t n)
{
    return strncpy(dest, src, n);
}

int32_t osa_strcmp(const char *s1, const char *s2)
{
    return strcmp(s1, s2);
}

int32_t osa_strncmp(const char *s1, const char *s2, size_t n)
{
    return strncmp(s1, s2, n);
}

size_t osa_strlen(const char *s)
{
    return strlen(s);
}

char *osa_strrchr(const char *s, int32_t c)
{
    return strrchr(s, c);
}

void *osa_memset(void *s, int32_t c, size_t n)
{
    return memset(s, c, n);
}

void *osa_memcpy(void *dest, const void *src, size_t n)
{
    return memcpy(dest, src, n);
}

int32_t osa_memcmp(const void *s1, const void *s2, size_t n)
{
    return memcmp(s1, s2, n);
}