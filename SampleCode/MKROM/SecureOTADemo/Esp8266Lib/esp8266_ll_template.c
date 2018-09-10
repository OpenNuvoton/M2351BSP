/**	
   ----------------------------------------------------------------------
    Copyright (c) 2016 Tilen Majerle

    Permission is hereby granted, free of charge, to any person
    obtaining a copy of this software and associated documentation
    files (the "Software"), to deal in the Software without restriction,
    including without limitation the rights to use, copy, modify, merge,
    publish, distribute, sublicense, and/or sell copies of the Software, 
    and to permit persons to whom the Software is furnished to do so, 
    subject to the following conditions:

    The above copyright notice and this permission notice shall be
    included in all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
    EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
    OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE
    AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
    HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
    WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
    FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
    OTHER DEALINGS IN THE SOFTWARE.
   ----------------------------------------------------------------------
 */
#include "esp8266_ll.h"

/******************************************************************************/
/******************************************************************************/
/***   Copy this file to project directory and rename it to "esp8266_ll.c"   **/
/******************************************************************************/
/******************************************************************************/


#if ESP_RTOS
osMutexId id;
#endif /* ESP_RTOS */

uint8_t ESP_LL_Callback(ESP_LL_Control_t ctrl, void* param, void* result) {
    switch (ctrl) {
        case ESP_LL_Control_Init: {                 /* Initialize low-level part of communication */
            ESP_LL_t* LL = (ESP_LL_t *)param;       /* Get low-level value from callback */
            
            /************************************/
            /*  Device specific initialization  */
            /************************************/

            
            if (result) {
                *(uint8_t *)result = 0;             /* Successfully initialized */
            }
            return 1;                               /* Return 1 = command was processed */
        }
        case ESP_LL_Control_Send: {
            ESP_LL_Send_t* send = (ESP_LL_Send_t *)param;   /* Get send parameters */
            
            /* Send actual data to UART, implement function to send data */
        	send(send->Data, send->Count);
            
            if (result) {
                *(uint8_t *)result = 0;             /* Successfully send */
            }
            return 1;                               /* Command processed */
        }
        case ESP_LL_Control_SetReset: {             /* Set reset value */
            uint8_t state = *(uint8_t *)param;      /* Get state packed in uint8_t variable */
            if (state == ESP_RESET_SET) {           /* Check state value */

            } else {

            }
            return 1;                               /* Command has been processed */
        }
        case ESP_LL_Control_SetRTS: {               /* Set RTS value */
            uint8_t state = *(uint8_t *)param;      /* Get state packed in uint8_t variable */
            (void)state;                            /* Prevent compiler warnings */
            return 1;                               /* Command has been processed */
        }
#if ESP_RTOS
        case ESP_LL_Control_SYS_Create: {           /* Create system synchronization object */
            ESP_RTOS_SYNC_t* Sync = (ESP_RTOS_SYNC_t *)param;   /* Get pointer to sync object */
            id = osMutexCreate(Sync);               /* Create mutex */
            
            if (result) {
                *(uint8_t *)result = id == NULL;    /*!< Set result value */
            }
            return 1;                               /* Command processed */
        }
        case ESP_LL_Control_SYS_Delete: {           /* Delete system synchronization object */
            ESP_RTOS_SYNC_t* Sync = (ESP_RTOS_SYNC_t *)param;   /* Get pointer to sync object */
            osMutexDelete(Sync);                    /* Delete mutex object */
            
            if (result) {
                *(uint8_t *)result = id == NULL;    /*!< Set result value */
            }
            return 1;                               /* Command processed */
        }
        case ESP_LL_Control_SYS_Request: {          /* Request system synchronization object */
            ESP_RTOS_SYNC_t* Sync = (ESP_RTOS_SYNC_t *)param;   /* Get pointer to sync object */
            (void)Sync;                             /* Prevent compiler warnings */
            
            *(uint8_t *)result = osMutexWait(id, 1000) == osOK ? 0 : 1; /* Set result according to response */
            return 1;                               /* Command processed */
        }
        case ESP_LL_Control_SYS_Release: {          /* Release system synchronization object */
            ESP_RTOS_SYNC_t* Sync = (ESP_RTOS_SYNC_t *)param;   /* Get pointer to sync object */
            (void)Sync;                             /* Prevent compiler warnings */
            
            *(uint8_t *)result = osMutexRelease(id) == osOK ? 0 : 1;    /* Set result according to response */
            return 1;                               /* Command processed */
        }
#endif /* ESP_RTOS */
        default: 
            return 0;
    }
}

/* UART receive interrupt handler */
void USART_RX_INTERRUPT_HANDLER_FUNCTION_NAME(void) {
	uint8_t ch;
	/* Get character from USART */
	
	
	/* Send received character to ESP stack */
	ESP_DataReceived(&ch, 1);
}
