/**
 * \author  Tilen Majerle
 * \email   tilen@majerle.eu
 * \website https://majerle.eu/projects/esp8266-at-commands-parser-for-embedded-systems
 * \ide     Keil uVision
 * \license GNU GPL v3
 * \brief   Low level, platform dependant, part for communicate with ESP module and platform.
 *  
\verbatim
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
\endverbatim
 */
#ifndef ESP_LL_H
#define ESP_LL_H

/* C++ detection */
#ifdef __cplusplus
extern "C" {
#endif

/**
 * \defgroup ESP_LL
 * \brief    Low level, platform dependant, part for communication with ESP module and platform.
 * \{
 *
 * This is a low-level part of ESP module library.
 *
 * It provides communication between ESP module and platform. There are some function, which needs to be implemented by user and with care.
 *
 * Everything is done in single function ESP_LL_Callback with command as parameter.
 * Each command has different param and result pointers which can be used to interact with device.
 *
 * Detailed description is available in \ref ESP_LL_Control_t enumeration.
 *
 * \par U(S)ART configuration
 *
 * ESP8266 module works with U(S)ART communication with device. For this purpose, library supposes init and send commands implemented in callback part.
 *
 * ESP stack module does not check for any incoming data from ESP8266 module to USART of your device.
 *
 * Most microcontrollers have USART RX capability, so when USART RX interrupt happens,
 * you should send this received byte to ESP8266 module using \ref ESP_DataReceived function to notify new incoming data.
 * Use interrupt handler routing to notify new data using previous mentioned function.
 *
 * \par SYSCALL
 *
 * In order to RTOS, you need system call implementation. 
 * A part of implementation is done in template esp8266_ll_template.c file to show on which part you have to response in order to get mutex to work.
 *
 * \par Time configuration
 *
 * ESP module part needs current time in milliseconds to take care of any possible timeouts on connections and similar things.
 *
 * You need to implement your own time source (systick interrupt, normal timer with interrupt capability, etc) to tell ESP stack current time.
 *
 * Use \ref ESP_TimeUpdate to notify ESP stack with new time.
 *
 * \par Dependencies
 *
\verbatim
 Platform based dependencies
\endverbatim
 */

/**
 * \defgroup LOWLEVEL
 * \brief    ESP Low-Level implementation including system calls for RTOS
 * \{
 */
    
#include "stdint.h"
#include "stdio.h"

/* Get config */
#include "esp8266_config.h"

#if ESP_RTOS
#include "cmsis_os.h"
#endif /* ESP_RTOS */
    
/**
 * \defgroup LOWLEVEL_Typedefs
 * \brief    ESP Low-Level
 * \{
 */

/**
 * \brief   Loe-level control enumeration with supported callbacks
 * 
 * This enumeration is used in \ref ESP_LL_Callback function with param and result parameters to function
 */
typedef enum _ESP_LL_Control_t {
    /**
     * \brief       Called to initialize low-level part of device, such as UART and GPIO configuration
     *
     * \param[in]   *param: Pointer to \ref ESP_LL_t structure with baudrate setup
     * \param[out]  *result: Pointer to \ref uint8_t variable with result. Set to 0 when OK, or non-zero on ERROR.
     */
    ESP_LL_Control_Init = 0x00,     /*!< Initialization control */
    
    /**
     * \brief       Called to send data to ESP device
     *
     * \param[in]   *param: Pointer to \ref ESP_LL_Send_t structure with data to send
     * \param[out]  *result: Pointer to \ref uint8_t variable with result. Set to 0 when OK, or non-zero on ERROR.
     */
    ESP_LL_Control_Send,            /*!< Send data control */
    
    /**
     * \brief       Called to set software RTS pin when necessary
     *
     * \param[in]   *param: Pointer to \ref uint8_t variable with RTS info. This parameter can be a value of \ref ESP_RTS_SET or \ref ESP_RTS_CLR macros
     * \param[out]  *result: Pointer to \ref uint8_t variable with result. Set to 0 when OK, or non-zero on ERROR.
     */
    ESP_LL_Control_SetRTS,          /*!< Set software RTS control */
    
    /**
     * \brief       Called to set reset pin when necessary
     *
     * \param[in]   *param: Pointer to \ref uint8_t variable with RESET info. This parameter can be a value of \ref ESP_RESET_SET or \ref ESP_RESET_CLR macros
     * \param[out]  *result: Pointer to \ref uint8_t variable with result. Set to 0 when OK, or non-zero on ERROR.
     */
    ESP_LL_Control_SetReset,        /*!< Set reset control */
    
    /**
     * \brief       Called to create system synchronization object on RTOS support
     *
     * \param[in]   *param: Pointer to \ref ESP_RTOS_SYNC_t variable with sync object
     * \param[out]  *result: Pointer to \ref uint8_t variable with result. Set to 0 when OK, or non-zero on ERROR.
     */
    ESP_LL_Control_SYS_Create,      /*!< Creates a synchronization object */
    
    /**
     * \brief       Called to delete system synchronization object on RTOS support
     *
     * \param[in]   *param: Pointer to \ref ESP_RTOS_SYNC_t variable with sync object
     * \param[out]  *result: Pointer to \ref uint8_t variable with result. Set to 0 when OK, or non-zero on ERROR.
     */
    ESP_LL_Control_SYS_Delete,      /*!< Deletes a synchronization object */
    
    /**
     * \brief       Called to grant access to sync object
     *
     * \param[in]   *param: Pointer to \ref ESP_RTOS_SYNC_t variable with sync object
     * \param[out]  *result: Pointer to \ref uint8_t variable with result. Set to 0 when OK, or non-zero on ERROR.
     */
    ESP_LL_Control_SYS_Request,     /*!< Requests grant for specific sync object */
    
    /**
     * \brief       Called to release access from sync object
     *
     * \param[in]   *param: Pointer to \ref ESP_RTOS_SYNC_t variable with sync object
     * \param[out]  *result: Pointer to \ref uint8_t variable with result. Set to 0 when OK, or non-zero on ERROR.
     */
    ESP_LL_Control_SYS_Release,     /*!< Releases grant for specific sync object */
} ESP_LL_Control_t;

/**
 * \brief   Structure for sending data to low-level part
 */
typedef struct _ESP_LL_Send_t {
    const uint8_t* Data;            /*!< Pointer to data to send */
    uint16_t Count;                 /*!< Number of bytes to send */
    uint8_t Result;                 /*!< Result of last send */
} ESP_LL_Send_t;

/**
 * \brief   Low level structure for driver
 * \note    For now it has basic settings only without hardware flow control.
 */
typedef struct _ESP_LL_t {
    uint32_t Baudrate;          /*!< Baudrate to be used for UART */
} ESP_LL_t;
    
/**
 * \}
 */
    
/* Include library */
#include "esp8266.h"
    
/**
 * \defgroup LOWLEVEL_Macros
 * \brief    ESP Low-Level macros
 * \{
 */

#define ESP_RTS_SET         1   /*!< RTS should be set high */
#define ESP_RTS_CLR         0   /*!< RTS should be set low */
#define ESP_RESET_SET       1   /*!< Reset pin should be set */
#define ESP_RESET_CLR       0   /*!< Reset pin should be cleared */
    
/**
 * \}
 */

/**
 * \defgroup    LOWLEVEL_Functions
 * \brief       ESP Low-Level implementation
 * \{
 */

/**
 * \brief       Low-level callback for interaction with device specific section
 * \param[in]   ctrl: Control to be done
 * \param[in]   *param: Pointer to parameter setup, depends on control type
 * \param[out]  *result: Optional result parameter in case of commands
 * \retval      1: Control command has been processed
 * \retval      0: Control command has not been processed
 */
uint8_t ESP_LL_Callback(ESP_LL_Control_t ctrl, void* param, void* result);
    
/**
 * \}
 */

/**
 * \}
 */

/* C++ detection */
#ifdef __cplusplus
}
#endif

#endif
