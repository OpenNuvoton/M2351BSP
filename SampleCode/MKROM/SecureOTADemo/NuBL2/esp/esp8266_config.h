/**
 * \author  Tilen Majerle
 * \email   tilen@majerle.eu
 * \website http://esp8266at.com
 * \license MIT
 * \brief   Configuration part for ESP8266 library
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
#ifndef ESP_CONFIG_H
#define ESP_CONFIG_H 200

/**************************************************************************/
/**************************************************************************/
/**************************************************************************/
/*  Edit file name to esp8266_config.h and edit values for your platform  */
/**************************************************************************/
/**************************************************************************/
/**************************************************************************/

/**
 * \defgroup CONFIG
 * \brief    Configuration parameters for ESP8266 library
 * \{
 */

/**
 * \brief   USART input buffer size. This buffer stores data received from USART RX interrupt 
 *          before stack use them.
 *
 *          Size of buffer depends on speed of calling \ref ESP_Update function, ESP baudrate and
 *          max ESP receive string size.
 *
 * \note    When possible, buffer should be at least 256 bytes.
 */
#define ESP_BUFFER_SIZE                     256

/**
 * \brief   Enables (1) or disables (0) single connection mode
 *
 *          In this mode, only one connection is allowed and only client can be used.
 *          Server mode is disabled and can not be enabled by user.
 */
#define ESP_SINGLE_CONN                     0

/**
 * \brief   Connection data receive buffer size in units of bytes.
 *          
 *          Since there can only be one received data packet at a time,
 *          library uses single buffer for connection receive data.
 *          It can be anysize, but recommended at least 1460 bytes as this is normal packet size from ESP.
 *
 *          Keep in mind that ESP8266 can return up to 4*1460 bytes in single packet.
 *
 * \note    If buffer is smaller than bytes received in packet, multiple callback functions will be called to read all the received data.
 */
#define ESP_CONNBUFFER_SIZE                 1460

/**
 * \brief   Enables (1) or disables (0) single buffer for all connections together
 *
 *          This is useful for low ram devices but not useful when having server mode active
 *          In this case, data may be lost because of strange AT output from ESP8266 software.
 *
 * \note    When server mode is active, this feature should not be enabled. In this case, 
 *          each connection will have separater buffer of \ref ESP_CONNBUFFER_SIZE size in bytes.
 */
#define ESP_CONN_SINGLEBUFFER               0

/**
 * \brief  Enables (1) or disables (0) RTOS support for library.
 *
 *         When using RTOS, some additional configuration must be set, listed below.
 */
#define ESP_RTOS                            0

/**
 * \brief  RTOS sync object for mutex
 */
#define ESP_RTOS_SYNC_t                     osMutexDef_t

/**
 * \brief  Timeout in milliseconds for mutex to access API
 *
 * \note   Should be set to the larger possible timeout from ESP8266
 */
#define ESP_RTOS_TIMEOUT                    180000

/**
 * \brief  Async data processing enabled (1) or disabled (0)
 *
 * \note   This feature has sense when in non-RTOS mode and you wanna process income data async (in interrupt).
 * 
 *         When this feature is enabled, you HAVE TO do processing (\ref ESP_Update) in interrupt.
 */
#define ESP_ASYNC                           0

/**
 * \brief   Enables (1) or disables (0) CTS pin on ESP8266 and software RTS pin on microcontroller
 *          
 *          When option is enabled, MCU must implement RTS software output pin to disable ESP
 *          module to send data before they are read from buffer and processed by ESP stack.
 *          This prevents loosing data if you have low RAM memory and you need to do hard work with data,
 *          like saving data to SDCARD.
 * 
 *          Function to control software RTS output on MCU is called from function
 *          which adds data to internal buffer of ESP stack.
 *         
 *          When buffer is read from \ref ESP_Update function, RTS should be set low again.
 *
 * \note    Software RTS pin from microcontroller (any pin selected by user)
 *           must be connected to CTS pin on ESP8266.
 */
#define ESP_USE_CTS                         0


/**
 * \}
 */

#endif
