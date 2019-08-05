/**************************************************************************//**
 * @file     SFLib.h
 * @version  V3.00
 * @brief    Secure Flash library header File
 *
 * @note
 * Copyright (C) 2016 Winbond Technology Ltd. All rights reserved.
 *****************************************************************************/
#ifndef __SFLIB_H__
#define __SFLIB_H__


/** @addtogroup LIBRARY Library
  @{
*/

/** @addtogroup SFLIB Secure Flash Library
  @{
*/


/** @addtogroup SFLIB_EXPORTED_CONSTANTS Secure Flash Library Exported Constants
  @{
*/

#ifdef __cplusplus
extern "C"
{
#endif


#define SFL_PAGE_SIZE   4096    /*!< Page Size of Erase is 4KB */

/*@}*/ /* end of group SFLIB_EXPORTED_CONSTANTS */


/** @addtogroup SFLIB_EXPORTED_STRUCTS Secure Flash Library Exported Structs
  @{
*/


/**
  * @brief  SFL_IF_T is enum for the SPI operation mode. It support Quad and Single mode to choice.
  */
typedef enum
{
    SFL_IF_SINGLE,              /*!< Set SPI of secure flash in single wire mode. */
    SFL_IF_QUAD,                /*!< Set SPI of secure flash in quard mode.       */
} SFL_IF_T;


/**
  * @brief  SFL_COMMAND_T is enum for the demond flash opeartion. It can use Read, Write and Erase opeartion for secure flash.
  */
typedef enum
{
    SFL_CMD_READ   = 0x1,       /*!< Read command for secure flash                 */
    SFL_CMD_WRITE  = 0x2,       /*!< Write command for secure flash                */
    SFL_CMD_ERASE  = 0x3,       /*!< Erease command for secure flash               */
} SFL_CMD_T;

/*@}*/ /* end of group SFLIB_EXPORTED_STRUCTS */

/** @addtogroup SFLIB_EXPORTED_FUNCTIONS Secure Flash Library Exported Functions
  @{
*/

/**
  * @brief      Initial Secure Flash Library
  *
  * @retval 0       Successful
  * @retval Others  Failed
  *
  * @details    This function is used to initial Secure Flash Library. 
  *             It must be called before using other library APIs.
  */
int32_t SFL_Init(void);

/**
  * @brief  Initial secure flash connect session.
  *
  * @retval 0       Successful
  * @retval Others  Failed
  *
  * @details This function must be executed first before opening sessions.   
  *             
  */
int32_t SFL_SessionInit(void);

/**
  * @brief  Operation function to perform read/write/erase secure flash.
  *
  * @retval 0       Successful
  * @retval Others  Failed
  *
  * @details This function is used to read/write/erase secure flash.
  *          User must open session first before using this function.
  *             
  */
int32_t SFL_Exec(SFL_CMD_T command, uint8_t *dataBuffer,uint32_t address,uint32_t BufferSize);

/**
  * @brief  To get secure flash XOM library version.
  *
  * @retval 0       Successful
  * @retval Others  Failed
  *
  * @details This function is used to get the version number of secure flash XOM library.
  *          Ver[31:16] = major
  *          Ver[15: 8] = minor
  *          Ver[ 7: 0] = revision
  */
uint32_t SFL_GetVersion(void);

/**
  * @brief  To get secure flash size
  *
  * @return Secure flash size
  *
  * @details This function is used to get secure flash maximum size in bytes.
  */
uint32_t SFL_GetFlashSize(void);



/**
  * @brief  To configure interface type (Single/Quad)
  * @param[in] intf   The mode of SPI interface for Secure Flash. It could be:
  *                   SFL_IF_SINGLE or SFL_IF_QUAD.
  *
  * @retval 0       Successful
  * @retval Others  Failed
  *
  * @details This routine configures SPI interface type. It could be set SPI in single mode or Quad mode.
  */
int32_t SFL_ConfigInterface(SFL_IF_T intf);



/**
  * @brief  Open a secure session to communicate with secure flash
  *
  * @retval 0       Successful
  * @retval Others  Failed
  *
  * @details This function is used to open a secure session channel to communicate with secure flash.
  *          User must open the secure session before accessing secure flash.
  */
int32_t SFL_SessionOpen(void);


/**
  * @brief  Close the communication session of secure flash.
  *
  * @retval 0       Successful
  * @retval Others  Failed
  *
  * @details This function is used to close the secure session channel with secure flash.
  */
int32_t SFL_SessionClose(void);




/**
  * @brief  Power down secure flash
  *
  * @retval 0       Successful
  * @retval Others  Failed
  *
  * @details This function is used to force secure flash into power down state.
  */
int32_t SFL_PowerDown(void);

/**
  * @brief  Wakeup secure flash
  *
  * @retval 0       Successful
  * @retval Others  Failed
  *
  * @details This function is used to wakeup secure flash into normal working mode.
  */
int32_t SFL_Wakeup(void);

#ifdef __cplusplus
}
#endif

#endif //__SFLIB_H__

/*@}*/ /* end of group SFLIB_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group SFLIB */

/*@}*/ /* end of group LIBRARY */

