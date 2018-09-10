/*********************************************************************
*                 SEGGER Software GmbH                               *
*        Solutions for real time microcontroller applications        *
**********************************************************************
*                                                                    *
*        (c) 1996 - 2018  SEGGER Microcontroller GmbH                *
*                                                                    *
*        Internet: www.segger.com    Support:  support@segger.com    *
*                                                                    *
**********************************************************************

** emWin V5.48 - Graphical user interface for embedded applications **
All  Intellectual Property rights in the Software belongs to  SEGGER.
emWin is protected by  international copyright laws.  Knowledge of the
source code may not be used to write a similar product. This file may
only be used in accordance with the following terms:

The  software has  been licensed by SEGGER Software GmbH to Nuvoton Technology Corporation
at the address: No. 4, Creation Rd. III, Hsinchu Science Park, Taiwan
for the purposes  of  creating  libraries  for its
Arm Cortex-M and  Arm9 32-bit microcontrollers, commercialized and distributed by Nuvoton Technology Corporation
under  the terms and conditions  of  an  End  User
License  Agreement  supplied  with  the libraries.
Full source code is available at: www.segger.com

We appreciate your understanding and fairness.
----------------------------------------------------------------------
Licensing information
Licensor:                 SEGGER Software GmbH
Licensed to:              Nuvoton Technology Corporation, No. 4, Creation Rd. III, Hsinchu Science Park, 30077 Hsinchu City, Taiwan
Licensed SEGGER software: emWin
License number:           GUI-00735
License model:            emWin License Agreement, signed February 27, 2018
Licensed platform:        Cortex-M and ARM9 32-bit series microcontroller designed and manufactured by or for Nuvoton Technology Corporation
----------------------------------------------------------------------
Support and Update Agreement (SUA)
SUA period:               2018-03-26 - 2019-03-27
Contact to extend SUA:    sales@segger.com
----------------------------------------------------------------------
File        : LCDConf.c
Purpose     : Display controller configuration (single layer)
---------------------------END-OF-HEADER------------------------------
*/

#include <stddef.h>
#include <stdio.h>

#include "GUI.h"
#include "GUIDRV_FlexColor.h"

#include "NuMicro.h"

#include "M2351TouchPanel.h"

#include "lcm.h"

//
// Hardware related
//
#define SPI_LCD_PORT  SPI1

#define GPIO_SPI1_SS PH9
#define GPIOPORT_SPI1_SS PH
#define PINMASK_SPI1_SS BIT9

#define GPIO_LCM_DC PA8
#define GPIOPORT_LCM_DC PA
#define PINMASK_LCM_DC BIT8

#define GPIO_LCM_RESET PA9
#define GPIOPORT_LCM_RESET PA
#define PINMASK_LCM_RESET BIT9

#define SPI_CS_SET    GPIO_SPI1_SS = 1
#define SPI_CS_CLR    GPIO_SPI1_SS = 0

#define LCM_DC_SET    GPIO_LCM_DC = 1
#define LCM_DC_CLR    GPIO_LCM_DC = 0

#define LCM_RESET_SET GPIO_LCM_RESET = 1
#define LCM_RESET_CLR GPIO_LCM_RESET = 0

/*********************************************************************
*
*       _Read1
*/
U8 _Read1(void)
{
#if 1
    /* FIXME if panel supports read back feature */
    return 0;
#else
    LCM_DC_SET;
    SPI_CS_CLR;
    SPI_WRITE_TX(SPI_LCD_PORT, 0x00);
    SPI_READ_RX(SPI_LCD_PORT);
    SPI_CS_SET;
    return (SPI_READ_RX(SPI_LCD_PORT));
#endif
}

/*********************************************************************
*
*       _ReadM1
*/
void _ReadM1(U8 * pData, int NumItems)
{
#if 1
    /* FIXME if panel supports read back feature */
#else
    LCM_DC_SET;
    SPI_CS_CLR;
    while(NumItems--)
    {
        SPI_WRITE_TX(SPI_LCD_PORT, 0x00);
        while(SPI_IS_BUSY(SPI_LCD_PORT));
        *pData++ = SPI_READ_RX(SPI_LCD_PORT);
    }
    SPI_CS_SET;
#endif
}

/*********************************************************************
*
*       _Write0
*/
void _Write0(U8 Cmd)
{
    LCM_DC_CLR;
    
    SPI_WRITE_TX(SPI_LCD_PORT, Cmd);
    while(SPI_IS_BUSY(SPI_LCD_PORT));
}

/*********************************************************************
*
*       _Write1
*/
void _Write1(U8 Data)
{
    LCM_DC_SET;
    
    SPI_WRITE_TX(SPI_LCD_PORT, Data);
    while(SPI_IS_BUSY(SPI_LCD_PORT));
}

/*********************************************************************
*
*       _WriteM1
*/
void _WriteM1(U8 * pData, int NumItems)
{
    LCM_DC_SET;
    while(NumItems--)
    {
        while(SPI_LCD_PORT->STATUS & SPI_STATUS_TXFULL_Msk);
        SPI_WRITE_TX(SPI_LCD_PORT, *pData++);
    }
}

static void _Open_SPI(void)
{
    GPIO_SetMode(GPIOPORT_LCM_DC, PINMASK_LCM_DC, GPIO_MODE_OUTPUT);
    GPIO_SetMode(GPIOPORT_LCM_RESET, PINMASK_LCM_RESET, GPIO_MODE_OUTPUT);
    GPIO_SetMode(GPIOPORT_SPI1_SS, PINMASK_SPI1_SS, GPIO_MODE_OUTPUT); //cs pin for gpiod

    /* Setup SPI1 multi-function pins */
    SYS->GPE_MFPL &= ~(SYS_GPE_MFPL_PE0MFP_Msk       | SYS_GPE_MFPL_PE1MFP_Msk);
    SYS->GPE_MFPL |=  (SYS_GPE_MFPL_PE0MFP_SPI1_MOSI | SYS_GPE_MFPL_PE1MFP_SPI1_MISO);
    
    SYS->GPH_MFPH &= ~(SYS_GPH_MFPH_PH8MFP_Msk | SYS_GPH_MFPH_PH9MFP_Msk);
    SYS->GPH_MFPH |=  (SYS_GPH_MFPH_PH8MFP_SPI1_CLK | SYS_GPH_MFPH_PH9MFP_SPI1_SS);

    /* Set IO to high slew rate */
    PE->SLEWCTL |= 3;
    PH->SLEWCTL |= (3 << 8);
    
    /* Enable SPI1 */
    CLK_EnableModuleClock(SPI1_MODULE);
    CLK_SetModuleClock(SPI1_MODULE, CLK_CLKSEL2_SPI1SEL_PCLK0, 0);

    SPI_Open(SPI_LCD_PORT, SPI_MASTER, SPI_MODE_0, 8, 32000000);
    
    /* Clear suspend interval */
    SPI_LCD_PORT->CTL &= (~SPI_CTL_SUSPITV_Msk);
    SPI_LCD_PORT->CTL |= (0 << SPI_CTL_SUSPITV_Pos);
    
    
    /* Disable auto SS function, control SS signal manually. */
    SPI_EnableAutoSS(SPI_LCD_PORT, SPI_SS, QSPI_SS_ACTIVE_LOW);
    SPI_ENABLE(SPI_LCD_PORT);
}

/*********************************************************************
*
*       _InitController
*
* Purpose:
*   Initializes the display controller
*/
void _InitController(void) {
    int i;
    static uint8_t s_InitOnce = 0;

    if(s_InitOnce == 0)
        s_InitOnce = 1;
    else
        return;

    _Open_SPI();

    LCM_RESET_SET;
    LCM_RESET_CLR;

    GUI_X_Delay(100);

    LCM_RESET_SET;

    GUI_X_Delay(120);
    _Write0(0x11);    //Exit Sleep
    GUI_X_Delay(120);
//------------------------------------------------------------------//
//-------------------Software Reset---------------------------------//

    _Write0(0xB1);
    _Write1(0x01);
    _Write1(0x2C);
    _Write1(0x2D);

    _Write0(0xB2);
    _Write1(0x01);
    _Write1(0x2C);
    _Write1(0x2D);

    _Write0(0xB3);
    _Write1(0x01);
    _Write1(0x2C);
    _Write1(0x2D);

    _Write1(0x01);
    _Write1(0x2C);
    _Write1(0x2D);


    _Write0(0xB4);  //Column inversion
    _Write1(0x07);
    //ST7735R Power Sequence
    _Write0(0xC0);
    _Write1(0xA2);
    _Write1(0x02);
    _Write1(0x84);
    _Write1(0xC1);
    _Write1(0xC5);
    _Write0(0xC2);
    _Write1(0x0A);
    _Write1(0x00);

    _Write0(0xC3);
    _Write1(0x8A);
    _Write1(0x2A);
    _Write1(0xC4);
    _Write1(0x8A);
    _Write1(0xEE);

    _Write0(0xC5); //VCOM
    _Write1(0x0E);

    _Write0(0x36); //MX, MY, RGB mode
    _Write1(0xC0);
    _Write1(0xC8); //??C8 ??08 A8

    //ST7735R Gamma Sequence
    _Write0(0xe0);
    _Write1(0x0f);
    _Write1(0x1a);
    _Write1(0x0f);
    _Write1(0x18);
    _Write1(0x2f);
    _Write1(0x28);
    _Write1(0x20);
    _Write1(0x22);
    _Write1(0x1f);
    _Write1(0x1b);
    _Write1(0x23);
    _Write1(0x37);
    _Write1(0x00);
    _Write1(0x07);
    _Write1(0x02);
    _Write1(0x10);

    _Write0(0xe1);
    _Write1(0x0f);
    _Write1(0x1b);
    _Write1(0x0f);
    _Write1(0x17);
    _Write1(0x33);
    _Write1(0x2c);
    _Write1(0x29);
    _Write1(0x2e);
    _Write1(0x30);
    _Write1(0x30);
    _Write1(0x39);
    _Write1(0x3f);
    _Write1(0x00);
    _Write1(0x07);
    _Write1(0x03);
    _Write1(0x10);

    _Write0(0x2a);
    _Write1(0x02);
    _Write1(0x00+2);
    _Write1(0x02);
    _Write1(0x7F+2);

    _Write0(0x2b);
    _Write1(0x01);
    _Write1(0x00+1);
    _Write1(0x01);
    _Write1(0x9F+1);

    _Write0(0xF0); //Enable test command
    _Write1(0x01);
    _Write0(0xF6); //Disable ram power save mode
    _Write1(0x00);

    _Write0(0x3A); //65k mode
    _Write1(0x05);

    _Write0(0x2c);
    for(i=0; i<0x5000; i++)
    {
        _Write1(0x00>>8);
        _Write1(0x00);
    }

    _Write0(0x29);    //Display on
}
