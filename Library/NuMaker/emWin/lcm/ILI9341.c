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

#define ILI9341_LED     PC11

#define SPI_TX_PDMA_CH  0

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

    while(SPI_LCD_PORT->STATUS & SPI_STATUS_TXFULL_Msk);
    SPI_WRITE_TX(SPI_LCD_PORT, Cmd);
    while((SPI_LCD_PORT->STATUS & SPI_STATUS_TXEMPTY_Msk) == 0);
}

/*********************************************************************
*
*       _Write1
*/
void _Write1(U8 Data)
{
    LCM_DC_SET;
    
    while(SPI_LCD_PORT->STATUS & SPI_STATUS_TXFULL_Msk);
    SPI_WRITE_TX(SPI_LCD_PORT, Data);
    while((SPI_LCD_PORT->STATUS & SPI_STATUS_TXEMPTY_Msk) == 0);
}

/*********************************************************************
*
*       _WriteM1
*/
void _WriteM1(U8 * pData, int NumItems)
{
    LCM_DC_SET;

    /* Set transfer count */
    PDMA_SET_TRANS_CNT(PDMA0, SPI_TX_PDMA_CH, NumItems);
    /* Set source address */
    PDMA_SET_SRC_ADDR(PDMA0, SPI_TX_PDMA_CH, (uint32_t)pData++);
    /* Set basic mode */
    PDMA0->DSCT[SPI_TX_PDMA_CH].CTL = (PDMA0->DSCT[SPI_TX_PDMA_CH].CTL & ~PDMA_DSCT_CTL_OPMODE_Msk) | PDMA_OP_BASIC;

    /* Enable SPI master's PDMA transfer function */
    SPI_TRIGGER_TX_PDMA(SPI_LCD_PORT);

    /* Check the PDMA transfer done flag */
    while((PDMA_GET_TD_STS(PDMA0) & (1 << SPI_TX_PDMA_CH)) == 0);

    /* Clear the PDMA transfer done flag */
    PDMA_CLR_TD_FLAG(PDMA0, (1 << SPI_TX_PDMA_CH));
}

static void _Open_SPI(void)
{
    GPIO_SetMode(GPIOPORT_LCM_DC, PINMASK_LCM_DC, GPIO_MODE_OUTPUT);
    GPIO_SetMode(GPIOPORT_LCM_RESET, PINMASK_LCM_RESET, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PC, BIT11, GPIO_MODE_OUTPUT);
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

static void _Open_PDMA(void)
{
    /* Reset PDMA module */
    SYS_ResetModule(PDMA0_RST);

    /* Enable PDMA channel */
    PDMA_Open(PDMA0, (1 << SPI_TX_PDMA_CH));

    /* Set transfer width (8 bits) */
    PDMA0->DSCT[SPI_TX_PDMA_CH].CTL = (PDMA0->DSCT[SPI_TX_PDMA_CH].CTL & ~PDMA_DSCT_CTL_TXWIDTH_Msk) | PDMA_WIDTH_8;
    /* Set destination address */
    PDMA_SET_DST_ADDR(PDMA0, SPI_TX_PDMA_CH, (uint32_t)&SPI_LCD_PORT->TX);
    /* Set source/destination attributes */
    PDMA0->DSCT[SPI_TX_PDMA_CH].CTL = (PDMA0->DSCT[SPI_TX_PDMA_CH].CTL & ~(PDMA_DSCT_CTL_SAINC_Msk | PDMA_DSCT_CTL_DAINC_Msk)) | (PDMA_SAR_INC | PDMA_DAR_FIX);
    /* Set request source; set basic mode. */
    PDMA_SetTransferMode(PDMA0, SPI_TX_PDMA_CH, PDMA_SPI1_TX, FALSE, 0);
    /* Single request type. SPI only support PDMA single request type. */
    PDMA_SetBurstType(PDMA0, SPI_TX_PDMA_CH, PDMA_REQ_SINGLE, 0);
    /* Disable table interrupt */
    PDMA0->DSCT[SPI_TX_PDMA_CH].CTL |= PDMA_TBINTDIS_DISABLE;
}

/*********************************************************************
*
*       _InitController
*
* Purpose:
*   Initializes the display controller
*/
void _InitController(void)
{
    static uint8_t s_InitOnce = 0;

    if(s_InitOnce == 0)
        s_InitOnce = 1;
    else
        return;

    _Open_SPI();
    _Open_PDMA();

    /* Configure DC/RESET/LED pins */
    GPIO_LCM_DC = 0;
    GPIO_LCM_RESET = 0;
    ILI9341_LED = 0;

    /* Configure LCD */
    GPIO_LCM_DC = 1;

    GPIO_LCM_RESET = 0;
    GUI_X_Delay(20);

    GPIO_LCM_RESET = 1;
    GUI_X_Delay(40);

    //************* Start Initial Sequence **********//

    _Write0(0xCF);
    _Write1(0x00);
    _Write1(0xD9);
    _Write1(0X30);

    _Write0(0xED);
    _Write1(0x64);
    _Write1(0x03);
    _Write1(0X12);
    _Write1(0X81);

    _Write0(0xE8);
    _Write1(0x85);
    _Write1(0x10);
    _Write1(0x78);

    _Write0(0xCB);
    _Write1(0x39);
    _Write1(0x2C);
    _Write1(0x00);
    _Write1(0x34);
    _Write1(0x02);

    _Write0(0xF7);
    _Write1(0x20);

    _Write0(0xEA);
    _Write1(0x00);
    _Write1(0x00);

    _Write0(0xC0);    //Power control
    _Write1(0x21);   //VRH[5:0]

    _Write0(0xC1);    //Power control
    _Write1(0x12);   //SAP[2:0];BT[3:0]

    _Write0(0xC5);    //VCM control
    _Write1(0x32);
    _Write1(0x3C);

    _Write0(0xC7);    //VCM control2
    _Write1(0XC1);

    _Write0(0x36);    // Memory Access Control
    _Write1(0xe8);

    _Write0(0x3A);
    _Write1(0x55);

    _Write0(0xB1);
    _Write1(0x00);
    _Write1(0x18);

    _Write0(0xB6);    // Display Function Control
    _Write1(0x0A);
    _Write1(0xA2);

    _Write0(0xF2);    // 3Gamma Function Disable
    _Write1(0x00);

    _Write0(0x26);    //Gamma curve selected
    _Write1(0x01);

    _Write0(0xE0);    //Set Gamma
    _Write1(0x0F);
    _Write1(0x20);
    _Write1(0x1E);
    _Write1(0x09);
    _Write1(0x12);
    _Write1(0x0B);
    _Write1(0x50);
    _Write1(0XBA);
    _Write1(0x44);
    _Write1(0x09);
    _Write1(0x14);
    _Write1(0x05);
    _Write1(0x23);
    _Write1(0x21);
    _Write1(0x00);

    _Write0(0XE1);    //Set Gamma
    _Write1(0x00);
    _Write1(0x19);
    _Write1(0x19);
    _Write1(0x00);
    _Write1(0x12);
    _Write1(0x07);
    _Write1(0x2D);
    _Write1(0x28);
    _Write1(0x3F);
    _Write1(0x02);
    _Write1(0x0A);
    _Write1(0x08);
    _Write1(0x25);
    _Write1(0x2D);
    _Write1(0x0F);

    _Write0(0x11);    //Exit Sleep
    GUI_X_Delay(120);
    _Write0(0x29);    //Display on

    ILI9341_LED = 1;
}
