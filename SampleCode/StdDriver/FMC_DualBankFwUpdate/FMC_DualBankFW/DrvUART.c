/*---------------------------------------------------------------------------------------------------------*/
/*                                                                                                         */
/* Copyright(c) 2009 Nuvoton Technology Corp. All rights reserved.                                         */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
#include <stdio.h>
#include "M2351.h"


/*---------------------------------------------------------------------------------------------------------*/
/* Includes of local headers                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
#include "DrvUART.h"


/*---------------------------------------------------------------------------------------------------------*/
/* Macro, type and constant definitions                                                                    */
/*---------------------------------------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
static PFN_DRVUART_CALLBACK *g_pfnUART0callback = NULL;
static PFN_DRVUART_CALLBACK *g_pfnUART1callback = NULL;
static PFN_DRVUART_CALLBACK *g_pfnUART2callback = NULL;
static PFN_DRVUART_CALLBACK *g_pfnUART3callback = NULL;
static PFN_DRVUART_CALLBACK *g_pfnUART4callback = NULL;
static PFN_DRVUART_CALLBACK *g_pfnUART5callback = NULL;


/*---------------------------------------------------------------------------------------------------------*/
/* Interrupt Handler                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void UART0_IRQHandler(void)
{
    if(g_pfnUART0callback != NULL)
    {
        g_pfnUART0callback();
    }
}

void UART1_IRQHandler(void)
{
    if(g_pfnUART1callback != NULL)
    {
        g_pfnUART1callback();
    }
}

void UART2_IRQHandler(void)
{
    if(g_pfnUART2callback != NULL)
    {
        g_pfnUART2callback();
    }
}

void UART3_IRQHandler(void)
{
    if(g_pfnUART3callback != NULL)
    {
        g_pfnUART3callback();
    }
}

void UART4_IRQHandler(void)
{
    if(g_pfnUART4callback != NULL)
    {
        g_pfnUART4callback();
    }
}

void UART5_IRQHandler(void)
{
    if(g_pfnUART5callback != NULL)
    {
        g_pfnUART5callback();
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/* GPIO Use                                                                                                */
/*---------------------------------------------------------------------------------------------------------*/
void SetPinValue(uint8_t u8Port, uint8_t u8Pin)
{
    GPIO_PIN_DATA(u8Port, u8Pin) = 1;
}

void ClrPinValue(uint8_t u8Port, uint8_t u8Pin)
{
    GPIO_PIN_DATA(u8Port, u8Pin) = 0;
}

uint32_t GetPinValue(uint8_t u8Port, uint8_t u8Pin)
{
    return GPIO_PIN_DATA(u8Port, u8Pin);
}

/*---------------------------------------------------------------------------------------------------------*/
/* UART Use                                                                                                */
/*---------------------------------------------------------------------------------------------------------*/
int32_t DrvUART_Write(E_UART_PORT u32Port, uint8_t *pu8TxBuf, uint32_t u32WriteBytes)
{
    uint32_t  u32Count, u32delayno;

    UART_T * tUART;
    tUART = (UART_T *)(UART0_BASE + u32Port);

    for(u32Count = 0; u32Count < u32WriteBytes; u32Count++)
    {
        u32delayno = 0;
        // while (tUART->FSR_BITS.TE_FLAG !=1)        /* Wait Tx empty and Time-out manner */
        while((tUART->FIFOSTS & UART_FIFOSTS_TXEMPTY_Msk) == 0)             /* Wait Tx empty and Time-out manner */
        {
            u32delayno++;
            if(u32delayno >= 0x40000000)
                return -1;
        }
        tUART->DAT = pu8TxBuf[u32Count];             /* Send UART Data from buffer */
    }

    return 0;
}

/*---------------------------------------------------------------------------------------------------------*/
/* Function:     DrvUART_EnableInt                                                                         */
/*                                                                                                         */
/* Parameter:                                                                                              */
/*               u32Port          -[in] UART Channel:  UART_PORT0 / UART_PORT1 / UART_PORT2                */
/*               u32InterruptFlag -[in] DRVUART_LININT/DRVUART_WAKEUPINT/DRVUART_BUFERRINT/DRVUART_RLSINT  */
/*                                      DRVUART_MOSINT/DRVUART_THREINT/DRVUART_RDAINT/DRVUART_TOUTINT      */
/*               pfncallback      -[in] A function pointer for callback function                           */
/* Returns:                                                                                                */
/*               None                                                                                      */
/* Description:                                                                                            */
/*               The function is used to enable specified UART interrupt, install the callback             */
/*               function and enable NVIC UART IRQ                                                         */
/*---------------------------------------------------------------------------------------------------------*/
void DrvUART_EnableInt(E_UART_PORT u32Port,
                       uint32_t u32InterruptFlag,
                       PFN_DRVUART_CALLBACK pfncallback)
{
    UART_T * tUART;
    tUART = (UART_T *)(UART0_BASE + u32Port);

    tUART->INTEN |= u32InterruptFlag;

    switch(u32Port)
    {
        case UART_PORT0:
        {
            g_pfnUART0callback = pfncallback;
            NVIC_EnableIRQ(UART0_IRQn);
        }
        break;

        case UART_PORT1:
        {
            g_pfnUART1callback = pfncallback;
            NVIC_EnableIRQ(UART1_IRQn);
        }
        break;

        case UART_PORT2:
        {
            g_pfnUART2callback = pfncallback;
            NVIC_EnableIRQ(UART2_IRQn);
        }
        break;

        case UART_PORT3:
        {
            g_pfnUART3callback = pfncallback;
            NVIC_EnableIRQ(UART3_IRQn);
        }
        break;

        case UART_PORT4:
        {
            g_pfnUART4callback = pfncallback;
            NVIC_EnableIRQ(UART4_IRQn);
        }
        break;

        case UART_PORT5:
        {
            g_pfnUART5callback = pfncallback;
            NVIC_EnableIRQ(UART5_IRQn);
        }
        break;

        default:
            break;
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/* Function:     DrvUART_DisableInt                                                                        */
/*                                                                                                         */
/* Parameter:                                                                                              */
/*               u32Port          -[in] UART Channel:  UART_PORT0 / UART_PORT1  /  UART_PORT2              */
/*               u32InterruptFlag -[in] DRVUART_LININT/DRVUART_WAKEUPINT/DRVUART_BUFERRINT/DRVUART_RLSINT  */
/*                                      DRVUART_MOSINT/DRVUART_THREINT/DRVUART_RDAINT/DRVUART_TOUTINT      */
/* Returns:                                                                                                */
/*               None                                                                                      */
/* Description:                                                                                            */
/*               The function is used to disable UART specified interrupt, uninstall the call back         */
/*               function and disable NVIC UART IRQ                                                        */
/*---------------------------------------------------------------------------------------------------------*/
void DrvUART_DisableInt(E_UART_PORT u32Port, uint32_t u32InterruptFlag)
{
    UART_T * tUART;
    tUART = (UART_T *)(UART0_BASE + u32Port);

    tUART->INTEN &= ~u32InterruptFlag;

    switch(u32Port)     /* Disable Callback function and NVIC */
    {
        case UART_PORT0:
        {
            g_pfnUART0callback = NULL;
            NVIC_DisableIRQ(UART0_IRQn);
        }
        break;

        case UART_PORT1:
        {
            g_pfnUART1callback = NULL;
            NVIC_DisableIRQ(UART1_IRQn);
        }
        break;

        case UART_PORT2:
        {
            g_pfnUART2callback = NULL;
            NVIC_DisableIRQ(UART2_IRQn);
        }
        break;

        case UART_PORT3:
        {
            g_pfnUART3callback = NULL;
            NVIC_DisableIRQ(UART3_IRQn);
        }
        break;

        case UART_PORT4:
        {
            g_pfnUART4callback = NULL;
            NVIC_DisableIRQ(UART4_IRQn);
        }
        break;

        case UART_PORT5:
        {
            g_pfnUART5callback = NULL;
            NVIC_DisableIRQ(UART5_IRQn);
        }
        break;

        default:
            break;
    }
}





/*---------------------------------------------------------------------------------------------------------*/
/* UART Clock setting                                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
void UART_Init(UART_T* uart, uint32_t u32ClkSrc, uint32_t u32ClkDiv, uint32_t u32BaudRate)
{
    uint32_t u32ClkTbl[4] = {__HXT, 0, __LXT, __HIRC};

    /* UART clock source and clock divider setting */
    switch((uint32_t)uart)
    {
        case UART0_BASE:
            CLK->CLKSEL1 = (CLK->CLKSEL1 & (~CLK_CLKSEL1_UART0SEL_Msk)) | (u32ClkSrc << CLK_CLKSEL1_UART0SEL_Pos);
            CLK->CLKDIV0 = (CLK->CLKDIV0 & (~CLK_CLKDIV0_UART0DIV_Msk)) | (u32ClkDiv << CLK_CLKDIV0_UART0DIV_Pos);
            break;
        case UART1_BASE:
            CLK->CLKSEL1 = (CLK->CLKSEL1 & (~CLK_CLKSEL1_UART1SEL_Msk)) | (u32ClkSrc << CLK_CLKSEL1_UART1SEL_Pos);
            CLK->CLKDIV0 = (CLK->CLKDIV0 & (~CLK_CLKDIV0_UART1DIV_Msk)) | (u32ClkDiv << CLK_CLKDIV0_UART1DIV_Pos);
            break;
        case UART2_BASE:
            CLK->CLKSEL3 = (CLK->CLKSEL3 & (~CLK_CLKSEL3_UART2SEL_Msk)) | (u32ClkSrc << CLK_CLKSEL3_UART2SEL_Pos);
            CLK->CLKDIV4 = (CLK->CLKDIV4 & (~CLK_CLKDIV4_UART2DIV_Msk)) | (u32ClkDiv << CLK_CLKDIV4_UART2DIV_Pos);
            break;
        case UART3_BASE:
            CLK->CLKSEL3 = (CLK->CLKSEL3 & (~CLK_CLKSEL3_UART3SEL_Msk)) | (u32ClkSrc << CLK_CLKSEL3_UART3SEL_Pos);
            CLK->CLKDIV4 = (CLK->CLKDIV4 & (~CLK_CLKDIV4_UART3DIV_Msk)) | (u32ClkDiv << CLK_CLKDIV4_UART3DIV_Pos);
            break;
        case UART4_BASE:
            CLK->CLKSEL3 = (CLK->CLKSEL3 & (~CLK_CLKSEL3_UART4SEL_Msk)) | (u32ClkSrc << CLK_CLKSEL3_UART4SEL_Pos);
            CLK->CLKDIV4 = (CLK->CLKDIV4 & (~CLK_CLKDIV4_UART4DIV_Msk)) | (u32ClkDiv << CLK_CLKDIV4_UART4DIV_Pos);
            break;
        case UART5_BASE:
            CLK->CLKSEL3 = (CLK->CLKSEL3 & (~CLK_CLKSEL3_UART5SEL_Msk)) | (u32ClkSrc << CLK_CLKSEL3_UART5SEL_Pos);
            CLK->CLKDIV4 = (CLK->CLKDIV4 & (~CLK_CLKDIV4_UART5DIV_Msk)) | (u32ClkDiv << CLK_CLKDIV4_UART5DIV_Pos);
            break;
    }

    /* Set UART line configuration */
    uart->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;

    /* Get PLL clock frequency if UART clock source selection is PLL */
    if(u32ClkSrc == 1)
        u32ClkTbl[1] = CLK_GetPLLClockFreq();

    /* Set UART baud rate */
    if(u32BaudRate != 0)
        uart->BAUD = (UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER((u32ClkTbl[u32ClkSrc]) / (u32ClkDiv + 1), u32BaudRate));

}




void DrvSYS_Delay(uint32_t us)
{
    SysTick->LOAD = us * CyclesPerUs;
    SysTick->VAL  = (0x00);
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

    /* Waiting for down-count to zero */
    while((SysTick->CTRL & (1 << 16)) == 0);

    SysTick->CTRL = 0;
}


uint32_t GetUartClkSrc(UART_T* uart)
{
    uint8_t u8UartClkSrcSel, u8UartClkDivNum;
    uint32_t u32ClkTbl[4] = {__HXT, 0, __LXT, __HIRC};

    /* Get UART clock source selection and UART clock divider number */
    switch((uint32_t)uart)
    {
        case UART0_BASE:
            u8UartClkSrcSel = (CLK->CLKSEL1 & CLK_CLKSEL1_UART0SEL_Msk) >> CLK_CLKSEL1_UART0SEL_Pos;
            u8UartClkDivNum = (CLK->CLKDIV0 & CLK_CLKDIV0_UART0DIV_Msk) >> CLK_CLKDIV0_UART0DIV_Pos;
            break;
        case UART1_BASE:
            u8UartClkSrcSel = (CLK->CLKSEL1 & CLK_CLKSEL1_UART1SEL_Msk) >> CLK_CLKSEL1_UART1SEL_Pos;
            u8UartClkDivNum = (CLK->CLKDIV0 & CLK_CLKDIV0_UART1DIV_Msk) >> CLK_CLKDIV0_UART1DIV_Pos;
            break;
        case UART2_BASE:
            u8UartClkSrcSel = (CLK->CLKSEL3 & CLK_CLKSEL3_UART2SEL_Msk) >> CLK_CLKSEL3_UART2SEL_Pos;
            u8UartClkDivNum = (CLK->CLKDIV4 & CLK_CLKDIV4_UART2DIV_Msk) >> CLK_CLKDIV4_UART2DIV_Pos;
            break;
        case UART3_BASE:
            u8UartClkSrcSel = (CLK->CLKSEL3 & CLK_CLKSEL3_UART3SEL_Msk) >> CLK_CLKSEL3_UART3SEL_Pos;
            u8UartClkDivNum = (CLK->CLKDIV4 & CLK_CLKDIV4_UART3DIV_Msk) >> CLK_CLKDIV4_UART3DIV_Pos;
            break;
        case UART4_BASE:
            u8UartClkSrcSel = (CLK->CLKSEL3 & CLK_CLKSEL3_UART4SEL_Msk) >> CLK_CLKSEL3_UART4SEL_Pos;
            u8UartClkDivNum = (CLK->CLKDIV4 & CLK_CLKDIV4_UART4DIV_Msk) >> CLK_CLKDIV4_UART4DIV_Pos;
            break;
        case UART5_BASE:
            u8UartClkSrcSel = (CLK->CLKSEL3 & CLK_CLKSEL3_UART5SEL_Msk) >> CLK_CLKSEL3_UART5SEL_Pos;
            u8UartClkDivNum = (CLK->CLKDIV4 & CLK_CLKDIV4_UART5DIV_Msk) >> CLK_CLKDIV4_UART5DIV_Pos;
            break;
    }

    /* Get PLL clock frequency if UART clock source selection is PLL */
    if(u8UartClkSrcSel == 1)
        u32ClkTbl[u8UartClkSrcSel] = CLK_GetPLLClockFreq();

    return (u32ClkTbl[u8UartClkSrcSel] / (u8UartClkDivNum + 1));

}


/*---------------------------------------------------------------------------------------------------------*/
/* set UART pin to GPIO for easy line connect in test                                                      */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t u8TestGpioPort, u8TestGpioPin;

//RX->GPIO
void SetUartRxPintoGPIO(E_UART_PORT u32Port)
{
    /* In Tx<->Rx Test, monitor Rx instead of change Rx setting */

    switch(u32Port)
    {
        case UART_PORT0:
            u8TestGpioPort = 0;
            u8TestGpioPin = 0;
            break; //PA.0
        case UART_PORT1:
            u8TestGpioPort = 1;
            u8TestGpioPin = 6;
            break; //PB.6
        case UART_PORT2:
            u8TestGpioPort = 2;
            u8TestGpioPin = 0;
            break; //PC.0
        case UART_PORT3:
            u8TestGpioPort = 3;
            u8TestGpioPin = 0;
            break; //PD.0
        case UART_PORT4:
            u8TestGpioPort = 7;
            u8TestGpioPin = 11;
            break; //PH.11
        case UART_PORT5:
            u8TestGpioPort = 1;
            u8TestGpioPin = 4;
            break; //PB.4
    }
}

//GPIO->RX
void SetGPIOPintoUartRx(E_UART_PORT u32Port)
{
    /* In Tx<->Rx Test, monitor Rx instead of change Rx setting */
}

//TX->GPIO
void SetUartTxPintoGPIO(E_UART_PORT u32Port)
{
    GPIO_T * tGPIO;

    switch(u32Port)
    {
        case UART_PORT0:
            u8TestGpioPort = 0;
            u8TestGpioPin = 1;
            break; //PA.1
        case UART_PORT1:
            u8TestGpioPort = 1;
            u8TestGpioPin = 7;
            break; //PB.7
        case UART_PORT2:
            u8TestGpioPort = 2;
            u8TestGpioPin = 1;
            break; //PC.1
        case UART_PORT3:
            u8TestGpioPort = 3;
            u8TestGpioPin = 1;
            break; //PD.1
        case UART_PORT4:
            u8TestGpioPort = 7;
            u8TestGpioPin = 10;
            break; //PH.10
        case UART_PORT5:
            u8TestGpioPort = 1;
            u8TestGpioPin = 5;
            break; //PB.5
    }

    tGPIO = (GPIO_T *)(GPIO_BASE + (u8TestGpioPort * 0x40));
    GPIO_PIN_DATA(u8TestGpioPort, u8TestGpioPin) = 1;
    GPIO_SetMode(tGPIO, 1 << u8TestGpioPin, GPIO_MODE_OUTPUT);

    switch(u32Port)
    {
        case UART_PORT0:
            SYS->GPA_MFPL &= (~SYS_GPA_MFPL_PA1MFP_Msk);
            break; //PA.1
        case UART_PORT1:
            SYS->GPB_MFPL &= (~SYS_GPB_MFPL_PB7MFP_Msk);
            break; //PB.7
        case UART_PORT2:
            SYS->GPC_MFPL &= (~SYS_GPC_MFPL_PC1MFP_Msk);
            break;  //PC.1
        case UART_PORT3:
            SYS->GPD_MFPL &= (~SYS_GPD_MFPL_PD1MFP_Msk);
            break; //PD.1
        case UART_PORT4:
            SYS->GPH_MFPH &= (~SYS_GPH_MFPH_PH10MFP_Msk);
            break; //PH.10
        case UART_PORT5:
            SYS->GPB_MFPL &= (~SYS_GPB_MFPL_PB5MFP_Msk);
            break;  //PB.5
    }
}

//GPIO->TX
void SetGPIOPintoUartTx(E_UART_PORT u32Port)
{

    switch(u32Port)
    {
        case UART_PORT0:
            SYS->GPA_MFPL |= SYS_GPA_MFPL_PA1MFP_UART0_TXD;
            break;  //PA.1
        case UART_PORT1:
            SYS->GPB_MFPL |= SYS_GPB_MFPL_PB7MFP_UART1_TXD;
            break;  //PB.7
        case UART_PORT2:
            SYS->GPC_MFPL |= SYS_GPC_MFPL_PC1MFP_UART2_TXD;
            break;    //PC.1
        case UART_PORT3:
            SYS->GPD_MFPL |= SYS_GPD_MFPL_PD1MFP_UART3_TXD;
            break;  //PD.1
        case UART_PORT4:
            SYS->GPH_MFPH |= SYS_GPH_MFPH_PH10MFP_UART4_TXD;
            break;  //PH.10
        case UART_PORT5:
            SYS->GPB_MFPL |= SYS_GPB_MFPL_PB5MFP_UART5_TXD;
            break;    //PB.5
    }

}

//RTS->GPIO
void SetUartRTSPintoGPIO(E_UART_PORT u32Port)
{
    GPIO_T * tGPIO;

    switch(u32Port)
    {
        case UART_PORT0:
            u8TestGpioPort = 0;
            u8TestGpioPin = 4;
            break; //PA.4
        case UART_PORT1:
            u8TestGpioPort = 1;
            u8TestGpioPin = 8;
            break; //PB.8
        case UART_PORT2:
            u8TestGpioPort = 2;
            u8TestGpioPin = 3;
            break; //PC.3
        case UART_PORT3:
            u8TestGpioPort = 3;
            u8TestGpioPin = 3;
            break; //PD.3
        case UART_PORT4:
            u8TestGpioPort = 4;
            u8TestGpioPin = 0;
            break; //PE.0
        case UART_PORT5:
            u8TestGpioPort = 1;
            u8TestGpioPin = 3;
            break; //PB.3
    }

    tGPIO = (GPIO_T *)(GPIO_BASE + (u8TestGpioPort * 0x40));
    GPIO_SetMode(tGPIO, 1 << u8TestGpioPin, GPIO_MODE_OUTPUT);

    switch(u32Port)
    {
        case UART_PORT0:
            SYS->GPA_MFPL &= (~SYS_GPA_MFPL_PA4MFP_Msk);
            break; //PA.4
        case UART_PORT1:
            SYS->GPB_MFPH &= (~SYS_GPB_MFPH_PB8MFP_Msk);
            break; //PB.8
        case UART_PORT2:
            SYS->GPC_MFPL &= (~SYS_GPC_MFPL_PC3MFP_Msk);
            break; //PC.3
        case UART_PORT3:
            SYS->GPD_MFPL &= (~SYS_GPD_MFPL_PD3MFP_Msk);
            break; //PD.3
        case UART_PORT4:
            SYS->GPE_MFPL &= (~SYS_GPE_MFPL_PE0MFP_Msk);
            break; //PE.0
        case UART_PORT5:
            SYS->GPB_MFPL &= (~SYS_GPB_MFPL_PB3MFP_Msk);
            break; //PB.3
    }
}

//GPIO->RTS
void SetGPIOPintoUartRTS(E_UART_PORT u32Port)
{

    switch(u32Port)
    {
        case UART_PORT0:
            SYS->GPA_MFPL |= SYS_GPA_MFPL_PA4MFP_UART0_nRTS;
            break; //PA.4
        case UART_PORT1:
            SYS->GPB_MFPH |= SYS_GPB_MFPH_PB8MFP_UART1_nRTS;
            break; //PB.8
        case UART_PORT2:
            SYS->GPC_MFPL |= SYS_GPC_MFPL_PC3MFP_UART2_nRTS;
            break; //PC.3
        case UART_PORT3:
            SYS->GPD_MFPL |= SYS_GPD_MFPL_PD3MFP_UART3_nRTS;
            break; //PD.3
        case UART_PORT4:
            SYS->GPE_MFPL |= SYS_GPE_MFPL_PE0MFP_UART4_nRTS;
            break; //PE.0
        case UART_PORT5:
            SYS->GPB_MFPL |= SYS_GPB_MFPL_PB3MFP_UART5_nRTS;
            break; //PB.3
    }

}


//CTS->GPIO
void SetUartCTSPintoGPIO(E_UART_PORT u32Port)
{
    /* Just get GPIO pin status, no need change MFP */

    switch(u32Port)
    {
        case UART_PORT0:
            u8TestGpioPort = 0;
            u8TestGpioPin = 5;
            break; //PA.5
        case UART_PORT1:
            u8TestGpioPort = 1;
            u8TestGpioPin = 9;
            break; //PB.9
        case UART_PORT2:
            u8TestGpioPort = 2;
            u8TestGpioPin = 2;
            break; //PC.2
        case UART_PORT3:
            u8TestGpioPort = 3;
            u8TestGpioPin = 2;
            break; //PD.2
        case UART_PORT4:
            u8TestGpioPort = 4;
            u8TestGpioPin = 1;
            break; //PE.1
        case UART_PORT5:
            u8TestGpioPort = 1;
            u8TestGpioPin = 2;
            break; //PB.2
    }

}

//GPIO->CTS
void SetGPIOPintoUartCTS(E_UART_PORT u32Port)
{
    /* Just get GPIO pin status, no need change MFP */
}


/*---------------------------------------------------------------------------------------------------------*/
/* Count how many 1 in data                                                                                */
/*---------------------------------------------------------------------------------------------------------*/
int count_bit_1(int n)
{

    int c = 0, i = 0;
    for(i = 0; i < 32; ++i)
    {
        if(n & (1 << i))
            c++;
    }
    return c;
}

/*---------------------------------------------------------------------------------------------------------*/
/* Setting befoe and after test                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
uint32_t au32ClkRegValue[5] = {0};

void BackupCLKSetting(void)
{
    UART_WAIT_TXEMPTYF_RXIDLE(DEBUG_PORT);
    au32ClkRegValue[0] = CLK->CLKSEL0;
    au32ClkRegValue[1] = CLK->CLKSEL1;
    au32ClkRegValue[2] = CLK->CLKSEL3;
    au32ClkRegValue[3] = CLK->CLKDIV0;
    au32ClkRegValue[4] = CLK->CLKDIV4;
}

void RestoreCLKSetting(void)
{
    UART_WAIT_TXEMPTYF_RXIDLE(DEBUG_PORT);
    CLK->CLKSEL0 = au32ClkRegValue[0];
    CLK->CLKSEL1 = au32ClkRegValue[1];
    CLK->CLKSEL3 = au32ClkRegValue[2];
    CLK->CLKDIV0 = au32ClkRegValue[3];
    CLK->CLKDIV4 = au32ClkRegValue[4];
}

uint32_t au32Uart0RegValue[19] = {0};
uint32_t au32Uart1RegValue[19] = {0};
uint32_t au32Uart2RegValue[19] = {0};
uint32_t au32Uart3RegValue[19] = {0};
uint32_t au32Uart4RegValue[19] = {0};
uint32_t au32Uart5RegValue[19] = {0};

void BackupUARTSetting(E_UART_PORT u32Port)
{
    int32_t i;

    switch(u32Port)
    {
        case UART_PORT0:
            for(i = 1; i < 14; i++)
            {
                au32Uart0RegValue[i] = inpw(UART0_BASE + UART_PORT0 + (i << 2));
            }
            break;

        case UART_PORT1:
            for(i = 1; i < 14; i++)
            {
                au32Uart1RegValue[i] = inpw(UART0_BASE + UART_PORT1 + (i << 2));
            }
            break;

        case UART_PORT2:
            for(i = 1; i < 14; i++)
            {
                au32Uart2RegValue[i] = inpw(UART0_BASE + UART_PORT2 + (i << 2));
            }
            break;

        case UART_PORT3:
            for(i = 1; i < 14; i++)
            {
                au32Uart3RegValue[i] = inpw(UART0_BASE + UART_PORT3 + (i << 2));
            }
            break;

        case UART_PORT4:
            for(i = 1; i < 14; i++)
            {
                au32Uart4RegValue[i] = inpw(UART0_BASE + UART_PORT4 + (i << 2));
            }
            break;

        case UART_PORT5:
            for(i = 1; i < 14; i++)
            {
                au32Uart5RegValue[i] = inpw(UART0_BASE + UART_PORT5 + (i << 2));
            }
            break;
    }
}

void RestoreUARTSetting(E_UART_PORT u32Port)
{
    int32_t i;

    switch(u32Port)
    {
        case UART_PORT0:
            for(i = 1; i < 14; i++)
            {
                if((i == 6) || (i == 7) || (i == 14) || (i == 17)) continue;
                if(inpw(UART0_BASE + UART_PORT0 + (i << 2)) != au32Uart0RegValue[i])
                    outpw(UART0_BASE + UART_PORT0 + (i << 2), au32Uart0RegValue[i]);
            }
            break;

        case UART_PORT1:
            for(i = 1; i < 14; i++)
            {
                if((i == 6) || (i == 7) || (i == 14) || (i == 17)) continue;
                if(inpw(UART0_BASE + UART_PORT1 + (i << 2)) != au32Uart1RegValue[i])
                    outpw(UART0_BASE + UART_PORT1 + (i << 2), au32Uart1RegValue[i]);
            }
            break;

        case UART_PORT2:
            for(i = 1; i < 14; i++)
            {
                if((i == 6) || (i == 7) || (i == 14) || (i == 17)) continue;
                if(inpw(UART0_BASE + UART_PORT2 + (i << 2)) != au32Uart2RegValue[i])
                    outpw(UART0_BASE + UART_PORT2 + (i << 2), au32Uart2RegValue[i]);
            }
            break;

        case UART_PORT3:
            for(i = 1; i < 14; i++)
            {
                if((i == 6) || (i == 7) || (i == 14) || (i == 17)) continue;
                if(inpw(UART0_BASE + UART_PORT3 + (i << 2)) != au32Uart3RegValue[i])
                    outpw(UART0_BASE + UART_PORT3 + (i << 2), au32Uart3RegValue[i]);
            }
            break;

        case UART_PORT4:
            for(i = 1; i < 14; i++)
            {
                if((i == 6) || (i == 7) || (i == 14) || (i == 17)) continue;
                if(inpw(UART0_BASE + UART_PORT4 + (i << 2)) != au32Uart4RegValue[i])
                    outpw(UART0_BASE + UART_PORT4 + (i << 2), au32Uart4RegValue[i]);
            }
            break;

        case UART_PORT5:
            for(i = 1; i < 14; i++)
            {
                if((i == 6) || (i == 7) || (i == 14) || (i == 17)) continue;
                if(inpw(UART0_BASE + UART_PORT5 + (i << 2)) != au32Uart5RegValue[i])
                    outpw(UART0_BASE + UART_PORT5 + (i << 2), au32Uart5RegValue[i]);
            }
            break;

    }
}

/*---------------------------------------------------------------------------------------------------------*/
/* Prepare Tx Rx Data                                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
void PrepareData8(uint8_t *pu8Data, uint32_t u32Size)
{
    uint32_t i;
    for(i = 0; i < u32Size; i++)
        pu8Data[i] = 0xFF;
}

void PrepareData16(uint16_t *pu16Data, uint32_t u32Size)
{
    uint32_t i;
    for(i = 0; i < u32Size; i++)
        pu16Data[i] = 0xFFFF;
}

uint32_t CompareData8(uint8_t *pu8Data, uint32_t u32Size)
{
    uint32_t i, u32Error = 0;

    for(i = 0; i < u32Size; i++)
    {
        if(*pu8Data != (i & 0xFF))
        {
            printf("\nError RxIndex[%d] exp[0x%X] get[0x%X]", i, i & 0xFF, *pu8Data);
            u32Error = 1;
        }
        pu8Data++;
    }

    if(u32Error)
        return 1;
    else
        return 0;

}

uint32_t CompareData8InOut(uint8_t InBuffer[], uint8_t OutBuffer[], int32_t len)
{
    int i, u32Error = 0;

    for(i = 0; i < len; i++)
    {
        if(InBuffer[i] != OutBuffer[i])
        {
            printf("\nFail Index[%d] exp[0x%X] get[0x%X]\n", i, InBuffer[i], OutBuffer[i]);
            u32Error = 1;
        }
    }

    if(u32Error)
        return 1;
    else
        return 0;
}

/*---------------------------------------------------------------------------------------------------------*/
/* Auto Test Use                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/


uint32_t u32LineConnectHint = 1;

void LineConnectHint(void)
{
    if(u32LineConnectHint)
    {
        printf(" Enter any key to continue.\n\n");
        GetChar();
    }
    else
        printf("\n");
}

