/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief
 *           Transmit and receive data from PC terminal through RS232 interface.
 *
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"



#define RXBUFSIZE   1024

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
static uint8_t s_u8RecData[RXBUFSIZE]  = {0};

static volatile uint32_t s_u32comRbytes = 0;
static volatile uint32_t s_u32comRhead  = 0;
static volatile uint32_t s_u32comRtail  = 0;
static volatile int32_t s_i32Wait       = TRUE;

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void);
void UART_TEST_HANDLE(void);
void UART_FunctionTest(void);

void SYS_Init(void);
void UART0_Init(void);
void UART0_IRQHandler(void);


void SYS_Init(void)
{

    /* Set PF multi-function pins for XT1_OUT(PF.2) and XT1_IN(PF.3) */
    SYS->GPF_MFPL = (SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF2MFP_Msk)) | SYS_GPF_MFPL_PF2MFP_XT1_OUT;
    SYS->GPF_MFPL = (SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF3MFP_Msk)) | SYS_GPF_MFPL_PF3MFP_XT1_IN;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and HCLK clock divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable HXT clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Wait for HXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Enable PLL */
    CLK->PLLCTL = CLK_PLLCTL_128MHz_HIRC;

    /* Waiting for PLL stable */
    CLK_WaitClockReady(CLK_STATUS_PLLSTB_Msk);

    /* Select HCLK clock source as PLL and HCLK source divider as 2 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_PLL, CLK_CLKDIV0_HCLK(2));

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART module clock source as HXT and UART module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HXT, CLK_CLKDIV0_UART0(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & (~(UART0_RXD_PB12_Msk | UART0_TXD_PB13_Msk))) | UART0_RXD_PB12 | UART0_TXD_PB13;

}

void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART0 */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/* UART Test Sample                                                                                        */
/* Test Item                                                                                               */
/* It sends the received data to HyperTerminal.                                                            */
/*---------------------------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);

    printf("\nUART Sample Program\n");

    /* UART sample function */
    UART_FunctionTest();

    printf("\nUART Sample Demo End.\n");

    while(1);

}

/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle UART Channel 0 interrupt event                                                            */
/*---------------------------------------------------------------------------------------------------------*/
void UART0_IRQHandler(void)
{
    UART_TEST_HANDLE();
}

/*---------------------------------------------------------------------------------------------------------*/
/* UART Callback function                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void UART_TEST_HANDLE(void)
{
    uint8_t u8InChar = 0xFF;
    uint32_t u32IntSts = UART0->INTSTS;

    /* Receive Data Available Interrupt Handle */
    if(u32IntSts & UART_INTSTS_RDAINT_Msk)      
    {                 
        printf("\nInput:");

        /* Get all the input characters */
        while( UART_GET_RX_EMPTY(UART0) == 0 )
        {         
            /* Receive Line Status Error Handle */ 
            if(u32IntSts & UART_INTSTS_RLSINT_Msk)
            {                
                /* Clear Receive Line Status Interrupt */
                UART_ClearIntFlag(UART0, UART_INTSTS_RLSINT_Msk);   
            }             
            
            /* Get the character from UART Buffer */
            u8InChar = (uint8_t)UART_READ(UART0);

            printf("%c ", u8InChar);

            if(u8InChar == '0')
            {
                s_i32Wait = FALSE;
            }

            /* Check if buffer full */
            if(s_u32comRbytes < RXBUFSIZE)
            {
                /* Enqueue the character */
                s_u8RecData[s_u32comRtail] = u8InChar;
                s_u32comRtail = (s_u32comRtail == (RXBUFSIZE - 1)) ? 0 : (s_u32comRtail + 1);
                s_u32comRbytes++;
            }               
            
            printf("\nTransmission Test:");                
        }           
    }

    /* Transmit Holding Register Empty Interrupt Handle */    
    if(u32IntSts & UART_INTSTS_THREINT_Msk)     
    {
        uint32_t u32Tmp;
        u32Tmp = s_u32comRtail;
        if(s_u32comRhead != u32Tmp)
        {
            u8InChar = s_u8RecData[s_u32comRhead];
            while(UART_IS_TX_FULL(UART0));      /* Wait Tx is not full to transmit data */
            UART_WRITE(UART0, u8InChar);
            s_u32comRhead = (s_u32comRhead == (RXBUFSIZE - 1)) ? 0 : (s_u32comRhead + 1);
            s_u32comRbytes--;
        }
    }  
    
    /* Buffer Error Interrupt Handle */    
    if(u32IntSts & UART_INTSTS_BUFERRINT_Msk)   
    {
        /* Clear Buffer Error Interrupt */
        UART_ClearIntFlag(UART0, UART_INTSTS_BUFERRINT_Msk);             
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  UART Function Test                                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
void UART_FunctionTest(void)
{
    printf("+-----------------------------------------------------------+\n");
    printf("|  UART Function Test                                       |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|  Description :                                            |\n");
    printf("|    The sample code will print input char on terminal      |\n");
    printf("|    Please enter any to start     (Press '0' to exit)      |\n");
    printf("+-----------------------------------------------------------+\n");

    /*
        Using a RS232 cable to connect UART0 and PC.
        UART0 is set to debug port. UART0 is enable RDA interrupt.
        When inputting char to terminal screen, RDA interrupt will happen and
        UART0 will print the received char on screen.
    */

    /* Enable UART RDA, THRE, RLS and Buffer Error interrupt */
    UART_EnableInt(UART0, (UART_INTEN_RDAIEN_Msk | UART_INTEN_THREIEN_Msk | UART_INTEN_RLSIEN_Msk | UART_INTEN_BUFERRIEN_Msk));
    while(s_i32Wait);

    /* Disable UART RDA, THRE, RLS and Buffer Error interrupt */
    UART_DisableInt(UART0, (UART_INTEN_RDAIEN_Msk | UART_INTEN_THREIEN_Msk | UART_INTEN_RLSIEN_Msk | UART_INTEN_BUFERRIEN_Msk));
    s_i32Wait = TRUE;

}

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
