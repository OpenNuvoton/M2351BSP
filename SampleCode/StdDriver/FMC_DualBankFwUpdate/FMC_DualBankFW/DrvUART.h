#ifndef _DRVUART_H_
#define _DRVUART_H_


#include "M2351.h"

//#define printf(...)

#define NO_LINE_DEBUG_HINT    //for not stop and connect line in test


//#define D_msg(...)
#define D_msg  printf



//#define UART_BASE  (TEST_PORT+UART0_BASE)
#define UART_BASE  UART0_BASE

#define MAX_FIFO_BYTES 16



//#define TEST_PORT   UART_PORT0
////UART_BASE need to modify for Register test in RegisterTest.c
//#define TX_PORT   UART_PORT1
//#define RX_PORT   UART_PORT0





/*---------------------------------------------------------------------------------------------------------*/
/*  GPIO Configure for Help Debug                                                                          */
/*---------------------------------------------------------------------------------------------------------*/





#define UART_BAUD_MODE1     (UART_BAUD_BAUDM1_Msk)


typedef void (PFN_DRVUART_CALLBACK)(void);
//typedef void (PFN_DRVUART_CALLBACK)(uint32_t userData);


/*=== BaudRate comvent divider table === */

//CASE3
#define BAUD_2400_3  (0x1386|0x30000000)
#define BAUD_4800_3  (0x9c2|0x30000000)
#define BAUD_9600_3  (0x4e0|0x30000000)
#define BAUD_14400_3 (0x33F|0x30000000)
#define BAUD_19200_3 (0x26F|0x30000000)
#define BAUD_28800_3 (0x19F|0x30000000)
#define BAUD_38400_3 (0x136|0x30000000)
#define BAUD_57600_3 (0xce|0x30000000)
#define BAUD_115200_3 (0x66|0x30000000)
#define BAUD_230400_3 (0x32|0x30000000)
#define BAUD_460800_3 (0x18|0x30000000)
#define BAUD_921600_3 (0xB|0x30000000)

//CASE2
#define BAUD_2400_2  (0x290001F2)
#define BAUD_4800_2  (0x290000F7)
#define BAUD_9600_2  (0x2900007A)
#define BAUD_14400_2 (0x29000051)
#define BAUD_19200_2 (0x2900003B)
#define BAUD_28800_2 (0x2900002A)
#define BAUD_38400_2 (0x2900001C)
#define BAUD_57600_2 (0x29000016)
#define BAUD_115200_2 (0x29000008)
#define BAUD_230400_2 (0x29000003)
#define BAUD_460800_2 (0x29000001)

//CASE1
// for 12 Mhz input clock
#define BAUD_2400_1  (310)
#define BAUD_4800_1  (154)
#define BAUD_9600_1  (76)
#define BAUD_14400_1 (50)
#define BAUD_19200_1 (37)
#define BAUD_28800_1 (24)
#define BAUD_38400_1 (17)
#define BAUD_57600_1 (11)
#define BAUD_115200_1 (4)
#define BAUD_230400_1 (1)
#define BAUD_460800_1 (0)

/*---------------------------------------------------------------------------------------------------------*/
/*  Define UART Channel Sturcture                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
typedef enum
{
    UART_PORT0 = 0x0,
    UART_PORT1 = 0x1000,
    UART_PORT2 = 0x2000,
    UART_PORT3 = 0x3000,
    UART_PORT4 = 0x4000,
    UART_PORT5 = 0x5000,
} E_UART_PORT;

/*---------------------------------------------------------------------------------------------------------*/
/*  Define UART Multi-function Pin Setting Set                                                             */
/*---------------------------------------------------------------------------------------------------------*/
typedef enum
{
    //UART0_RX_TX_RTS_CTS
    E_UART0_C11_C12,
    E_UART0_F2_F3,
    E_UART0_A6_A7_C6_C7,
    E_UART0_A0_A1_A4_A5,
    E_UART0_D2_D3,
    E_UART0_A15_A14,
    E_UART0_H11_H10,
    E_UART0_B12_B13_B14_B15,
    E_UART0_B8_B9_B10_B11,

    //UART1_RX_TX_RTS_CTS
    E_UART1_B2_B3,
    E_UART1_A8_A9,
    E_UART1_D10_D11,
    E_UART1_C8_E13_E12_E11,
    E_UART1_A2_A3_A0_A1,
    E_UART1_F1_F0,
    E_UART1_D6_D7,
    E_UART1_H9_H8,
    E_UART1_B6_B7_B8_B9,

    //UART2_RX_TX_RTS_CTS
    E_UART2_B0_B1,
    E_UART2_D12_C13,
    E_UART2_F5_F4,
    E_UART2_E9_E8,
    E_UART2_E15_E14,
    E_UART2_C4_C5_D8_D9,
    E_UART2_C0_C1_C3_C2,

    //UART3_RX_TX_RTS_CTS
    E_UART3_C9_C10,
    E_UART3_E11_E10,
    E_UART3_C2_C3,
    E_UART3_D0_D1_D3_D2,
    E_UART3_E0_E1_H8_H9,
    E_UART3_B14_B15_B13_B12,

    //UART4_RX_TX_RTS_CTS
    E_UART4_F6_F7,
    E_UART4_C6_C7_E13_C8,
    E_UART4_A2_A3,
    E_UART4_C4_C5,
    E_UART4_A13_A12,
    E_UART4_H11_H10_E0_E1,
    E_UART4_B10_B11,

    //UART5_RX_TX_RTS_CTS
    E_UART5_B4_B5_B3_B2,
    E_UART5_A4_A5,
    E_UART5_E6_E7,


} E_UART_SET;

/*---------------------------------------------------------------------------------------------------------*/
/* define UART Interrupt Source                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
#define  DRVUART_TXENDINT       ((uint32_t)0x00400000)  /* Tx End Interrupt */
#define  DRVUART_ABAUDINT       ((uint32_t)0x00040000)  /* Auto Baud Rate Interrupt */
//#define  DRVUART_DATWAKEUPINT   ((uint32_t)0x00000400)  /* Data Wakeup Enable */
//#define  DRVUART_CTSWAKEUPINT   ((uint32_t)0x00000200)  /* CTS Wakeup Enable */
#define  DRVUART_LININT         ((uint32_t)0x00000100)  /* LIN RX Break Field Detected Interrupt */
#define  DRVUART_WAKEUPINT      ((uint32_t)0x00000040)  /* Wakeup Interrupt */
#define  DRVUART_BUFERRINT      ((uint32_t)0x00000020)  /* Buffer Error Interrupt */
#define  DRVUART_TOUTINT        ((uint32_t)0x00000010)  /* RX Time out Interrupt */
#define  DRVUART_MOSINT         ((uint32_t)0x00000008)  /* MODEM Status Interrupt */
#define  DRVUART_RLSINT         ((uint32_t)0x00000004)  /* Receive Line Status Interrupt*/
#define  DRVUART_THREINT        ((uint32_t)0x00000002)  /* Transmit Holding Register Empty Interrupt */
#define  DRVUART_RDAINT         ((uint32_t)0x00000001)  /* Receive Data Available Interrupt */









void DrvUART_EnableInt(E_UART_PORT u32Port, uint32_t u32InterruptFlag, PFN_DRVUART_CALLBACK pfncallback);
void DrvUART_DisableInt(E_UART_PORT u32Port, uint32_t u32InterruptFlag);
uint32_t GetBaudRate(E_UART_PORT u32Port, uint32_t u32ClkVal);
uint32_t CLK_GetUARTFreq(void);
int32_t DrvUART_Write(E_UART_PORT u32Port, uint8_t *pu8TxBuf, uint32_t u32WriteBytes);
int32_t DrvUART_Read(E_UART_PORT u32Port, uint8_t *pu8RxBuf, uint32_t u32ReadBytes);



/*---------------------------------------------------------------------------------------------------------*/
/* Macro                                                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
#define UART_WAIT_TXEMPTYF_RXIDLE(uart) \
    while( ((uart)->FIFOSTS & (UART_FIFOSTS_TXEMPTYF_Msk|UART_FIFOSTS_RXIDLE_Msk)) \
                            !=(UART_FIFOSTS_TXEMPTYF_Msk|UART_FIFOSTS_RXIDLE_Msk))





/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
extern char GetChar(void);
extern void SendChar(int ch);


/*========== DrvUART.c ==========*/

//Initial setting
void UART_MFP_Init(UART_T* uart, E_UART_SET u32Set, uint32_t u32MFOS);
#define UART_CLK_SRC_HXT    0
#define UART_CLK_SRC_PLL    1
#define UART_CLK_SRC_LXT    2
#define UART_CLK_SRC_HIRC   3
#define UART_CLK_DIV(x)     (x)-1
void UART_Init(UART_T* uart, uint32_t u32ClkSrc, uint32_t u32ClkDiv, uint32_t u32BaudRate);

//setting before/after test
void BackupCLKSetting(void);
void RestoreCLKSetting(void);
void BackupUARTSetting(E_UART_PORT u32Port);
void RestoreUARTSetting(E_UART_PORT u32Port);

//pin control
void SetPinValue(uint8_t u8Port, uint8_t u8Pin);
void ClrPinValue(uint8_t u8Port, uint8_t u8Pin);
uint32_t GetPinValue(uint8_t u8Port, uint8_t u8Pin);

//pin configure
extern uint8_t u8TestGpioPort, u8TestGpioPin;

void SetUartTxPintoGPIO(E_UART_PORT u32Port);
void SetGPIOPintoUartTx(E_UART_PORT u32Port);
void SetUartRxPintoGPIO(E_UART_PORT u32Port);
void SetGPIOPintoUartRx(E_UART_PORT u32Port);
void SetUartRTSPintoGPIO(E_UART_PORT u32Port);
void SetGPIOPintoUartRTS(E_UART_PORT u32Port);
void SetUartCTSPintoGPIO(E_UART_PORT u32Port);
void SetGPIOPintoUartCTS(E_UART_PORT u32Port);

//caculation
int count_bit_1(int n);

//data
void PrepareData8(uint8_t *pu8Data, uint32_t u32Size);
void PrepareData16(uint16_t *pu16Data, uint32_t u32Size);
uint32_t CompareData8(uint8_t *pu8Data, uint32_t u32Size);
uint32_t CompareData8InOut(uint8_t InBuffer[], uint8_t OutBuffer[], int32_t len);

//baud rate
uint32_t GetUartClkSrc(UART_T* uart);

//prepare Tx/Rx data
void PrepareData8(uint8_t *pu8Data, uint32_t u32Size);
void PrepareData16(uint16_t *pu16Data, uint32_t u32Size);


extern uint32_t u32LineConnectHint;
void LineConnectHint(void);


/*===== TestMenu.c =====*/
//Connection
void UART_OnePortTest(void);
void UART_TwoPortTest(void);
void UART_WithPCTest(void);
//Function
void UART_Menu_All(void);
void UART_Menu_Function_1(void);
void UART_Menu_Function_2(void);
void UART_Menu_Function_3(void);
void UART_Menu_Wakeup(void);
void UART_Menu_PDMA(void);
void UART_Menu_RS485(void);
void UART_Menu_LIN(void);



//RegisterTest.c
void RegisterTest(E_UART_PORT u32Port);

//FunctionTest.c
/*----- For ABAUD Scan Test -----*/
extern UART_T *u32TestPortTx, *u32TestPort, *u32TestPortRx;
void UART_Menu_Function(void);
void TX_empty_INT_Test(E_UART_PORT u32Port);
void RxReadyINT_Test(E_UART_PORT u32Port, uint32_t testMode);
void INT_Test(E_UART_PORT u32Port);
void SetBaudRateTest(E_UART_PORT u32Port);
void LineBreakControl_Test(E_UART_PORT u32Port);
void RxTimeOut_Test(E_UART_PORT u32Port);
void RTS_TEST(E_UART_PORT u32Port);
void CTS_TEST(E_UART_PORT u32Port);
void Test_Loopback(E_UART_PORT u32Port);
void RxTimeOutTiming_Test(E_UART_PORT u32Port);
void RTS_DriveTest(E_UART_PORT u32Port);
void FIFO_Test(E_UART_PORT u32Port);
void BufferError_INTTest(E_UART_PORT u32Port);
void ModemStatusINT_Test(E_UART_PORT u32Port);
void TxEnd_INT_Test(E_UART_PORT u32Port);
void FIFO_Reset_Test(E_UART_PORT u32Port);
void CTS_Toggle_Test(E_UART_PORT u32Port);

void Tx_Glitch_Test(E_UART_PORT u32PortTX, E_UART_PORT u32PortRX);
void TxRxDisable_Test(E_UART_PORT u32PortTX, E_UART_PORT u32PortRX);

extern volatile uint32_t delaynum;
void RxFIFO_InOutTest(E_UART_PORT u32PortTX, E_UART_PORT u32PortRX);
void TxFIFO_InOutTest(E_UART_PORT u32PortTX, E_UART_PORT u32PortRX);

/*===== FaultIOTest.c =====*/
void UART_Menu_FunctionPlus(void);
void BreakCotrol_Test(E_UART_PORT u32Port);
void FrameError_Test(E_UART_PORT u32Port);
void ParityError_Test(E_UART_PORT u32Port);

/*===== FreqMaxTest.c =====*/
#define TEST_BAUD_RATE_DIV          30
#define TEST_BAUD_RATE_DIV_FOR_LXT  5

void UART_FreqMax_Test(E_UART_PORT u32Port);



/*===== SendToPC.c =====*/
void RecevieAndSendBack(E_UART_PORT u32Port);
void TwoStopBitTest(E_UART_PORT u32Port);

/*===== LargeData.c =====*/
void LargeDataTestCompare(E_UART_PORT u32Port);
#define TWO_BOARD_CONTROL_PIN_IN  PD4
#define TWO_BOARD_CONTROL_PIN_OUT PD5

/*===== ParitySourceTest.c =====*/
void ParitySource_Tx_Test(E_UART_PORT u32PortTX, E_UART_PORT u32PortRX);
void ParitySource_Rx_Test(E_UART_PORT u32PortTX, E_UART_PORT u32PortRX);
void ParitySource_TxRx_Test(E_UART_PORT u32Port);
void RLSINT_FEF_Test(E_UART_PORT u32PortTX, E_UART_PORT u32PortRX);
void RLSINT_PEF_Test(E_UART_PORT u32PortTX, E_UART_PORT u32PortRX);


/*===== AutoBaudRate.c =====*/
void ABAUD_Scan_Test(E_UART_PORT u32PortTX, E_UART_PORT u32PortRX);
void ABAUD_DetectBit_INTTest(E_UART_PORT u32PortTX, E_UART_PORT u32PortRX);
void ABAUD_Tout_Test(E_UART_PORT u32PortTX, E_UART_PORT u32PortRX);
void ABAUD_withoutFlag_Test(E_UART_PORT u32PortTX, E_UART_PORT u32PortRX);
void ABAUD_PC_Test(E_UART_PORT u32Port);
void ABAUD_TwoBoard_Test(E_UART_PORT u32Port);

/*===== BRCompTest.c =====*/
void BRComp_Time_Test(E_UART_PORT u32Port);
void BRComp_Loopback_Test(E_UART_PORT u32Port);
void BRComp_PC_Test(E_UART_PORT u32Port);

/*===== PDMA.c =====*/
void UART_Menu_PDMA(void);
void PDMA_Tx_Test(E_UART_PORT u32Port);
void PDMA_Rx_Test(E_UART_PORT u32Port);
void PDMA_Self_Loopback(E_UART_PORT u32Port);
void PDMA_BufErr_INT(E_UART_PORT u32Port);
void PDMA_TimeOut_INT(E_UART_PORT u32Port);
void PDMA_ModemStatus_INT(E_UART_PORT u32Port);
void PDMA_RLS_INT(E_UART_PORT u32Port);
void PDMA_Stop_Test(E_UART_PORT u32Port);
void PDMA_Timeout_Test(E_UART_PORT u32Port);
void PDMA_TxScatter_Test(E_UART_PORT u32Port);
void PDMA_RxScatter_Test(E_UART_PORT u32Port);
void PDMA_LoopbackScatter_Test(E_UART_PORT u32Port);
void PDMA_Tx16_Test(E_UART_PORT u32Port);
void PDMA_Rx16_Test(E_UART_PORT u32Port);
void PDMA_Loopback16_Test(E_UART_PORT u32Port);
void PDMA_TxAck_Test(E_UART_PORT u32Port);

/*===== WakeUp.c =====*/
void CTS_WakeUp(E_UART_PORT u32Port);
void Data_WakeUp(E_UART_PORT u32Port);
void RxTH_WakeUp_useINT(E_UART_PORT u32Port);
void RxTHTO_WakeUp_useINT(E_UART_PORT u32Port);
void RS485_WakeUp_useINT(E_UART_PORT u32Port);
void RS485_WKTHTO_useINT(E_UART_PORT u32Port);
void RxBusy_PowerDwon_useINT(E_UART_PORT u32Port);
void RxBusy_Test(E_UART_PORT u32Port);
void Wakeup_TwoBoard_Test(E_UART_PORT u32Port);


/*===== RS485Test.c =====*/
void RS485_TransmitTest(E_UART_PORT u32Port);
void RS485_ReceiveTest(E_UART_PORT u32Port, uint32_t u32Mode);
void RS485_FreqMax_Test(E_UART_PORT u32PortTX, E_UART_PORT u32PortRX, uint32_t u32Mode);
void RS485_Test(E_UART_PORT u32Port, uint32_t u32Mode);

/*===== LINTest.c =====*/
void LIN_FunctionTest(E_UART_PORT u32Port);
void LIN_Tx_Test(E_UART_PORT u32Port);
void LIN_Rx_Test(E_UART_PORT u32Port);

/*===== TwoRegLINTest.c =====*/
void LIN_MasterHeaderNoErrorMonitor_Test(void);
void LIN_MasterIdParity_Test(void);
void LIN_MasterHeaderErrorMonitor_Test(void);
void LIN_SlaveHeaderDetect_Test(void);
void LIN_SlaveHeaderTout_Test(void);
void LIN_BitError_Test(void);
void LIN_AutoResync_Test(void);
void LIN_DividerUpdate_Test(void);
void LIN_Sync56_Test(void);
void LIN_MuteMode_Test(void);
void LIN_ErrorCasePolling_Test(void);
void LIN_ErrorCaseInterrupt_Test(void);
void LIN_UartMaster_Test(void);
void LIN_UartSlave_Test(void);
void LIN_TwoBoardMaster_Test(void);
void LIN_TwoBoardSlave_Test(void);
//int32_t sysGetNum(void);


/*===== IrDATest.c =====*/
void Test_IrDA(E_UART_PORT u32Port);


/*===== MFPTest.c =====*/
#define MFOS_OPENDRAIN_OUTPUT 1
#define MFOS_PUSHPULL_OUTPUT 0
void MFOS_Test(E_UART_PORT u32Port);
void UART_MFOS_Test_PushPull(UART_T* tUART, uint32_t u32testRTSCTS);
void UART_MFOS_Test_OpenDrain(UART_T* tUART, uint32_t u32testRTSCTS);


/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/

//extern volatile uint32_t comStatus, comInDataWanted, comOutData, data_to_send;
//extern volatile uint8_t comInData;

extern volatile uint32_t TEST_PORT, TX_PORT, RX_PORT, u32ToutCnt;



/*===== for Tx and Rx data =====*/
#define TXBUFSIZE 0x213c
extern volatile uint32_t u32TxIndex, u32RxIndex;
extern uint8_t  u8RxData[TXBUFSIZE];
//extern uint16_t u16TxData[RXBUFSIZE], u16RxData[RXBUFSIZE];

/*===== for monitor test status =====*/
extern volatile uint32_t IsTestOver;
extern uint32_t u32ErrFlag, u32ErrFlagTotal;
//extern uint32_t u32TestPort, u32TestPortTx, u32TestPortRx;




/*===== for wake-up test =====*/
extern uint32_t u32WakeupFlag;



#define NO_SUPPORT 0
extern const uint32_t au32UARTtestBRfromHIRC[];
extern const uint32_t au32UARTtestBRfromHXT[];


/*---------------------------------------------------------------------------------------------------------*/
/* MFP Configuration (Rx, Tx, RTS, CTS)                                                                    */
/*---------------------------------------------------------------------------------------------------------*/

//#define TEST_GPIO_UART0 0
//#define TEST_GPIO_UART1 1
//#define TEST_GPIO_UART2 2
//#define TEST_GPIO_RX    0
//#define TEST_GPIO_TX    1
//#define TEST_GPIO_RTS   2
//#define TEST_GPIO_CTS   3

//extern uint32_t u32TestGpioPort, u32TestGpioPin;



void UartMFP_Test(E_UART_PORT u32Port);



#endif  // _DRVUART_H_
