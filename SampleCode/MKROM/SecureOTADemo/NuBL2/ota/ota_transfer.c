/**************************************************************************//**
 * @file     ota_transfer.c
 * @version  V1.00
 * @brief    Wifi demo code
 *
 * @note
 * Copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "M2351.h"
#include "ota_transfer.h"
#include "ota_api.h"
#include "esp8266.h"

//#define printf(...)

/*---------------------------------------------------------------------------------------------------------*/
/* Macro, type and constant definitions                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t g_au8SendBuf[BUF_SIZE]="CONNECT0\r\n"; //request new NuBL32 and NuBL33 FW
volatile uint32_t g_u32SendbytesLen = 8;
volatile uint8_t g_u8SendbytesFlag = 1;
volatile uint8_t g_u8ResetFlag = 0;
volatile uint8_t g_u8DisconnFlag = 0;
static volatile uint8_t s_u8InitialWifiFail = 0;

#if INPUT_WIFI_PROMPT
uint8_t g_au8WifiName[32];
uint8_t g_au8WifiPass[32];
uint8_t g_au8WifiIp[32];
//uint8_t g_au8WifiPort[32];
#endif

#define WIFI_PORT   UART3    // Used to connect to WIFI module

#define LED_OFF         PA11 // Green LED
#define PWR_OFF         PD7
#define FW_UPDATE_OFF   PD6
#define IOCTL_INIT      { \
    PD->MODE = (GPIO_MODE_OUTPUT << 6*2) | (GPIO_MODE_OUTPUT << 7*2); \
    PA->MODE = (GPIO_MODE_OUTPUT << 10*2) | (GPIO_MODE_OUTPUT << 11*2) ;}


/* ESP working structure and result enumeration */
static evol ESP_t ESP;
static ESP_Result_t espRes;

/* Client connection pointer */
static ESP_CONN_t* conn;

/* Connection manupulation */
static uint32_t s_u32BW;

/* ESP callback declaration */
int ESP_Callback(ESP_Event_t evt, ESP_EventParams_t* params);

static struct pt PT;

void SysTick_Handler(void);
__WEAK void UART3_ReceiveHandler(uint8_t c);
void UART3_IRQHandler(void);
void UART3_Init(void);
    
static
PT_THREAD(CLIENT_THREAD(struct pt* pt)) {
    PT_BEGIN(pt);

    PT_WAIT_UNTIL(pt, ESP_IsReady(&ESP) == espOK);          /* Wait stack to be ready first */
    espRes = ESP_GetLastReturnStatus(&ESP);             /* Get actual response status */

    /* Check connected status */
    if (espRes == espOK) {
//          printf("Connection to server has been successfull!\r\n");

        /* Send HTTP request for example in non-blocking mode */
        if ((espRes = ESP_CONN_Send(&ESP, conn, g_au8SendBuf, g_u32SendbytesLen, &s_u32BW, 0)) == espOK) {
            //DEBUG_MSG("Data sending has started successfully\r\n");

            PT_WAIT_UNTIL(pt, ESP_IsReady(&ESP) == espOK);  /* Wait for stack to finish */
            espRes = ESP_GetLastReturnStatus(&ESP);         /* Get actual response status */

            /* Check if data sent */
            if (espRes == espOK) {
                //printf("Data sent! Number of bytes sent: %d. We expect connection will be closed by remote server\r\n", s_u32BW);
                DEBUG_MSG("Data sent! Number of bytes sent: %d.\n", s_u32BW);
            } else {
                printf("Data were not sent: %d\r\n", espRes);
            }

            /* Anything received from server is handled in callback function! */
        } else {
            printf("Problems trying to start sending data!\r\n");
        }
    } else {
        printf("Problems to connect to server : %s(%d)\r\n\n", (espRes == espOK)?"OK":"ERROR", espRes);
    }
    g_u8SendbytesFlag = 0;
    PT_END(pt)
}

#if (PERIODIC_CHK_NEW_VER)
uint8_t Transfer_Connect(void)
{
    ESP_ConnectedStation_t ConnStation;
    uint16_t u16ConnNumbers;

    /* Check if connection was existed. */
    ESP_AP_ListConnectedStations(&ESP, (ESP_ConnectedStation_t *)&ConnStation, 1, (uint16_t *)&u16ConnNumbers, 1);

    /* Try to connect to wifi network in blocking mode */
    if ((espRes = ESP_STA_Connect(&ESP, WIFINAME, WIFIPASS, NULL, 0, 1)) == espOK) {
        printf("Connected to network\r\n");
    } else {
        printf("Problems trying to connect to network: %s(%d)\r\n", (espRes == espOK)?"OK":"ERROR", espRes);
    }

    if ((espRes = ESP_CONN_Start(&ESP, &conn, ESP_CONN_Type_TCP, WIFIIP, WIFIPORT, 0)) == espOK) {//can get IP address of AP by ESP_AP_GetIP()
        printf("\nConnection to server has started!\r\n");

    } else {
        printf("Problems trying to start connection to server as client: %d\r\n", espRes);
    }

    return espRes;
}
#endif

/**
  * @brief      System tick handler for transfer task
  * @param[in]  u32Ticks  System ticks
  * @retval     0         success
  * @retval     Other     fail
  * @details    The function is the system tick handler for transfer task.
  */
uint8_t Transfer_SysTickProcess(uint32_t u32Ticks)
{
    (void)u32Ticks;
    static uint32_t u32ChkNewVerTick = 0;
#if (PERIODIC_CHK_NEW_VER)
    uint8_t u8Ret;
#endif

    u32ChkNewVerTick++;
    ESP_UpdateTime(&ESP, 1);                                /* Update ESP library time for 1 ms */

#if (PERIODIC_CHK_NEW_VER)
    if (u32ChkNewVerTick == 60000)
    {
        uint8_t au8SendConn[]="CONNECT0\r\n"; //request new NuBL32 and NuBL33 FW
        u8Ret = Transfer_Connect();
        printf("Transfer_Connect: %d\n", u8Ret);
        if (u8Ret == 0)
        {
            printf("check new version\n");
            memcpy(g_au8SendBuf, au8SendConn, sizeof(au8SendConn));
            g_u8SendbytesFlag = 1;
        }
        u32ChkNewVerTick = 0;
    }
#endif
    return 0;
}

/*----------------------------------------------------------------------------
  SysTick IRQ Handler
 *----------------------------------------------------------------------------*/
void SysTick_Handler(void)
{
    static uint32_t u32Ticks;
    static uint32_t u32WDTTicks;

    u32WDTTicks++;

    if (u32WDTTicks == 10000) /* 10s */
    {
        /* Reset WDT counter */
        WDT_RESET_COUNTER();
        u32WDTTicks = 0;
    }

    /* Process system tick handler for transfer task */
    if (Transfer_SysTickProcess(u32Ticks))
        u32Ticks = 0;
    else
        u32Ticks++;

}

__WEAK void UART3_ReceiveHandler(uint8_t c) {(void)c; }

/**
  * @brief        Read received transfer data
  * @param        None
  * @return       None
  * @details      The function is used to read Rx data from WiFi module.
  */
void Transfer_WiFiProcess(void)
{
	if (UART_GET_INT_FLAG(WIFI_PORT, UART_INTSTS_RDAINT_Msk))
	{
        while(UART_IS_RX_READY(WIFI_PORT))
		{
//            printf("%c",UART_READ(WIFI_PORT));
            UART3_ReceiveHandler((uint8_t)UART_READ(WIFI_PORT));
        }
    }
}

/*----------------------------------------------------------------------------
  UART3 IRQ Handler(Wifi module)
 *----------------------------------------------------------------------------*/
void UART3_IRQHandler(void)
{
    Transfer_WiFiProcess();
}

/***********************************************/
/**               Library callback            **/
/***********************************************/
int ESP_Callback(ESP_Event_t evt, ESP_EventParams_t* params) {
    switch ((uint8_t)evt) {                              /* Check events */
        case espEventIdle:
//            printf("Stack is IDLE!\r\n");
            break;
        case espEventConnActive: {
            ESP_CONN_t* conn = (ESP_CONN_t *)(uint32_t)params->CP1;   /* Get connection for event */
            printf("Connection %d just became active!\r\n", conn->Number);

            break;
        }
        case espEventConnClosed: {
            ESP_CONN_t* conn = (ESP_CONN_t *)(uint32_t)params->CP1;   /* Get connection for event */
            printf("Connection %d was just closed!\r\n", conn->Number);
            if (g_u8ResetFlag)
            {
                //while(!(UART_IS_TX_EMPTY(DEBUG_PORT)));
                SYS_ResetChip();
            }
            break;
        }
        case espEventDataReceived: {
            //ESP_CONN_t* conn = (ESP_CONN_t *)params->CP1;   /* Get connection for event */
            conn = (ESP_CONN_t *)(uint32_t)params->CP1;
            uint8_t* data = (uint8_t *)(uint32_t)params->CP2;         /* Get actual received data */
            uint16_t datalen = (uint16_t)params->UI;                  /* Print length of received data in current part packet */

            DEBUG_MSG("%d bytes of data received\r\n", datalen);
            (void)(data);                                   /* Prevent compiler warnings */

            /* Call packet received call back function */
            OTA_API_RecvCallBack(data, datalen, 0, datalen);

            break;
        }
        default:
            break;
    }

    return 0;
}

/* Init UART3 settings(WiFi module) */
void UART3_Init(void)
{
    CLK->APBCLK0 |= CLK_APBCLK0_UART3CKEN_Msk;
    //CLK->CLKSEL3 = (CLK->CLKSEL3 & (~CLK_CLKSEL3_UART3SEL_Msk)) | CLK_CLKSEL3_UART3SEL_HIRC;
    CLK->CLKSEL3 = (CLK->CLKSEL3 & (~CLK_CLKSEL3_UART3SEL_Msk)) | CLK_CLKSEL3_UART3SEL_HXT;

    UART3->LINE = UART_PARITY_NONE | UART_STOP_BIT_1 | UART_WORD_LEN_8;
    //UART3->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC, 115200);
    UART3->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HXT, 115200);

    /* Set multi-function pins for UART3 RXD and TXD */
    SYS->GPD_MFPL = (SYS->GPD_MFPL & (~(UART3_RXD_PD0_Msk | UART3_TXD_PD1_Msk))) | UART3_RXD_PD0 | UART3_TXD_PD1;
}

/**
  * @brief      Init hardware for transfer task
  * @param      None
  * @retval     None
  * @details    This API is used for init hardware configure of transfer task
  */
void Transfer_Init(void)
{
    UART3_Init();

    IOCTL_INIT
    LED_OFF = 1;
    PWR_OFF = 1;
    FW_UPDATE_OFF = 1;

    CLK_SysTickLongDelay(3000000);

    //FW_UPDATE_OFF = 0; // Set 0 to enable WIFI module firmware update.
    FW_UPDATE_OFF = 1; // Set 1 to Disable WIFI module firmware update.
    CLK_SysTickLongDelay(1000000);

    LED_OFF = 0;
    PWR_OFF = 0;

    /* Generate Systick interrupt each 1 ms */
    SysTick_Config(SystemCoreClock / 1000);
}

/**
  * @brief Transfer task process
  * @param      None
  * @retval     0             Success
  * @retval     others        Failed
  * @details    Transfer task process
  */
int8_t Transfer_Process(void)
{
    /* Init ESP */
    if ((espRes = ESP_Init(&ESP, 115200, ESP_Callback)) == espOK) {
        printf("\nESP module init successfully!\r\n");
    } else {
        printf("\nESP Init error. Status: %d\r\n", espRes);
    }

#if INPUT_WIFI_PROMPT
    {
        uint8_t au8InputBuffer[32];
        uint8_t u8InputTmp;
        uint8_t u8Idx = 0;
//        uint16_t u16Port = 0;

        printf("\n========= Input connection settings =========\n");
        memset(au8InputBuffer, 0, sizeof(au8InputBuffer)/sizeof(uint8_t));
        memset(g_au8WifiName, 0, sizeof(g_au8WifiName)/sizeof(uint8_t));

        printf("WIFI AP NAME    : ");
        u8Idx = 0;
        while(u8InputTmp != 0x0D) /* enter key */
        {
            u8InputTmp = getchar();
            printf("%c",u8InputTmp);
            au8InputBuffer[u8Idx] = u8InputTmp;
            u8Idx++;
            if (u8Idx == ((sizeof(au8InputBuffer)/sizeof(uint8_t)) - 1))
            {
                printf("\n[ERROR]The length of input name is too long! \n");
                while(1){}
            }
        }
        memcpy(g_au8WifiName, au8InputBuffer, u8Idx - 1);

        memset(au8InputBuffer, 0, sizeof(au8InputBuffer)/sizeof(uint8_t));
        memset(g_au8WifiPass, 0, sizeof(g_au8WifiPass)/sizeof(uint8_t));
        u8InputTmp = 0;
        printf("\nWIFI AP PASSWORD: ");
        u8Idx = 0;
        while(u8InputTmp != 0x0D) /* enter key */
        {
            u8InputTmp = getchar();
            printf("%c",u8InputTmp);
            au8InputBuffer[u8Idx] = u8InputTmp;
            u8Idx++;
            if (u8Idx == ((sizeof(au8InputBuffer)/sizeof(uint8_t)) - 1))
            {
                printf("\n[ERROR]The length of input password is too long! \n");
                while(1){}
            }
        }
        memcpy(g_au8WifiPass, au8InputBuffer, u8Idx - 1);

        memset(au8InputBuffer, 0, sizeof(au8InputBuffer)/sizeof(uint8_t));
        memset(g_au8WifiIp, 0, sizeof(g_au8WifiIp)/sizeof(uint8_t));
        u8InputTmp = 0;
        printf("\nServer IP       : ");
        u8Idx = 0;
        while(u8InputTmp != 0x0D) /* enter key */
        {
            u8InputTmp = getchar();
            printf("%c",u8InputTmp);
            au8InputBuffer[u8Idx] = u8InputTmp;
            u8Idx++;
            if (u8Idx == ((sizeof(au8InputBuffer)/sizeof(uint8_t)) - 1))
            {
                printf("\n[ERROR]The length of input ip is too long! \n");
                while(1){}
            }
        }
        memcpy(g_au8WifiIp, au8InputBuffer, u8Idx - 1);
        u8InputTmp = 0;
        u8Idx = 0;

//        memset(au8InputBuffer, 0, sizeof(au8InputBuffer)/sizeof(uint8_t));
//        printf("\nWIFI PORT: ");
//        u8Idx = 0;
//        while(getchar() != 0x0D) /* enter key */
//        {
//            u8InputTmp = getchar();
//            printf("%c",u8InputTmp);
//            au8InputBuffer[u8Idx] = u8InputTmp;
//            u8Idx++;
//        }
//        memcpy(g_au8WifiPort, au8InputBuffer, sizeof(au8InputBuffer)/sizeof(uint8_t));
//
//        u16Port = ParseNumber(au8InputBuffer, NULL);

        printf("\n\n============== Start connection ==============\n");
    }

    /* Try to connect to wifi network in blocking mode */
    if ((espRes = ESP_STA_Connect(&ESP, (const char *)g_au8WifiName, (const char *)g_au8WifiPass, NULL, 0, 1)) == espOK) {
        printf("Connected to network successed.\r\n\n");
    } else {
        printf("Problems trying to connect to network: %s(%d)\r\n", (espRes == espOK)?"OK":"ERROR", espRes);
    }

    if ((espRes = ESP_CONN_Start(&ESP, &conn, ESP_CONN_Type_TCP, (const char *)g_au8WifiIp, 1111, 0)) == espOK) {//can get IP address of AP by ESP_AP_GetIP()
        printf("\nConnection to server has started!\r\n");
        s_u8InitialWifiFail = 0;

    } else {
        printf("Problems trying to start connection to server as client: %d\r\n", espRes);
        s_u8InitialWifiFail = 1;
    }
#else
    printf("\n\n============== Start connection ==============\n");
    /* Try to connect to wifi network in blocking mode */
    if ((espRes = ESP_STA_Connect(&ESP, WIFINAME, WIFIPASS, NULL, 0, 1)) == espOK) {
        printf("Connected to network successed.\r\n\n");
    } else {
        printf("Problems trying to connect to network: %s(%d)\r\n", (espRes == espOK)?"OK":"ERROR", espRes);
    }

    if ((espRes = ESP_CONN_Start(&ESP, &conn, ESP_CONN_Type_TCP, WIFIIP, WIFIPORT, 0)) == espOK) {//can get IP address of AP by ESP_AP_GetIP()
        printf("\nConnection to server has started!\r\n");

    } else {
        printf("Problems trying to start connection to server as client: %d\r\n", espRes);
    }
#endif

    while (1)
    {
        ESP_Update(&ESP); /* Update ESP stack */

        if (g_u8SendbytesFlag) /* Check if we should process */
        {
            CLIENT_THREAD(&PT); /* Process thread */
        }
        else
        {
            if (g_u8DisconnFlag == 1)
            {
                printf("Data sent! Number of bytes sent: %d.\n", s_u32BW);
                g_u8DisconnFlag = 0;
                ESP_CONN_Close(&ESP, conn, 0);
//                if ((FMC_GetVECMAP() == 0x40000)||(FMC_GetVECMAP() == 0x60000))
//                {
//                    /* exit NuBL2 loop, return to NuBL32 */
//                    break;
//                }
            }
            /* Do other stuff */
            //if (OTA_API_GetFwUpgradeDone())
            else if (OTA_API_GetFwUpgradeDone())
            {
                /* Exit this loop when firmware was upgrade done. */
                #if (!(OTA_UPGRADE_FROM_SD))
                //ESP_CONN_Close(&ESP, conn, 0);
                printf("g_u8DisconnFlag : %d\n",g_u8DisconnFlag);
                printf("waiting for WiFi disconnected...\n");
    //            while(!g_u8DisconnFlag){}
    //            g_u8DisconnFlag = 0;
                #endif
                break;
            }
        }
        if (s_u8InitialWifiFail == 1)
        {
            s_u8InitialWifiFail = 0;
            SYS_UnlockReg();
            FMC_Open();
            //check current firmware
            printf("VECMAP = 0x%x\n", FMC_GetVECMAP());
            //if (FMC_GetVECMAP() == 0x60000)
            if ((FMC_GetVECMAP() == 0x40000)||(FMC_GetVECMAP() == 0x60000))
            {
                /* exit NuBL2 loop, return to NuBL32 */
                break;
            }
        }
//        if (g_u8ResetFlag)
//        {
//            SYS_ResetChip();
//        }
    }

    return 0;
}

/**
  *    @brief        Send transfer data
  *    @param[in]    pu8TxBuf        The buffer to send the data
  *    @param[in]    u32Len          The data lengths
  *    @return       None
  *    @details      The function is to write data into send buffer to transmit data.
  */
void Transfer_SendBytes(uint8_t pu8TxBuf[], uint32_t u32Len)
{
    //printf("Transfer_SendBytes(%d),%d\n", u32Len,g_u8SendbytesFlag);
    if (g_u8SendbytesFlag == 0)
    {
        memcpy(g_au8SendBuf, pu8TxBuf, u32Len);
        g_u32SendbytesLen = u32Len;

        g_u8SendbytesFlag = 1;
    }
}

/**
  * @brief        Disconnect transfer connection
  * @param        None
  * @return       None
  * @details      The function is used to disconnect transfer connection.
  */
void Transfer_ConnClose(void)
{
    ESP_CONN_Close(&ESP, conn, 0);
}

/**
  * @brief        Set Reset flag for transfer task
  * @param        None
  * @return       None
  * @details      The function is used to set reset flag for transfer task.
  */
void Transfer_SetResetFlag(void)
{
    g_u8ResetFlag = 1;
}

/**
  * @brief        Set disconnect flag for transfer task
  * @param        None
  * @return       None
  * @details      The function is used to set disconnect flag for transfer task.
  */
void Transfer_SetDisconnFlag(void)
{
    g_u8DisconnFlag = 1;
}

/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/
