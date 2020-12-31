/**
 * |----------------------------------------------------------------------
 * | Copyright (c) 2016 Tilen Majerle
 * |
 * | Permission is hereby granted, free of charge, to any person
 * | obtaining a copy of this software and associated documentation
 * | files (the "Software"), to deal in the Software without restriction,
 * | including without limitation the rights to use, copy, modify, merge,
 * | publish, distribute, sublicense, and/or sell copies of the Software,
 * | and to permit persons to whom the Software is furnished to do so,
 * | subject to the following conditions:
 * |
 * | The above copyright notice and this permission notice shall be
 * | included in all copies or substantial portions of the Software.
 * |
 * | THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * | EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * | OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE
 * | AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * | HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * | WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * | FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * | OTHER DEALINGS IN THE SOFTWARE.
 * |----------------------------------------------------------------------
 */
#include "esp8266.h"

/******************************************************************************/
/******************************************************************************/
/***                           Private structures                            **/
/******************************************************************************/
/******************************************************************************/
typedef struct {
    uint8_t Length;
    uint8_t Data[128];
} Received_t;
#define RECEIVED_ADD(c)                     do { Received.Data[Received.Length++] = (c); Received.Data[Received.Length] = 0; } while (0)
#define RECEIVED_RESET()                    do { Received.Length = 0; Received.Data[0] = 0; } while (0)
#define RECEIVED_SHIFT()                    do { uint16_t i = 0; for (i = 0; i < Received.Length; i++) { Received.Data[i] = Received.Data[i + 1]; } Received.Data[i] = 0; if (Received.Length) { Received.Length--; } } while (0);
#define RECEIVED_LENGTH()                   Received.Length

typedef struct {
    evol const void* CPtr1;
    evol const void* CPtr2;
    evol const void* CPtr3;
    evol void* Ptr1;
    evol void* Ptr2;
    evol void** PPtr1;
    evol uint32_t UI;
} Pointers_t;

/******************************************************************************/
/******************************************************************************/
/***                           Private definitions                           **/
/******************************************************************************/
/******************************************************************************/
#define CHARISNUM(x)                        ((x) >= '0' && (x) <= '9')
#define CHARISHEXNUM(x)                     (((x) >= '0' && (x) <= '9') || ((x) >= 'a' && (x) <= 'f') || ((x) >= 'A' && (x) <= 'F'))
#define CHARTONUM(x)                        ((x) - '0')
#define CHARHEXTONUM(x)                     (((x) >= '0' && (x) <= '9') ? ((x) - '0') : (((x) >= 'a' && (x) <= 'f') ? ((x) - 'a' + 10) : (((x) >= 'A' && (x) <= 'F') ? ((x) - 'A' + 10) : 0)))
#define ISVALIDASCII(x)                     (((x) >= 32 && (x) <= 126) || (x) == '\r' || (x) == '\n')
#define FROMMEM(x)                          ((const char *)(x))

/* LL drivers */
#define UART_SEND_STR(str)                  do { Send.Data = (const uint8_t *)(str); Send.Count = strlen((const char *)(str)); ESP_LL_Callback(ESP_LL_Control_Send, &Send, &Send.Result); } while (0)
#define UART_SEND(str, len)                 do { Send.Data = (const uint8_t *)(str); Send.Count = (len); ESP_LL_Callback(ESP_LL_Control_Send, &Send, &Send.Result); } while (0)
#define UART_SEND_CH(ch)                    do { Send.Data = (const uint8_t *)(ch); Send.Count = 1; ESP_LL_Callback(ESP_LL_Control_Send, &Send, &Send.Result); } while (0)

#define RESP_OK                             FROMMEM("OK\r\n")
#define RESP_ERROR                          FROMMEM("ERROR\r\n")
#define RESP_BUSY                           FROMMEM("busy p...\r\n")
#define RESP_READY                          FROMMEM("ready\r\n")
#define _CRLF                               FROMMEM("\r\n")

/* List of commands */
#define CMD_IDLE                            ((uint16_t)0x0000)

/* Basic set of commands */
#define CMD_BASIC                           ((uint16_t)0x1000)
#define CMD_BASIC_AT                        ((uint16_t)0x1001)
#define CMD_BASIC_RST                       ((uint16_t)0x1002)
#define CMD_BASIC_GMR                       ((uint16_t)0x1003)
#define CMD_BASIC_GSLP                      ((uint16_t)0x1004)
#define CMD_BASIC_ATE                       ((uint16_t)0x1005)
#define CMD_BASIC_RESTORE                   ((uint16_t)0x1006)
#define CMD_BASIC_UART                      ((uint16_t)0x1007)
#define CMD_BASIC_SLEEP                     ((uint16_t)0x1008)
#define CMD_BASIC_WAKEUPGPIO                ((uint16_t)0x1009)
#define CMD_BASIC_RFPOWER                   ((uint16_t)0x100A)
#define CMD_BASIC_RFVDD                     ((uint16_t)0x100B)
#define CMD_BASIC_GETSYSRAM                 ((uint16_t)0x100C)
#define CMD_BASIC_GETSYSADC                 ((uint16_t)0x100D)
#define CMD_BASIC_SYSIOSETCFG               ((uint16_t)0x100E)
#define CMD_BASIC_SYSIOGETCFG               ((uint16_t)0x100F)
#define CMD_BASIC_SYSGPIOSETDIR             ((uint16_t)0x1010)
#define CMD_BASIC_SYSGPIOGETDIR             ((uint16_t)0x1011)
#define CMD_BASIC_SYSGPIOWRITE              ((uint16_t)0x1012)
#define CMD_BASIC_SYSGPIOREAD               ((uint16_t)0x1013)
#define CMD_IS_ACTIVE_BASIC(p)              ((p)->ActiveCmd >= 0x1000 && (p)->ActiveCmd < 0x2000)

/* Wifi commands */
#define CMD_WIFI                            ((uint16_t)0x2000)
#define CMD_WIFI_CWMODE                     ((uint16_t)0x2001)
#define CMD_WIFI_CWJAP                      ((uint16_t)0x2002)
#define CMD_WIFI_CWLAPOPT                   ((uint16_t)0x2003)
#define CMD_WIFI_CWLAP                      ((uint16_t)0x2004)
#define CMD_WIFI_CWQAP                      ((uint16_t)0x2005)
#define CMD_WIFI_CWSAP                      ((uint16_t)0x2006)
#define CMD_WIFI_CWLIF                      ((uint16_t)0x2007)
#define CMD_WIFI_CWDHCP                     ((uint16_t)0x2008)
#define CMD_WIFI_CWDHCPS                    ((uint16_t)0x2009)
#define CMD_WIFI_CWAUTOCONN                 ((uint16_t)0x200A)
#define CMD_WIFI_CIPSTAMAC                  ((uint16_t)0x200B)
#define CMD_WIFI_CIPAPMAC                   ((uint16_t)0x200C)
#define CMD_WIFI_CIPSTA                     ((uint16_t)0x200D)
#define CMD_WIFI_CIPAP                      ((uint16_t)0x200E)
#define CMD_WIFI_CWSTARTSMART               ((uint16_t)0x200F)
#define CMD_WIFI_CWSTOPSMART                ((uint16_t)0x2010)
#define CMD_WIFI_CWSTARTDISCOVER            ((uint16_t)0x2011)
#define CMD_WIFI_CWSTOPDISCOVER             ((uint16_t)0x2012)
#define CMD_WIFI_WPS                        ((uint16_t)0x2013)
#define CMD_WIFI_MDNS                       ((uint16_t)0x2014)

#define CMD_WIFI_GETSTAMAC                  ((uint16_t)0x2101)
#define CMD_WIFI_GETAPMAC                   ((uint16_t)0x2102)
#define CMD_WIFI_GETSTAIP                   ((uint16_t)0x2103)
#define CMD_WIFI_GETAPIP                    ((uint16_t)0x2104)
#define CMD_WIFI_SETSTAMAC                  ((uint16_t)0x2105)
#define CMD_WIFI_SETAPMAC                   ((uint16_t)0x2106)
#define CMD_WIFI_LISTACCESSPOINTS           ((uint16_t)0x2107)
#define CMD_WIFI_GETCWJAP                   ((uint16_t)0x2108)
#define CMD_WIFI_GETCWSAP                   ((uint16_t)0x2109)
#define CMD_WIFI_SETCWSAP                   ((uint16_t)0x210A)
#define CMD_WIFI_SETSTAIP                   ((uint16_t)0x210B)
#define CMD_WIFI_SETAPIP                    ((uint16_t)0x210C)
#define CMD_WIFI_SETHOSTNAME                ((uint16_t)0x200D)
#define CMD_WIFI_GETHOSTNAME                ((uint16_t)0x200E)
#define CMD_IS_ACTIVE_WIFI(p)               ((p)->ActiveCmd >= 0x2000 && (p)->ActiveCmd < 0x3000)

#define CMD_TCPIP                           ((uint16_t)0x3000)
#define CMD_TCPIP_CIPSTATUS                 ((uint16_t)0x3001)
#define CMD_TCPIP_CIPDOMAIN                 ((uint16_t)0x3002)
#define CMD_TCPIP_CIPSTART                  ((uint16_t)0x3003)
#define CMD_TCPIP_CIPSSLSIZE                ((uint16_t)0x3004)
#define CMD_TCPIP_CIPSEND                   ((uint16_t)0x3005)
#define CMD_TCPIP_CIPSENDEX                 ((uint16_t)0x3006)
#define CMD_TCPIP_CIPSENDBUF                ((uint16_t)0x3007)
#define CMD_TCPIP_CIPBUFSTATUS              ((uint16_t)0x3008)
#define CMD_TCPIP_CIPCHECKSEQ               ((uint16_t)0x3009)
#define CMD_TCPIP_CIPBUFRESET               ((uint16_t)0x300A)
#define CMD_TCPIP_CIPCLOSE                  ((uint16_t)0x300B)
#define CMD_TCPIP_CIFSR                     ((uint16_t)0x300C)
#define CMD_TCPIP_CIPMUX                    ((uint16_t)0x300D)
#define CMD_TCPIP_CIPSERVER                 ((uint16_t)0x300E)
#define CMD_TCPIP_CIPMODE                   ((uint16_t)0x300F)
#define CMD_TCPIP_SAVETRANSLINK             ((uint16_t)0x3010)
#define CMD_TCPIP_CIPSTO                    ((uint16_t)0x3011)
#define CMD_TCPIP_PING                      ((uint16_t)0x3012)
#define CMD_TCPIP_CIUPDATE                  ((uint16_t)0x3013)
#define CMD_TCPIP_CIPDINFO                  ((uint16_t)0x3014)
#define CMD_TCPIP_IPD                       ((uint16_t)0x3015)
#define CMD_TCPIP_TRANSFER_SEND             ((uint16_t)0x3016)
#define CMD_TCPIP_TRANSFER_STOP             ((uint16_t)0x3017)
#define CMD_TCPIP_CIPSNTPTIME               ((uint16_t)0x3018)
#define CMD_TCPIP_CIPDNS                    ((uint16_t)0x3119)

#define CMD_TCPIP_SERVERENABLE              ((uint16_t)0x3101)
#define CMD_TCPIP_SERVERDISABLE             ((uint16_t)0x3102)
#define CMD_TCPIP_SNTPSETCFG                ((uint16_t)0x3103)
#define CMD_TCPIP_SNTPGETCFG                ((uint16_t)0x3104)
#define CMD_TCPIP_CIPSETDNS                 ((uint16_t)0x3105)
#define CMD_TCPIP_CIPGETDNS                 ((uint16_t)0x3106)
#define CMD_IS_ACTIVE_TCPIP(p)              ((p)->ActiveCmd >= 0x3000 && (p)->ActiveCmd < 0x4000)

#define ESP_DEFAULT_BAUDRATE                115200              /* Default ESP8266 baudrate */
#define ESP_TIMEOUT                         30000               /* Timeout value in milliseconds */

/* In case ESP_RTOS_YIELD hasn't been defined */
#ifndef ESP_RTOS_YIELD
#define ESP_RTOS_YIELD()
#endif


/* Debug */
#define __DEBUG(fmt, ...)                   printf(fmt, ##__VA_ARGS__)

/* Delay milliseconds */
#if ESP_RTOS
#define __DELAYMS(ESP, x)                   do { volatile uint32_t t = (ESP)->Time; while (((ESP)->Time - t) < (x)) { ESP_RTOS_YIELD(); } } while (0)
#else
#define __DELAYMS(ESP, x)                   do { volatile uint32_t t = (ESP)->Time; while (((ESP)->Time - t) < (x)); } while (0)
#endif

/* Constants */
#define ESP_MAX_RFPWR                       82
#define ESP_MAX_SEND_DATA_LEN               2048

#if ESP_RTOS
#define __IS_BUSY(p)                        ((p)->ActiveCmd != CMD_IDLE || (p)->Flags.F.Call_Idle != 0)
#else
#define __IS_BUSY(p)                        ((p)->ActiveCmd != CMD_IDLE || (p)->Flags.F.Call_Idle != 0)
#endif
#define __IS_READY(p)                       (!__IS_BUSY(p))
#define __CHECK_BUSY(p)                     do { if (__IS_BUSY(p)) { __RETURN(ESP, espBUSY); } } while (0)
#define __CHECK_INPUTS(c)                   do { if (!(c)) { __RETURN(ESP, espPARERROR); } } while (0)

#define __CONN_RESET(c)                     do { uint8_t number = (c)->Number; memset((void *)(c), 0x00, sizeof(ESP_CONN_t)); (c)->Number = number; } while (0)
#define __CONN_UPDATE_TIME(e, c)            (c)->PollTime = (e)->Time

#if ESP_RTOS
#define __IDLE(p)                           do {\
    uint8_t result = 1;                         \
    if (ESP_LL_Callback(ESP_LL_Control_SYS_Release, (void *)&(p)->Sync, &result) || result) {   \
                                                \
    }                                           \
    (p)->ActiveCmd = CMD_IDLE;                  \
    __RESET_THREADS(p);                         \
    if (!(p)->Flags.F.IsBlocking) {             \
        (p)->Flags.F.Call_Idle = 1;             \
    }                                           \
    memset((void *)&Pointers, 0x00, sizeof(Pointers));  \
} while (0)
#else
#define __IDLE(p)                           do {\
    (p)->ActiveCmd = CMD_IDLE;                  \
    __RESET_THREADS(p);                         \
    if (!(p)->Flags.F.IsBlocking) {             \
        (p)->Flags.F.Call_Idle = 1;             \
    }                                           \
    memset((void *)&Pointers, 0x00, sizeof(Pointers));  \
} while (0)
#endif

#if ESP_RTOS
#define __ACTIVE_CMD(p, cmd)                do {\
    uint8_t result = 1;                         \
    if (ESP_LL_Callback(ESP_LL_Control_SYS_Request, (void *)&(p)->Sync, &result) || result) {   \
        /* __RETURN(p, espTIMEOUT); */          \
    }                                           \
    if ((p)->ActiveCmd == CMD_IDLE) {           \
        (p)->ActiveCmdStart = (p)->Time;        \
    }                                           \
    (p)->ActiveCmd = (cmd);                     \
} while (0)
#else
#define __ACTIVE_CMD(p, cmd)                do {\
    if ((p)->ActiveCmd == CMD_IDLE) {           \
        (p)->ActiveCmdStart = (ESP)->Time;      \
    }                                           \
    (p)->ActiveCmd = (cmd);                     \
} while (0)
#endif /* ESP_RTOS */

#define __CMD_SAVE(p)                       (p)->ActiveCmdSaved = (p)->ActiveCmd
#define __CMD_RESTORE(p)                    (p)->ActiveCmd = (p)->ActiveCmdSaved

#define __RETURN(p, v)                      do { (p)->RetVal = (v); return (v); } while (0)
#define __RETURN_BLOCKING(p, b, mt)         return __return_blocking(p, b, mt);

#define __RST_EVENTS_RESP(p)                do { (p)->Events.Value = 0; (p)->ActiveCmdStart = (p)->Time; } while (0)

#define ESP_CALL_CALLBACK(p, e)             (p)->Callback(e, (ESP_EventParams_t *)&(p)->CallbackParams);
#define ESP_CALL_CONN_CALLBACK(p, c, e)     (c)->Cb(e, (ESP_EventParams_t *)&(p)->CallbackParams);

#if ESP_USE_CTS
#define ESP_SET_RTS(p, s)                   do {\
    if (RTSStatus != (s) && !(p)->Flags.F.RTSForced) {  \
        RTSStatus = (s);                        \
        ESP_LL_SetRTS((ESP_LL_t *)&(p)->LL, (s));   \
    }                                           \
} while (0)
#else
#define ESP_SET_RTS(p, s)                   (void)0
#endif /* ESP_USE_CTS */

/* Check device CIPSTATUS */
#define __CHECK_CIPSTATUS(p)                do {\
    __RST_EVENTS_RESP((ESP));                   \
    UART_SEND_STR(FROMMEM("AT+CIPSTATUS"));     \
    UART_SEND_STR(FROMMEM(_CRLF));              \
    StartCommand((ESP), CMD_TCPIP_CIPSTATUS, NULL); \
    PT_WAIT_UNTIL(pt, (ESP)->Events.F.RespOk || \
                    (ESP)->Events.F.RespError); \
} while (0)

/******************************************************************************/
/******************************************************************************/
/***                            Private variables                            **/
/******************************************************************************/
/******************************************************************************/
#if ESP_USE_CTS
static
uint8_t RTSStatus;                                          /* RTS pin status */
#endif /* ESP_USE_CTS */

/* Buffers */
static BUFFER_t Buffer;                                     /* Buffer structure */
static uint8_t Buffer_Data[ESP_BUFFER_SIZE + 1];            /* Buffer data array */
static Received_t Received;                                 /* Received data structure */
static Pointers_t Pointers;                                 /* Pointers object */
#if ESP_CONN_SINGLEBUFFER
static uint8_t IPD_Data[ESP_CONNBUFFER_SIZE + 1];           /* Data buffer for incoming connection */
#endif /* ESP_CONN_SINGLEBUFFER */

static
struct pt pt_BASIC, pt_WIFI, pt_TCPIP;                      /* Protothread setup */

static
ESP_LL_Send_t Send;                                         /* Send data setup */

#define __RESET_THREADS(ESP)                  do {          \
PT_INIT(&pt_BASIC); PT_INIT(&pt_WIFI); PT_INIT(&pt_TCPIP);  \
} while (0);

/******************************************************************************/
/******************************************************************************/
/***                            Private functions                            **/
/******************************************************************************/
/******************************************************************************/
/* Blocking return */
ESP_Result_t __return_blocking(evol ESP_t* p, uint32_t b, uint32_t mt) {
    ESP_Result_t res;
    (p)->ActiveCmdTimeout = mt;
    if (!(b)) {
        (p)->Flags.F.IsBlocking = 0;
        __RETURN(p, espOK);
    }
    (p)->Flags.F.IsBlocking = 1;
    res = ESP_WaitReady(p, mt);
    if (res == espTIMEOUT) {
        return espTIMEOUT;
    }
    res = (p)->ActiveResult;
    (p)->ActiveResult = espOK;
    return res;
}

/* Default callback for events */
estatic
int ESP_CallbackDefault(ESP_Event_t evt, ESP_EventParams_t* params) {
    (void)evt;
    (void)params;
    return 0;
}

/* Returns number from hex value */
estatic
uint8_t Hex2Num(char a) {
    if (a >= '0' && a <= '9') {                             /* Char is num */
        return a - '0';
    } else if (a >= 'a' && a <= 'f') {                      /* Char is lowercase character A - Z (hex) */
        return (a - 'a') + 10;
    } else if (a >= 'A' && a <= 'F') {                      /* Char is uppercase character A - Z (hex) */
        return (a - 'A') + 10;
    }

    return 0;
}

/* Parses and returns number from string */
estatic
int32_t ParseNumber(const char* ptr, uint8_t* cnt) {
    uint8_t minus = 0, i = 0;
    int32_t sum = 0;

    if (*ptr == '-') {                                		/* Check for minus character */
        minus = 1;
        ptr++;
        i++;
    }
    while (CHARISNUM(*ptr)) {                        		/* Parse number */
        sum = 10 * sum + CHARTONUM(*ptr);
        ptr++;
        i++;
    }
    if (cnt) {                                		        /* Save number of characters used for number */
        *cnt = i;
    }
    if (minus) {                                    		/* Minus detected */
        return 0 - sum;
    }
    return sum;                                       		/* Return number */
}

/* Parses and returns HEX number from string */
estatic
uint32_t ParseHexNumber(const char* ptr, uint8_t* cnt) {
    uint32_t sum = 0;
    uint8_t i = 0;

    while (CHARISHEXNUM(*ptr)) {                    		/* Parse number */
        sum <<= 4;
        sum += Hex2Num(*ptr);
        ptr++;
        i++;
    }

    if (cnt) {                               		        /* Save number of characters used for number */
        *cnt = i;
    }
    return sum;                                        		/* Return number */
}

/* Parse MAC number in string format xx:xx:xx:xx:xx:xx */
estatic
void ParseMAC(evol ESP_t* ESP, const char* str, uint8_t* mac, uint8_t* cnt) {
    uint8_t i = 6;

    (void)ESP;                                              /* Process unused */

    while (i--) {
        *mac++ = ParseHexNumber(str, NULL);
        str += 3;
    }
    if (cnt) {
        *cnt = 17;
    }
}

/* Parse IP number in string format xxx.xxx.xxx.xxx */
estatic
void ParseIP(evol ESP_t* ESP, const char* str, uint8_t* ip, uint8_t* cnt) {
    uint8_t i = 4;
    uint8_t c = 0;

    (void)ESP;                                              /* Process unused */

    if (cnt) {
        *cnt = 0;
    }
    while (i--) {
        *ip++ = ParseNumber(str, &c);
        str += c + 1;
        if (cnt) {
            *cnt += c;
            if (i) {
                *cnt += 1;
            }
        }
    }
}

/* Parse +CWLAP statement */
estatic
void ParseCWLAP(evol ESP_t* ESP, const char* str, ESP_AP_t* AP) {
    uint8_t cnt;

    (void)ESP;                                              /* Process unused */

    if (*str == '(') {                                      /* Remove opening bracket */
        str++;
    }

    memset((void *)AP, 0x00, sizeof(ESP_AP_t));             /* Reset structure first */

    AP->Ecn = (ESP_Ecn_t)ParseNumber(str, &cnt);            /* Parse ECN value */
    str += cnt + 1;

    if (*str == '"') {                                      /* Remove opening " */
        str++;
    }

    cnt = 0;                                                /* Parse SSID */
    while (*str) {
        if (*str == '"' && *(str + 1) == ',') {
            break;
        }
        if (cnt < sizeof(AP->SSID) - 1) {
            AP->SSID[cnt] = *str;
        }

        cnt++;
        str++;
    }

    str += 2;                                               /* Parse RSSI */
    AP->RSSI = ParseNumber(str, &cnt);

    str += cnt + 1;
    if (*str == '"') {
        str++;
    }
    ParseMAC(ESP, str, AP->MAC, NULL);                      /* Parse MAC */
    str += 19;                                              /* Ignore mac, " and comma */
    AP->Channel = ParseNumber(str, &cnt);                   /* Parse channel for wifi */
    str += cnt + 1;
    AP->Offset = ParseNumber(str, &cnt);                    /* Parse offset */
    str += cnt + 1;
    AP->Calibration = ParseNumber(str, &cnt);               /* Parse calibration number */
    str += cnt + 1;
}

/* Parse +CWJAP statement */
estatic
void ParseCWJAP(evol ESP_t* ESP, const char* ptr, ESP_ConnectedAP_t* AP) {
    uint8_t i, cnt;

    (void)ESP;                                              /* Process unused */

    while (*ptr && *ptr != '"') {                    		/* Find first " character */
        ptr++;
    }
    if (!*ptr) {                                    		/* Check if zero detected */
        return;
    }
    ptr++;                                            		/* Remove first " for SSID */
    i = 0;                                            		/* Parse SSID part */
    while (*ptr && (*ptr != '"' || *(ptr + 1) != ',' || *(ptr + 2) != '"')) {
        AP->SSID[i++] = *ptr++;
    }
    AP->SSID[i++] = 0;
    ptr += 3;                                        		/* Increase pointer by 3, ignore "," part */
    ParseMAC(ESP, ptr, AP->MAC, NULL);    	                /* Get MAC */
    ptr += 19;                                    		    /* Increase counter by elements in MAC address and ", part */
    AP->Channel = ParseNumber(ptr, &cnt);	                /* Get channel */
    ptr += cnt + 1;                                    		/* Increase position */
    AP->RSSI = ParseNumber(ptr, &cnt);    /* Get RSSI */
}

/* Parse CWLIF statement with IP and MAC */
estatic
void ParseCWLIF(evol ESP_t* ESP, const char* str, ESP_ConnectedStation_t* station) {
    uint8_t cnt;

    ParseIP(ESP, str, station->IP, &cnt);                   /* Parse IP address */
    str += cnt + 1;
    ParseMAC(ESP, str, station->MAC, &cnt);                 /* Parse MAC */
}

/* Parse incoming IPD statement */
estatic
void ParseIPD(evol ESP_t* ESP, const char* str, ESP_IPD_t* IPD) {
    uint8_t cnt;

    memset((void *)IPD, 0x00, sizeof(ESP_IPD_t));           /* Reset structure */

#if !ESP_SINGLE_CONN
    IPD->Conn = (ESP_CONN_t *)&ESP->Conn[ParseNumber(str, &cnt)];   /* Get connection */
    str += cnt + 1;
#else
    IPD->Conn = (ESP_CONN_t *)&ESP->Conn[0];                /* Get connection */
#endif /* !ESP_SINGLE_CONN */
    __CONN_UPDATE_TIME(ESP, IPD->Conn);                     /* Update connection access time */
    IPD->BytesRemaining = ParseNumber(str, &cnt);           /* Set bytes remaining to read */
}

/* Parses +CWSAP statement */
estatic
void ParseCWSAP(evol ESP_t* ESP, const char* ptr, ESP_APConfig_t* AP) {
    uint8_t cnt, i;

    (void)ESP;                                              /* Process unused */

    memset((void *)AP, 0x00, sizeof(ESP_APConfig_t));       /* Reset structure */

    i = 0;                                            		/* Copy till "," which indicates end of SSID string and start of password part */
    while (*ptr && (*ptr != '"' || *(ptr + 1) != ',' || *(ptr + 2) != '"')) {
        AP->SSID[i++] = *ptr++;
    }
    AP->SSID[i++] = 0;
    ptr += 3;                                        		/* Increase pointer by 3, ignore "," part */
    i = 0;                                            		/* Copy till ", which indicates end of password string and start of number */
    while (*ptr && (*ptr != '"' || *(ptr + 1) != ',')) {
        AP->Pass[i++] = *ptr++;
    }
    AP->Pass[i++] = 0;
    ptr += 2;                                        		/* Increase pointer by 2 */
    AP->Channel = ParseNumber(ptr, &cnt);    		        /* Get channel number */
    ptr += cnt + 1;                                    		/* Increase pointer and comma */
    AP->Ecn = (ESP_Ecn_t)ParseNumber(ptr, &cnt);            /* Get ECN value */
    ptr += cnt + 1;                                    		/* Increase pointer and comma */
    AP->MaxConnections = ParseNumber(ptr, &cnt);            /* Get max connections value */
    ptr += cnt + 1;                                    		/* Increase pointer and comma */
    AP->Hidden = ParseNumber(ptr, &cnt);    		        /* Get hidden value */
	(void)ESP;
}

/* Parse CIPSTATUS value */
estatic
void ParseCIPSTATUS(evol ESP_t* ESP, uint8_t* value, const char* str) {
    uint8_t i, cnt;
    uint8_t connNumber = 0;

#if !ESP_SINGLE_CONN
    connNumber = CHARTONUM(*str);                           /* Get connection number */
#endif /* !ESP_SINGLE_CONN */
    *value |= 1 << connNumber;                              /* Set bit according to active connection */

    /* Parse connection parameters */
    str += 2;
    while (*str && *str != ',') {
        str++;
    }
    str++;

    /* Parse connection IP */
    str++;
    for (i = 0; i < 4; i++) {
        ESP->Conn[connNumber].RemoteIP[i] = ParseNumber(str, &cnt);  /* Get remote IP */
        str += cnt + 1;
    }
    str++;

    /* Parse Remove PORT */
    ESP->Conn[connNumber].RemotePort = ParseNumber(str, &cnt);
    str += cnt + 1;

    /* Parse local port */
    ESP->Conn[connNumber].LocalPort = ParseNumber(str, &cnt);
    str += cnt + 1;

    /* Parse client/server type */
    ESP->Conn[connNumber].Flags.F.Client = CHARTONUM(*str) == 0;
}

/* Parse SYSGPIOREAD value */
estatic
void ParseSysGPIORead(evol ESP_t* ESP, const char* str, uint8_t* level, ESP_GPIO_Dir_t* dir) {
    uint8_t cnt;

    (void)ESP;                                              /* Process unused */

    ParseNumber(str, &cnt);                                 /* Parse GPIO number */
    str += cnt + 1;

    if (dir) {                                              /* If pointer to direction is set */
        *dir = (ESP_GPIO_Dir_t)ParseNumber(str, &cnt);      /* Parse number and return value */
    } else {
        ParseNumber(str, &cnt);                             /* Perform dummy read only */
    }
    str += cnt + 1;

    if (level) {                                            /* Save level value */
        *level = ParseNumber(str, NULL);
    }
}

/* Parse CWHOSTNAME value */
estatic
void ParseHostName(evol ESP_t* ESP, const char* str, char* dest) {
    (void)ESP;
    if (*str == '"') {                                      /* Ignore " on beginning */
        str++;
    }

    while (*str) {                                          /* Parse entire string */
        if (*str == '"' && (*(str + 1) == ',' || *(str + 1) == '\n')) {
            break;
        }
        *dest++ = *str++;
    }
    *dest = 0;
}

/* Parse CIPSNTPTIME value: "Thu Aug 04 14:48:05 2016" */
estatic
void ParseSNTPTime(evol ESP_t* ESP, const char* str, ESP_DateTime_t* dt) {
    uint8_t cnt;

    (void)ESP;                                              /* Process unused */

    /* Find day in a week */
    if (strncmp(str, FROMMEM("Mon"), 3) == 0) {
        dt->Day = 1;
    } else if (strncmp(str, FROMMEM("Tue"), 3) == 0) {
        dt->Day = 2;
    } else if (strncmp(str, FROMMEM("Wed"), 3) == 0) {
        dt->Day = 3;
    } else if (strncmp(str, FROMMEM("Thu"), 3) == 0) {
        dt->Day = 4;
    } else if (strncmp(str, FROMMEM("Fri"), 3) == 0) {
        dt->Day = 5;
    } else if (strncmp(str, FROMMEM("Sat"), 3) == 0) {
        dt->Day = 6;
    } else if (strncmp(str, FROMMEM("Sun"), 3) == 0) {
        dt->Day = 7;
    }
    str += 4;

    /* Find month in a year */
    if (strncmp(str, FROMMEM("Jan"), 3) == 0) {
        dt->Month = 1;
    } else if (strncmp(str, FROMMEM("Feb"), 3) == 0) {
        dt->Month = 2;
    } else if (strncmp(str, FROMMEM("Mar"), 3) == 0) {
        dt->Month = 3;
    } else if (strncmp(str, FROMMEM("Apr"), 3) == 0) {
        dt->Month = 4;
    } else if (strncmp(str, FROMMEM("May"), 3) == 0) {
        dt->Month = 5;
    } else if (strncmp(str, FROMMEM("Jun"), 3) == 0) {
        dt->Month = 6;
    } else if (strncmp(str, FROMMEM("Jul"), 3) == 0) {
        dt->Month = 7;
    } else if (strncmp(str, FROMMEM("Aug"), 3) == 0) {
        dt->Month = 8;
    } else if (strncmp(str, FROMMEM("Sep"), 3) == 0) {
        dt->Month = 9;
    } else if (strncmp(str, FROMMEM("Oct"), 3) == 0) {
        dt->Month = 10;
    } else if (strncmp(str, FROMMEM("Nov"), 3) == 0) {
        dt->Month = 11;
    } else if (strncmp(str, FROMMEM("Dec"), 3) == 0) {
        dt->Month = 12;
    }
    str += 3;
    while (str && *str != ' ') {                            /* Ignore all possible entries (JunE, JulY, etc) from month and go to next valid entry */
        str++;
    }
    str++;

    dt->Date = ParseNumber(str, &cnt);                      /* Get day in month */
    str += cnt + 1;
    dt->Hours = ParseNumber(str, &cnt);                     /* Get hours in day */
    str += cnt + 1;
    dt->Minutes = ParseNumber(str, &cnt);                   /* Get minutes in hour */
    str += cnt + 1;
    dt->Seconds = ParseNumber(str, &cnt);                   /* Get seconds in minute */
    str += cnt + 1;
    dt->Year = ParseNumber(str, &cnt);                      /* Get year */
}

/* Parse SNTP config */
estatic
void ParseSNTPConfig(evol ESP_t* ESP, const char* str, ESP_SNTP_t* conf) {
    uint8_t cnt, i;
    char *dst;

    (void)ESP;                                              /* Process unused */

    conf->Enable = ParseNumber(str, &cnt);                  /* Get enabled status */
    str += cnt + 1;

    conf->Timezone = (int8_t)ParseNumber(str, &cnt);        /* Get timezone */
    str += cnt + 1;

    /* Parse server addresses */
    for (i = 0; i < sizeof(conf->Addr) / sizeof(conf->Addr[0]); i++) {
        if (!conf->Addr[i]) {                               /* Check if memory is set */
            break;
        }
        dst = conf->Addr[i];                                /* Set destination pointer */
        if (*str == '"') {
            str++;
        }
        while (*str) {                                      /* Process entire string */
            if (*str == '"') {                              /* Check if end of server received */
                if (*(str + 1) == ',') {                    /* If comma is next */
                    str += 2;                               /* Ignore this char and next one */
                    break;                                  /* Stop parsing this server and go to next one */
                } else if (*(str + 1) == '\r') {            /* If new line received */
                    return;                                 /* Stop function execution */
                }
            }
            *dst = *str;
            str++;
            dst++;
        }
        *dst = 0;
    }
}

/* Parse SYSIOGETCFG value */
estatic
void ParseSysIOGetCfg(evol ESP_t* ESP, const char* str, ESP_GPIO_t* conf) {
    uint8_t cnt;

    (void)ESP;                                              /* Process unused */

    conf->Pin = ParseNumber(str, &cnt);                     /* Parse pin number */
    str += cnt + 1;

    conf->Mode = (ESP_GPIO_Mode_t)ParseNumber(str, &cnt);   /* Get GPIO mode */
    str += cnt + 1;

    conf->Pull = (ESP_GPIO_Pull_t)ParseNumber(str, &cnt);   /* Set pull resistor value */
}

/* Parse CIPDNS value */
estatic
void ParseCIPDNS(evol ESP_t* ESP, const char* str, ESP_DNS_t* dns) {
    uint8_t i = 0, cnt;

    (void)ESP;                                              /* Process unused */

    if (dns->_ptr >= (sizeof(dns->Addr) / sizeof(dns->Addr[0]))) {  /* Check if any available memory */
        return;
    }
    if (*str == '"') {
        str++;
    }
    for (i = 0; i < 4; i++) {
        dns->Addr[dns->_ptr][i] = ParseNumber(str, &cnt);   /* Parse IP number */
        str += cnt + 1;                                     /* Go to next number */
    }
    dns->_ptr++;                                            /* Increase DNS pointer by 1 */
}

/* Starts command and sets pointer for return statement */
estatic
ESP_Result_t StartCommand(evol ESP_t* ESP, uint16_t cmd, const char* cmdResp) {
    ESP->ActiveCmd = cmd;
    ESP->ActiveCmdResp = (char *)cmdResp;
    ESP->ActiveCmdStart = ESP->Time;
    ESP->ActiveResult = espOK;

    if (cmd == CMD_TCPIP_CIPSTATUS) {                       /* On CIPSTATUS command */
        ESP->ActiveConnsResp = 0;                           /* Reset active connections status */
    }

    return espOK;
}

/* Converts number to string */
estatic
void NumberToString(char* str, uint32_t number) {
    sprintf(str, "%lu", (unsigned long)number);
}

/* Converts number to hex for MAC */
estatic
void HexNumberToString(char* str, uint8_t number) {
    sprintf(str, "%02X", (unsigned)number);
}

/* Escapes string and sends directly to output stream */
estatic
void EscapeStringAndSend(const char* str) {
    char special = '\\';

    while (*str) {                                    		/* Go through string */
        if (*str == ',' || *str == '"' || *str == '\\') {	/* Check for special character */
            UART_SEND_CH(&special);                         /* Send special character */
        }
        UART_SEND_CH(str++);                                /* Send character */
    }
}

/* Process received character */
estatic
void ParseReceived(evol ESP_t* ESP, Received_t* Received_p) {
    char* str = (char *)Received_p->Data;

    uint8_t is_ok = 0, is_error = 0, len;
    len = Received_p->Length;                               /* String length */

    if (*str == '\r' && *(str + 1) == '\n') {               /* Check empty line */
        return;
    }

    is_ok = strcmp(str, RESP_OK) == 0;                      /* Check if OK received */
    if (!is_ok) {
        is_error = strcmp(str, RESP_ERROR) == 0;            /* Check if error received */
        if (!is_error) {
            is_error = strcmp(str, RESP_BUSY) == 0;         /* Check if busy */
        }
    }

    if (!is_ok && !is_error) {
        if (strcmp(str, RESP_READY) == 0) {
            ESP->Events.F.RespReady = 1;                    /* Device is ready flag */
        }
    }

    /* Device info */
    if (ESP->ActiveCmd == CMD_BASIC_GMR) {
        if ((str[0] == 'A' || str[0] == 'a') && Pointers.CPtr1 && len > 13) {   /* "AT version:" received */
            strncpy((char *)Pointers.CPtr1, &str[11], len - 13);    /* Save AT version */
            ((char *)Pointers.CPtr1)[len - 13] = 0;
        }
        if ((str[0] == 'S' || str[0] == 's') && Pointers.CPtr2) {   /* "SDK version:" received */
            strncpy((char *)Pointers.CPtr2, &str[12], len - 14);    /* Save SDK version */
            ((char *)Pointers.CPtr2)[len - 14] = 0;         /* End of strings */
        }
        if ((str[0] == 'C' || str[0] == 'c') && Pointers.CPtr3) {   /* "compile time:" received */
            strncpy((char *)Pointers.CPtr3, &str[13], len - 15);    /* Save compile time version */
            ((char *)Pointers.CPtr3)[len - 15] = 0;         /* End of strings */
        }
    }

    if (ESP->ActiveCmd == CMD_WIFI_CWLIF && CHARISNUM(str[0])) {    /* IP of device connected to AP received */
        if (*(uint16_t *)Pointers.Ptr2 < Pointers.UI) {     /* Check if memory still available */
            ParseCWLIF(ESP, str, (void *)Pointers.Ptr1);    /* Parse CWLIF statement */
            Pointers.Ptr1 = ((ESP_ConnectedStation_t *)Pointers.Ptr1) + 1;
            *(uint16_t *)Pointers.Ptr2 = (*(uint16_t *)Pointers.Ptr2) + 1;  /* Increase number of parsed elements */
        }
    }

    /* We received string starting with + sign = some useful data! */
    if (*str == '+') {
        if (strncmp(str, FROMMEM("+IPD"), 4) == 0) {        /* Check for incoming data */
            ParseIPD(ESP, str + 5, (void *)&ESP->IPD);      /* Parse incoming data string */
            ESP->IPD.InIPD = 1;                             /* Start with data reading */
            if (!ESP->IPD.Conn->TotalBytesReceived) {
                ESP->IPD.Conn->DataStartTime = (uint32_t)ESP->Time; /* Set time when first IPD received on connection */
            }
            ESP->IPD.Conn->TotalBytesReceived += ESP->IPD.BytesRemaining;   /* Increase total bytes received so far */
        } else if (ESP->ActiveCmd == CMD_WIFI_CWLAP && strncmp(str, FROMMEM("+CWLAP"), 6) == 0) {  /* When active command is listing wifi stations */
            if (*(uint16_t *)Pointers.Ptr2 < Pointers.UI) { /* Check if memory still available */
                ParseCWLAP(ESP, str + 7, (void *)Pointers.Ptr1);    /* Parse CWLAP statement */
                Pointers.Ptr1 = ((ESP_AP_t *)Pointers.Ptr1) + 1;
                *(uint32_t *)Pointers.Ptr2 = (*(uint16_t *)Pointers.Ptr2) + 1;  /* Increase number of parsed elements */
            }
        } else if (ESP->ActiveCmd == CMD_WIFI_CWSAP && strncmp(str, FROMMEM("+CWSAP"), 6) == 0) {   /* Check CWSAP response */
            ParseCWSAP(ESP, str + 12, (void *)&ESP->APConf);    /* Parse config from AP */
        } else if (ESP->ActiveCmd == CMD_TCPIP_PING && CHARISNUM(str[1])) {
            *(uint32_t *)Pointers.Ptr1 = ParseNumber(str + 1, NULL);    /* Parse response time */
        } else if (ESP->ActiveCmd == CMD_BASIC_GETSYSRAM && strncmp(str, FROMMEM("+SYSRAM"), 7) == 0) {
            *(uint32_t *)Pointers.Ptr1 = ParseNumber(str + 8, NULL);    /* Parse RAM value */
        } else if (ESP->ActiveCmd == CMD_BASIC_GETSYSADC && strncmp(str, FROMMEM("+SYSADC"), 7) == 0) {
            *(uint32_t *)Pointers.Ptr1 = ParseNumber(str + 8, NULL);    /* Parse ADC value */
        } else if (ESP->ActiveCmd == CMD_BASIC_SYSGPIOREAD && strncmp(str, FROMMEM("+SYSGPIOREAD"), 12) == 0) {
            ParseSysGPIORead(ESP, str + 13, (void *)Pointers.Ptr1, (void *)Pointers.Ptr2);
        } else if (ESP->ActiveCmd == CMD_BASIC_SYSIOGETCFG && strncmp(str, FROMMEM("+SYSIOGETCFG"), 12) == 0) {
            ParseSysIOGetCfg(ESP, str + 13, (void *)Pointers.Ptr1);
        } else if (ESP->ActiveCmd == CMD_WIFI_CIPAP && strncmp(str, FROMMEM("+CIPAP_"), 7) == 0) {  /* +CIPAP received */
            if (str[11] == 'i') {                           /* +CIPAP_CUR:ip received */
                ParseIP(ESP, str + 15, (void *)&ESP->APIP, NULL);    /* Parse IP string */
                if (Pointers.Ptr1) {
                    memcpy((void *)Pointers.Ptr1, (void *)&ESP->APIP, 4);
                }
            } else if (str[11] == 'g') {
                ParseIP(ESP, str + 20, (void *)&ESP->APGateway, NULL);  /* Parse IP string */
            } else if (str[11] == 'n') {
                ParseIP(ESP, str + 20, (void *)&ESP->APNetmask, NULL);  /* Parse IP string */
            }
        } else  if (ESP->ActiveCmd == CMD_WIFI_CIPSTA && strncmp(str, FROMMEM("+CIPSTA_"), 8) == 0) {   /* +CIPSTA received */
            if (str[12] == 'i') {                               /* +CIPSTA_CUR:ip received */
                ParseIP(ESP, str + 16, (void *)&ESP->STAIP, NULL);  /* Parse IP string */
                if (Pointers.Ptr1) {
                    memcpy((void *)Pointers.Ptr1, (void *)&ESP->STAIP, 4);
                }
            } else if (str[12] == 'g') {
                ParseIP(ESP, str + 21, (void *)&ESP->STAGateway, NULL); /* Parse IP string */
            } else if (str[12] == 'n') {
                ParseIP(ESP, str + 21, (void *)&ESP->STANetmask, NULL); /* Parse IP string */
            }
        } else  if (ESP->ActiveCmd == CMD_WIFI_CIPSTA && strncmp(str, FROMMEM("+CIPSTA"), 7) == 0) {   /* +CIPSTA received */
            //print connected information of client
            __DEBUG("[Client] %s", str + 8);
        } else if (ESP->ActiveCmd == CMD_WIFI_CIPSTAMAC && strncmp(str, FROMMEM("+CIPSTAMAC"), 10) == 0) {  /* On CIPSTAMAC active command */
            __DEBUG("[Client] STA MAC address %s", str + 10);
            ParseMAC(ESP, str + 16, (void *)&ESP->STAMAC, NULL);    /* Parse MAC */
            if (Pointers.Ptr1) {
                memcpy((void *)Pointers.Ptr1, (void *)&ESP->STAMAC, 6);
            }
        } else if (ESP->ActiveCmd == CMD_WIFI_CIPAPMAC && strncmp(str, FROMMEM("+CIPAPMAC"), 9) == 0) { /* On CIPAPMAC active command */
            __DEBUG("[Client] AP MAC address %s", str + 9);
            ParseMAC(ESP, str + 15, (void *)&ESP->APMAC, NULL); /* Parse MAC */
            if (Pointers.Ptr1) {
                memcpy((void *)Pointers.Ptr1, (void *)&ESP->APMAC, 6);
            }
        } else if (ESP->ActiveCmd == CMD_WIFI_GETHOSTNAME && strncmp(str, FROMMEM("+CWHOSTNAME"), 11) == 0) {
            ParseHostName(ESP, str + 12, (void *)Pointers.Ptr1);    /* Parse IP and save it to user location */
        } else if (ESP->ActiveCmd == CMD_TCPIP_CIPSNTPTIME && strncmp(str, FROMMEM("+CIPSNTPTIME"), 12) == 0) {
            ParseSNTPTime(ESP, str + 13, (void *)Pointers.Ptr1);    /* Parse received time */
        } else if (ESP->ActiveCmd == CMD_TCPIP_SNTPGETCFG && strncmp(str, FROMMEM("+CIPSNTPCFG"), 11) == 0) {
            ParseSNTPConfig(ESP, str + 12, (void *)Pointers.Ptr1);  /* Parse received time */
        } else if (ESP->ActiveCmd == CMD_TCPIP_CIPDOMAIN && strncmp(str, FROMMEM("+CIPDOMAIN"), 10) == 0) {
            ParseIP(ESP, str + 11, (void *)Pointers.Ptr1, NULL);    /* Parse IP and save it to user location */
        } else if (ESP->ActiveCmd == CMD_TCPIP_CIPGETDNS && strncmp(str, FROMMEM("+CIPDNS_"), 8) == 0) {
            ParseCIPDNS(ESP, str + 12, (void *)Pointers.Ptr1);  /* Parse DNS server */
        }
    }

    /* Connecting to AP */
    if (ESP->ActiveCmd == CMD_WIFI_CWJAP) {                 /* Trying to connect to wifi network */
        if (strcmp(str, FROMMEM("FAIL\r\n")) == 0) {        /* Fail received */
            __DEBUG("Connecting to WIFP AP: FAIL\n");
            is_error = 1;
        //} else if (strncmp(str, FROMMEM("+CWJAP_CUR"), 10) == 0) {  /* Received currently connected AP info */
        } else if (strncmp(str, FROMMEM("+CWJAP"), 10) == 0) {  /* Received currently connected AP info */
            __DEBUG("%s", str + 10);
            ParseCWJAP(ESP, str + 10, (void *)Pointers.Ptr1);   /* Parse and save */
        }
        __DEBUG("\n");
    }

    /* Wifi management informations */
    if (strcmp(str, FROMMEM("WIFI CONNECTED\r\n")) == 0) {  /* Just connected */
        __DEBUG("%s", str);
        if (ESP->ActiveCmd == CMD_WIFI_CWJAP) {             /* If trying to join AP */
            ESP->Events.F.RespWifiConnected = 1;
        }
        ESP->CallbackFlags.F.WifiConnected = 1;
    } else if (strcmp(str, FROMMEM("WIFI DISCONNECT\r\n")) == 0) {  /* Just disconnected */
        __DEBUG("%s", str);
        if (ESP->ActiveCmd == CMD_WIFI_CWJAP) {             /* If trying to join AP */
            ESP->Events.F.RespWifiDisconnected = 1;
        }
        ESP->CallbackFlags.F.WifiDisconnected = 1;
    } else if (strcmp(str, FROMMEM("WIFI GOT IP\r\n")) == 0) {  /* ESP got assigned IP address from DHCP */
        if (ESP->ActiveCmd == CMD_WIFI_CWJAP) {             /* If trying to join AP */
            ESP->Events.F.RespWifiGotIp = 1;
        }
        ESP->CallbackFlags.F.WifiGotIP = 1;
    }

    /* Connection management */
#if !ESP_SINGLE_CONN
    if (strncmp(&str[1], FROMMEM(",CONNECT"), 8) == 0) {
        ESP_CONN_t* conn = (void *)&ESP->Conn[CHARTONUM(str[0])];   /* Get connection from number */
        conn->Number = CHARTONUM(str[0]);                   /* Set connection number */
        conn->Flags.F.Active = 1;                           /* Connection is active */
        conn->Callback.F.Connect = 1;
        __CONN_UPDATE_TIME(ESP, conn);                      /* Update connection access time */
    } else if (strncmp(&str[1], FROMMEM(",CLOSED"), 7) == 0) {
        ESP_CONN_t* conn = (void *)&ESP->Conn[CHARTONUM(str[0])];   /* Get connection from number */
        ESP_EventCallback_t cb = conn->Cb;
        __CONN_RESET(conn);                                 /* Reset connection */
        conn->Callback.F.Closed = 1;
        conn->Cb = cb;
    }
#else
    if (strncmp(str, FROMMEM("CONNECT"), 7) == 0) {
        ESP_CONN_t* conn = (void *)&ESP->Conn[0];           /* Get connection from number */
        conn->Number = 0;                                   /* Set connection number */
        conn->Flags.F.Active = 1;                           /* Connection is active */
        conn->Callback.F.Connect = 1;
        __CONN_UPDATE_TIME(ESP, conn);                      /* Update connection access time */
    } else if (strncmp(str, FROMMEM("CLOSED"), 6) == 0) {
        ESP_CONN_t* conn = (void *)&ESP->Conn[0];           /* Get connection from number */
        ESP_EventCallback_t cb = conn->Cb;
        __CONN_RESET(conn);                                 /* Reset connection */
        conn->Callback.F.Closed = 1;
        conn->Cb = cb;
    }
#endif /* !ESP_SINGLE_CONN */

    /* Manage connection status */
    if (ESP->ActiveCmd == CMD_TCPIP_CIPSTATUS) {
        if (strncmp(str, FROMMEM("+CIPSTATUS"), 10) == 0) { /* +CIPSTATUS received */
            __DEBUG("%s\n", str);
            ParseCIPSTATUS(ESP, (void *)&ESP->ActiveConnsResp, str + 11);   /* Parse CIPSTATUS response */
        } else if (is_ok) {                                 /* OK received */
            /* Check and merge all connections from ESP */
            uint8_t i = 0;
            for (i = 0; i < ESP_MAX_CONNECTIONS; i++) {
                ESP->Conn[i].Flags.F.Active = (ESP->ActiveConnsResp & (1 << i)) == (1 << i);
            }
            ESP->ActiveConns = ESP->ActiveConnsResp;        /* Copy current value */
        }
    }

    /* Manage send data */
    if (ESP->ActiveCmd == CMD_TCPIP_CIPSEND) {
        if (strncmp(str, FROMMEM("SEND OK"), 7) == 0) {     /* Data successfully sent */
            ESP->Events.F.RespSendOk = 1;
        } else if (strncmp(str, FROMMEM("SEND FAIL"), 9) == 0) {    /* Data sent error */
            ESP->Events.F.RespSendFail = 1;
        }
    }

    if (is_ok) {
        ESP->Events.F.RespOk = 1;
        ESP->Events.F.RespError = 0;
    } else if (is_error) {
        ESP->Events.F.RespOk = 0;
        ESP->Events.F.RespError = 1;
    }
}

/******************************************************************************/
/******************************************************************************/
/***                              Protothreads                               **/
/******************************************************************************/
/******************************************************************************/
estatic
PT_THREAD(PT_Thread_BASIC(struct pt* pt, evol ESP_t* ESP)) {
    static volatile uint32_t time;
    static uint8_t rst;
    char str[8];
    PT_BEGIN(pt);

    if (ESP->ActiveCmd == CMD_BASIC_AT) {                   /* Send AT and check for any response */
        __RST_EVENTS_RESP(ESP);                             /* Reset all events */

        UART_SEND_STR(FROMMEM("AT"));                       /* Send data */
        UART_SEND_STR(_CRLF);
        StartCommand(ESP, CMD_BASIC_AT, NULL);              /* Start command */

        PT_WAIT_UNTIL(pt, ESP->Events.F.RespOk ||
                            ESP->Events.F.RespError);       /* Wait for response */

        ESP->ActiveResult = ESP->Events.F.RespOk ? espOK : espERROR;    /* Check response */

        __IDLE(ESP);                                        /* Go IDLE mode */
    } else if (ESP->ActiveCmd == CMD_BASIC_ATE) {           /* Set echo */
        NumberToString(str, Pointers.UI);                   /* Get echo parameter as string */

        __RST_EVENTS_RESP(ESP);                             /* Reset all events */

        UART_SEND_STR(FROMMEM("ATE"));                       /* Send data */
        UART_SEND_STR(FROMMEM(str));
        UART_SEND_STR(_CRLF);
        StartCommand(ESP, CMD_BASIC_ATE, NULL);              /* Start command */

        PT_WAIT_UNTIL(pt, ESP->Events.F.RespOk ||
                            ESP->Events.F.RespError);       /* Wait for response */

        ESP->ActiveResult = ESP->Events.F.RespOk ? espOK : espERROR;    /* Check response */

        __IDLE(ESP);                                        /* Go IDLE mode */
    } else if (ESP->ActiveCmd == CMD_BASIC_RST) {           /* Process device reset */
        __RST_EVENTS_RESP(ESP);                             /* Reset all events */

        /***** Hardware reset *****/
        __RST_EVENTS_RESP(ESP);                             /* Reset all events */
        rst = ESP_RESET_SET;
        ESP_LL_Callback(ESP_LL_Control_SetReset, &rst, 0);  /* Process callback with reset set */
        time = ESP->Time;
        PT_WAIT_UNTIL(pt, ESP->Time - time > 2);            /* Wait reset time */
        rst = ESP_RESET_CLR;
        ESP_LL_Callback(ESP_LL_Control_SetReset, &rst, 0);  /* Process callback with reset clear */

        PT_WAIT_UNTIL(pt, ESP->Events.F.RespReady ||
                            ESP->Events.F.RespError);   /* Wait for response */

        ESP->ActiveResult = ESP->Events.F.RespReady ? espOK : espERROR; /* Check response */

        /***** Software reset *****/
        if (ESP->ActiveResult != espOK) {
            UART_SEND_STR(FROMMEM("AT+RST"));               /* Send data */
            UART_SEND_STR(_CRLF);
            StartCommand(ESP, CMD_BASIC_RST, NULL);         /* Start command */

            PT_WAIT_UNTIL(pt, ESP->Events.F.RespReady ||
                                ESP->Events.F.RespError);   /* Wait for response */

            ESP->ActiveResult = ESP->Events.F.RespReady ? espOK : espERROR; /* Check response */
        }

        __IDLE(ESP);                                        /* Go IDLE mode */
    } else if (ESP->ActiveCmd == CMD_BASIC_GMR) {           /* Get informations about AT software */
        __RST_EVENTS_RESP(ESP);                             /* Reset all events */

        UART_SEND_STR(FROMMEM("AT+GMR"));                   /* Send data */
        UART_SEND_STR(_CRLF);
        StartCommand(ESP, CMD_BASIC_GMR, NULL);             /* Start command */

        PT_WAIT_UNTIL(pt, ESP->Events.F.RespOk ||
                            ESP->Events.F.RespError);       /* Wait for response */

        ESP->ActiveResult = ESP->Events.F.RespOk ? espOK : espERROR;    /* Check response */

        __IDLE(ESP);                                        /* Go IDLE mode */
    } else if (ESP->ActiveCmd == CMD_BASIC_UART) {          /* Set UART */
        NumberToString(str, Pointers.UI);                   /* Get baudrate as string */

        /* Send UART command */
        __RST_EVENTS_RESP(ESP);                             /* Reset all events */
        UART_SEND_STR(FROMMEM("AT+UART_"));                 /* Send data */
        UART_SEND_STR(FROMMEM(Pointers.CPtr1));
        UART_SEND_STR(FROMMEM("="));
        UART_SEND_STR(FROMMEM(str));
#if ESP_USE_CTS
        UART_SEND_STR(FROMMEM(",8,1,0,2"));                 /* Enable hardware CTS pin on ESP device */
#else
        UART_SEND_STR(FROMMEM(",8,1,0,0"));                 /* No flow control for ESP */
#endif /* ESP_USE_CTS */
        UART_SEND_STR(_CRLF);
        StartCommand(ESP, CMD_BASIC_UART, NULL);            /* Start command */

        PT_WAIT_UNTIL(pt, ESP->Events.F.RespOk ||
                            ESP->Events.F.RespError);       /* Wait for response */

        ESP->ActiveResult = ESP->Events.F.RespOk ? espOK : espERROR;    /* Check response */
        if (ESP->ActiveResult == espOK) {
            uint8_t result;
            BUFFER_Reset(&Buffer);                          /* Reset buffer */

            /* Reinit low-level with new baudrate */
            ESP->LL.Baudrate = Pointers.UI;
            ESP_LL_Callback(ESP_LL_Control_Init, (void *)&ESP->LL, &result);    /* Init low-level layer again */
        }

        /* Now let's read default baudrate for reinit purpose */
        time = ESP->Time;
        PT_WAIT_UNTIL(pt, ESP->Time - time > 2);            /* Wait reset time */

        __RST_EVENTS_RESP(ESP);                             /* Reset all events */
        UART_SEND_STR(FROMMEM("AT+UART_DEF?"));             /* Send data */
        StartCommand(ESP, CMD_BASIC_UART, NULL);            /* Start command */
        UART_SEND_STR(_CRLF);

        PT_WAIT_UNTIL(pt, ESP->Events.F.RespOk ||
                            ESP->Events.F.RespError);       /* Wait for response */

        __IDLE(ESP);                                        /* Go IDLE mode */
    } else if (ESP->ActiveCmd == CMD_BASIC_RFPOWER) {       /* Set RF power */
        NumberToString(str, Pointers.UI);                   /* Convert mode to string */
        __RST_EVENTS_RESP(ESP);                             /* Reset all events */
        UART_SEND_STR(FROMMEM("AT+RFPOWER="));              /* Send data */
        UART_SEND_STR(FROMMEM(str));
        UART_SEND_STR(_CRLF);
        StartCommand(ESP, CMD_BASIC_RFPOWER, NULL);         /* Start command */

        PT_WAIT_UNTIL(pt, ESP->Events.F.RespOk ||
                            ESP->Events.F.RespError);       /* Wait for response */

        ESP->ActiveResult = ESP->Events.F.RespOk ? espOK : espERROR;    /* Check response */

        __IDLE(ESP);                                        /* Go IDLE mode */
    } else if (ESP->ActiveCmd == CMD_BASIC_RESTORE) {       /* Restore device */
        __RST_EVENTS_RESP(ESP);                             /* Reset all events */

        UART_SEND_STR(FROMMEM("AT+RESTORE"));               /* Send data */
        UART_SEND_STR(_CRLF);
        StartCommand(ESP, CMD_BASIC_RESTORE, NULL);         /* Start command */

        PT_WAIT_UNTIL(pt, ESP->Events.F.RespOk ||
                            ESP->Events.F.RespError);       /* Wait for response */

        ESP->ActiveResult = ESP->Events.F.RespOk ? espOK : espERROR;    /* Check response */

        PT_WAIT_UNTIL(pt, ESP->Events.F.RespReady ||
                            ESP->Events.F.RespError);       /* Wait for response */

        if (!ESP->Events.F.RespReady) {
            ESP->ActiveResult = espERROR;
        }

        __IDLE(ESP);                                        /* Go IDLE mode */
    } else if (ESP->ActiveCmd == CMD_BASIC_GETSYSRAM) {     /* Get available RAM */
        __RST_EVENTS_RESP(ESP);                             /* Reset all events */

        UART_SEND_STR(FROMMEM("AT+SYSRAM?"));               /* Send data */
        UART_SEND_STR(_CRLF);
        StartCommand(ESP, CMD_BASIC_GETSYSRAM, NULL);       /* Start command */

        PT_WAIT_UNTIL(pt, ESP->Events.F.RespOk ||
                            ESP->Events.F.RespError);       /* Wait for response */

        ESP->ActiveResult = ESP->Events.F.RespReady ? espOK : espERROR;    /* Check response */

        __IDLE(ESP);                                        /* Go IDLE mode */
    } else if (ESP->ActiveCmd == CMD_BASIC_GETSYSADC) {     /* Read ADC channel */
        __RST_EVENTS_RESP(ESP);                             /* Reset all events */

        UART_SEND_STR(FROMMEM("AT+SYSADC?"));               /* Send data */
        UART_SEND_STR(_CRLF);
        StartCommand(ESP, CMD_BASIC_GETSYSADC, NULL);       /* Start command */

        PT_WAIT_UNTIL(pt, ESP->Events.F.RespOk ||
                            ESP->Events.F.RespError);       /* Wait for response */

        ESP->ActiveResult = ESP->Events.F.RespReady ? espOK : espERROR;    /* Check response */

        __IDLE(ESP);                                        /* Go IDLE mode */
    } else if (ESP->ActiveCmd == CMD_BASIC_SYSIOSETCFG) {   /* Set GPIO config */
        __RST_EVENTS_RESP(ESP);                             /* Reset all events */

        UART_SEND_STR(FROMMEM("AT+SYSIOSETCFG="));          /* Send data */
        NumberToString(str, Pointers.UI);                   /* Convert pin number to string */
        UART_SEND_STR(FROMMEM(str));
        UART_SEND_STR(FROMMEM(","));
        NumberToString(str, ((ESP_GPIO_t *)Pointers.CPtr1)->Mode);  /* Convert pin mode to string */
        UART_SEND_STR(FROMMEM(str));
        UART_SEND_STR(FROMMEM(","));
        NumberToString(str, ((ESP_GPIO_t *)Pointers.CPtr1)->Pull);  /* Convert pin pull to string */
        UART_SEND_STR(FROMMEM(str));
        UART_SEND_STR(_CRLF);
        StartCommand(ESP, CMD_BASIC_SYSIOSETCFG, NULL);     /* Start command */

        PT_WAIT_UNTIL(pt, ESP->Events.F.RespOk ||
                            ESP->Events.F.RespError);       /* Wait for response */

        ESP->ActiveResult = ESP->Events.F.RespOk ? espOK : espERROR;    /* Check response */

        __IDLE(ESP);                                        /* Go IDLE mode */
    } else if (ESP->ActiveCmd == CMD_BASIC_SYSIOGETCFG) {   /* Get GPIO config */
        __RST_EVENTS_RESP(ESP);                             /* Reset all events */

        UART_SEND_STR(FROMMEM("AT+SYSIOGETCFG="));          /* Send data */
        NumberToString(str, Pointers.UI);                   /* Convert pin number to string */
        UART_SEND_STR(FROMMEM(str));
        UART_SEND_STR(_CRLF);
        StartCommand(ESP, CMD_BASIC_SYSIOGETCFG, NULL);     /* Start command */

        PT_WAIT_UNTIL(pt, ESP->Events.F.RespOk ||
                            ESP->Events.F.RespError);       /* Wait for response */

        ESP->ActiveResult = ESP->Events.F.RespOk ? espOK : espERROR;    /* Check response */

        __IDLE(ESP);                                        /* Go IDLE mode */
    } else if (ESP->ActiveCmd == CMD_BASIC_SYSGPIOSETDIR) { /* Set GPIO direction */
        __RST_EVENTS_RESP(ESP);                             /* Reset all events */

        UART_SEND_STR(FROMMEM("AT+SYSGPIODIR="));           /* Send data */
        NumberToString(str, Pointers.UI);                   /* Convert pin number to string */
        UART_SEND_STR(FROMMEM(str));
        UART_SEND_STR(FROMMEM(","));
        NumberToString(str, ((ESP_GPIO_t *)Pointers.CPtr1)->Dir);   /* Convert pin direction to string */
        UART_SEND_STR(FROMMEM(str));
        UART_SEND_STR(_CRLF);
        StartCommand(ESP, CMD_BASIC_SYSGPIOSETDIR, NULL);   /* Start command */

        PT_WAIT_UNTIL(pt, ESP->Events.F.RespOk ||
                            ESP->Events.F.RespError);       /* Wait for response */

        ESP->ActiveResult = ESP->Events.F.RespOk ? espOK : espERROR;    /* Check response */

        __IDLE(ESP);                                        /* Go IDLE mode */
    } else if (ESP->ActiveCmd == CMD_BASIC_SYSGPIOREAD) {   /* Read GPIO value */
        __RST_EVENTS_RESP(ESP);                             /* Reset all events */

        UART_SEND_STR(FROMMEM("AT+SYSGPIOREAD="));          /* Send data */
        NumberToString(str, Pointers.UI);                   /* Convert pin number to string */
        UART_SEND_STR(FROMMEM(str));
        UART_SEND_STR(_CRLF);
        StartCommand(ESP, CMD_BASIC_SYSGPIOREAD, NULL);   /* Start command */

        PT_WAIT_UNTIL(pt, ESP->Events.F.RespOk ||
                            ESP->Events.F.RespError);       /* Wait for response */

        ESP->ActiveResult = ESP->Events.F.RespOk ? espOK : espERROR;    /* Check response */

        __IDLE(ESP);                                        /* Go IDLE mode */
    } else if (ESP->ActiveCmd == CMD_BASIC_SYSGPIOWRITE) {  /* Write GPIO value */
        __RST_EVENTS_RESP(ESP);                             /* Reset all events */

        UART_SEND_STR(FROMMEM("AT+SYSGPIOWRITE="));         /* Send data */
        NumberToString(str, (Pointers.UI) & 0xFF);          /* Convert pin number to string */
        UART_SEND_STR(FROMMEM(str));
        UART_SEND_STR(FROMMEM(","));
        NumberToString(str, (Pointers.UI >> 8) & 0xFF);     /* Convert pin value to string */
        UART_SEND_STR(FROMMEM(str));
        UART_SEND_STR(_CRLF);
        StartCommand(ESP, CMD_BASIC_SYSGPIOREAD, NULL);     /* Start command */

        PT_WAIT_UNTIL(pt, ESP->Events.F.RespOk ||
                            ESP->Events.F.RespError);       /* Wait for response */

        ESP->ActiveResult = ESP->Events.F.RespOk ? espOK : espERROR;    /* Check response */

        __IDLE(ESP);                                        /* Go IDLE mode */
    }
    PT_END(pt);
}

estatic
PT_THREAD(PT_Thread_WIFI(struct pt* pt, evol ESP_t* ESP)) {
    char str[7], ch = ':', i = 6;
    uint8_t* ptr;
    PT_BEGIN(pt);

    if (ESP->ActiveCmd == CMD_WIFI_CWMODE) {                /* Set device mode */
        NumberToString(str, Pointers.UI);                   /* Convert mode to string */
        __RST_EVENTS_RESP(ESP);                             /* Reset all events */
        UART_SEND_STR(FROMMEM("AT+CWMODE="));               /* Send data */
        UART_SEND_STR(FROMMEM(str));
        UART_SEND_STR(_CRLF);
        StartCommand(ESP, CMD_WIFI_CWMODE, NULL);           /* Start command */

        PT_WAIT_UNTIL(pt, ESP->Events.F.RespOk ||
                            ESP->Events.F.RespError);       /* Wait for response */

        ESP->ActiveResult = ESP->Events.F.RespOk ? espOK : espERROR;    /* Check response */

        __IDLE(ESP);                                        /* Go IDLE mode */
    } else if (ESP->ActiveCmd == CMD_WIFI_GETSTAMAC) {      /* Get station MAC address */
        __RST_EVENTS_RESP(ESP);                             /* Reset all events */
        //UART_SEND_STR(FROMMEM("AT+CIPSTAMAC_CUR?"));        /* Send data */
        UART_SEND_STR(FROMMEM("AT+CIPSTAMAC?"));        /* Send data */
        UART_SEND_STR(_CRLF);
        StartCommand(ESP, CMD_WIFI_CIPSTAMAC, NULL);        /* Start command */

        PT_WAIT_UNTIL(pt, ESP->Events.F.RespOk ||
                            ESP->Events.F.RespError);       /* Wait for response */

        ESP->ActiveResult = ESP->Events.F.RespOk ? espOK : espERROR;    /* Check response */

        __IDLE(ESP);                                        /* Go IDLE mode */
    } else if (ESP->ActiveCmd == CMD_WIFI_GETAPMAC) {       /* Get AP MAC address */
        __RST_EVENTS_RESP(ESP);                             /* Reset all events */
        //UART_SEND_STR(FROMMEM("AT+CIPAPMAC_CUR?"));         /* Send data */
        UART_SEND_STR(FROMMEM("AT+CIPAPMAC?"));         /* Send data */
        UART_SEND_STR(_CRLF);
        StartCommand(ESP, CMD_WIFI_CIPAPMAC, NULL);         /* Start command */

        PT_WAIT_UNTIL(pt, ESP->Events.F.RespOk ||
                            ESP->Events.F.RespError);       /* Wait for response */

        ESP->ActiveResult = ESP->Events.F.RespOk ? espOK : espERROR;    /* Check response */

        __IDLE(ESP);                                        /* Go IDLE mode */
    } else if (ESP->ActiveCmd == CMD_WIFI_GETSTAIP) {       /* Get station IP address */
        __RST_EVENTS_RESP(ESP);                             /* Reset all events */
        //UART_SEND_STR(FROMMEM("AT+CIPSTA_CUR?"));           /* Send data */
        UART_SEND_STR(FROMMEM("AT+CIPSTA?"));           /* Send data */
        UART_SEND_STR(_CRLF);
        StartCommand(ESP, CMD_WIFI_CIPSTA, NULL);           /* Start command */

        PT_WAIT_UNTIL(pt, ESP->Events.F.RespOk ||
                            ESP->Events.F.RespError);       /* Wait for response */

        ESP->ActiveResult = ESP->Events.F.RespOk ? espOK : espERROR;    /* Check response */

        __IDLE(ESP);                                        /* Go IDLE mode */
    } else if (ESP->ActiveCmd == CMD_WIFI_GETAPIP) {        /* Get AP IP address */
        __RST_EVENTS_RESP(ESP);                             /* Reset all events */
        //UART_SEND_STR(FROMMEM("AT+CIPAP_CUR?"));            /* Send data */
        UART_SEND_STR(FROMMEM("AT+CIPAP?"));            /* Send data */
        UART_SEND_STR(_CRLF);
        StartCommand(ESP, CMD_WIFI_CIPAP, NULL);

        PT_WAIT_UNTIL(pt, ESP->Events.F.RespOk ||
                            ESP->Events.F.RespError);       /* Wait for response */

        ESP->ActiveResult = ESP->Events.F.RespOk ? espOK : espERROR;    /* Check response */

        __IDLE(ESP);                                        /* Go IDLE mode */
    } else if (ESP->ActiveCmd == CMD_WIFI_SETSTAMAC) {      /* Get AP IP address */
        ptr = (uint8_t *) Pointers.CPtr2;

        __RST_EVENTS_RESP(ESP);                             /* Reset all events */
        UART_SEND_STR(FROMMEM("AT+CIPSTAMAC_"));            /* Send data */
        UART_SEND_STR(FROMMEM(Pointers.CPtr1));             /* Default or current */
        UART_SEND_STR(FROMMEM("=\""));
        i = 6; ch = ':';
        while (i--) {
            HexNumberToString(str, *ptr++);                 /* Convert to hex number */
            UART_SEND_STR(FROMMEM(str));
            if (i) {
                UART_SEND_CH(FROMMEM(&ch));
            }
        }
        UART_SEND_STR(FROMMEM("\""));
        UART_SEND_STR(_CRLF);
        StartCommand(ESP, CMD_WIFI_CIPSTAMAC, NULL);        /* Start command */

        PT_WAIT_UNTIL(pt, ESP->Events.F.RespOk ||
                            ESP->Events.F.RespError);       /* Wait for response */

        ESP->ActiveResult = ESP->Events.F.RespOk ? espOK : espERROR;    /* Check response */
        if (ESP->ActiveResult == espOK) {                   /* Copy data as new MAC address */
            memcpy((void *)&ESP->APMAC, (void *)Pointers.CPtr2, 6); /* Copy new MAC */
        }

        __IDLE(ESP);                                        /* Go IDLE mode */
    } else if (ESP->ActiveCmd == CMD_WIFI_SETAPMAC) {       /* Get AP IP address */
        ptr = (uint8_t *) Pointers.CPtr2;

        __RST_EVENTS_RESP(ESP);                             /* Reset all events */
        UART_SEND_STR(FROMMEM("AT+CIPAPMAC_"));             /* Send data */
        UART_SEND_STR(FROMMEM(Pointers.CPtr1));             /* Default or current */
        UART_SEND_STR(FROMMEM("=\""));
        i = 6; ch = ':';
        while (i--) {
            HexNumberToString(str, *ptr++);                 /* Convert to hex number */
            UART_SEND_STR(FROMMEM(str));
            if (i) {
                UART_SEND_CH(FROMMEM(&ch));
            }
        }
        UART_SEND_STR(FROMMEM("\""));
        UART_SEND_STR(_CRLF);
        StartCommand(ESP, CMD_WIFI_CIPAPMAC, NULL);         /* Start command */

        PT_WAIT_UNTIL(pt, ESP->Events.F.RespOk ||
                            ESP->Events.F.RespError);       /* Wait for response */

        ESP->ActiveResult = ESP->Events.F.RespOk ? espOK : espERROR;    /* Check response */
        if (ESP->ActiveResult == espOK) {                   /* Copy data as new MAC address */
            memcpy((void *)&ESP->APMAC, (void *)Pointers.CPtr2, 6); /* Copy new MAC */
        }

        __IDLE(ESP);                                        /* Go IDLE mode */
    } else if (ESP->ActiveCmd == CMD_WIFI_SETSTAIP) {       /* Set AP IP address */
        ptr = (uint8_t *) Pointers.CPtr2;

        __RST_EVENTS_RESP(ESP);                             /* Reset all events */
        UART_SEND_STR(FROMMEM("AT+CIPSTA_"));               /* Send data */
        UART_SEND_STR(FROMMEM(Pointers.CPtr1));             /* Default or current */
        UART_SEND_STR(FROMMEM("=\""));
        i = 4; ch = '.';
        while (i--) {
            NumberToString(str, *ptr++);                    /* Convert to hex number */
            UART_SEND_STR(FROMMEM(str));
            if (i) {
                UART_SEND_CH(FROMMEM(&ch));
            }
        }
        UART_SEND_STR(FROMMEM("\""));
        if (Pointers.CPtr3 != NULL) {                       /* Check for gateway and netmask addresses */
            ptr = (uint8_t *) Pointers.CPtr3;
            UART_SEND_STR(FROMMEM(",\""));
            i = 4;
            while (i--) {                                   /* Send gateway address */
                NumberToString(str, *ptr++);                /* Convert to hex number */
                UART_SEND_STR(FROMMEM(str));
                if (i) {
                    UART_SEND_CH(FROMMEM(&ch));
                }
            }
            UART_SEND_STR(FROMMEM("\",\""));
            i = 4;
            while (i--) {                                   /* Send net mask */
                NumberToString(str, *ptr++);                /* Convert to hex number */
                UART_SEND_STR(FROMMEM(str));
                if (i) {
                    UART_SEND_CH(FROMMEM(&ch));
                }
            }
            UART_SEND_STR(FROMMEM("\""));
        }
        UART_SEND_STR(_CRLF);
        StartCommand(ESP, CMD_WIFI_CIPSTA, NULL);           /* Start command */

        PT_WAIT_UNTIL(pt, ESP->Events.F.RespOk ||
                            ESP->Events.F.RespError);       /* Wait for response */

        ESP->ActiveResult = ESP->Events.F.RespOk ? espOK : espERROR;    /* Check response */
        if (ESP->ActiveResult == espOK) {                   /* Copy data as new MAC address */
            memcpy((void *)&ESP->STAIP, (void *)Pointers.CPtr2, 4);
            if (Pointers.CPtr3 != NULL) {                   /* Check network mask and gateway */
                memcpy((void *)&ESP->STAGateway, ((uint8_t *) Pointers.CPtr3), 4);  /* Copy gateway address */
                memcpy((void *)&ESP->STANetmask, ((uint8_t *) Pointers.CPtr3) + 4, 4);  /* Copy netmas address */
            }
        }

        __IDLE(ESP);                                        /* Go IDLE mode */
    } else if (ESP->ActiveCmd == CMD_WIFI_SETAPIP) {        /* Set AP IP address */
        ptr = (uint8_t *) Pointers.CPtr2;

        __RST_EVENTS_RESP(ESP);                             /* Reset all events */
        UART_SEND_STR(FROMMEM("AT+CIPAP_"));                /* Send data */
        UART_SEND_STR(FROMMEM(Pointers.CPtr1));             /* Default or current */
        UART_SEND_STR(FROMMEM("=\""));
        i = 4; ch = '.';
        while (i--) {
            NumberToString(str, *ptr++);                    /* Convert to hex number */
            UART_SEND_STR(FROMMEM(str));
            if (i) {
                UART_SEND_CH(FROMMEM(&ch));
            }
        }
        UART_SEND_STR(FROMMEM("\""));
        UART_SEND_STR(_CRLF);
        StartCommand(ESP, CMD_WIFI_CIPSTA, NULL);           /* Start command */

        PT_WAIT_UNTIL(pt, ESP->Events.F.RespOk ||
                            ESP->Events.F.RespError);       /* Wait for response */

        ESP->ActiveResult = ESP->Events.F.RespOk ? espOK : espERROR;    /* Check response */
        if (ESP->ActiveResult == espOK) {                   /* Copy data as new MAC address */
            memcpy((void *)&ESP->STAIP, ptr - 4, 4);
        }

        __IDLE(ESP);                                        /* Go IDLE mode */
    } else if (ESP->ActiveCmd == CMD_WIFI_LISTACCESSPOINTS) {   /* List available access points */
        /***** Setup options returned by list access *****/
        __RST_EVENTS_RESP(ESP);                             /* Reset all events */
        UART_SEND_STR(FROMMEM("AT+CWLAPOPT=1,127"));        /* Send data */
        UART_SEND_STR(_CRLF);
        StartCommand(ESP, CMD_WIFI_CWLAPOPT, NULL);

        PT_WAIT_UNTIL(pt, ESP->Events.F.RespOk ||
                            ESP->Events.F.RespError);       /* Wait for response */

        ESP->ActiveResult = ESP->Events.F.RespOk ? espOK : espERROR;    /* Check response */
        if (ESP->ActiveResult != espOK) {
            goto cmd_wifi_listaccesspoints_clean;           /* Clean thread and stop execution */
        }

        /***** Execute access point search *****/
        __RST_EVENTS_RESP(ESP);                             /* Reset all events */
        UART_SEND_STR(FROMMEM("AT+CWLAP"));                 /* Send data */
        UART_SEND_STR(_CRLF);
        StartCommand(ESP, CMD_WIFI_CWLAP, NULL);            /* Start command */

        PT_WAIT_UNTIL(pt, ESP->Events.F.RespOk ||
                            ESP->Events.F.RespError);       /* Wait for response */

        ESP->ActiveResult = ESP->Events.F.RespOk ? espOK : espERROR;    /* Check response */

cmd_wifi_listaccesspoints_clean:
        __IDLE(ESP);                                        /* Go IDLE mode */
    } else if (ESP->ActiveCmd == CMD_WIFI_CWJAP) {          /* Connect to network */
        ptr = (uint8_t *) Pointers.Ptr1;
        __RST_EVENTS_RESP(ESP);                             /* Reset all events */
        __DEBUG("Connecting to WIFI AP: ssid: %s, pass: %s\n", Pointers.CPtr2, Pointers.CPtr3);
//        UART_SEND_STR(FROMMEM("AT+CWJAP_"));                /* Send data */
        UART_SEND_STR(FROMMEM("AT+CWJAP"));                /* Send data */
//        UART_SEND_STR(FROMMEM(Pointers.CPtr1));
        UART_SEND_STR(FROMMEM("=\""));
        EscapeStringAndSend(FROMMEM(Pointers.CPtr2));
        UART_SEND_STR(FROMMEM("\",\""));
        EscapeStringAndSend(FROMMEM(Pointers.CPtr3));
        UART_SEND_STR(FROMMEM("\""));
        if (ptr) {                                          /* Send MAC address */
            UART_SEND_STR(FROMMEM(",\""));
            i = 6; ch = ':';
            while (i--) {
                HexNumberToString(str, *ptr++);             /* Convert to hex number */
                UART_SEND_STR(FROMMEM(str));
                if (i) {
                    UART_SEND_CH(FROMMEM(&ch));
                }
            }
            UART_SEND_STR(FROMMEM("\""));
        }
        UART_SEND_STR(_CRLF);
        StartCommand(ESP, CMD_WIFI_CWJAP, NULL);            /* Start command */

        PT_WAIT_UNTIL(pt, ESP->Events.F.RespOk ||
                            ESP->Events.F.RespError);       /* Wait for response */

        ESP->ActiveResult = ESP->Events.F.RespOk ? espOK : espERROR;    /* Check response */
        if (ESP->ActiveResult == espOK) {
            ESP->ActiveCmd = CMD_WIFI_GETSTAIP;
            //__IDLE(ESP);                                    /* Go IDLE mode */
            //__ACTIVE_CMD(ESP, CMD_WIFI_GETSTAIP);           /* Set new active CMD to retrieve info about station data */
        } else {
            __IDLE(ESP);                                    /* Go IDLE mode */
        }
    } else if (ESP->ActiveCmd == CMD_WIFI_CWQAP) {          /* Disconnect from network */
        __RST_EVENTS_RESP(ESP);                             /* Reset all events */
        UART_SEND_STR(FROMMEM("AT+CWQAP"));                 /* Send data */
        UART_SEND_STR(_CRLF);
        StartCommand(ESP, CMD_WIFI_CWQAP, NULL);            /* Start command */

        PT_WAIT_UNTIL(pt, ESP->Events.F.RespOk ||
                            ESP->Events.F.RespError);       /* Wait for response */

        ESP->ActiveResult = ESP->Events.F.RespOk ? espOK : espERROR;    /* Check response */

        __IDLE(ESP);                                        /* Go IDLE mode */
    } else if (ESP->ActiveCmd == CMD_WIFI_GETCWJAP) {       /* Get AP we are connected with */
        __RST_EVENTS_RESP(ESP);                             /* Reset all events */
        //UART_SEND_STR(FROMMEM("AT+CWJAP_CUR?"));            /* Send data */
        UART_SEND_STR(FROMMEM("AT+CWJAP?"));            /* Send data */
        UART_SEND_STR(_CRLF);
        StartCommand(ESP, CMD_WIFI_CWJAP, NULL);            /* Start command */

        PT_WAIT_UNTIL(pt, ESP->Events.F.RespOk ||
                            ESP->Events.F.RespError);       /* Wait for response */

        ESP->ActiveResult = ESP->Events.F.RespOk ? espOK : espERROR;    /* Check response */

        __IDLE(ESP);                                        /* Go IDLE mode */
    } else if (ESP->ActiveCmd == CMD_WIFI_CWAUTOCONN) {     /* Set autoconnect status */
        __RST_EVENTS_RESP(ESP);                             /* Reset all events */
        UART_SEND_STR(FROMMEM("AT+CWAUTOCONN="));           /* Send data */
        UART_SEND_STR(Pointers.UI ? FROMMEM("1") : FROMMEM("0"));
        UART_SEND_STR(_CRLF);
        StartCommand(ESP, CMD_WIFI_CWAUTOCONN, NULL);       /* Start command */

        PT_WAIT_UNTIL(pt, ESP->Events.F.RespOk ||
                            ESP->Events.F.RespError);       /* Wait for response */

        ESP->ActiveResult = ESP->Events.F.RespOk ? espOK : espERROR;    /* Check response */

        __IDLE(ESP);                                        /* Go IDLE mode */
    } else if (ESP->ActiveCmd == CMD_WIFI_CWLIF) {          /* List connected stations */
        __RST_EVENTS_RESP(ESP);                             /* Reset all events */
        UART_SEND_STR(FROMMEM("AT+CWLIF"));                 /* Send data */
        UART_SEND_STR(_CRLF);
        StartCommand(ESP, CMD_WIFI_CWLIF, NULL);            /* Start command */

        PT_WAIT_UNTIL(pt, ESP->Events.F.RespOk ||
                            ESP->Events.F.RespError);       /* Wait for response */

        ESP->ActiveResult = ESP->Events.F.RespOk ? espOK : espERROR;    /* Check response */

        __IDLE(ESP);                                        /* Go IDLE mode */
    } else if (ESP->ActiveCmd == CMD_WIFI_GETCWSAP) {       /* GET AP settings */
        __RST_EVENTS_RESP(ESP);                             /* Reset all events */

        //UART_SEND_STR(FROMMEM("AT+CWSAP_CUR?"));            /* Send data */
        UART_SEND_STR(FROMMEM("AT+CWSAP?"));            /* Send data */
        UART_SEND_STR(_CRLF);
        StartCommand(ESP, CMD_WIFI_CWSAP, NULL);            /* Start command */

        PT_WAIT_UNTIL(pt, ESP->Events.F.RespOk ||
                            ESP->Events.F.RespError);       /* Wait for response */

        ESP->ActiveResult = ESP->Events.F.RespOk ? espOK : espERROR;    /* Check response */

        __IDLE(ESP);                                        /* Go IDLE mode */
    } else if (ESP->ActiveCmd == CMD_WIFI_SETCWSAP) {       /* Set AP settings */
        __RST_EVENTS_RESP(ESP);                             /* Reset all events */

        UART_SEND_STR(FROMMEM("AT+CWSAP_"));                /* Send data */
        UART_SEND_STR(FROMMEM(Pointers.CPtr1));             /* Send data */
        UART_SEND_STR(FROMMEM("=\""));
        EscapeStringAndSend(((ESP_APConfig_t *)Pointers.CPtr2)->SSID);
        UART_SEND_STR(FROMMEM("\",\""));
        EscapeStringAndSend(((ESP_APConfig_t *)Pointers.CPtr2)->Pass);
        UART_SEND_STR(FROMMEM("\","));
        NumberToString(str, ((ESP_APConfig_t *)Pointers.CPtr2)->Channel);
        UART_SEND_STR(FROMMEM(str));
        UART_SEND_STR(FROMMEM(","));
        NumberToString(str, ((ESP_APConfig_t *)Pointers.CPtr2)->Ecn);
        UART_SEND_STR(FROMMEM(str));
        UART_SEND_STR(FROMMEM(","));
        NumberToString(str, ((ESP_APConfig_t *)Pointers.CPtr2)->MaxConnections);
        UART_SEND_STR(FROMMEM(str));
        UART_SEND_STR(FROMMEM(","));
        NumberToString(str, ((ESP_APConfig_t *)Pointers.CPtr2)->Hidden);
        UART_SEND_STR(FROMMEM(str));
        UART_SEND_STR(_CRLF);
        StartCommand(ESP, CMD_WIFI_CWSAP, NULL);            /* Start command */

        PT_WAIT_UNTIL(pt, ESP->Events.F.RespOk ||
                            ESP->Events.F.RespError);       /* Wait for response */

        ESP->ActiveResult = ESP->Events.F.RespOk ? espOK : espERROR;    /* Check response */
        if (ESP->ActiveResult != espOK) {
            __IDLE(ESP);                                    /* Go IDLE mode */
        } else {
            __IDLE(ESP);                                    /* Go IDLE mode */
            __ACTIVE_CMD(ESP, CMD_WIFI_GETCWSAP);           /* Get info and save to structure */
        }
    } else if (ESP->ActiveCmd == CMD_WIFI_WPS) {            /* Set WPS function */
        __RST_EVENTS_RESP(ESP);                             /* Reset all events */
        UART_SEND_STR(FROMMEM("AT+WPS="));                  /* Send data */
        UART_SEND_STR(Pointers.UI ? FROMMEM("1") : FROMMEM("0"));
        UART_SEND_STR(_CRLF);
        StartCommand(ESP, CMD_WIFI_WPS, NULL);              /* Start command */

        PT_WAIT_UNTIL(pt, ESP->Events.F.RespOk ||
                            ESP->Events.F.RespError);       /* Wait for response */

        ESP->ActiveResult = ESP->Events.F.RespOk ? espOK : espERROR;    /* Check response */

        __IDLE(ESP);                                        /* Go IDLE mode */
    } else if (ESP->ActiveCmd == CMD_WIFI_SETHOSTNAME) {    /* Set device hostname */
        __RST_EVENTS_RESP(ESP);                             /* Reset all events */

        UART_SEND_STR(FROMMEM("AT+CWHOSTNAME=\""));         /* Send data */
        EscapeStringAndSend((char *)Pointers.CPtr1);
        UART_SEND_STR(FROMMEM("\""));
        UART_SEND_STR(_CRLF);
        StartCommand(ESP, CMD_WIFI_SETHOSTNAME, NULL);      /* Start command */

        PT_WAIT_UNTIL(pt, ESP->Events.F.RespOk ||
                            ESP->Events.F.RespError);       /* Wait for response */

        ESP->ActiveResult = ESP->Events.F.RespOk ? espOK : espERROR;    /* Check response */

        __IDLE(ESP);                                        /* Go IDLE mode */
    } else if (ESP->ActiveCmd == CMD_WIFI_GETHOSTNAME) {    /* Set device hostname */
        __RST_EVENTS_RESP(ESP);                             /* Reset all events */

        UART_SEND_STR(FROMMEM("AT+CWHOSTNAME?"));           /* Send data */
        UART_SEND_STR(_CRLF);
        StartCommand(ESP, CMD_WIFI_GETHOSTNAME, NULL);      /* Start command */

        PT_WAIT_UNTIL(pt, ESP->Events.F.RespOk ||
                            ESP->Events.F.RespError);       /* Wait for response */

        ESP->ActiveResult = ESP->Events.F.RespOk ? espOK : espERROR;    /* Check response */

        __IDLE(ESP);                                        /* Go IDLE mode */
    }

    PT_END(pt);
}

estatic
PT_THREAD(PT_Thread_TCPIP(struct pt* pt, evol ESP_t* ESP)) {
    char str[7];
    static uint8_t i;
    static uint8_t tries;
    static uint32_t btw = 0;

    PT_BEGIN(pt);

    if (ESP->ActiveCmd == CMD_TCPIP_CIPMUX) {               /* Set device mode */
        NumberToString(str, Pointers.UI);                   /* Convert mode to string */
        __RST_EVENTS_RESP(ESP);                             /* Reset all events */
        UART_SEND_STR(FROMMEM("AT+CIPMUX="));               /* Send data */
        UART_SEND_STR(FROMMEM(str));
        UART_SEND_STR(_CRLF);
        StartCommand(ESP, CMD_TCPIP_CIPMUX, NULL);          /* Start command */

        PT_WAIT_UNTIL(pt, ESP->Events.F.RespOk ||
                            ESP->Events.F.RespError);       /* Wait for response */

        ESP->ActiveResult = ESP->Events.F.RespOk ? espOK : espERROR;    /* Check response */

        __IDLE(ESP);                                        /* Go IDLE mode */
    } else if (ESP->ActiveCmd == CMD_TCPIP_CIPDINFO) {      /* Set info on +IPD statement */
        __RST_EVENTS_RESP(ESP);                             /* Reset all events */
        UART_SEND_STR(FROMMEM("AT+CIPDINFO=1"));            /* Send data */
        UART_SEND_STR(_CRLF);
        StartCommand(ESP, CMD_TCPIP_CIPDINFO, NULL);        /* Start command */

        PT_WAIT_UNTIL(pt, ESP->Events.F.RespOk ||
                            ESP->Events.F.RespError);       /* Wait for response */

        ESP->ActiveResult = ESP->Events.F.RespOk ? espOK : espERROR;    /* Check response */

        __IDLE(ESP);                                        /* Go IDLE mode */
    } else if (ESP->ActiveCmd == CMD_TCPIP_SERVERENABLE) {  /* Enable server mode */
        NumberToString(str, Pointers.UI);                   /* Convert mode to string */
        __RST_EVENTS_RESP(ESP);                             /* Reset all events */
        UART_SEND_STR(FROMMEM("AT+CIPSERVER=1,"));          /* Send data */
        UART_SEND_STR(FROMMEM(str));
        UART_SEND_STR(_CRLF);
        StartCommand(ESP, CMD_TCPIP_CIPSERVER, NULL);       /* Start command */

        PT_WAIT_UNTIL(pt, ESP->Events.F.RespOk ||
                            ESP->Events.F.RespError);       /* Wait for response */

        ESP->ActiveResult = ESP->Events.F.RespOk ? espOK : espERROR;    /* Check response */

        __IDLE(ESP);                                        /* Go IDLE mode */
    } else if (ESP->ActiveCmd == CMD_TCPIP_SERVERDISABLE) { /* Disable server mode */
        __RST_EVENTS_RESP(ESP);                             /* Reset all events */
        UART_SEND_STR(FROMMEM("AT+CIPSERVER=0"));           /* Send data */
        UART_SEND_STR(_CRLF);
        StartCommand(ESP, CMD_TCPIP_CIPSERVER, NULL);       /* Start command */

        PT_WAIT_UNTIL(pt, ESP->Events.F.RespOk ||
                            ESP->Events.F.RespError);       /* Wait for response */

        ESP->ActiveResult = ESP->Events.F.RespOk ? espOK : espERROR;    /* Check response */

        __IDLE(ESP);                                        /* Go IDLE mode */
    } else if (ESP->ActiveCmd == CMD_TCPIP_CIPSTO) {        /* Set server timeout */
        NumberToString(str, Pointers.UI);                   /* Convert mode to string */
        __RST_EVENTS_RESP(ESP);                             /* Reset all events */
        UART_SEND_STR(FROMMEM("AT+CIPSTO="));               /* Send data */
        UART_SEND_STR(FROMMEM(str));
        UART_SEND_STR(_CRLF);
        StartCommand(ESP, CMD_TCPIP_CIPSERVER, NULL);       /* Start command */

        PT_WAIT_UNTIL(pt, ESP->Events.F.RespOk ||
                            ESP->Events.F.RespError);       /* Wait for response */

        ESP->ActiveResult = ESP->Events.F.RespOk ? espOK : espERROR;    /* Check response */

        __IDLE(ESP);                                        /* Go IDLE mode */
    } else if (ESP->ActiveCmd == CMD_TCPIP_CIPSTART) {      /* Start a new connection */
        __CMD_SAVE(ESP);                                    /* Save current command */

        /* Check CIPSTATUS */
        __CHECK_CIPSTATUS(ESP);

        /* Check response */
        if (ESP->Events.F.RespError) {
            ESP->ActiveResult = espERROR;
            goto cmd_tcpip_cipstart_clean;
        }

        /* Find available connection */
        for (i = 0; i < ESP_MAX_CONNECTIONS; i++) {
            if (!ESP->Conn[i].Flags.F.Active || (ESP->ActiveConns & (1 << i)) == 0) {
                ESP->Conn[i].Number = i;
                *(ESP_CONN_t **)Pointers.PPtr1 = (ESP_CONN_t *)&ESP->Conn[i];
                break;
            }
        }

        /* Check valid connection */
        if (!(*(ESP_CONN_t **)Pointers.PPtr1) || i == ESP_MAX_CONNECTIONS) {
            ESP->ActiveResult = espERROR;
            goto cmd_tcpip_cipstart_clean;
        }

        /* Check if there is an active connection and is SSL */
        if (((ESP_CONN_Type_t)(Pointers.UI >> 16)) == ESP_CONN_Type_SSL) {
            for (i = 0; i < ESP_MAX_CONNECTIONS; i++) {
                if (ESP->Conn[i].Flags.F.Active && ESP->Conn[i].Flags.F.SSL) {
                    ESP->ActiveResult = espSSLERROR;
                    goto cmd_tcpip_cipstart_clean;
                }
            }
        }

        (*(ESP_CONN_t **)Pointers.PPtr1)->Flags.F.Client = 1;   /* Connection made as client */
        (*(ESP_CONN_t **)Pointers.PPtr1)->Flags.F.SSL = ((ESP_CONN_Type_t)(Pointers.UI >> 16)) == ESP_CONN_Type_SSL;  /* Connection type is SSL */

        __RST_EVENTS_RESP(ESP);                             /* Reset all events */
        UART_SEND_STR(FROMMEM("AT+CIPSTART="));             /* Send data */
#if !ESP_SINGLE_CONN
        NumberToString(str, (*(ESP_CONN_t **)Pointers.PPtr1)->Number);  /* Convert mode to string */
        UART_SEND_STR(FROMMEM(str));
        UART_SEND_STR(FROMMEM(","));
        __DEBUG("[Server] ID(Link No.): %s\n", str);
#endif /* ESP_SINGLE_CONN */
        if (((ESP_CONN_Type_t)(Pointers.UI >> 16)) == ESP_CONN_Type_TCP) {
            UART_SEND_STR(FROMMEM("\"TCP"));
            __DEBUG("[Server] Type        : TCP\n");
        } else if (((ESP_CONN_Type_t)(Pointers.UI >> 16)) == ESP_CONN_Type_UDP) {
            UART_SEND_STR(FROMMEM("\"UDP"));
            __DEBUG("[Server] Type        : UDP\n");
        } else if (((ESP_CONN_Type_t)(Pointers.UI >> 16)) == ESP_CONN_Type_SSL) {
            UART_SEND_STR(FROMMEM("\"SSL"));
            __DEBUG("[Server] Type        : SSL\n");
        }
        UART_SEND_STR(FROMMEM("\",\""));
        UART_SEND_STR(FROMMEM(Pointers.CPtr1));
        __DEBUG("[Server] IP          : %s\n", Pointers.CPtr1);
        UART_SEND_STR(FROMMEM("\","));
        NumberToString(str, Pointers.UI & 0xFFFF);
        UART_SEND_STR(FROMMEM(str));
        UART_SEND_STR(_CRLF);
        __DEBUG("[Server] Port        : %s\n", str);

        StartCommand(ESP, CMD_TCPIP_CIPSTART, NULL);        /* Start command */

        PT_WAIT_UNTIL(pt, ESP->Events.F.RespOk ||
                            ESP->Events.F.RespError);       /* Wait for response */

        ESP->ActiveResult = ESP->Events.F.RespOk ? espOK : espERROR;    /* Check response */

        if (ESP->ActiveResult != espOK) {                   /* Failed, reset connection */
            __CONN_RESET((*(ESP_CONN_t **)Pointers.PPtr1));
            goto cmd_tcpip_cipstart_clean;
        }

        /* Execute CIPSTATUS */
        __CHECK_CIPSTATUS(ESP);                             /* Check CIPSTATUS response and parse connection parameters */

cmd_tcpip_cipstart_clean:
        __CMD_RESTORE(ESP);                                 /* Restore command */
        __IDLE(ESP);                                        /* Go IDLE mode */
    } else if (ESP->ActiveCmd == CMD_TCPIP_CIPCLOSE) {      /* Close connection */
        __CMD_SAVE(ESP);                                    /* Save current command */

        __RST_EVENTS_RESP(ESP);                             /* Reset all events */
#if !ESP_SINGLE_CONN
        NumberToString(str, Pointers.UI);                   /* Close specific connection */
        UART_SEND_STR(FROMMEM("AT+CIPCLOSE="));             /* Send data */
        UART_SEND_STR(FROMMEM(str));
#else
        UART_SEND_STR(FROMMEM("AT+CIPCLOSE"));              /* Send data */
#endif /* !ESP_SINGLE_CONN */
        UART_SEND_STR(_CRLF);
        StartCommand(ESP, CMD_TCPIP_CIPCLOSE, NULL);        /* Start command */

        PT_WAIT_UNTIL(pt, ESP->Events.F.RespOk ||
                            ESP->Events.F.RespError);       /* Wait for response */

        ESP->ActiveResult = ESP->Events.F.RespOk ? espOK : espERROR;    /* Check response */

        /* Execute CIPSTATUS */
        __CHECK_CIPSTATUS(ESP);                             /* Check CIPSTATUS and ignore response */

        __CMD_RESTORE(ESP);                                 /* Restore command */
        __IDLE(ESP);                                        /* Go IDLE mode */
    } else if (ESP->ActiveCmd == CMD_TCPIP_CIPSEND) {       /* Send data on connection */
        __CMD_SAVE(ESP);                                    /* Save command */

#if ESP_SINGLE_CONN
        if (ESP->TransferMode == ESP_TransferMode_Transparent) {/* If connection is not specified and transparent mode is in use */
            __RST_EVENTS_RESP(ESP);                         /* Reset events */
            UART_SEND_STR(FROMMEM("AT+CIPSEND"));           /* Send number to ESP */
            UART_SEND_STR(_CRLF);
            StartCommand(ESP, CMD_TCPIP_CIPSEND, NULL);     /* Start command */

            PT_WAIT_UNTIL(pt, ESP->Events.F.RespBracket ||
                                ESP->Events.F.RespError);   /* Wait for > character and timeout */

            ESP->ActiveResult = ESP->Events.F.RespBracket ? espOK : espERROR;
            ESP->Flags.F.InTransparentMode = ESP->ActiveResult == espOK;    /* Transfer mode status */
        } else
#endif /* ESP_SINGLE_CONN */
        {
            if (Pointers.Ptr2 != NULL) {
                *(uint32_t *)Pointers.Ptr2 = 0;             /* Set sent bytes to zero first */
            }

            tries = 3;                                      /* Give 3 tries to send each packet */
            do {
                btw = Pointers.UI > ESP_MAX_SEND_DATA_LEN ? ESP_MAX_SEND_DATA_LEN : Pointers.UI;    /* Set length to send */

                __RST_EVENTS_RESP(ESP);                     /* Reset events */
                UART_SEND_STR(FROMMEM("AT+CIPSEND="));      /* Send number to ESP */
#if !ESP_SINGLE_CONN
                NumberToString(str, ((ESP_CONN_t *)Pointers.Ptr1)->Number);
                UART_SEND_STR(FROMMEM(str));
                UART_SEND_STR(FROMMEM(","));
#endif /* ESP_SINGLE_CONN */
                NumberToString(str, btw);                   /* Get string from number */
                UART_SEND_STR(str);
                UART_SEND_STR(_CRLF);
                StartCommand(ESP, CMD_TCPIP_CIPSEND, NULL); /* Start command */

                PT_WAIT_UNTIL(pt, ESP->Events.F.RespBracket ||
                                    ESP->Events.F.RespError);   /* Wait for > character and timeout */

                if (ESP->Events.F.RespBracket) {            /* We received bracket */
                    __RST_EVENTS_RESP(ESP);                 /* Reset events */
                    UART_SEND((uint8_t *)Pointers.CPtr1, btw);  /* Send data */

                    PT_WAIT_UNTIL(pt, ESP->Events.F.RespSendOk ||
                                        ESP->Events.F.RespSendFail ||
                                        ESP->Events.F.RespError);   /* Wait for OK or ERROR */

                    ESP->ActiveResult = ESP->Events.F.RespSendOk ? espOK : espSENDERROR; /* Set result to return */
                    __CONN_UPDATE_TIME(ESP, (ESP_CONN_t *)Pointers.Ptr1);   /* Update connection access time */

                    if (ESP->ActiveResult == espOK) {
                        if (Pointers.Ptr2 != NULL) {
                            *(uint32_t *)Pointers.Ptr2 = *(uint32_t *)Pointers.Ptr2 + btw;  /* Increase number of sent bytes */
                        }
                    }
                } else if (ESP->Events.F.RespError) {
                    ESP->ActiveResult = espERROR;           /* Process error */
                }
                if (ESP->ActiveResult == espOK) {
                    tries = 3;                              /* Reset number of tries */

                    Pointers.UI -= btw;                     /* Decrease number of sent bytes */
                    Pointers.CPtr1 = (uint8_t *)Pointers.CPtr1 + btw;   /* Set new data memory location to send */
                } else if (ESP->Events.F.RespSendFail) {    /* Send failed */
                    tries--;                                /* We failed, decrease number of tries and start over */
                } else {                                    /* Error was received, link is probably not active */
                    tries = 0;                              /* Stop execution here */
                }
            } while (Pointers.UI && tries);                 /* Until anything to send or max tries reached */

            if (tries) {
                ((ESP_CONN_t *)Pointers.Ptr1)->Callback.F.DataSent = 1; /* Set flag for callback */
            } else {
                ((ESP_CONN_t *)Pointers.Ptr1)->Callback.F.DataError = 1;/* Set flag for callback */
            }
        }

        __CMD_RESTORE(ESP);                                 /* Restore command */
        __IDLE(ESP);                                        /* Go IDLE mode */
    } else if (ESP->ActiveCmd == CMD_TCPIP_CIPSSLSIZE) {    /* Set SSL buffer size */
        NumberToString(str, Pointers.UI);                   /* Close specific connection */
        __RST_EVENTS_RESP(ESP);                             /* Reset all events */
        UART_SEND_STR(FROMMEM("AT+CIPSSLSIZE="));           /* Send data */
        UART_SEND_STR(FROMMEM(str));
        UART_SEND_STR(_CRLF);
        StartCommand(ESP, CMD_TCPIP_CIPSSLSIZE, NULL);      /* Start command */

        PT_WAIT_UNTIL(pt, ESP->Events.F.RespOk ||
                            ESP->Events.F.RespError);       /* Wait for response */

        ESP->ActiveResult = ESP->Events.F.RespOk ? espOK : espERROR;    /* Check response */

        __IDLE(ESP);                                        /* Go IDLE mode */
    } else if (ESP->ActiveCmd == CMD_TCPIP_PING) {          /* Ping domain or IP */
        __RST_EVENTS_RESP(ESP);                             /* Reset all events */
        UART_SEND_STR(FROMMEM("AT+PING=\""));               /* Send data */
        UART_SEND_STR(FROMMEM(Pointers.CPtr1));
        UART_SEND_STR(FROMMEM("\""));
        UART_SEND_STR(_CRLF);
        StartCommand(ESP, CMD_TCPIP_PING, NULL);            /* Start command */

        PT_WAIT_UNTIL(pt, ESP->Events.F.RespOk ||
                            ESP->Events.F.RespError);       /* Wait for response */

        ESP->ActiveResult = ESP->Events.F.RespOk ? espOK : espERROR;    /* Check response */

        __IDLE(ESP);                                        /* Go IDLE mode */
    } else if (ESP->ActiveCmd == CMD_TCPIP_CIUPDATE) {      /* Update firmware from cloud */
        __RST_EVENTS_RESP(ESP);                             /* Reset all events */
        UART_SEND_STR(FROMMEM("AT+CIUPDATE"));              /* Send data */
        UART_SEND_STR(_CRLF);
        StartCommand(ESP, CMD_TCPIP_CIUPDATE, NULL);        /* Start command */

        PT_WAIT_UNTIL(pt, ESP->Events.F.RespOk ||
                            ESP->Events.F.RespError);       /* Wait for response */

        ESP->ActiveResult = ESP->Events.F.RespOk ? espOK : espERROR;    /* Check response */

        __IDLE(ESP);                                        /* Go IDLE mode */
    }
#if ESP_SINGLE_CONN
    else if (ESP->ActiveCmd == CMD_TCPIP_CIPMODE) {         /* Set transfer CIP mode */
        NumberToString(str, Pointers.UI);                   /* Close specific connection */
        __RST_EVENTS_RESP(ESP);                             /* Reset all events */
        UART_SEND_STR(FROMMEM("AT+CIPMODE="));              /* Send data */
        UART_SEND_STR(FROMMEM(str));
        UART_SEND_STR(_CRLF);
        StartCommand(ESP, CMD_TCPIP_CIPMODE, NULL);         /* Start command */

        PT_WAIT_UNTIL(pt, ESP->Events.F.RespOk ||
                            ESP->Events.F.RespError);       /* Wait for response */

        ESP->ActiveResult = ESP->Events.F.RespOk ? espOK : espERROR;    /* Check response */
        if (ESP->ActiveResult == espOK) {
            ESP->TransferMode = (ESP_TransferMode_t) Pointers.UI;
        }

        __IDLE(ESP);                                        /* Go IDLE mode */
    } else if (ESP->ActiveCmd == CMD_TCPIP_TRANSFER_STOP) {
        /****** Execute AT check ******/
        btw = ESP->Time;                                    /* Set start time */
        PT_WAIT_UNTIL(pt, (ESP->Time - btw) > 100);         /* Wait some time first */

        UART_SEND((uint8_t *)"+++", 3);                     /* Send data to stop transfer mode */

        btw = ESP->Time;                                    /* Set new start time */
        PT_WAIT_UNTIL(pt, (ESP->Time - btw) > 1100);        /* Wait at least 1 second for next command */

        ESP->Flags.F.InTransparentMode = 0;                 /* Temporarly disable transparent mode */
        RECEIVED_RESET();

        /****** Execute AT check ******/
        __RST_EVENTS_RESP(ESP);                             /* Reset all events */
        UART_SEND_STR(FROMMEM("AT"));                       /* Send data */
        UART_SEND_STR(_CRLF);
        StartCommand(ESP, CMD_TCPIP_TRANSFER_STOP, NULL);   /* Start command, use CMD_TCPIP_TRANSFER_STOP to prevent proto thread switch */

        PT_WAIT_UNTIL(pt, ESP->Events.F.RespOk ||
                            ESP->Events.F.RespError);       /* Wait for response */

        ESP->ActiveResult = ESP->Events.F.RespOk ? espOK : espERROR;    /* Check response */
        ESP->Flags.F.InTransparentMode = ESP->ActiveResult == espOK ? 0 : 1;    /* Set transparent mode status */

        __IDLE(ESP);                                        /* Go IDLE mode */
    }
#endif /* ESP_SINGLE_CONN */
    else if (ESP->ActiveCmd == CMD_TCPIP_SNTPSETCFG) {      /* Set SNTP configuration */
        __RST_EVENTS_RESP(ESP);                             /* Reset all events */

        UART_SEND_STR(FROMMEM("AT+CIPSNTPCFG="));           /* Send data */
        NumberToString(str, !!((ESP_SNTP_t *)Pointers.CPtr1)->Enable);
        UART_SEND_STR(FROMMEM(str));
        if (((ESP_SNTP_t *)Pointers.CPtr1)->Enable) {       /* SNTP is enabled, send other settings */
            sprintf(str, "%d", (signed)((ESP_SNTP_t *)Pointers.CPtr1)->Timezone);
            UART_SEND_STR(FROMMEM(","));
            UART_SEND_STR(FROMMEM(str));                    /* Send timezone */
            for (i = 0; i < sizeof(((ESP_SNTP_t *)Pointers.CPtr1)->Addr) / sizeof(((ESP_SNTP_t *)Pointers.CPtr1)->Addr[0]); i++) {  /* Check all servers if exists */
                if (((ESP_SNTP_t *)Pointers.CPtr1)->Addr[i] && strlen(((ESP_SNTP_t *)Pointers.CPtr1)->Addr[i])) {
                    UART_SEND_STR(FROMMEM(",\""));
                    UART_SEND_STR(FROMMEM(((ESP_SNTP_t *)Pointers.CPtr1)->Addr[i]));    /* Send first server address */
                    UART_SEND_STR(FROMMEM("\""));
                }
            }
        }
        UART_SEND_STR(_CRLF);
        StartCommand(ESP, CMD_TCPIP_SNTPSETCFG, NULL);      /* Start command */

        PT_WAIT_UNTIL(pt, ESP->Events.F.RespOk ||
                            ESP->Events.F.RespError);       /* Wait for response */

        ESP->ActiveResult = ESP->Events.F.RespOk ? espOK : espERROR;    /* Check response */

        __IDLE(ESP);                                        /* Go IDLE mode */
    } else if (ESP->ActiveCmd == CMD_TCPIP_SNTPGETCFG) {    /* Get SNTP config */
        __RST_EVENTS_RESP(ESP);                             /* Reset all events */
        UART_SEND_STR(FROMMEM("AT+CIPSNTPCFG?"));           /* Send data */
        UART_SEND_STR(_CRLF);
        StartCommand(ESP, CMD_TCPIP_SNTPGETCFG, NULL);      /* Start command */

        PT_WAIT_UNTIL(pt, ESP->Events.F.RespOk ||
                            ESP->Events.F.RespError);       /* Wait for response */

        ESP->ActiveResult = ESP->Events.F.RespOk ? espOK : espERROR;    /* Check response */

        __IDLE(ESP);                                        /* Go IDLE mode */
    } else if (ESP->ActiveCmd == CMD_TCPIP_CIPSNTPTIME) {   /* Get time via SNTP */
        __RST_EVENTS_RESP(ESP);                             /* Reset all events */
        UART_SEND_STR(FROMMEM("AT+CIPSNTPTIME?"));          /* Send data */
        UART_SEND_STR(_CRLF);
        StartCommand(ESP, CMD_TCPIP_CIPSNTPTIME, NULL);     /* Start command */

        PT_WAIT_UNTIL(pt, ESP->Events.F.RespOk ||
                            ESP->Events.F.RespError);       /* Wait for response */

        ESP->ActiveResult = ESP->Events.F.RespOk ? espOK : espERROR;    /* Check response */

        __IDLE(ESP);                                        /* Go IDLE mode */
    } else if (ESP->ActiveCmd == CMD_TCPIP_CIPSETDNS) {     /* Set DNS configuration */

        __RST_EVENTS_RESP(ESP);                             /* Reset all events */
        UART_SEND_STR(FROMMEM("AT+CIPDNS_"));               /* Send data */
        UART_SEND_STR(FROMMEM(Pointers.CPtr1));
        UART_SEND_STR(FROMMEM("="));

        /* Send addresses */
        do {
            const ESP_DNS_t* dns = (ESP_DNS_t *)Pointers.CPtr2;

            NumberToString(str, !!dns->Enable);
            UART_SEND_STR(FROMMEM(str));

            for (i = 0; i < sizeof(dns->Addr) / sizeof(dns->Addr[0]); i++) {
                /* When address is not 0.0.0.0 */
                if (*(uint32_t *)dns->Addr[i] != (uint32_t)0x00000000UL) {
                    uint8_t tmp_i;

                    UART_SEND_STR(FROMMEM(",\""));
                    for (tmp_i = 0; tmp_i < 4; tmp_i++) {       /* Send 4 pieces of IP address */
                        NumberToString(str, dns->Addr[i][tmp_i]);
                        UART_SEND_STR(FROMMEM(str));
                        if (tmp_i != 3) {
                            UART_SEND_STR(FROMMEM("."));
                        }
                    }
                    UART_SEND_STR(FROMMEM("\""));
                }
            }
        } while (0);

        UART_SEND_STR(_CRLF);
        StartCommand(ESP, CMD_TCPIP_CIPSETDNS, NULL);       /* Start command */

        PT_WAIT_UNTIL(pt, ESP->Events.F.RespOk ||
                            ESP->Events.F.RespError);       /* Wait for response */

        ESP->ActiveResult = ESP->Events.F.RespOk ? espOK : espERROR;    /* Check response */

        __IDLE(ESP);                                        /* Go IDLE mode */
    } else if (ESP->ActiveCmd == CMD_TCPIP_CIPGETDNS) {     /* Get DNS configuration */
        __RST_EVENTS_RESP(ESP);                             /* Reset all events */
        UART_SEND_STR(FROMMEM("AT+CIPDNS_"));               /* Send data */
        UART_SEND_STR(FROMMEM(Pointers.CPtr1));
        UART_SEND_STR(FROMMEM("?"));

        UART_SEND_STR(_CRLF);
        StartCommand(ESP, CMD_TCPIP_CIPGETDNS, NULL);       /* Start command */

        PT_WAIT_UNTIL(pt, ESP->Events.F.RespOk ||
                            ESP->Events.F.RespError);       /* Wait for response */

        ESP->ActiveResult = ESP->Events.F.RespOk ? espOK : espERROR;    /* Check response */

        __IDLE(ESP);                                        /* Go IDLE mode */
    } else if (ESP->ActiveCmd == CMD_TCPIP_CIPDOMAIN) {     /* Get IP address from domain name */
        __RST_EVENTS_RESP(ESP);                             /* Reset all events */
        UART_SEND_STR(FROMMEM("AT+CIPDOMAIN=\""));          /* Send data */
        UART_SEND_STR(FROMMEM(Pointers.CPtr1));
        UART_SEND_STR(FROMMEM("\""));
        UART_SEND_STR(_CRLF);
        StartCommand(ESP, CMD_TCPIP_CIPDOMAIN, NULL);       /* Start command */

        PT_WAIT_UNTIL(pt, ESP->Events.F.RespOk ||
                            ESP->Events.F.RespError);       /* Wait for response */

        ESP->ActiveResult = ESP->Events.F.RespOk ? espOK : espERROR;    /* Check response */

        __IDLE(ESP);                                        /* Go IDLE mode */
    }

    PT_END(pt);
}

/* Process all thread calls */
ESP_Result_t ProcessThreads(evol ESP_t* ESP) {
    if (CMD_IS_ACTIVE_BASIC(ESP)) {                         /* General related commands */
        PT_Thread_BASIC(&pt_BASIC, ESP);
    }
    if (CMD_IS_ACTIVE_WIFI(ESP)) {                          /* General related commands */
        PT_Thread_WIFI(&pt_WIFI, ESP);
    }
    if (CMD_IS_ACTIVE_TCPIP(ESP)) {                         /* On active PIN related command */
        PT_Thread_TCPIP(&pt_TCPIP, ESP);
    }
#if !ESP_RTOS && !ESP_ASYNC
    ESP_ProcessCallbacks(ESP);                              /* Process callbacks when not in RTOS or ASYNC mode */
#endif
    __RETURN(ESP, espOK);
}

/* Initialize necessary parts */
static
ESP_Result_t __Init(evol ESP_t* ESP) {
    size_t i;
    /* Reset protothreads */
    __RESET_THREADS(ESP);

    /* Close all connections if not already */
    memset((void *)&ESP->Conn, 0x00, sizeof(ESP->Conn));    /* Reset connection structure */

    /* Send initialization commands */
    ESP->Flags.F.IsBlocking = 1;                            /* Process blocking calls */
    //ESP->ActiveCmdTimeout = 5000;                           /* Set response timeout */
    ESP->ActiveCmdTimeout = 20000;                           /* Set response timeout */
    i = 50;
//    while (i) {
//        __ACTIVE_CMD(ESP, CMD_BASIC_RST);                   /* Reset device */
//        ESP_WaitReady(ESP, ESP->ActiveCmdTimeout);
//        __IDLE(ESP);
//        if (ESP->ActiveResult == espOK) {
//            break;
//        }
//        i--;
//    }
    ESP->ActiveCmdTimeout = 100;                            /* Set response timeout */
    while (i) {
        __ACTIVE_CMD(ESP, CMD_BASIC_AT);                    /* Check AT response */
        ESP_WaitReady(ESP, ESP->ActiveCmdTimeout);
        __IDLE(ESP);
        if (ESP->ActiveResult == espOK) {
            break;
        }
        i--;
    }
    while (i) {
        Pointers.UI = ESP_ECHO ? 1 : 0;
        __ACTIVE_CMD(ESP, CMD_BASIC_ATE);                    /* Check ATE response */
        ESP_WaitReady(ESP, ESP->ActiveCmdTimeout);
        __IDLE(ESP);
        if (ESP->ActiveResult == espOK) {
            break;
        }
        i--;
    }
    while (i) {
        __ACTIVE_CMD(ESP, CMD_BASIC_GMR);                   /* Check AT software */
        ESP_WaitReady(ESP, ESP->ActiveCmdTimeout);
        __IDLE(ESP);
        if (ESP->ActiveResult == espOK) {
            break;
        }
        i--;
    }
#if ESP_USE_CTS
    while (i) {
        Pointers.CPtr1 = FROMMEM("CUR");
        Pointers.UI = baudrate;
        __ACTIVE_CMD(ESP, CMD_BASIC_UART);                  /* Check AT response */
        ESP_WaitReady(ESP, ESP->ActiveCmdTimeout);
        __IDLE(ESP);
        if (ESP->ActiveResult == espOK) {
            break;
        }
        i--;
    }
#endif /* ESP_USE_CTS */
    while (i) {
        //Pointers.UI = 3;
    #if IS_CLIENT
        Pointers.UI = 1;//STATION mode
        //Pointers.UI = 3;//AP+STATION mode
    #else
        Pointers.UI = 2;//AP mode
    #endif
        __ACTIVE_CMD(ESP, CMD_WIFI_CWMODE);                 /* Set device mode */
        ESP_WaitReady(ESP, ESP->ActiveCmdTimeout);
        __IDLE(ESP);
        if (ESP->ActiveResult == espOK) {
            break;
        }
        i--;
    }
//#if IS_SERVER
    while (i) {
        Pointers.UI = !ESP_SINGLE_CONN && 1;                /* Set up for single connection mode */
        __ACTIVE_CMD(ESP, CMD_TCPIP_CIPMUX);                /* Set device mux */
        ESP_WaitReady(ESP, ESP->ActiveCmdTimeout);
        __IDLE(ESP);
        if (ESP->ActiveResult == espOK) {
            break;
        }
        i--;
    }
//#endif
//    while (i) {
//        Pointers.UI = !ESP_SINGLE_CONN && 1;                /* In single connection mode, we don't need informations on IPD data = CLIENT ONLY */
//        __ACTIVE_CMD(ESP, CMD_TCPIP_CIPDINFO);              /* Enable informations about connection on +IPD statement */
//        ESP_WaitReady(ESP, ESP->ActiveCmdTimeout);
//        __IDLE(ESP);
//        if (ESP->ActiveResult == espOK) {
//            break;
//        }
//        i--;
//    }
    while (i) {
        __ACTIVE_CMD(ESP, CMD_WIFI_GETSTAMAC);              /* Get station MAC address */
        Pointers.UI = 1;
        ESP_WaitReady(ESP, ESP->ActiveCmdTimeout);
        __IDLE(ESP);
        if (ESP->ActiveResult == espOK) {
            break;
        }
        i--;
    }
    while (i) {
        __ACTIVE_CMD(ESP, CMD_WIFI_GETAPMAC);               /* Get AP MAC address */
        Pointers.UI = 1;
        ESP_WaitReady(ESP, ESP->ActiveCmdTimeout);
        __IDLE(ESP);
        if (ESP->ActiveResult == espOK) {
            break;
        }
        i--;
    }
    while (i) {
        __ACTIVE_CMD(ESP, CMD_WIFI_GETSTAIP);               /* Get station IP */
        Pointers.UI = 1;
        ESP_WaitReady(ESP, ESP->ActiveCmdTimeout);
        __IDLE(ESP);
        if (ESP->ActiveResult == espOK) {
            break;
        }
        i--;
    }
    while (i) {
        __ACTIVE_CMD(ESP, CMD_WIFI_GETAPIP);                /* Get AP IP */
        Pointers.UI = 1;
        ESP_WaitReady(ESP, ESP->ActiveCmdTimeout);
        __IDLE(ESP);
        if (ESP->ActiveResult == espOK) {
            break;
        }
        i--;
    }
#if IS_SERVER
    while (i) {
        __ACTIVE_CMD(ESP, CMD_WIFI_GETCWSAP);               /* Get AP settings */
        ESP_WaitReady(ESP, ESP->ActiveCmdTimeout);
        __IDLE(ESP);
        if (ESP->ActiveResult == espOK) {
            break;
        }
        i--;
    }
    while (i) {
        __ACTIVE_CMD(ESP, CMD_TCPIP_SERVERDISABLE);         /* Get AP settings */
        ESP_WaitReady(ESP, ESP->ActiveCmdTimeout);
        ESP->ActiveResult = espOK;
        __IDLE(ESP);
        break;                                              /* Ignore response */
    }
#endif
    __IDLE(ESP);                                            /* Process IDLE */
    ESP->Flags.F.IsBlocking = 0;                            /* Reset blocking calls */
    ESP->Flags.F.Call_Idle = 0;

    return ESP->ActiveResult;
}

/******************************************************************************/
/******************************************************************************/
/***                              Public API                                 **/
/******************************************************************************/
/******************************************************************************/
ESP_Result_t ESP_Init(evol ESP_t* ESP, uint32_t baudrate, ESP_EventCallback_t callback) {
    BUFFER_t* Buff = &Buffer;
    uint8_t result;

    memset((void *)ESP, 0x00, sizeof(ESP_t));               /* Clear structure first */

    ESP->Callback = callback;                               /* Set event callback */
    if (callback == NULL) {
        ESP->Callback = ESP_CallbackDefault;                /* Set default callback function */
    }

#if ESP_CONN_SINGLEBUFFER
    do {
        uint8_t i = 0;
        for (i = 0; i < ESP_MAX_CONNECTIONS; i++) {
            ESP->Conn[i].Data = IPD_Data;
        }
    } while (0);
#endif /*!< ESP_CONN_SINGLEBUFFER */

    ESP->Time = 0;                                          /* Reset time start time */
    BUFFER_Init(Buff, sizeof(Buffer_Data) - 1, Buffer_Data);    /* Init buffer for receive */

    /* Low-Level initialization */
    result = 1;                                             /* Set to default value first */
    ESP->LL.Baudrate = baudrate;
    if (!ESP_LL_Callback(ESP_LL_Control_Init, (void *)&ESP->LL, &result) || result) {   /* Init low-level */
        __RETURN(ESP, espLLERROR);                          /* Return error */
    }

#if ESP_RTOS
    /* RTOS support */
    do {
        uint8_t result = 1;
        if (!ESP_LL_Callback(ESP_LL_Control_SYS_Create, (void *)&ESP->Sync, &result) || result) {
            __RETURN(ESP, espSYSERROR);
        }
    } while (0);
#endif /*!< ESP_RTOS */

    /* Send init commands */
    __RETURN(ESP, __Init(ESP));                             /* Return active status */
}

ESP_Result_t ESP_ReInit(evol ESP_t* ESP) {
    __RETURN(ESP, __Init(ESP));                             /* Process with init */
}

ESP_Result_t ESP_DeInit(evol ESP_t* ESP) {
    BUFFER_Free(&Buffer);                                   /* Clear USART buffer */

    __RETURN(ESP, espOK);                                   /* Return OK from function */
}

ESP_Result_t ESP_Update(evol ESP_t* ESP) {
    char ch;
    static char prev1_ch = 0x00, prev2_ch = 0x00;
    BUFFER_t* Buff = &Buffer;
    uint16_t processedCount = 500;

    if (ESP->ActiveCmd != CMD_IDLE && ESP->Time - ESP->ActiveCmdStart > ESP->ActiveCmdTimeout) {
        ESP->Events.F.RespError = 1;                        /* Set active error and process */
    }

    while (
#if !ESP_RTOS && ESP_ASYNC
        processedCount-- &&
#else
        processedCount-- &&
#endif /* !ESP_RTOS && ESP_ASYNC */
        BUFFER_Read(Buff, (uint8_t *)&ch, 1)                /* Read single character from buffer */
    ) {
        ESP_SET_RTS(ESP, ESP_RTS_CLR);                      /* Clear RTS pin */
        if (ESP->IPD.InIPD && ESP->IPD.BytesRemaining) {    /* Read network data */
            if (ESP->ActiveCmd == CMD_IDLE) {
                __ACTIVE_CMD(ESP, CMD_TCPIP_IPD);           /* Set active command! */
                ESP->Flags.F.IsBlocking = 1;                /* Set as it was blocking call */
            }
            ESP->IPD.Conn->Data[ESP->IPD.BytesRead] = ch;   /* Add character to receive buffer */
            ESP->IPD.BytesRead++;                           /* Increase number of bytes read in this packet */
            ESP->IPD.BytesRemaining--;                      /* Decrease number of bytes remaining to read in entire IPD packet */
            __CONN_UPDATE_TIME(ESP, ESP->IPD.Conn);         /* Update connection access time */

            if (!ESP->IPD.BytesRemaining) {                 /* We read all the data? */
                ESP->IPD.InIPD = 0;
                if (ESP->ActiveCmd == CMD_TCPIP_IPD) {      /* If we set as TCPIP then release it */
                    __IDLE(ESP);                            /* Go to IDLE mode */
                }
            }
            if (ESP->IPD.BytesRead >= (ESP_CONNBUFFER_SIZE - 1) || !ESP->IPD.BytesRemaining) { /* Receive buffer full or we read all the data? */
                ESP->CallbackParams.CP1 = ESP->IPD.Conn;
                ESP->CallbackParams.CP2 = ESP->IPD.Conn->Data;
                ESP->IPD.Conn->DataLength = ESP->IPD.BytesRead;
                ESP->CallbackParams.UI = ESP->IPD.BytesRead;
                if (ESP->IPD.BytesRemaining
#if ESP_CONN_SINGLEBUFFER
                    || (!ESP->IPD.BytesRemaining && ESP->ActiveCmd == CMD_IDLE) /*!< Do this only if low of RAM (Do not USE RTOS in this mode) */
#endif /* ESP_CONN_SINGLEBUFFER */
                ) {
                    ESP_CALL_CALLBACK(ESP, espEventDataReceived);   /* Process callback */
                    ESP->IPD.Conn->Callback.F.CallLastPartOfPacketReceived = 0;
                } else {
                    ESP->IPD.Conn->Callback.F.CallLastPartOfPacketReceived = 1;
                }
                ESP->IPD.BytesRead = 0;                     /* Reset buffer and prepare for new packet */
            }
#if ESP_SINGLE_CONN
        } else if (ESP->TransferMode == ESP_TransferMode_Transparent && ESP->Flags.F.InTransparentMode) {
            RECEIVED_ADD(ch);                               /* Add character to received data */
            if (RECEIVED_LENGTH() >= 8) {                   /* If greater than 8 for some reason */
                while (RECEIVED_LENGTH() > 8) {             /* Shift it up to 8 bytes long */
                    RECEIVED_SHIFT();
                }
                if (Received.Data[0] == 'C' && strncmp((const char *)Received.Data, FROMMEM("CLOSED\r\n"), 8) == 0) {
                    ESP->Flags.F.InTransparentMode = 0;     /* Not in transparent mode anymore */
                    __CONN_RESET(&ESP->Conn[0]);            /* Reset connection */
                    ESP->Conn[0].Callback.F.Closed = 1;     /* Set callback for connection */
                }
            }
            if (ESP->ActiveCmd == CMD_IDLE) {
                ESP->CallbackParams.CP1 = (const void *)&ESP->Conn[0];
                ESP->CallbackParams.CP2 = (const void *)&ch;
                ESP->CallbackParams.UI = 1;
                ESP_CALL_CALLBACK(ESP, espEventTransparentReceived);
            }
#endif /* ESP_SINGLE_CONN */
        } else {
            if (ISVALIDASCII(ch)) { /* Handle transparent mode receive data */
                switch (ch) {
                    case '\n':
                        RECEIVED_ADD(ch);                   /* Add character */
                        ParseReceived(ESP, &Received);      /* Parse received string */
                        RECEIVED_RESET();
                        break;
                    default:
                        if ((ch == ' ' && prev1_ch == '>' && prev2_ch == '\n')
#if ESP_SINGLE_CONN
                            || (ESP->TransferMode == ESP_TransferMode_Transparent && ch == '>' && prev1_ch == '\n')
#endif /* ESP_SINGLE_CONN */
                        ) {   /* Check if bracket received */
                            ESP->Events.F.RespBracket = 1;  /* We receive bracket on command */
                        } else {
                            RECEIVED_ADD(ch);               /* Add character to buffer */

                            /*!< Check IPD statement */
                            if (ch == ':' && RECEIVED_LENGTH() > 4) {   /* Maybe +IPD was received* */
                                if (Received.Data[0] == '+' && strncmp(FROMMEM(Received.Data), FROMMEM("+IPD"), 4) == 0) {  /* Check for IPD statement */
                                    ParseReceived(ESP, &Received);  /* Process parsing received data */
                                    RECEIVED_RESET();       /* Reset received object! */
                                }
                            }
                        }
                        break;
                }
            } else {
                RECEIVED_RESET();                           /* Reset invalid received character */
            }
        }
        prev2_ch = prev1_ch;                                /* Save previous character to prevprev character */
        prev1_ch = ch;                                      /* Save current character as previous */
        if (ESP->IPD.Conn && ESP->IPD.Conn->Callback.F.CallLastPartOfPacketReceived) {
            break;
        }
    }

    return ProcessThreads(ESP);                             /* Process stack */
}

ESP_Result_t ESP_ProcessCallbacks(evol ESP_t* ESP) {
    uint8_t i = 0;
    /* Process callbacks */
    if (ESP->ActiveCmd == CMD_IDLE && ESP->Flags.F.Call_Idle) { /* Process IDLE call */
        ESP->Flags.F.Call_Idle = 0;
        ESP_CALL_CALLBACK(ESP, espEventIdle);
    }
    if (ESP->ActiveCmd == CMD_IDLE && ESP->CallbackFlags.F.WifiConnected) { /* Wifi just connected */
        ESP->CallbackFlags.F.WifiConnected = 0;
        ESP_CALL_CALLBACK(ESP, espEventWifiConnected);
    }
    if (ESP->ActiveCmd == CMD_IDLE && ESP->CallbackFlags.F.WifiDisconnected) {  /* Wifi just connected */
        ESP->CallbackFlags.F.WifiDisconnected = 0;
        ESP_CALL_CALLBACK(ESP, espEventWifiDisconnected);
    }
    if (ESP->ActiveCmd == CMD_IDLE && ESP->CallbackFlags.F.WifiGotIP) { /* Wifi just connected */
        ESP->CallbackFlags.F.WifiGotIP = 0;
        ESP_CALL_CALLBACK(ESP, espEventWifiGotIP);
    }

    for (i = 0; i < sizeof(ESP->Conn) / sizeof(ESP->Conn[0]); i++) {
        ESP_CONN_t* c = (ESP_CONN_t *)&ESP->Conn[i];

        if (!c->Cb) {
            c->Cb = ESP->Callback;                          /* Check if callback is set */
        }

        /* Maybe move this part directly to __IDLE() command */
        /* More test required first to see usability of this */
        if (__IS_READY(ESP) && c->Callback.F.CallLastPartOfPacketReceived) {    /* Notify user about last packet */
            c->Callback.F.CallLastPartOfPacketReceived = 0;
            ESP->CallbackParams.CP1 = c;
            ESP->CallbackParams.CP2 = c->Data;
            ESP->CallbackParams.UI = c->DataLength;
            ESP_CALL_CONN_CALLBACK(ESP, c, espEventDataReceived);
        }
        if (__IS_READY(ESP) && c->Callback.F.DataSent) {    /* Data sent ok */
            c->Callback.F.DataSent = 0;
            ESP->CallbackParams.CP1 = c;
            ESP_CALL_CONN_CALLBACK(ESP, c, espEventDataSent);
        }
        if (__IS_READY(ESP) && c->Callback.F.DataError) {   /* Data sent error */
            c->Callback.F.DataError = 0;
            ESP->CallbackParams.CP1 = c;
            ESP_CALL_CONN_CALLBACK(ESP, c, espEventDataSentError);
        }
        if (__IS_READY(ESP) && c->Callback.F.Connect) {     /* Connection just active */
            c->Callback.F.Connect = 0;
            ESP->CallbackParams.CP1 = c;
            ESP_CALL_CONN_CALLBACK(ESP, c, espEventConnActive);
        }
        if (__IS_READY(ESP) && c->Callback.F.Closed) {      /* Connection just closed */
            c->Callback.F.Closed = 0;
            ESP->CallbackParams.CP1 = c;
            ESP_CALL_CONN_CALLBACK(ESP, c, espEventConnClosed);
        }
        if (__IS_READY(ESP) && c->Flags.F.Active) {
            if (c->PollTime == 0 || c->PollTimeInterval == 0) {
                c->PollTimeInterval = 1000;
                c->PollTime = ESP->Time + c->PollTimeInterval;
            }
            if (ESP->Time > c->PollTime) {
                c->PollTime += c->PollTimeInterval;
                ESP->CallbackParams.CP1 = c;
                ESP_CALL_CONN_CALLBACK(ESP, c, espEventConnPoll);
            }
        }
    }
    __RETURN(ESP, espOK);
}

void ESP_UpdateTime(evol ESP_t* ESP, uint32_t time_increase) {
    ESP->Time += time_increase;                             /* Increase time */
}

ESP_Result_t ESP_GetLastReturnStatus(evol ESP_t* ESP) {
    ESP_Result_t tmp = ESP->ActiveResult;
    ESP->ActiveResult = espOK;

    return tmp;
}

uint16_t ESP_DataReceived(uint8_t* ch, uint16_t count) {
    uint16_t r;
    r = BUFFER_Write(&Buffer, ch, count);                   /* Writes data to USART buffer */
#if ESP_USE_CTS
    if (BUFFER_GetFree(&Buffer) <= 3) {
        ESP_SET_RTS(_ESP, ESP_RTS_SET);                     /* Set RTS pin */
    }
#endif /* ESP_USE_CTS */
    return r;
}

/******************************************************************************/
/*                              Device ready status                           */
/******************************************************************************/
ESP_Result_t ESP_WaitReady(evol ESP_t* ESP, uint32_t timeout) {
    ESP->ActiveCmdTimeout = timeout;                        /* Set timeout value */
    do {
#if !ESP_RTOS && !ESP_ASYNC
        ESP_Update(ESP);                                    /* Update stack if we are in synchronous mode */
#else
        ESP_ProcessCallbacks(ESP);                          /* Process callbacks when not in synchronous mode */
        ESP_RTOS_YIELD();
#endif /* !ESP_RTOS && !ESP_ASYNC */
    } while (__IS_BUSY(ESP));
    __RETURN(ESP, ESP->ActiveResult);                       /* Return active result from command */
}

ESP_Result_t ESP_Delay(evol ESP_t* ESP, uint32_t timeout) {
    evol uint32_t start = ESP->Time;
    do {
#if !ESP_RTOS && !ESP_ASYNC
        ESP_Update(ESP);
#else
        ESP_RTOS_YIELD();
#endif /* !ESP_RTOS && !ESP_ASYNC */
    } while (ESP->Time - start < timeout);
    __RETURN(ESP, espOK);
}

ESP_Result_t ESP_IsReady(evol ESP_t* ESP) {
    return ESP->ActiveCmd != CMD_IDLE ? espERROR : espOK;
}

/******************************************************************************/
/***                              Device settings                            **/
/******************************************************************************/
ESP_Result_t ESP_RestoreDefault(evol ESP_t* ESP, uint32_t blocking) {
    __CHECK_BUSY(ESP);                                      /* Check busy status */
    __ACTIVE_CMD(ESP, CMD_BASIC_RESTORE);                   /* Set active command */

    __RETURN_BLOCKING(ESP, blocking, 2000);                 /* Return with blocking support */
}

ESP_Result_t ESP_SetUART(evol ESP_t* ESP, uint32_t baudrate, uint32_t def, uint32_t blocking) {
    __CHECK_INPUTS(baudrate >= 110 && baudrate <= (40 * 115200));   /* Check inputs */
    __CHECK_BUSY(ESP);                                      /* Check busy status */
    __ACTIVE_CMD(ESP, CMD_BASIC_UART);                      /* Set active command */

    Pointers.CPtr1 = def ? FROMMEM("DEF") : FROMMEM("CUR");
    Pointers.UI = baudrate;

    __RETURN_BLOCKING(ESP, blocking, 1000);                 /* Return with blocking support */
}

ESP_Result_t ESP_SetRFPower(evol ESP_t* ESP, float pwr, uint32_t blocking) {
    __CHECK_INPUTS(pwr > 0 && (pwr / 0.25f) <= ESP_MAX_RFPWR);  /* Check inputs */
    __CHECK_BUSY(ESP);                                      /* Check busy status */
    __ACTIVE_CMD(ESP, CMD_BASIC_RFPOWER);                   /* Set active command */

    Pointers.UI = (uint8_t)(pwr / 0.25f);

    __RETURN_BLOCKING(ESP, blocking, 1000);                 /* Return with blocking support */
}

ESP_Result_t ESP_FirmwareUpdate(evol ESP_t* ESP, uint32_t blocking) {
    __CHECK_BUSY(ESP);                                      /* Check busy status */
    __ACTIVE_CMD(ESP, CMD_TCPIP_CIUPDATE);                  /* Set active command */

    __RETURN_BLOCKING(ESP, blocking, 180000);               /* Return with blocking support */
}

ESP_Result_t ESP_GetSoftwareInfo(evol ESP_t* ESP, char* atv, char* sdkv, char* cmpt, uint32_t blocking) {
    __CHECK_INPUTS(atv || sdkv || cmpt);                    /* Check inputs, at least one must be valid to start with this command */
    __CHECK_BUSY(ESP);                                      /* Check busy status */
    __ACTIVE_CMD(ESP, CMD_BASIC_GMR);                       /* Set active command */

    /* Use const pointers because of missing structure data and to prevent RAM usage */
    Pointers.CPtr1 = (const void *)atv;
    Pointers.CPtr2 = (const void *)sdkv;
    Pointers.CPtr3 = (const void *)cmpt;

    __RETURN_BLOCKING(ESP, blocking, 180000);               /* Return with blocking support */
}

/******************************************************************************/
/***                         STATION AND AP settings                         **/
/******************************************************************************/
ESP_Result_t ESP_STA_GetIP(evol ESP_t* ESP, uint8_t* ip, uint32_t blocking) {
    __CHECK_BUSY(ESP);                                      /* Check busy status */
    __ACTIVE_CMD(ESP, CMD_WIFI_GETSTAIP);                   /* Set active command */

    Pointers.Ptr1 = ip;                                     /* Save pointer to save IP to */

    __RETURN_BLOCKING(ESP, blocking, 1000);                 /* Return with blocking support */
}

ESP_Result_t ESP_STA_SetIP(evol ESP_t* ESP, const uint8_t* ip, const uint8_t* gw_msk, uint8_t def, uint32_t blocking) {
    __CHECK_BUSY(ESP);                                      /* Check busy status */
    __ACTIVE_CMD(ESP, CMD_WIFI_SETSTAIP);                   /* Set active command */

    Pointers.CPtr1 = def ? FROMMEM("DEF") : FROMMEM("CUR");
    Pointers.CPtr2 = ip;
    Pointers.CPtr3 = gw_msk;

    __RETURN_BLOCKING(ESP, blocking, 1000);                 /* Return with blocking support */
}

ESP_Result_t ESP_STA_GetMAC(evol ESP_t* ESP, uint8_t* mac, uint32_t blocking) {
    __CHECK_BUSY(ESP);                                      /* Check busy status */
    __ACTIVE_CMD(ESP, CMD_WIFI_GETSTAMAC);                  /* Set active command */

    Pointers.Ptr1 = mac;                                    /* Save pointer to save MAC to */

    __RETURN_BLOCKING(ESP, blocking, 1000);                 /* Return with blocking support */
}

ESP_Result_t ESP_STA_SetMAC(evol ESP_t* ESP, const uint8_t* mac, uint32_t def, uint32_t blocking) {
    __CHECK_INPUTS(mac && (*mac & 0x01) == 0);              /* Check inputs, bit 0 of first byte cannot be set to 1 on station */
    __CHECK_BUSY(ESP);                                      /* Check busy status */
    __ACTIVE_CMD(ESP, CMD_WIFI_SETSTAMAC);                  /* Set active command */

    Pointers.CPtr1 = def ? FROMMEM("DEF") : FROMMEM("CUR");
    Pointers.CPtr2 = mac;

    __RETURN_BLOCKING(ESP, blocking, 1000);                 /* Return with blocking support */
}

ESP_Result_t ESP_AP_GetIP(evol ESP_t* ESP, uint8_t* ip, uint32_t blocking) {
    __CHECK_BUSY(ESP);                                      /* Check busy status */
    __ACTIVE_CMD(ESP, CMD_WIFI_GETAPIP);                    /* Set active command */

    Pointers.Ptr1 = ip;                                     /* Save pointer to save IP to */

    __RETURN_BLOCKING(ESP, blocking, 1000);                 /* Return with blocking support */
}

ESP_Result_t ESP_AP_SetIP(evol ESP_t* ESP, const uint8_t* ip, uint8_t def, uint32_t blocking) {
    __CHECK_BUSY(ESP);                                      /* Check busy status */
    __ACTIVE_CMD(ESP, CMD_WIFI_SETAPIP);                    /* Set active command */

    Pointers.CPtr1 = def ? FROMMEM("DEF") : FROMMEM("CUR");
    Pointers.CPtr2 = ip;

    __RETURN_BLOCKING(ESP, blocking, 1000);                 /* Return with blocking support */
}

ESP_Result_t ESP_AP_GetMAC(evol ESP_t* ESP, uint8_t* mac, uint32_t blocking) {
    __CHECK_BUSY(ESP);                                      /* Check busy status */
    __ACTIVE_CMD(ESP, CMD_WIFI_GETAPMAC);                   /* Set active command */

    Pointers.Ptr1 = mac;                                    /* Save pointer to save MAC to */

    __RETURN_BLOCKING(ESP, blocking, 1000);                 /* Return with blocking support */
}

ESP_Result_t ESP_AP_SetMAC(evol ESP_t* ESP, const uint8_t* mac, uint32_t def, uint32_t blocking) {
    __CHECK_INPUTS(mac);                                    /* Check inputs */
    __CHECK_BUSY(ESP);                                      /* Check busy status */
    __ACTIVE_CMD(ESP, CMD_WIFI_SETAPMAC);                   /* Set active command */

    Pointers.CPtr1 = def ? FROMMEM("DEF") : FROMMEM("CUR");
    Pointers.CPtr2 = mac;

    __RETURN_BLOCKING(ESP, blocking, 1000);                 /* Return with blocking support */
}

ESP_Result_t ESP_AP_GetConfig(evol ESP_t* ESP, uint32_t blocking) {
    __CHECK_BUSY(ESP);                                      /* Check busy status */
    __ACTIVE_CMD(ESP, CMD_WIFI_GETCWSAP);                   /* Set active command */

    __RETURN_BLOCKING(ESP, blocking, 1000);                 /* Return with blocking support */
}

ESP_Result_t ESP_AP_SetConfig(evol ESP_t* ESP, ESP_APConfig_t* conf, uint8_t def, uint32_t blocking) {
    __CHECK_INPUTS(conf);                                   /* Check inputs */
    __CHECK_BUSY(ESP);                                      /* Check busy status */
    __ACTIVE_CMD(ESP, CMD_WIFI_SETCWSAP);                   /* Set active command */

    Pointers.CPtr1 = def ? FROMMEM("DEF") : FROMMEM("CUR");
    Pointers.CPtr2 = conf;

    __RETURN_BLOCKING(ESP, blocking, 1000);                 /* Return with blocking support */
}

/******************************************************************************/
/***                            SYSTEM settings                              **/
/******************************************************************************/
ESP_Result_t ESP_SYS_GetAvailableRAM(evol ESP_t* ESP, uint32_t* ram, uint32_t blocking) {
    __CHECK_INPUTS(ram);                                    /* Check inputs */
    __CHECK_BUSY(ESP);                                      /* Check busy status */
    __ACTIVE_CMD(ESP, CMD_BASIC_GETSYSRAM);                 /* Set active command */

    Pointers.Ptr1 = ram;

    __RETURN_BLOCKING(ESP, blocking, 1000);                 /* Return with blocking support */
}

ESP_Result_t ESP_SYS_ReadADC(evol ESP_t* ESP, uint32_t* adc, uint32_t blocking) {
    __CHECK_INPUTS(adc);                                    /* Check inputs */
    __CHECK_BUSY(ESP);                                      /* Check busy status */
    __ACTIVE_CMD(ESP, CMD_BASIC_GETSYSADC);                 /* Set active command */

    Pointers.Ptr1 = adc;

    __RETURN_BLOCKING(ESP, blocking, 1000);                 /* Return with blocking support */
}

ESP_Result_t ESP_SYS_GPIO_Read(evol ESP_t* ESP, uint8_t gpionum, uint8_t* level, ESP_GPIO_Dir_t* dir, uint32_t blocking) {
    __CHECK_INPUTS(level);                                  /* Check inputs */
    __CHECK_BUSY(ESP);                                      /* Check busy status */
    __ACTIVE_CMD(ESP, CMD_BASIC_SYSGPIOREAD);               /* Set active command */

    Pointers.Ptr1 = level;
    Pointers.Ptr2 = dir;
    Pointers.UI = gpionum;

    __RETURN_BLOCKING(ESP, blocking, 1000);                 /* Return with blocking support */
}

ESP_Result_t ESP_SYS_GPIO_Write(evol ESP_t* ESP, uint8_t gpionum, uint8_t val, uint32_t blocking) {
    __CHECK_BUSY(ESP);                                      /* Check busy status */
    __ACTIVE_CMD(ESP, CMD_BASIC_SYSGPIOWRITE);              /* Set active command */

    Pointers.UI = (!!val) << 8 | gpionum;                   /* Save values for gpio number and value to write */

    __RETURN_BLOCKING(ESP, blocking, 1000);                 /* Return with blocking support */
}

ESP_Result_t ESP_SYS_GPIO_SetConfig(evol ESP_t* ESP, uint8_t gpionum, const ESP_GPIO_t* conf, uint32_t blocking) {
    __CHECK_INPUTS(conf);                                   /* Check inputs */
    __CHECK_BUSY(ESP);                                      /* Check busy status */
    __ACTIVE_CMD(ESP, CMD_BASIC_SYSIOSETCFG);               /* Set active command */

    Pointers.CPtr1 = conf;
    Pointers.UI = gpionum;

    __RETURN_BLOCKING(ESP, blocking, 1000);                 /* Return with blocking support */
}

ESP_Result_t ESP_SYS_GPIO_GetConfig(evol ESP_t* ESP, uint8_t gpionum, ESP_GPIO_t* conf, uint32_t blocking) {
    __CHECK_INPUTS(conf);                                   /* Check inputs */
    __CHECK_BUSY(ESP);                                      /* Check busy status */
    __ACTIVE_CMD(ESP, CMD_BASIC_SYSIOGETCFG);               /* Set active command */

    Pointers.Ptr1 = conf;
    Pointers.UI = gpionum;

    __RETURN_BLOCKING(ESP, blocking, 1000);                 /* Return with blocking support */
}

ESP_Result_t ESP_SYS_GPIO_SetDir(evol ESP_t* ESP, uint8_t gpionum, const ESP_GPIO_t* conf, uint32_t blocking) {
    __CHECK_INPUTS(conf);                                   /* Check inputs */
    __CHECK_BUSY(ESP);                                      /* Check busy status */
    __ACTIVE_CMD(ESP, CMD_BASIC_SYSGPIOSETDIR);             /* Set active command */

    Pointers.CPtr1 = conf;
    Pointers.UI = gpionum;

    __RETURN_BLOCKING(ESP, blocking, 1000);                 /* Return with blocking support */
}

#if !ESP_SINGLE_CONN
/******************************************************************************/
/***                            SERVER settings                              **/
/******************************************************************************/
ESP_Result_t ESP_SERVER_Enable(evol ESP_t* ESP, uint16_t port, uint32_t blocking) {
    __CHECK_INPUTS(port > 0);                               /* Check inputs */
    __CHECK_BUSY(ESP);                                      /* Check busy status */
    __ACTIVE_CMD(ESP, CMD_TCPIP_SERVERENABLE);              /* Set active command */

    Pointers.UI = port;                                     /* port number */

    __RETURN_BLOCKING(ESP, blocking, 1000);                 /* Return with blocking support */
}

ESP_Result_t ESP_SERVER_Disable(evol ESP_t* ESP, uint32_t blocking) {
    __CHECK_BUSY(ESP);                                      /* Check busy status */
    __ACTIVE_CMD(ESP, CMD_TCPIP_SERVERDISABLE);             /* Set active command */

    __RETURN_BLOCKING(ESP, blocking, 1000);                 /* Return with blocking support */
}

ESP_Result_t ESP_SERVER_SetTimeout(evol ESP_t* ESP, uint16_t timeout, uint32_t blocking) {
    __CHECK_BUSY(ESP);                                      /* Check busy status */
    __ACTIVE_CMD(ESP, CMD_TCPIP_CIPSTO);                    /* Set active command */

    Pointers.UI = timeout;                                  /* Save timeout value */

    __RETURN_BLOCKING(ESP, blocking, 1000);                 /* Return with blocking support */
}
#endif /* ESP_SINGLE_CONN */

/******************************************************************************/
/***                            List wifi stations                           **/
/******************************************************************************/
ESP_Result_t ESP_STA_ListAccessPoints(evol ESP_t* ESP, ESP_AP_t* APs, uint16_t atr, uint16_t* ar, uint32_t blocking) {
    __CHECK_INPUTS(APs && atr && ar);                       /* Check inputs */
    __CHECK_BUSY(ESP);                                      /* Check busy status */
    __ACTIVE_CMD(ESP, CMD_WIFI_LISTACCESSPOINTS);           /* Set active command */

    *ar = 0;
    Pointers.Ptr1 = APs;
    Pointers.Ptr2 = ar;
    Pointers.UI = atr;

    __RETURN_BLOCKING(ESP, blocking, 10000);                /* Return with blocking support */
}

ESP_Result_t ESP_AP_ListConnectedStations(evol ESP_t* ESP, ESP_ConnectedStation_t* stations, uint16_t size, uint16_t* sr, uint32_t blocking) {
    __CHECK_INPUTS(stations && size && sr);                 /* Check inputs */
    __CHECK_BUSY(ESP);                                      /* Check busy status */
    __ACTIVE_CMD(ESP, CMD_WIFI_CWLIF);                      /* Set active command */

    *sr = 0;
    Pointers.Ptr1 = stations;
    Pointers.Ptr2 = sr;
    Pointers.UI = size;

    __RETURN_BLOCKING(ESP, blocking, 10000);                /* Return with blocking support */
}

/******************************************************************************/
/***                            Connect to network                           **/
/******************************************************************************/
ESP_Result_t ESP_STA_Connect(evol ESP_t* ESP, const char* ssid, const char* pass, const uint8_t* mac, uint32_t def, uint32_t blocking) {
    __CHECK_INPUTS(ssid && pass);                           /* Check inputs */
    __CHECK_BUSY(ESP);                                      /* Check busy status */
    __ACTIVE_CMD(ESP, CMD_WIFI_CWJAP);                      /* Set active command */

    //Pointers.CPtr1 = def ? FROMMEM("DEF") : FROMMEM("CUR");
    Pointers.CPtr1 = def ? FROMMEM("DEF") : FROMMEM("");
    Pointers.CPtr2 = ssid;
    Pointers.CPtr3 = pass;
    Pointers.Ptr1 = (void *)mac;

    __RETURN_BLOCKING(ESP, blocking, 30000);                /* Return with blocking support */
}

ESP_Result_t ESP_STA_GetConnected(evol ESP_t* ESP, ESP_ConnectedAP_t* AP, uint32_t blocking) {
    __CHECK_BUSY(ESP);                                      /* Check busy status */
    __ACTIVE_CMD(ESP, CMD_WIFI_GETCWJAP);                   /* Set active command */

    Pointers.Ptr1 = AP;

    __RETURN_BLOCKING(ESP, blocking, 10000);                /* Return with blocking support */
}

ESP_Result_t ESP_STA_Disconnect(evol ESP_t* ESP, uint32_t blocking) {
    __CHECK_BUSY(ESP);                                      /* Check busy status */
    __ACTIVE_CMD(ESP, CMD_WIFI_CWQAP);                      /* Set active command */

    __RETURN_BLOCKING(ESP, blocking, 10000);                /* Return with blocking support */
}

ESP_Result_t ESP_STA_SetAutoConnect(evol ESP_t* ESP, uint8_t autoconn, uint32_t blocking) {
    __CHECK_BUSY(ESP);                                      /* Check busy status */
    __ACTIVE_CMD(ESP, CMD_WIFI_CWAUTOCONN);                 /* Set active command */

    Pointers.UI = autoconn ? 1 : 0;

    __RETURN_BLOCKING(ESP, blocking, 10000);                /* Return with blocking support */
}

/******************************************************************************/
/***                           Connection management                         **/
/******************************************************************************/
ESP_Result_t ESP_CONN_Start(evol ESP_t* ESP, ESP_CONN_t** conn, ESP_CONN_Type_t type, const char* domain, uint16_t port, uint32_t blocking) {
    __CHECK_INPUTS(conn && domain && port);                 /* Check inputs */
    __CHECK_BUSY(ESP);                                      /* Check busy status */
    __ACTIVE_CMD(ESP, CMD_TCPIP_CIPSTART);                  /* Set active command */

    Pointers.PPtr1 = (evol void **)conn;
    Pointers.CPtr1 = domain;
    Pointers.UI = type << 16 | port;

    __RETURN_BLOCKING(ESP, blocking, 180000);               /* Return with blocking support */
}

ESP_Result_t ESP_CONN_Send(evol ESP_t* ESP, ESP_CONN_t* conn, const uint8_t* data, uint32_t btw, uint32_t* bw, uint32_t blocking) {
    __CHECK_INPUTS(conn && data && btw);                    /* Check inputs */
    __CHECK_BUSY(ESP);                                      /* Check busy status */
    __ACTIVE_CMD(ESP, CMD_TCPIP_CIPSEND);                   /* Set active command */

    Pointers.Ptr1 = conn;
    Pointers.Ptr2 = bw;
    Pointers.CPtr1 = data;
    Pointers.UI = btw;

    __RETURN_BLOCKING(ESP, blocking, 10000);                /* Return with blocking support */
}

ESP_Result_t ESP_CONN_Close(evol ESP_t* ESP, ESP_CONN_t* conn, uint32_t blocking) {
    __CHECK_INPUTS(conn);                                   /* Check inputs */
    __CHECK_BUSY(ESP);                                      /* Check busy status */
    __ACTIVE_CMD(ESP, CMD_TCPIP_CIPCLOSE);                  /* Set active command */

    Pointers.UI = conn->Number;

    __RETURN_BLOCKING(ESP, blocking, 10000);                /* Return with blocking support */
}

ESP_Result_t ESP_CONN_CloseAll(evol ESP_t* ESP, uint32_t blocking) {
    __CHECK_BUSY(ESP);                                      /* Check busy status */
    __ACTIVE_CMD(ESP, CMD_TCPIP_CIPCLOSE);                  /* Set active command */

    Pointers.UI = ESP_MAX_CONNECTIONS;                      /* Close all connections */

    __RETURN_BLOCKING(ESP, blocking, 5000);                 /* Return with blocking support */
}

ESP_Result_t ESP_CONN_SetArg(evol ESP_t* ESP, ESP_CONN_t* conn, void* arg, uint32_t blocking) {
    __CHECK_INPUTS(conn);                                   /* Check inputs */
    conn->Arg = arg;
    return espOK;
}

void* ESP_CONN_GetArg(evol ESP_t* ESP, ESP_CONN_t* conn) {
    return conn->Arg;
}

ESP_Result_t ESP_CONN_SetCallback(evol ESP_t* ESP, ESP_CONN_t* conn, ESP_EventCallback_t cb, uint32_t blocking) {
    __CHECK_INPUTS(conn);                                   /* Check inputs */
    conn->Cb = cb ? cb : ESP->Callback;                     /* Set connection callback */
    return espOK;
}

ESP_Result_t ESP_SetSSLBufferSize(evol ESP_t* ESP, uint32_t size, uint32_t blocking) {
    __CHECK_INPUTS(size >= 2048 && size <= 4096);           /* Check inputs */
    __CHECK_BUSY(ESP);                                      /* Check busy status */
    __ACTIVE_CMD(ESP, CMD_TCPIP_CIPSSLSIZE);                /* Set active command */

    Pointers.UI = size;                                     /* Close all connections */

    __RETURN_BLOCKING(ESP, blocking, 1000);                 /* Return with blocking support */
}

#if ESP_SINGLE_CONN
/******************************************************************************/
/***                           Transparent transfer                          **/
/******************************************************************************/
ESP_Result_t ESP_TRANSFER_SetMode(evol ESP_t* ESP, ESP_TransferMode_t Mode, uint32_t blocking) {
    __CHECK_BUSY(ESP);                                      /* Check busy status */
    __ACTIVE_CMD(ESP, CMD_TCPIP_CIPMODE);                   /* Set active command */

    Pointers.UI = (uint8_t)Mode;                            /* Close all connections */

    __RETURN_BLOCKING(ESP, blocking, 1000);                 /* Return with blocking support */
}

ESP_Result_t ESP_TRANSFER_Start(evol ESP_t* ESP, uint32_t blocking) {
    __CHECK_BUSY(ESP);                                      /* Check busy status */
    if (ESP->TransferMode != ESP_TransferMode_Transparent) {
        return espERROR;
    }
    __ACTIVE_CMD(ESP, CMD_TCPIP_CIPSEND);                   /* Set active command */

    /* Set up send without parameters, execute only AT+CIPSEND command to start transparent mode */
    memset((void *)&Pointers, 0x00, sizeof(Pointers));

    __RETURN_BLOCKING(ESP, blocking, 10000);                /* Return with blocking support */
}

ESP_Result_t ESP_TRANSFER_Send(evol ESP_t* ESP, const void* data, uint32_t length, uint32_t blocking) {
    __CHECK_BUSY(ESP);                                      /* Check busy status */
    if (!ESP->Flags.F.InTransparentMode) {
        return espERROR;
    }
    __ACTIVE_CMD(ESP, CMD_TCPIP_TRANSFER_SEND);             /* Set active command */

    UART_SEND((uint8_t *)data, length);                     /* Send data to wifi directly */

    ESP->ActiveResult = espOK;                              /* Return OK */
    __IDLE(ESP);
    __RETURN_BLOCKING(ESP, blocking, 1000);                 /* Return with blocking support */
}

ESP_Result_t ESP_TRANSFER_Stop(evol ESP_t* ESP, uint32_t blocking) {
    if (!ESP->Flags.F.InTransparentMode) {
        return espERROR;
    }
    __ACTIVE_CMD(ESP, CMD_TCPIP_TRANSFER_STOP);             /* Set active command */

    __RETURN_BLOCKING(ESP, blocking, 5000);                 /* Return with blocking support */
}
#endif /* ESP_SINGLE_CONN */

/******************************************************************************/
/***                              SNTP API                                   **/
/******************************************************************************/
ESP_Result_t ESP_SNTP_SetConfig(evol ESP_t* ESP, const ESP_SNTP_t* sntp, uint32_t blocking) {
    __CHECK_INPUTS(sntp && (!sntp->Enable || (sntp->Enable && sntp->Timezone >= -11 && sntp->Timezone <= 13))); /* Check input parameters */
    __CHECK_BUSY(ESP);                                      /* Check busy status */
    __ACTIVE_CMD(ESP, CMD_TCPIP_SNTPSETCFG);                /* Set active command */

    Pointers.CPtr1 = sntp;

    __RETURN_BLOCKING(ESP, blocking, 1000);                 /* Return with blocking support */
}

ESP_Result_t ESP_SNTP_GetConfig(evol ESP_t* ESP, ESP_SNTP_t* sntp, uint32_t blocking) {
    __CHECK_INPUTS(sntp);                                   /* Check input parameters */
    __CHECK_BUSY(ESP);                                      /* Check busy status */
    __ACTIVE_CMD(ESP, CMD_TCPIP_SNTPGETCFG);                /* Set active command */

    Pointers.Ptr1 = sntp;

    __RETURN_BLOCKING(ESP, blocking, 1000);                 /* Return with blocking support */
}

ESP_Result_t ESP_SNTP_GetDateTime(evol ESP_t* ESP, ESP_DateTime_t* dt, uint32_t blocking) {
    __CHECK_INPUTS(dt);                                     /* Check input parameters */
    __CHECK_BUSY(ESP);                                      /* Check busy status */
    __ACTIVE_CMD(ESP, CMD_TCPIP_CIPSNTPTIME);               /* Set active command */

    Pointers.Ptr1 = dt;

    __RETURN_BLOCKING(ESP, blocking, 1000);                 /* Return with blocking support */
}

/******************************************************************************/
/***                              DNS API                                    **/
/******************************************************************************/
ESP_Result_t ESP_DNS_SetConfig(evol ESP_t* ESP, const ESP_DNS_t* dns, uint8_t def, uint32_t blocking) {
    __CHECK_INPUTS(dns);                                    /* Check inputs */
    __CHECK_BUSY(ESP);                                      /* Check busy status */
    __ACTIVE_CMD(ESP, CMD_TCPIP_CIPSETDNS);                 /* Set active command */

    Pointers.CPtr1 = def ? FROMMEM("DEF") : FROMMEM("CUR");
    Pointers.CPtr2 = dns;

    __RETURN_BLOCKING(ESP, blocking, 1000);                 /* Return with blocking support */
}

ESP_Result_t ESP_DNS_GetConfig(evol ESP_t* ESP, ESP_DNS_t* dns, uint8_t def, uint32_t blocking) {
    __CHECK_INPUTS(dns);                                    /* Check inputs */
    __CHECK_BUSY(ESP);                                      /* Check busy status */
    __ACTIVE_CMD(ESP, CMD_TCPIP_CIPGETDNS);                 /* Set active command */

    Pointers.CPtr1 = def ? FROMMEM("DEF") : FROMMEM("CUR");
    Pointers.Ptr1 = dns;
    dns->_ptr = 0;

    __RETURN_BLOCKING(ESP, blocking, 1000);                 /* Return with blocking support */
}

ESP_Result_t ESP_DNS_GetIp(evol ESP_t* ESP, const char* domain, uint8_t* ip, uint32_t blocking) {
    __CHECK_INPUTS(ip);                                     /* Check inputs */
    __CHECK_BUSY(ESP);                                      /* Check busy status */
    __ACTIVE_CMD(ESP, CMD_TCPIP_CIPDOMAIN);                 /* Set active command */

    Pointers.CPtr1 = domain;
    Pointers.Ptr1 = ip;

    __RETURN_BLOCKING(ESP, blocking, 10000);                /* Return with blocking support */
}

/******************************************************************************/
/***                            Miscellanious                                **/
/******************************************************************************/
ESP_Result_t ESP_Ping(evol ESP_t* ESP, const char* addr, uint32_t* time, uint32_t blocking) {
    __CHECK_INPUTS(addr && time);                           /* Check inputs */
    __CHECK_BUSY(ESP);                                      /* Check busy status */
    __ACTIVE_CMD(ESP, CMD_TCPIP_PING);                      /* Set active command */

    *time = 0;
    Pointers.CPtr1 = addr;
    Pointers.Ptr1 = time;

    __RETURN_BLOCKING(ESP, blocking, 10000);                /* Return with blocking support */
}

ESP_Result_t ESP_SetWPS(evol ESP_t* ESP, uint8_t wps, uint32_t blocking) {
    __CHECK_BUSY(ESP);                                      /* Check busy status */
    __ACTIVE_CMD(ESP, CMD_WIFI_WPS);                        /* Set active command */

    Pointers.UI = wps ? 1 : 0;

    __RETURN_BLOCKING(ESP, blocking, 1000);                 /* Return with blocking support */
}

ESP_Result_t ESP_SetHostName(evol ESP_t* ESP, const char* hostname, uint32_t blocking) {
    __CHECK_INPUTS(hostname);                               /* Check inputs */
    __CHECK_BUSY(ESP);                                      /* Check busy status */
    __ACTIVE_CMD(ESP, CMD_WIFI_SETHOSTNAME);                /* Set active command */

    Pointers.CPtr1 = hostname;

    __RETURN_BLOCKING(ESP, blocking, 1000);                 /* Return with blocking support */
}

ESP_Result_t ESP_GetHostName(evol ESP_t* ESP, char* hostname, uint32_t blocking) {
    __CHECK_INPUTS(hostname);                               /* Check inputs */
    __CHECK_BUSY(ESP);                                      /* Check busy status */
    __ACTIVE_CMD(ESP, CMD_WIFI_GETHOSTNAME);                /* Set active command */

    Pointers.Ptr1 = hostname;

    __RETURN_BLOCKING(ESP, blocking, 1000);                 /* Return with blocking support */
}

void ESP_AssertRTS(evol ESP_t* ESP) {
    uint8_t state = ESP_RTS_SET;
    uint8_t result = 1;

    ESP->Flags.F.RTSForced = 1;
    ESP_LL_Callback(ESP_LL_Control_SetRTS, &state, &result);
}

void ESP_DesertRTS(evol ESP_t* ESP) {
    if (ESP->Flags.F.RTSForced) {
        uint8_t state = ESP_RTS_CLR;
        uint8_t result = 1;

        ESP_LL_Callback(ESP_LL_Control_SetRTS, &state, &result);
        ESP->Flags.F.RTSForced = 1;
    }
}
