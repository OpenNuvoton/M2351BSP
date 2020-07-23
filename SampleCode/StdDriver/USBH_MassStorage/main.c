/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    This sample uses a command-shell-like interface to demonstrate
 *           how to use USBH mass storage driver and make it working as a disk
 *           driver under FATFS file system.
 *
 *
 * @copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <string.h>

#include "NuMicro.h"

#include "usbh_lib.h"
#include "ff.h"
#include "diskio.h"

#define CLK_PLLCTL_192MHz_HXT   (CLK_PLLCTL_PLLSRC_HXT  | CLK_PLLCTL_NR(2) | CLK_PLLCTL_NF( 16) | CLK_PLLCTL_NO_1)

#define BUFF_SIZE       (4*1024)

static UINT g_u8Len = BUFF_SIZE;
static uint32_t s_u32AccSize;                         /* Work register for fs command */
static uint16_t s_u16AccFiles, s_u16AccDirs;
static FILINFO s_Finfo;

static char s_achLine[256];                         /* Console input buffer */
#if _USE_LFN
char g_achLfname[512];
#endif

#ifdef __ICCARM__
#pragma data_alignment=32
BYTE s_au8BuffPool[BUFF_SIZE];       /* Working buffer */
BYTE s_au8BuffPool2[BUFF_SIZE];      /* Working buffer 2 */
#else
static uint8_t s_au8BuffPool[BUFF_SIZE] __attribute__((aligned(32)));       /* Working buffer */
static uint8_t s_au8BuffPool2[BUFF_SIZE] __attribute__((aligned(32)));      /* Working buffer 2 */
#endif

static uint8_t  *s_pu8Buff;
static uint8_t  *s_pu8Buff2;


#ifdef __ICCARM__
#pragma data_alignment=4
uint8_t  s_au8Buff1[BUFF_SIZE] ;       /* Working buffer */
uint8_t  s_au8Buff2[BUFF_SIZE] ;       /* Working buffer */
#else
static uint8_t  s_au8Buff1[BUFF_SIZE] __attribute__((aligned(4)));       /* Working buffer */
static uint8_t  s_au8Buff2[BUFF_SIZE] __attribute__((aligned(4)));       /* Working buffer */
#endif


static volatile int32_t  s_i32IntInCnt, s_i32IntOutCnt, s_i32IsoInCnt, s_i32IsoOutCnt;


static volatile uint32_t  s_u32TickCnt;
static uint32_t  s_u32T0;
void SysTick_Handler(void);
void enable_sys_tick(int ticks_per_second);
void timer_init(void);
uint32_t get_timer_value(void);
void  dump_buff_hex(uint8_t *pu8Buff, int i8Bytes);
int xatoi(          /* 0:Failed, 1:Successful */
    TCHAR **str,    /* Pointer to pointer to the string */
    long *res       /* Pointer to a variable to store the value */
);
void put_dump(
    const unsigned char* buff,  /* Pointer to the byte array to be dumped */
    unsigned long addr,         /* Heading address value */
    int cnt                     /* Number of bytes to be dumped */
);
void put_rc(FRESULT rc);
void get_line(char *buff, int len);
void int_xfer_read(uint8_t *pu8DataBuff, int *pu8DataLen);
void int_xfer_write(uint8_t *pu8DataBuff, int *pu8DataLen);
void iso_xfer_write(uint8_t *pu8DataBuff, int u8DataLen);
void iso_xfer_read(uint8_t *pu8DataBuff, int u8DataLen);
void SYS_Init(void);
void UART0_Init(void);


void SysTick_Handler(void)
{
    s_u32TickCnt++;
}

void enable_sys_tick(int ticks_per_second)
{
    s_u32TickCnt = 0;
    if(SysTick_Config(SystemCoreClock / (uint32_t)ticks_per_second))
    {
        /* Setup SysTick Timer for 1 second interrupts  */
        printf("Set system tick error!!\n");
        while(1);
    }
}

uint32_t get_ticks()
{
    return s_u32TickCnt;
}

void timer_init()
{
    s_u32T0 = get_ticks();
}

uint32_t get_timer_value()
{
    return (get_ticks() - s_u32T0);
}

/*
 *  This function is necessary for USB Host library.
 */
void delay_us(int usec)
{
    /*
     *  Configure Timer0, clock source from XTL_12M. Prescale 12
     */
    /* TIMER0 clock from HXT */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & (~CLK_CLKSEL1_TMR0SEL_Msk)) | CLK_CLKSEL1_TMR0SEL_HXT;
    CLK->APBCLK0 |= CLK_APBCLK0_TMR0CKEN_Msk;
    TIMER0->CTL = 0;        /* disable timer */
    TIMER0->INTSTS = 0x3;   /* write 1 to clear for safty */
    TIMER0->CMP = usec;
    TIMER0->CTL = (11 << TIMER_CTL_PSC_Pos) | TIMER_ONESHOT_MODE | TIMER_CTL_CNTEN_Msk;

    while(!TIMER0->INTSTS);
}

void  dump_buff_hex(uint8_t *pu8Buff, int i8Bytes)
{
    int     i8Idx, i8Cnt;

    i8Idx = 0;
    while(i8Bytes > 0)
    {
        printf("0x%04X  ", i8Idx);
        for(i8Cnt = 0; i8Cnt < 16; i8Cnt++)
            printf("%02x ", pu8Buff[i8Idx + i8Cnt]);
        printf("  ");
        for(i8Cnt = 0; i8Cnt < 16; i8Cnt++)
        {
            if((pu8Buff[i8Idx + i8Cnt] >= 0x20) && (pu8Buff[i8Idx + i8Cnt] < 127))
                printf("%c", pu8Buff[i8Idx + i8Cnt]);
            else
                printf(".");
            i8Bytes--;
        }
        i8Idx += 16;
        printf("\n");
    }
    printf("\n");
}


/*--------------------------------------------------------------------------*/
/* Monitor                                                                  */

/*----------------------------------------------*/
/* Get a value of the string                    */
/*----------------------------------------------*/
/*  "123 -5   0x3ff 0b1111 0377  w "
        ^                           1st call returns 123 and next ptr
           ^                        2nd call returns -5 and next ptr
                   ^                3rd call returns 1023 and next ptr
                          ^         4th call returns 15 and next ptr
                               ^    5th call returns 255 and next ptr
                                  ^ 6th call fails and returns 0
*/

int xatoi(          /* 0:Failed, 1:Successful */
    TCHAR **str,    /* Pointer to pointer to the string */
    long *res       /* Pointer to a variable to store the value */
)
{
    long val;
    unsigned char r, s = 0;
    TCHAR c;


    *res = 0;
    while((c = **str) == ' ')(*str)++;      /* Skip leading spaces */

    if(c == '-')        /* negative? */
    {
        s = 1;
        c = *(++(*str));
    }

    if(c == '0')
    {
        c = *(++(*str));
        switch(c)
        {
            case 'x':       /* hexadecimal */
                r = 16;
                c = *(++(*str));
                break;
            case 'b':       /* binary */
                r = 2;
                c = *(++(*str));
                break;
            default:
                if(c <= ' ') return 1;  /* single zero */
                if(c < '0' || c > '9') return 0;    /* invalid char */
                r = 8;      /* octal */
        }
    }
    else
    {
        if(c < '0' || c > '9') return 0;    /* EOL or invalid char */
        r = 10;         /* decimal */
    }

    val = 0;
    while(c > ' ')
    {
        if(c >= 'a') c -= 0x20;
        c -= '0';
        if(c >= 17)
        {
            c -= 7;
            if(c <= 9) return 0;    /* invalid char */
        }
        if(c >= r) return 0;        /* invalid char for current radix */
        val = val * r + c;
        c = *(++(*str));
    }
    if(s) val = 0 - val;            /* apply sign if needed */

    *res = val;
    return 1;
}


/*----------------------------------------------*/
/* Dump a block of byte array                   */

void put_dump(
    const unsigned char* buff,  /* Pointer to the byte array to be dumped */
    unsigned long addr,         /* Heading address value */
    int cnt                     /* Number of bytes to be dumped */
)
{
    int i;


    printf("%08x ", (UINT)addr);

    for(i = 0; i < cnt; i++)
        printf(" %02x", buff[i]);

    printf(" ");
    for(i = 0; i < cnt; i++)
        putchar((TCHAR)((buff[i] >= ' ' && buff[i] <= '~') ? buff[i] : '.'));

    printf("\n");
}


/*--------------------------------------------------------------------------*/
/* Monitor                                                                  */
/*--------------------------------------------------------------------------*/

static
FRESULT scan_files(
    char* path      /* Pointer to the path name working buffer */
)
{
    DIR dirs;
    FRESULT res;
    size_t i;
    char *fn;


    if((res = f_opendir(&dirs, path)) == FR_OK)
    {
        i = strlen(path);
        while(((res = f_readdir(&dirs, &s_Finfo)) == FR_OK) && s_Finfo.fname[0])
        {
            if(_FS_RPATH && s_Finfo.fname[0] == '.') continue;
#if _USE_LFN
            fn = *s_Finfo.lfname ? s_Finfo.lfname : s_Finfo.fname;
#else
            fn = s_Finfo.fname;
#endif
            if(s_Finfo.fattrib & AM_DIR)
            {
                s_u16AccDirs++;
                *(path + i) = '/';
                strcpy(path + i + 1, fn);
                res = scan_files(path);
                *(path + i) = '\0';
                if(res != FR_OK) break;
            }
            else
            {
                /*              printf("%s/%s\n", path, fn); */
                s_u16AccFiles++;
                s_u32AccSize += s_Finfo.fsize;
            }
        }
    }

    return res;
}



void put_rc(FRESULT rc)
{
    const TCHAR *p =
        _T("OK\0DISK_ERR\0INT_ERR\0NOT_READY\0NO_FILE\0NO_PATH\0INVALID_NAME\0")
        _T("DENIED\0EXIST\0INVALID_OBJECT\0WRITE_PROTECTED\0INVALID_DRIVE\0")
        _T("NOT_ENABLED\0NO_FILE_SYSTEM\0MKFS_ABORTED\0TIMEOUT\0LOCKED\0")
        _T("NOT_ENOUGH_CORE\0TOO_MANY_OPEN_FILES\0");
    //FRESULT i;
    uint32_t i;
    for(i = 0; (i != (UINT)rc) && *p; i++)
    {
        while(*p++) ;
    }
    printf(_T("rc=%d FR_%s\n"), (UINT)rc, p);
}

/*----------------------------------------------*/
/* Get a line from the input                    */
/*----------------------------------------------*/

void get_line(char *buff, int len)
{
    int c;
    int idx = 0;
//  DWORD dw;


    for(;;)
    {
        c = getchar();
        putchar(c);
        if(c == '\r') break;
        if((c == '\b') && idx) idx--;
        if((c >= ' ') && (idx < len - 1)) buff[idx++] = (char)c;
    }
    buff[idx] = 0;

    putchar('\n');

}

/*---------------------------------------------------------*/
/* User Provided RTC Function for FatFs module             */
/*---------------------------------------------------------*/
/* This is a real time clock service to be called from     */
/* FatFs module. Any valid time must be returned even if   */
/* the system does not support an RTC.                     */
/* This function is not required in read-only cfg.         */

unsigned long get_fattime(void)
{
    unsigned long tmr;

    tmr = 0x00000;

    return tmr;
}


static FIL file1, file2;        /* File objects */


/*
 *  Interrupt-In transfer data delivery callback function.
 */
void int_xfer_read(uint8_t *pu8DataBuff, int *pu8DataLen)
{
    //  Receive interrupt in transfer data. The data len is (*pu8DataLen).
    //  NOTICE: This callback function is in USB Host interrupt context!
    (void)pu8DataBuff;
    (void)pu8DataLen;
    s_i32IntInCnt++;
}


/*
 *  Interrupt-Out transfer data request callback function.
 */
void int_xfer_write(uint8_t *pu8DataBuff, int *pu8DataLen)
{
    //  LBK request the interrupt out transfer data.
    //  NOTICE: This callback function is in USB Host interrupt context!
    s_i32IntOutCnt++;
    *pu8DataLen = 8;
    memset(pu8DataBuff, s_i32IntOutCnt & 0xff, 8);
}

/*
 *  Isochronous-Out transfer data request callback function.
 */
void iso_xfer_write(uint8_t *pu8DataBuff, int u8DataLen)
{
    //  Application feeds Isochronous-Out data here.
    //  NOTICE: This callback function is in USB Host interrupt context!
    s_i32IsoOutCnt++;
    memset(pu8DataBuff, s_i32IsoOutCnt & 0xff, (size_t)u8DataLen);
}

/*
 *  Isochronous-In transfer data request callback function.
 */
void iso_xfer_read(uint8_t *pu8DataBuff, int u8DataLen)
{
    //  Application gets Isochronous-In data here.
    //  NOTICE: This callback function is in USB Host interrupt context!
    (void)pu8DataBuff;
    (void)u8DataLen;
    s_i32IsoInCnt++;
}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode */
    PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);

    /* Enable HXT clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Wait for HXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Switch HCLK clock source to HXT */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HXT, CLK_CLKDIV0_HCLK(1));

    /* Set PLL frequency */
    CLK->PLLCTL = CLK_PLLCTL_192MHz_HXT;

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_STATUS_PLLSTB_Msk);

    /* Switch HCLK clock source to PLL */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_PLL, CLK_CLKDIV0_HCLK(3));

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART module clock source as HXT and UART module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HXT, CLK_CLKDIV0_UART0(1));

    /* Enable USBH module clock */
    CLK_EnableModuleClock(USBH_MODULE);

    /* USB Host desired input clock is 48 MHz. Set as PLL divided by 4 (192/4 = 48) */
    CLK->CLKDIV0 = (CLK->CLKDIV0 & ~CLK_CLKDIV0_USBDIV_Msk) | CLK_CLKDIV0_USB(4);

    /* Enable USBD and OTG clock */
    CLK->APBCLK0 |= CLK_APBCLK0_USBDCKEN_Msk | CLK_APBCLK0_OTGCKEN_Msk;

    /* Set OTG as USB Host role */
    SYS->USBPHY = SYS_USBPHY_OTGPHYEN_Msk | SYS_USBPHY_SBO_Msk | (0x1 << SYS_USBPHY_USBROLE_Pos);

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & (~(UART0_RXD_PB12_Msk | UART0_TXD_PB13_Msk))) | UART0_RXD_PB12 | UART0_TXD_PB13;

    /* USB_VBUS_EN (USB 1.1 VBUS power enable pin) multi-function pin - PB.15     */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~SYS_GPB_MFPH_PB15MFP_Msk) | SYS_GPB_MFPH_PB15MFP_USB_VBUS_EN;

    /* USB_VBUS_ST (USB 1.1 over-current detect pin) multi-function pin - PB.14   */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~SYS_GPB_MFPH_PB14MFP_Msk) | SYS_GPB_MFPH_PB14MFP_USB_VBUS_ST;

    /* USB 1.1 port multi-function pin VBUS, D+, D-, and ID pins */
    SYS->GPA_MFPH &= ~(SYS_GPA_MFPH_PA12MFP_Msk | SYS_GPA_MFPH_PA13MFP_Msk |
                       SYS_GPA_MFPH_PA14MFP_Msk | SYS_GPA_MFPH_PA15MFP_Msk);
    SYS->GPA_MFPH |= SYS_GPA_MFPH_PA12MFP_USB_VBUS | SYS_GPA_MFPH_PA13MFP_USB_D_N |
                     SYS_GPA_MFPH_PA14MFP_USB_D_P | SYS_GPA_MFPH_PA15MFP_USB_OTG_ID;

    /* Lock protected registers */
    SYS_LockReg();
}

void UART0_Init(void)
{
    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}


int32_t main(void)
{
    char        *ptr, *ptr2;
    long        p1, p2, p3, sect = 0;
    BYTE        *buf;
    FATFS       *fs;              /* Pointer to file system object */
    TCHAR       usb_path[] = { '3', ':', 0 };    /* USB drive started from 3 */
    FRESULT     res;

    DIR dir;                /* Directory object */
    UINT s1, s2, cnt, sector_no;
    static const BYTE ft[] = {0, 12, 16, 32};
    DWORD ofs = 0;

    SYS_Init();                        /* Init System, IP clock and multi-function I/O */

    UART0_Init();                      /* Initialize UART0 */

    enable_sys_tick(100);

    printf("\n\n");
    printf("+--------------------------------------------+\n");
    printf("|                                            |\n");
    printf("|    USB Host Mass Storage sample program    |\n");
    printf("|                                            |\n");
    printf("+--------------------------------------------+\n");

    s_pu8Buff = (BYTE *)((uint32_t)&s_au8BuffPool[0]);
    s_pu8Buff2 = (BYTE *)((uint32_t)&s_au8BuffPool2[0]);

    usbh_core_init();
    usbh_umas_init();
    usbh_pooling_hubs();

    printf("Init time: %d\n", get_ticks());

    s_i32IntInCnt = s_i32IntOutCnt = s_i32IsoInCnt = s_i32IsoOutCnt = 0;

    f_chdrive(usb_path);               /* set default path                                */

    for(;;)
    {
        usbh_pooling_hubs();

        usbh_memory_used();            /* print out UsbHostLib memory usage information   */

        printf(_T(">"));
        ptr = s_achLine;

        get_line(ptr, sizeof(s_achLine));

        switch(*ptr++)
        {

            case 'q' :  /* Exit program */
                return 0;

            case '5':
                for(sector_no = 0; sector_no < 128000; sector_no++)
                {
                    if(sector_no % 1000 == 0)
                        printf("\nLBK transfer count: Ii:%d, Io: %d, Si:%d, So:%d\n", s_i32IntInCnt, s_i32IntOutCnt, s_i32IsoInCnt, s_i32IsoOutCnt);

                    printf("Test sector %d\r", sector_no);

                    memset(s_au8Buff1, 87, 512);
                    memset(s_au8Buff2, 0x87, 512);

                    res = (FRESULT)disk_read(3, s_au8Buff1, sector_no, 1);
                    if(res)
                    {
                        printf("read failed at %d, rc=%d\n", sector_no, (WORD)res);
                        put_rc(res);
                        break;
                    }

                    res = (FRESULT)disk_read(3, s_au8Buff2, sector_no, 1);
                    if(res)
                    {
                        printf("read failed at %d, rc=%d\n", sector_no, (WORD)res);
                        put_rc(res);
                        break;
                    }

                    if(memcmp(s_au8Buff1, s_au8Buff2, 512) != 0)
                    {
                        printf("\nData compare failed!!\n");
                        break;
                    }
                }
                break;

            case '4' :
            case '6' :
            case '7' :
                ptr--;
                *(ptr + 1) = ':';
                *(ptr + 2) = 0;
                put_rc(f_chdrive((TCHAR *)ptr));
                break;

            case 'd' :
                switch(*ptr++)
                {
                    case 'd' :  /* dd [<lba>] - Dump sector */
                        if(!xatoi(&ptr, &p2)) p2 = sect;
                        res = (FRESULT)disk_read(3, s_pu8Buff, (DWORD)p2, 1);
                        if(res)
                        {
                            printf("rc=%d\n", (WORD)res);
                            break;
                        }
                        sect = p2 + 1;
                        printf("Sector:%d\n", (INT)p2);
                        for(buf = (unsigned char*)s_pu8Buff, ofs = 0; ofs < 0x200; buf += 16, ofs += 16)
                            put_dump(buf, ofs, 16);
                        break;

                    case 't' :  /* dt - raw sector read/write performance test */
                        printf("Raw sector read performance test...\n");
                        timer_init();
                        for(s1 = 0; s1 < (0x800000 / BUFF_SIZE); s1++)
                        {
                            s2 = BUFF_SIZE / 512;   /* sector count for each read  */
                            res = (FRESULT)disk_read(3, s_pu8Buff, 10000 + s1 * s2, s2);
                            if(res)
                            {
                                printf("read failed at %d, rc=%d\n", 10000 + s1 * s2, (WORD)res);
                                put_rc(res);
                                break;
                            }
                            if(s1 % 256 == 0)
                                printf("%d KB\n", (s1 * BUFF_SIZE) / 1024);
                        }
                        p1 = (long)get_timer_value();
                        printf("time = %d.%02d\n", (INT)(p1 / 100), (INT)(p1 % 100));
                        printf("Raw read speed: %d KB/s\n", (INT)(((0x800000 * 100) / p1) / 1024));

                        printf("Raw sector write performance test...\n");
                        timer_init();
                        for(s1 = 0; s1 < (0x800000 / BUFF_SIZE); s1++)
                        {
                            s2 = BUFF_SIZE / 512;   /* sector count for each read  */
                            res = (FRESULT)disk_write(3, s_pu8Buff, 20000 + s1 * s2, s2);
                            if(res)
                            {
                                printf("write failed at %d, rc=%d\n", 20000 + s1 * s2, (WORD)res);
                                put_rc(res);
                                break;
                            }
                            if(s1 % 256 == 0)
                                printf("%d KB\n", (s1 * BUFF_SIZE) / 1024);
                        }
                        p1 = (long)get_timer_value();
                        printf("time = %d.%02d\n", (INT)(p1 / 100), (INT)(p1 % 100));
                        printf("Raw write speed: %d KB/s\n", (INT)(((0x800000 * 100) / p1) / 1024));
                        break;

                    case 'z' :  /* dz - file read/write performance test */
#if 0
                        printf("File write performance test...\n");
                        res = f_open(&file1, "3:\\tfile", FA_CREATE_ALWAYS | FA_WRITE);
                        if(res)
                        {
                            put_rc(res);
                            break;
                        }
                        timer_init();
                        for(s1 = 0; s1 < (0x800000 / BUFF_SIZE); s1++)
                        {
                            res = f_write(&file1, s_pu8Buff, BUFF_SIZE, &cnt);
                            if(res || (cnt != BUFF_SIZE))
                            {
                                put_rc(res);
                                break;
                            }
                            if(s1 % 128 == 0)
                                printf("%d KB\n", (s1 * BUFF_SIZE) / 1024);
                        }
                        p1 = get_timer_value();
                        f_close(&file1);
                        printf("time = %d.%02d\n", p1 / 100, p1 % 100);
                        printf("File write speed: %d KB/s\n", ((0x800000 * 100) / p1) / 1024);
#endif
                        printf("File read performance test...\n");
                        res = f_open(&file1, "3:\\tfile", FA_READ);
                        if(res)
                        {
                            put_rc(res);
                            break;
                        }
                        timer_init();
                        for(s1 = 0; s1 < (0x800000 / BUFF_SIZE); s1++)
                        {
                            res = f_read(&file1, s_pu8Buff, BUFF_SIZE, &cnt);
                            if(res || (cnt != BUFF_SIZE))
                            {
                                put_rc(res);
                                break;
                            }
                            if(s1 % 128 == 0)
                                printf("%d KB\n", (s1 * BUFF_SIZE) / 1024);
                        }
                        p1 = (long)get_timer_value();
                        f_close(&file1);
                        printf("time = %d.%02d\n", (INT)(p1 / 100), (INT)(p1 % 100));
                        printf("File read speed: %d KB/s\n", (INT)(((0x800000 * 100) / p1) / 1024));
                        break;
                }
                break;

            case 'b' :
                switch(*ptr++)
                {
                    case 'd' :  /* bd <addr> - Dump R/W buffer */
                        if(!xatoi(&ptr, &p1)) break;
                        for(ptr = (char*)&s_pu8Buff[p1], ofs = (DWORD)p1, cnt = 32; cnt; cnt--, ptr += 16, ofs += 16)
                            put_dump((BYTE*)ptr, ofs, 16);
                        break;

                    case 'e' :  /* be <addr> [<data>] ... - Edit R/W buffer */
                        if(!xatoi(&ptr, &p1)) break;
                        if(xatoi(&ptr, &p2))
                        {
                            do
                            {
                                s_pu8Buff[p1++] = (BYTE)p2;
                            }
                            while(xatoi(&ptr, &p2));
                            break;
                        }
                        for(;;)
                        {
                            printf("%04X %02X-", (WORD)p1, s_pu8Buff[p1]);
                            get_line(s_achLine, sizeof(s_achLine));
                            ptr = s_achLine;
                            if(*ptr == '.') break;
                            if(*ptr < ' ')
                            {
                                p1++;
                                continue;
                            }
                            if(xatoi(&ptr, &p2))
                                s_pu8Buff[p1++] = (BYTE)p2;
                            else
                                printf("???\n");
                        }
                        break;

                    case 'r' :  /* br <sector> [<n>] - Read disk into R/W buffer */
                        if(!xatoi(&ptr, &p2)) break;
                        if(!xatoi(&ptr, &p3)) p3 = 1;
                        printf("rc=%d\n", disk_read(3, s_pu8Buff, (DWORD)p2, (UINT)p3));
                        break;

                    case 'w' :  /* bw <sector> [<n>] - Write R/W buffer into disk */
                        if(!xatoi(&ptr, &p2)) break;
                        if(!xatoi(&ptr, &p3)) p3 = 1;
                        printf("rc=%d\n", disk_write(3, s_pu8Buff, (DWORD)p2, (UINT)p3));
                        break;

                    case 'f' :  /* bf <n> - Fill working buffer */
                        if(!xatoi(&ptr, &p1)) break;
                        memset(s_pu8Buff, (int)p1, BUFF_SIZE);
                        break;

                }
                break;



            case 'f' :
                switch(*ptr++)
                {

                    case 's' :  /* fs - Show logical drive status */
                        res = f_getfree("", (DWORD*)&p2, &fs);
                        if(res)
                        {
                            put_rc(res);
                            break;
                        }
                        printf("FAT type = FAT%d\nBytes/Cluster = %d\nNumber of FATs = %d\n"
                               "Root DIR entries = %d\nSectors/FAT = %d\nNumber of clusters = %d\n"
                               "FAT start (lba) = %d\nDIR start (lba,clustor) = %d\nData start (lba) = %d\n\n...",
                               ft[fs->fs_type & 3], (INT)(fs->csize * 512UL), fs->n_fats,
                               fs->n_rootdir, (INT)fs->fsize, (INT)(fs->n_fatent - 2),
                               (INT)fs->fatbase, (INT)fs->dirbase, (INT)fs->database
                              );
                        s_u32AccSize = s_u16AccFiles = s_u16AccDirs = 0;
#if _USE_LFN
                        s_Finfo.lfname = g_achLfname;
                        s_Finfo.lfsize = sizeof(g_achLfname);
#endif
                        res = scan_files(ptr);
                        if(res)
                        {
                            put_rc(res);
                            break;
                        }
                        printf("\r%d files, %d bytes.\n%d folders.\n"
                               "%d KB total disk space.\n%d KB available.\n",
                               s_u16AccFiles, (INT)s_u32AccSize, s_u16AccDirs,
                               (INT)((fs->n_fatent - 2) * (fs->csize / 2)), (INT)(p2 * (fs->csize / 2))
                              );
                        break;
                    case 'l' :  /* fl [<path>] - Directory listing */
                        while(*ptr == ' ') ptr++;
                        res = f_opendir(&dir, ptr);
                        if(res)
                        {
                            put_rc(res);
                            break;
                        }
                        p1 = s1 = s2 = 0;
                        for(;;)
                        {
                            res = f_readdir(&dir, &s_Finfo);
                            if((res != FR_OK) || !s_Finfo.fname[0]) break;
                            if(s_Finfo.fattrib & AM_DIR)
                            {
                                s2++;
                            }
                            else
                            {
                                s1++;
                                p1 += s_Finfo.fsize;
                            }
                            printf("%c%c%c%c%c %d/%02d/%02d %02d:%02d    %9d  %s",
                                   (s_Finfo.fattrib & AM_DIR) ? 'D' : '-',
                                   (s_Finfo.fattrib & AM_RDO) ? 'R' : '-',
                                   (s_Finfo.fattrib & AM_HID) ? 'H' : '-',
                                   (s_Finfo.fattrib & AM_SYS) ? 'S' : '-',
                                   (s_Finfo.fattrib & AM_ARC) ? 'A' : '-',
                                   (s_Finfo.fdate >> 9) + 1980, (s_Finfo.fdate >> 5) & 15, s_Finfo.fdate & 31,
                                   (s_Finfo.ftime >> 11), (s_Finfo.ftime >> 5) & 63, (INT)s_Finfo.fsize, s_Finfo.fname);
#if _USE_LFN
                            for(p2 = strlen(s_Finfo.fname); p2 < 14; p2++)
                                printf(" ");
                            printf("%s\n", g_achLfname);
#else
                            printf("\n");
#endif
                        }
                        printf("%4d File(s),%10d bytes total\n%4d Dir(s)", s1, (INT)p1, s2);
                        if(f_getfree(ptr, (DWORD*)&p1, &fs) == FR_OK)
                            printf(", %10d bytes free\n", (INT)(p1 * fs->csize * 512));
                        break;


                    case 'o' :  /* fo <mode> <file> - Open a file */
                        if(!xatoi(&ptr, &p1)) break;
                        while(*ptr == ' ') ptr++;
                        put_rc(f_open(&file1, ptr, (BYTE)p1));
                        break;

                    case 'c' :  /* fc - Close a file */
                        put_rc(f_close(&file1));
                        break;

                    case 'e' :  /* fe - Seek file pointer */
                        if(!xatoi(&ptr, &p1)) break;
                        res = f_lseek(&file1, (DWORD)p1);
                        put_rc(res);
                        if(res == FR_OK)
                            printf("fptr=%d(0x%lX)\n", (INT)file1.fptr, file1.fptr);
                        break;

                    case 'd' :  /* fd <len> - read and dump file from current fp */
                        if(!xatoi(&ptr, &p1)) break;
                        ofs = file1.fptr;
                        while(p1)
                        {
                            if((UINT)p1 >= 16)
                            {
                                cnt = 16;
                                p1 -= 16;
                            }
                            else
                            {
                                cnt = (UINT)p1;
                                p1 = 0;
                            }
                            res = f_read(&file1, s_pu8Buff, cnt, &cnt);
                            if(res != FR_OK)
                            {
                                put_rc(res);
                                break;
                            }
                            if(!cnt) break;
                            put_dump(s_pu8Buff, ofs, (int)cnt);
                            ofs += 16;
                        }
                        break;

                    case 'r' :  /* fr <len> - read file */
                        if(!xatoi(&ptr, &p1)) break;
                        p2 = 0;
                        timer_init();
                        while(p1)
                        {
                            if((UINT)p1 >= g_u8Len)
                            {
                                cnt = g_u8Len;
                                p1 -= g_u8Len;
                            }
                            else
                            {
                                cnt = (UINT)p1;
                                p1 = 0;
                            }
                            res = f_read(&file1, s_pu8Buff, cnt, &s2);
                            if(res != FR_OK)
                            {
                                put_rc(res);
                                break;
                            }
                            p2 += s2;
                            if(cnt != s2) break;
                        }
                        p1 = (long)get_timer_value();
                        if(p1)
                            printf("%d bytes read with %d kB/sec.\n", (INT)p2, (INT)(((p2 * 100) / p1) / 1024));
                        break;

                    case 'w' :  /* fw <len> <val> - write file */
                        if(!xatoi(&ptr, &p1) || !xatoi(&ptr, &p2)) break;
                        memset(s_pu8Buff, (BYTE)p2, g_u8Len);
                        p2 = 0;
                        timer_init();
                        while(p1)
                        {
                            if((UINT)p1 >= g_u8Len)
                            {
                                cnt = g_u8Len;
                                p1 -= g_u8Len;
                            }
                            else
                            {
                                cnt = (UINT)p1;
                                p1 = 0;
                            }
                            res = f_write(&file1, s_pu8Buff, cnt, &s2);
                            if(res != FR_OK)
                            {
                                put_rc(res);
                                break;
                            }
                            p2 += s2;
                            if(cnt != s2) break;
                        }
                        p1 = (long)get_timer_value();
                        if(p1)
                            printf("%d bytes written with %d kB/sec.\n", (INT)p2, (INT)(((p2 * 100) / p1) / 1024));
                        break;

                    case 'n' :  /* fn <old_name> <new_name> - Change file/dir name */
                        while(*ptr == ' ') ptr++;
                        ptr2 = strchr(ptr, ' ');
                        if(!ptr2) break;
                        *ptr2++ = 0;
                        while(*ptr2 == ' ') ptr2++;
                        put_rc(f_rename(ptr, ptr2));
                        break;

                    case 'u' :  /* fu <name> - Unlink a file or dir */
                        while(*ptr == ' ') ptr++;
                        put_rc(f_unlink(ptr));
                        break;

                    case 'v' :  /* fv - Truncate file */
                        put_rc(f_truncate(&file1));
                        break;

                    case 'k' :  /* fk <name> - Create a directory */
                        while(*ptr == ' ') ptr++;
                        put_rc(f_mkdir(ptr));
                        break;

                    case 'a' :  /* fa <atrr> <mask> <name> - Change file/dir attribute */
                        if(!xatoi(&ptr, &p1) || !xatoi(&ptr, &p2)) break;
                        while(*ptr == ' ') ptr++;
                        put_rc(f_chmod(ptr, (BYTE)p1, (BYTE)p2));
                        break;

                    case 't' :  /* ft <year> <month> <day> <hour> <min> <sec> <name> - Change timestamp */
                        if(!xatoi(&ptr, &p1) || !xatoi(&ptr, &p2) || !xatoi(&ptr, &p3)) break;
                        s_Finfo.fdate = (WORD)(((p1 - 1980) << 9) | ((p2 & 15) << 5) | (p3 & 31));
                        if(!xatoi(&ptr, &p1) || !xatoi(&ptr, &p2) || !xatoi(&ptr, &p3)) break;
                        s_Finfo.ftime = (WORD)(((p1 & 31) << 11) | ((p1 & 63) << 5) | ((p1 >> 1) & 31));
                        put_rc(f_utime(ptr, &s_Finfo));
                        break;

                    case 'x' : /* fx <src_name> <dst_name> - Copy file */
                        while(*ptr == ' ') ptr++;
                        ptr2 = strchr(ptr, ' ');
                        if(!ptr2) break;
                        *ptr2++ = 0;
                        while(*ptr2 == ' ') ptr2++;
                        printf("Opening \"%s\"", ptr);
                        res = f_open(&file1, ptr, FA_OPEN_EXISTING | FA_READ);
                        printf("\n");
                        if(res)
                        {
                            put_rc(res);
                            break;
                        }
                        printf("Creating \"%s\"", ptr2);
                        res = f_open(&file2, ptr2, FA_CREATE_ALWAYS | FA_WRITE);
                        putchar('\n');
                        if(res)
                        {
                            put_rc(res);
                            f_close(&file1);
                            break;
                        }
                        printf("Copying...");
                        p1 = 0;
                        for(;;)
                        {
                            res = f_read(&file1, s_pu8Buff, BUFF_SIZE, &s1);
                            if(res || s1 == 0) break;    /* error or eof */
                            res = f_write(&file2, s_pu8Buff, s1, &s2);
                            p1 += s2;
                            if(res || s2 < s1) break;    /* error or disk full */

                            if((p1 % 0x10000) == 0)
                                printf("\n%d KB copyed.", (INT)(p1 / 1024));
                            printf(".");
                        }
                        printf("\n%d bytes copied.\n", (INT)p1);
                        f_close(&file1);
                        f_close(&file2);
                        break;

                    case 'y' : /* fz <src_name> <dst_name> - Compare file */
                        while(*ptr == ' ') ptr++;
                        ptr2 = strchr(ptr, ' ');
                        if(!ptr2) break;
                        *ptr2++ = 0;
                        while(*ptr2 == ' ') ptr2++;
                        printf("Opening \"%s\"", ptr);
                        res = f_open(&file1, ptr, FA_OPEN_EXISTING | FA_READ);
                        printf("\n");
                        if(res)
                        {
                            put_rc(res);
                            break;
                        }
                        printf("Opening \"%s\"", ptr2);
                        res = f_open(&file2, ptr2, FA_OPEN_EXISTING | FA_READ);
                        putchar('\n');
                        if(res)
                        {
                            put_rc(res);
                            f_close(&file1);
                            break;
                        }
                        printf("Compare...");
                        p1 = 0;
                        for(;;)
                        {
                            res = f_read(&file1, s_pu8Buff, BUFF_SIZE, &s1);
                            if(res || s1 == 0)
                            {
                                printf("\nRead file %s terminated. (%d)\n", ptr, res);
                                break;     /* error or eof */
                            }

                            res = f_read(&file2, s_pu8Buff2, BUFF_SIZE, &s2);
                            if(res || s2 == 0)
                            {
                                printf("\nRead file %s terminated. (%d)\n", ptr2, res);
                                break;     /* error or eof */
                            }

                            p1 += s2;
                            if(res || s2 < s1) break;    /* error or disk full */

                            if(memcmp(s_pu8Buff, s_pu8Buff2, s1) != 0)
                            {
                                printf("Compre failed!! %d %d\n", s1, s2);
                                break;
                            }

                            if((p1 % 0x10000) == 0)
                                printf("\n%d KB compared.", (INT)(p1 / 1024));
                            printf(".");
                        }
                        if(s1 == 0)
                            printf("\nPASS. \n ");
                        f_close(&file1);
                        f_close(&file2);
                        break;

#if _FS_RPATH
                    case 'g' :  /* fg <path> - Change current directory */
                        while(*ptr == ' ') ptr++;
                        put_rc(f_chdir(ptr));
                        break;

                    case 'j' :  /* fj <drive#> - Change current drive */
                        while(*ptr == ' ') ptr++;
                        dump_buff_hex((uint8_t *)&p1, 16);
                        put_rc(f_chdrive((TCHAR *)ptr));
                        break;
#endif
#if _USE_MKFS
                    case 'm' :  /* fm <partition rule> <sect/clust> - Create file system */
                        if(!xatoi(&ptr, &p2) || !xatoi(&ptr, &p3)) break;
                        printf("The memory card will be formatted. Are you sure? (Y/n)=");
                        get_line(ptr, sizeof(s_achLine));
                        if(*ptr == 'Y')
                            put_rc(f_mkfs(0, (BYTE)p2, (WORD)p3));
                        break;
#endif
                    case 'z' :  /* fz [<rw size>] - Change R/W length for fr/fw/fx command */
                        if(xatoi(&ptr, &p1) && p1 >= 1 && (size_t)p1 <= BUFF_SIZE)
                            g_u8Len = (UINT)p1;
                        printf("blen=%d\n", g_u8Len);
                        break;
                }
                break;
            case '?':       /* Show usage */
                printf(
                    _T("n: - Change default drive (USB drive is 3~7)\n")
                    _T("dd [<lba>] - Dump sector\n")
                    //_T("ds <pd#> - Show disk status\n")
                    _T("\n")
                    _T("bd <ofs> - Dump working buffer\n")
                    _T("be <ofs> [<data>] ... - Edit working buffer\n")
                    _T("br <pd#> <sect> [<num>] - Read disk into working buffer\n")
                    _T("bw <pd#> <sect> [<num>] - Write working buffer into disk\n")
                    _T("bf <val> - Fill working buffer\n")
                    _T("\n")
                    _T("fs - Show volume status\n")
                    _T("fl [<path>] - Show a directory\n")
                    _T("fo <mode> <file> - Open a file\n")
                    _T("fc - Close the file\n")
                    _T("fe <ofs> - Move fp in normal seek\n")
                    //_T("fE <ofs> - Move fp in fast seek or Create link table\n")
                    _T("fd <len> - Read and dump the file\n")
                    _T("fr <len> - Read the file\n")
                    _T("fw <len> <val> - Write to the file\n")
                    _T("fn <object name> <new name> - Rename an object\n")
                    _T("fu <object name> - Unlink an object\n")
                    _T("fv - Truncate the file at current fp\n")
                    _T("fk <dir name> - Create a directory\n")
                    _T("fa <atrr> <mask> <object name> - Change object attribute\n")
                    _T("ft <year> <month> <day> <hour> <min> <sec> <object name> - Change timestamp of an object\n")
                    _T("fx <src file> <dst file> - Copy a file\n")
                    _T("fg <path> - Change current directory\n")
                    _T("fj <ld#> - Change current drive. For example: <fj 4:>\n")
                    _T("fm <ld#> <rule> <cluster size> - Create file system\n")
                    _T("\n")
                );
                break;
        }
    }
    //while(1);
}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/

