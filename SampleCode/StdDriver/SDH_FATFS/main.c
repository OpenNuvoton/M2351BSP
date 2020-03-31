/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    Access a SD card formatted in FAT file system
 *
 *
 * @copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <string.h>

#include "NuMicro.h"
#include "diskio.h"
#include "ff.h"

#define BUFF_SIZE       (8*1024)

static uint32_t u32Blen = BUFF_SIZE;
static DWORD s_u32AccSize;                         /* Work register for fs command */
static WORD s_u16AccFiles, s_u16AccDirs;
static FILINFO Finfo;

static char s_achLine[256];                         /* Console input buffer */

#ifdef __ICCARM__
#pragma data_alignment=4
uint8_t s_au8BuffPool[BUFF_SIZE] ;       /* Working buffer */
#else
static __attribute__((aligned)) uint8_t s_au8BuffPool[BUFF_SIZE] ;       /* Working buffer */
#endif
static uint8_t  *s_pu8Buff;
static uint32_t volatile s_u32Sec = 0;


void TMR0_IRQHandler(void);
void SDH0_IRQHandler(void);
void timer_init(void);
uint32_t get_timer_value(void);
void  dump_buff_hex(uint8_t *pucBuff, int8_t i8Bytes);
int xatoi(          /* 0:Failed, 1:Successful */
    TCHAR **str,    /* Pointer to pointer to the string */
    unsigned long *pi32Res       /* Pointer to a variable to store the value */
);
void put_dump(
    const uint8_t* pu8Buff,  /* Pointer to the byte array to be dumped */
    uint32_t u32Addr,         /* Heading address value */
    int32_t i32Cnt                     /* Number of bytes to be dumped */
);
void get_line(char *buff, int32_t i32Len);
void put_rc(FRESULT rc);
void SYS_Init(void);
		

void TMR0_IRQHandler(void)
{
    s_u32Sec++;

    /* clear timer interrupt flag */
    TIMER_ClearIntFlag(TIMER0);

}

void timer_init()
{
    /* Set timer frequency to 1HZ */
    TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, 1000);

    /* Enable timer interrupt */
    TIMER_EnableInt(TIMER0);
    NVIC_EnableIRQ(TMR0_IRQn);


    /* Start Timer 0 */
    TIMER_Start(TIMER0);
}

uint32_t get_timer_value()
{
    return s_u32Sec;
}

void  dump_buff_hex(uint8_t *pucBuff, int8_t i8Bytes)
{
    int8_t     i8Idx, i;

    i8Idx = 0;
    while(i8Bytes > 0)
    {
        printf("0x%04X  ", i8Idx);
        for(i = 0; i < 16; i++)
            printf("%02x ", pucBuff[i8Idx + i]);
        printf("  ");
        for(i = 0; i < 16; i++)
        {
            if((pucBuff[i8Idx + i] >= 0x20) && (pucBuff[i8Idx + i] < 127))
                printf("%c", pucBuff[i8Idx + i]);
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
    unsigned long *pi32Res       /* Pointer to a variable to store the value */
)
{
    uint32_t u32Val;
    uint8_t u8R, u8S = 0;
    TCHAR c;

    *pi32Res = 0;
    while((c = **str) == ' ')(*str)++;      /* Skip leading spaces */

    if(c == '-')        /* negative? */
    {
        u8S = 1;
        c = *(++(*str));
    }

    if(c == '0')
    {
        c = *(++(*str));
        switch(c)
        {
            case 'x':       /* hexadecimal */
                u8R = 16;
                c = *(++(*str));
                break;
            case 'b':       /* binary */
                u8R = 2;
                c = *(++(*str));
                break;
            default:
                if(c <= ' ') return 1;  /* single zero */
                if(c < '0' || c > '9') return 0;    /* invalid char */
                u8R = 8;      /* octal */
        }
    }
    else
    {
        if(c < '0' || c > '9') return 0;    /* EOL or invalid char */
        u8R = 10;         /* decimal */
    }

    u32Val = 0;
    while(c > ' ')
    {
        if(c >= 'a') c -= 0x20;
        c -= '0';
        if(c >= 17)
        {
            c -= 7;
            if(c <= 9) return 0;    /* invalid char */
        }
        if(c >= u8R) return 0;        /* invalid char for current radix */
        u32Val = u32Val * u8R + c;
        c = *(++(*str));
    }
    if(u8S) u32Val = 0 - u32Val;            /* apply sign if needed */

    *pi32Res = u32Val;
    return 1;
}


/*----------------------------------------------*/
/* Dump a block of byte array                   */

void put_dump(
    const uint8_t* pu8Buff,  /* Pointer to the byte array to be dumped */
    uint32_t u32Addr,         /* Heading address value */
    int32_t i32Cnt                     /* Number of bytes to be dumped */
)
{
    int i;

    printf("%08x ", u32Addr);

    for(i = 0; i < i32Cnt; i++)
        printf(" %02x", pu8Buff[i]);

    printf(" ");
    for(i = 0; i < i32Cnt; i++)
        putchar((TCHAR)((pu8Buff[i] >= ' ' && pu8Buff[i] <= '~') ? pu8Buff[i] : '.'));

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
    uint32_t i;
    char *fn;


    if((res = f_opendir(&dirs, path)) == FR_OK)
    {
        i = strlen(path);
        while(((res = f_readdir(&dirs, &Finfo)) == FR_OK) && Finfo.fname[0])
        {
            if(_FS_RPATH && Finfo.fname[0] == '.') continue;

            fn = Finfo.fname;

            if(Finfo.fattrib & AM_DIR)
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
                s_u32AccSize += Finfo.fsize;
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

    uint32_t i;
    for(i = 0; (i != (UINT)rc) && *p; i++)
    {
        while(*p++) ;
    }
    printf(_T("rc=%u FR_%s\n"), (UINT)rc, p);
}

/*----------------------------------------------*/
/* Get a line from the input                    */
/*----------------------------------------------*/

void get_line(char *buff, int32_t i32Len)
{
    int32_t i32C;
    int32_t i32Idx = 0;

    for(;;)
    {
        i32C = getchar();
        putchar(i32C);
        if(i32C == '\r') break;
        if((i32C == '\b') && i32Idx) i32Idx--;
        if((i32C >= ' ') && (i32Idx < i32Len - 1)) buff[i32Idx++] = (char)i32C;
    }
    buff[i32Idx] = 0;

    putchar('\n');

}


void SDH0_IRQHandler(void)
{
    uint32_t volatile u32Isr;

    /* FMI data abort interrupt */
    if(SDH0->GINTSTS & SDH_GINTSTS_DTAIF_Msk)
    {
        /* ResetAllEngine() */
        SDH0->GCTL |= SDH_GCTL_GCTLRST_Msk;
    }

    /* ----- SD interrupt status */
    u32Isr = SDH0->INTSTS;
    if(u32Isr & SDH_INTSTS_BLKDIF_Msk)
    {
        /* Block down */
        g_u8SDDataReadyFlag = TRUE;
        SDH0->INTSTS = SDH_INTSTS_BLKDIF_Msk;
    }

    if(u32Isr & SDH_INTSTS_CDIF_Msk)    // card detect
    {
        /* ----- SD interrupt status */
        /* it is work to delay 50 times for SD_CLK = 200KHz */
        {
            int volatile i;
            for(i = 0; i < 0x500; i++); /* delay to make sure got updated value from REG_SDISR. */
            u32Isr = SDH0->INTSTS;
        }

        if(u32Isr & SDH_INTSTS_CDSTS_Msk)
        {
            printf("\n***** card remove !\n");
            SD0.IsCardInsert = FALSE;   /* SDISR_CD_Card = 1 means card remove for GPIO mode */
            memset(&SD0, 0, sizeof(SDH_INFO_T));
        }
        else
        {
            printf("***** card insert !\n");
            SDH_Open(SDH0, CardDetect_From_GPIO);
            //SDH_Open(SDH0, CardDetect_From_DAT3);
            SDH_Probe(SDH0);
        }

        SDH0->INTSTS = SDH_INTSTS_CDIF_Msk;
    }

    /* CRC error interrupt */
    if(u32Isr & SDH_INTSTS_CRCIF_Msk)
    {
        if(!(u32Isr & SDH_INTSTS_CRC16_Msk))
        {
            //printf("***** ISR sdioIntHandler(): CRC_16 error !\n");
            // handle CRC error
        }
        else if(!(u32Isr & SDH_INTSTS_CRC7_Msk))
        {
            if(!g_u8R3Flag)
            {
                //printf("***** ISR sdioIntHandler(): CRC_7 error !\n");
                // handle CRC error
            }
        }
        /* Clear interrupt flag */
        SDH0->INTSTS = SDH_INTSTS_CRCIF_Msk;
    }

    if(u32Isr & SDH_INTSTS_DITOIF_Msk)
    {
        printf("***** ISR: data in timeout !\n");
        SDH0->INTSTS |= SDH_INTSTS_DITOIF_Msk;
    }

    /* Response in timeout interrupt */
    if(u32Isr & SDH_INTSTS_RTOIF_Msk)
    {
        printf("***** ISR: response in timeout !\n");
        SDH0->INTSTS |= SDH_INTSTS_RTOIF_Msk;
    }
}

void SYS_Init(void)
{
    /* Set PF multi-function pins for XT1_OUT(PF.2) and XT1_IN(PF.3) */
    SYS->GPF_MFPL = (SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF2MFP_Msk)) | XT1_OUT_PF2;
    SYS->GPF_MFPL = (SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF3MFP_Msk)) | XT1_IN_PF3;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable External XTAL (4~24 MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Waiting for 12MHz clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Switch HCLK clock source to HXT */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HXT, CLK_CLKDIV0_HCLK(1));

    /* Set PLL to power down mode and PLL_STB bit in CLKSTATUS register will be cleared by hardware.*/
    CLK->PLLCTL |= CLK_PLLCTL_PD_Msk;

    /* Set PLL frequency */
    CLK->PLLCTL = CLK_PLLCTL_128MHz_HIRC;

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_STATUS_PLLSTB_Msk);

    /* Switch HCLK clock source to PLL and set HCLK divider to 2 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_PLL, CLK_CLKDIV0_HCLK(2));

    CLK->CLKSEL0 = (CLK->CLKSEL0 & ~CLK_CLKSEL0_SDH0SEL_Msk) | CLK_CLKSEL0_SDH0SEL_HCLK;

    /* select multi-function pin */
    /* CD: PB12(9), PD13(3) */
    //SYS->GPB_MFPH = (SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB12MFP_Msk)) | SD0_nCD_PB12;
    SYS->GPD_MFPH = (SYS->GPD_MFPH & (~SYS_GPD_MFPH_PD13MFP_Msk)) | SD0_nCD_PD13;

    /* CLK: PB1(3), PE6(3) */
    //SYS->GPB_MFPL = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB1MFP_Msk)) | SD0_CLK_PB1;
    SYS->GPE_MFPL = (SYS->GPE_MFPL & (~SYS_GPE_MFPL_PE6MFP_Msk)) | SD0_CLK_PE6;

    /* CMD: PB0(3), PE7(3) */
    //SYS->GPB_MFPL = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB0MFP_Msk)) | SD0_CMD_PB0;
    SYS->GPE_MFPL = (SYS->GPE_MFPL & (~SYS_GPE_MFPL_PE7MFP_Msk)) | SD0_CMD_PE7;

    /* D0: PB2(3), PE2(3) */
    //SYS->GPB_MFPL = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB2MFP_Msk)) | SD0_DAT0_PB2;
    SYS->GPE_MFPL = (SYS->GPE_MFPL & (~SYS_GPE_MFPL_PE2MFP_Msk)) | SD0_DAT0_PE2;

    /* D1: PB3(3), PE3(3) */
    //SYS->GPB_MFPL = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB3MFP_Msk)) | SD0_DAT1_PB3;
    SYS->GPE_MFPL = (SYS->GPE_MFPL & (~SYS_GPE_MFPL_PE3MFP_Msk)) | SD0_DAT1_PE3;

    /* D2: PB4(3), PE4(3) */
    //SYS->GPB_MFPL = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB4MFP_Msk)) | SD0_DAT2_PB4;
    SYS->GPE_MFPL = (SYS->GPE_MFPL & (~SYS_GPE_MFPL_PE4MFP_Msk)) | SD0_DAT2_PE4;

    /* D3: PB5(3)-, PE5(3) */
    //SYS->GPB_MFPL = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB5MFP_Msk)) | SD0_DAT3_PB5;
    SYS->GPE_MFPL = (SYS->GPE_MFPL & (~SYS_GPE_MFPL_PE5MFP_Msk)) | SD0_DAT3_PE5;

    /* Enable IP clock */
    CLK->AHBCLK |= CLK_AHBCLK_SDH0CKEN_Msk; // SD Card driving clock.

    /* Enable IP clock */
    CLK->APBCLK0 |= CLK_APBCLK0_UART0CKEN_Msk; // UART0 Clock Enable

    /* Select IP clock source */
    CLK->CLKSEL1 &= ~CLK_CLKSEL1_UART0SEL_Msk;  /* clock source is HXT */

    CLK_EnableModuleClock(TMR0_MODULE);

    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HXT, 0);

    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /* Set multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & (~(UART0_RXD_PB12_Msk | UART0_TXD_PB13_Msk))) | UART0_RXD_PB12 | UART0_TXD_PB13;

    /* Lock protected registers */
    SYS_LockReg();

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
    unsigned long u32Tmr;

    u32Tmr = 0x00000;

    return u32Tmr;
}


static FIL file1, file2;        /* File objects */

/*----------------------------------------------------------------------------
  MAIN function
 *----------------------------------------------------------------------------*/
int32_t main(void)
{
    char        *ptr, *ptr2;
    unsigned long  i32P1, i32P2, i32P3;
    uint8_t        *pu8Buf;
    FATFS       *fs;              /* Pointer to file system object */
    BYTE        SD_Drv = 0;
    TCHAR       sd_path[] = { '0', ':', 0 };    /* SD drive started from 0 */
    FRESULT     res;

    DIR dir;                /* Directory object */
    UINT u32S1, u32S2, u32Cnt;
    static const uint8_t au8Ft[] = {0, 12, 16, 32};
    DWORD ofs = 0, sect = 0;

    s_pu8Buff = (BYTE *)s_au8BuffPool;

    SYS_Init();
    UART_Open(UART0, 115200);
    timer_init();

    printf("\n");
    printf("====================================\n");
    printf("          SDH Testing               \n");
    printf("====================================\n");

    printf("\n\nM2351 SDH FATFS TEST!\n");
    /* Enable NVIC SDH0 IRQ */
    NVIC_EnableIRQ(SDH0_IRQn);
    /*
        SD initial state needs 400KHz clock output, driver will use HIRC for SD initial clock source.
        And then switch back to the user's setting.
    */
    SDH_Open_Disk(SDH0, CardDetect_From_GPIO);
    //SDH_Open_Disk(SDH0, CardDetect_From_DAT3);
    f_chdrive(sd_path);          /* set default path */

    for(;;)
    {
        if(!(SDH_CardDetection(SDH0)))
            continue;

        printf(_T(">"));
        ptr = s_achLine;
        get_line(ptr, sizeof(s_achLine));
        switch(*ptr++)
        {
            case 'q' :  /* Exit program */
                return 0;

            case 'd' :
                switch(*ptr++)
                {
                    case 'd' :  /* dd [<lba>] - Dump sector */
                        if(!xatoi(&ptr, &i32P2)) i32P2 = sect;
                        res = (FRESULT)disk_read(SD_Drv, s_pu8Buff, i32P2, 1);
                        if(res)
                        {
                            printf("rc=%d\n", (WORD)res);
                            break;
                        }
                        sect = i32P2 + 1;
                        printf("Sector:%lu\n", i32P2);
                        for(pu8Buf = (unsigned char*)s_pu8Buff, ofs = 0; ofs < 0x200; pu8Buf += 16, ofs += 16)
                            put_dump(pu8Buf, ofs, 16);
                        break;

                }
                break;

            case 'b' :
                switch(*ptr++)
                {
                    case 'd' :  /* bd <addr> - Dump R/W buffer */
                        if(!xatoi(&ptr, &i32P1)) break;
                        for(ptr = (char*)&s_pu8Buff[i32P1], ofs = i32P1, u32Cnt = 32; u32Cnt; u32Cnt--, ptr += 16, ofs += 16)
                            put_dump((BYTE*)ptr, ofs, 16);
                        break;

                    case 'e' :  /* be <addr> [<data>] ... - Edit R/W buffer */
                        if(!xatoi(&ptr, &i32P1)) break;
                        if(xatoi(&ptr, &i32P2))
                        {
                            do
                            {
                                s_pu8Buff[i32P1++] = (BYTE)i32P2;
                            }
                            while(xatoi(&ptr, &i32P2));
                            break;
                        }
                        for(;;)
                        {
                            printf("%04X %02X-", (WORD)i32P1, s_pu8Buff[i32P1]);
                            get_line(s_achLine, sizeof(s_achLine));
                            ptr = s_achLine;
                            if(*ptr == '.') break;
                            if(*ptr < ' ')
                            {
                                i32P1++;
                                continue;
                            }
                            if(xatoi(&ptr, &i32P2))
                                s_pu8Buff[i32P1++] = (BYTE)i32P2;
                            else
                                printf("???\n");
                        }
                        break;

                    case 'r' :  /* br <sector> [<n>] - Read disk into R/W buffer */
                        if(!xatoi(&ptr, &i32P2)) break;
                        if(!xatoi(&ptr, &i32P3)) i32P3 = 1;
                        printf("rc=%d\n", disk_read(SD_Drv, s_pu8Buff, i32P2, i32P3));
                        break;

                    case 'w' :  /* bw <sector> [<n>] - Write R/W buffer into disk */
                        if(!xatoi(&ptr, &i32P2)) break;
                        if(!xatoi(&ptr, &i32P3)) i32P3 = 1;
                        printf("rc=%d\n", disk_write(SD_Drv, s_pu8Buff, i32P2, i32P3));
                        break;

                    case 'f' :  /* bf <n> - Fill working buffer */
                        if(!xatoi(&ptr, &i32P1)) break;
                        memset(s_pu8Buff, (int)i32P1, BUFF_SIZE);
                        break;

                }
                break;

            case 'f' :
                switch(*ptr++)
                {

                    case 's' :  /* fs - Show logical drive status */
                        res = f_getfree("", (DWORD*)&i32P2, &fs);
                        if(res)
                        {
                            put_rc(res);
                            break;
                        }
                        printf("FAT type = FAT%u\nBytes/Cluster = %lu\nNumber of FATs = %u\n"
                               "Root DIR entries = %u\nSectors/FAT = %lu\nNumber of clusters = %lu\n"
                               "FAT start (lba) = %lu\nDIR start (lba,clustor) = %lu\nData start (lba) = %lu\n\n...",
                               au8Ft[fs->fs_type & 3], fs->csize * 512UL, fs->n_fats,
                               fs->n_rootdir, fs->fsize, fs->n_fatent - 2,
                               fs->fatbase, fs->dirbase, fs->database
                              );
                        s_u32AccSize = s_u16AccFiles = s_u16AccDirs = 0;

                        res = scan_files(ptr);
                        if(res)
                        {
                            put_rc(res);
                            break;
                        }
                        printf("\r%u files, %lu bytes.\n%u folders.\n"
                               "%lu KB total disk space.\n%lu KB available.\n",
                               s_u16AccFiles, s_u32AccSize, s_u16AccDirs,
                               (fs->n_fatent - 2) * (fs->csize / 2), i32P2 * (fs->csize / 2)
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
                        i32P1 = u32S1 = u32S2 = 0;
                        for(;;)
                        {
                            res = f_readdir(&dir, &Finfo);
                            if((res != FR_OK) || !Finfo.fname[0]) break;
                            if(Finfo.fattrib & AM_DIR)
                            {
                                u32S2++;
                            }
                            else
                            {
                                u32S1++;
                                i32P1 += Finfo.fsize;
                            }
                            printf("%c%c%c%c%c %d/%02d/%02d %02d:%02d    %9lu  %s",
                                   (Finfo.fattrib & AM_DIR) ? 'D' : '-',
                                   (Finfo.fattrib & AM_RDO) ? 'R' : '-',
                                   (Finfo.fattrib & AM_HID) ? 'H' : '-',
                                   (Finfo.fattrib & AM_SYS) ? 'S' : '-',
                                   (Finfo.fattrib & AM_ARC) ? 'A' : '-',
                                   (Finfo.fdate >> 9) + 1980, (Finfo.fdate >> 5) & 15, Finfo.fdate & 31,
                                   (Finfo.ftime >> 11), (Finfo.ftime >> 5) & 63, Finfo.fsize, Finfo.fname);

                            printf("\n");
                        }
                        printf("%4u File(s),%10lu bytes total\n%4u Dir(s)", u32S1, i32P1, u32S2);
                        if(f_getfree(ptr, (DWORD*)&i32P1, &fs) == FR_OK)
                            printf(", %10lu bytes free\n", i32P1 * fs->csize * 512);
                        break;


                    case 'o' :  /* fo <mode> <file> - Open a file */
                        if(!xatoi(&ptr, &i32P1)) break;
                        while(*ptr == ' ') ptr++;
                        put_rc(f_open(&file1, ptr, (BYTE)i32P1));
                        break;

                    case 'c' :  /* fc - Close a file */
                        put_rc(f_close(&file1));
                        break;

                    case 'e' :  /* fe - Seek file pointer */
                        if(!xatoi(&ptr, &i32P1)) break;
                        res = f_lseek(&file1, i32P1);
                        put_rc(res);
                        if(res == FR_OK)
                            printf("fptr=%lu(0x%lX)\n", file1.fptr, file1.fptr);
                        break;

                    case 'd' :  /* fd <len> - read and dump file from current fp */
                        if(!xatoi(&ptr, &i32P1)) break;
                        ofs = file1.fptr;
                        while(i32P1)
                        {
                            if((UINT)i32P1 >= 16)
                            {
                                u32Cnt = 16;
                                i32P1 -= 16;
                            }
                            else
                            {
                                u32Cnt = i32P1;
                                i32P1 = 0;
                            }
                            res = f_read(&file1, s_pu8Buff, u32Cnt, &u32Cnt);
                            if(res != FR_OK)
                            {
                                put_rc(res);
                                break;
                            }
                            if(!u32Cnt) break;
                            put_dump(s_pu8Buff, ofs, (int32_t)u32Cnt);
                            ofs += 16;
                        }
                        break;

                    case 'r' :  /* fr <len> - read file */
                        if(!xatoi(&ptr, &i32P1)) break;
                        i32P2 = 0;
                        while(i32P1)
                        {
                            if((UINT)i32P1 >= u32Blen)
                            {
                                u32Cnt = u32Blen;
                                i32P1 -= u32Blen;
                            }
                            else
                            {
                                u32Cnt = i32P1;
                                i32P1 = 0;
                            }
                            res = f_read(&file1, s_pu8Buff, u32Cnt, &u32S2);
                            if(res != FR_OK)
                            {
                                put_rc(res);
                                break;
                            }
                            i32P2 += u32S2;
                            if(u32Cnt != u32S2) break;
                        }

                        if(i32P1)
                            printf("%lu bytes read with %lu kB/sec.\n", i32P2, ((i32P2 * 100) / i32P1) / 1024);
                        break;

                    case 'w' :  /* fw <len> <val> - write file */
                        if(!xatoi(&ptr, &i32P1) || !xatoi(&ptr, &i32P2)) break;
                        memset(s_pu8Buff, (BYTE)i32P2, u32Blen);
                        i32P2 = 0;
                        while(i32P1)
                        {
                            if((UINT)i32P1 >= u32Blen)
                            {
                                u32Cnt = u32Blen;
                                i32P1 -= u32Blen;
                            }
                            else
                            {
                                u32Cnt = i32P1;
                                i32P1 = 0;
                            }
                            res = f_write(&file1, s_pu8Buff, u32Cnt, &u32S2);
                            if(res != FR_OK)
                            {
                                put_rc(res);
                                break;
                            }
                            i32P2 += u32S2;
                            if(u32Cnt != u32S2) break;
                        }
                        if(i32P1)
                            printf("%lu bytes written with %lu kB/sec.\n", i32P2, ((i32P2 * 100) / i32P1) / 1024);
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
                        if(!xatoi(&ptr, &i32P1) || !xatoi(&ptr, &i32P2)) break;
                        while(*ptr == ' ') ptr++;
                        put_rc(f_chmod(ptr, (BYTE)i32P1, (BYTE)i32P2));
                        break;

                    case 't' :  /* ft <year> <month> <day> <hour> <min> <sec> <name> - Change timestamp */
                        if(!xatoi(&ptr, &i32P1) || !xatoi(&ptr, &i32P2) || !xatoi(&ptr, &i32P3)) break;
                        Finfo.fdate = (WORD)(((i32P1 - 1980) << 9) | ((i32P2 & 15) << 5) | (i32P3 & 31));
                        if(!xatoi(&ptr, &i32P1) || !xatoi(&ptr, &i32P2) || !xatoi(&ptr, &i32P3)) break;
                        Finfo.ftime = (WORD)(((i32P1 & 31) << 11) | ((i32P1 & 63) << 5) | ((i32P1 >> 1) & 31));
                        put_rc(f_utime(ptr, &Finfo));
                        break;

                    case 'x' : /* fx <src_name> <dst_name> - Copy file */
                    {
                        uint32_t volatile u32Btime;

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
                        i32P1 = 0;
                        u32Btime = get_timer_value();
                        for(;;)
                        {
                            res = f_read(&file1, s_pu8Buff, BUFF_SIZE, &u32S1);
                            if(res || u32S1 == 0) break;    /* error or eof */
                            res = f_write(&file2, s_pu8Buff, u32S1, &u32S2);
                            i32P1 += u32S2;
                            if(res || u32S2 < u32S1) break;    /* error or disk full */
                        }
                        printf("\n%lu bytes copied. %d\n", i32P1, (get_timer_value() - u32Btime));
                        f_close(&file1);
                        f_close(&file2);
                    }
                    break;
#if _FS_RPATH
                    case 'g' :  /* fg <path> - Change current directory */
                        while(*ptr == ' ') ptr++;
                        put_rc(f_chdir(ptr));
                        break;

                    case 'j' :  /* fj <drive#> - Change current drive */
                        while(*ptr == ' ') ptr++;
                        dump_buff_hex((uint8_t *)&i32P1, 16);
                        put_rc(f_chdrive((TCHAR *)ptr));
                        break;
#endif
#if _USE_MKFS
                    case 'm' :  /* fm <partition rule> <sect/clust> - Create file system */
                        if(!xatoi(&ptr, &i32P2) || !xatoi(&ptr, &i32P3)) break;
                        printf("The memory card will be formatted. Are you sure? (Y/n)=");
                        get_line(ptr, sizeof(s_achLine));
                        if(*ptr == 'Y')
                            put_rc(f_mkfs(0, (BYTE)i32P2, (WORD)i32P3));
                        break;
#endif
                    case 'z' :  /* fz [<rw size>] - Change R/W length for fr/fw/fx command */
                        if(xatoi(&ptr, &i32P1) && i32P1 >= 1 && (size_t)i32P1 <= BUFF_SIZE)
                            u32Blen = i32P1;
                        printf("blen=%d\n", u32Blen);
                        break;
                }
                break;
            case '?':       /* Show usage */
                printf(
                    _T("n: - Change default drive (SD drive is 0~1)\n")
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

}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
