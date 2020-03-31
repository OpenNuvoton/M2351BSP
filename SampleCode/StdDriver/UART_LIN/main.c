/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief
 *           Transmit LIN frame including header and response in UART LIN mode.
 *
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/

#include <stdio.h>
#include "NuMicro.h"



/* CheckSum Method */
#define MODE_CLASSIC    2
#define MODE_ENHANCED   1

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
static volatile int32_t s_i32Pointer;
static uint8_t s_au8SendData[12] ;

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
extern char GetChar(void);
void LIN_FunctionTest(void);
void LIN_FunctionTestUsingLinCtlReg(void);
void LIN_MasterTest(uint32_t u32id, uint32_t u32ModeSel);
void LIN_MasterTestUsingLinCtlReg(uint32_t u32id, uint32_t u32ModeSel);
void LIN_SendHeader(uint32_t u32id);
void LIN_SendHeaderUsingLinCtlReg(uint32_t u32id, uint32_t u32HeaderSel);
void LIN_SendResponse(int32_t checkSumOption, uint32_t *pu32TxBuf);
void LIN_SendResponseWithByteCnt(int32_t checkSumOption, uint32_t *pu32TxBuf, uint32_t u32ByteCnt);
uint32_t GetCheckSumValue(uint8_t *pu8Buf, uint32_t u32ModeSel);
uint8_t ComputeChksumValue(uint8_t *pu8Buf, uint32_t u32ByteCnt);
void TestItem(void);
void LIN_TestItem(void);
uint8_t GetParityValue(uint32_t u32id);
void SYS_Init(void);
void UART0_Init(void);
void UART1_Init(void);

/*---------------------------------------------------------------------------------------------------------*/
/*  Sample Code Menu                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void TestItem(void)
{
    printf("+-----------------------------------------------------------+\n");
    printf("|                LIN Sample Program                         |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("| LIN Master function test                            - [1] |\n");
    printf("| LIN Master function test using UART_LINCTL register - [2] |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("| Quit                                              - [ESC] |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("Please Select key (1~2): ");
}

void LIN_TestItem()
{
    printf("\n\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|     LIN Master Function Test                              |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("| [1] Master send header with ID = 0x30                     |\n");
    printf("| [2] Master send header and response with classic checksum |\n");
    printf("| [3] Master send header and response with enhanced checksum|\n");
    printf("|                                                           |\n");
    printf("| To measure UART1_TXD(PB.7) to check waveform ...          |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("| Quit                                             -  [ESC] |\n");
    printf("+-----------------------------------------------------------+\n\n");
}

/*---------------------------------------------------------------------------------------------------------*/
/*  LIN Function Test                                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
void LIN_FunctionTest()
{
    int32_t i32Item;

    /* Set UART Configuration, LIN Max Speed is 20K */
    UART_SetLineConfig(UART1, 9600, UART_WORD_LEN_8, UART_PARITY_NONE, UART_STOP_BIT_1);

    /* === CASE 1====
        The sample code will send a LIN header with a 12-bit break field,
        0x55 sync field and ID field is 0x30. Measurement the UART1 Tx pin to check it.
    */

    /* === CASE 2====
        The sample code will send a LIN header with ID is 0x35 and response field.
        The response field with 8 data bytes and checksum without including ID.
        Measurement the UART1 Tx pin to check it.
    */

    /* === CASE 3====
        The sample code will send a LIN header with ID is 0x12 and response field.
        The response field with 8 data bytes and checksum with including ID.
        Measurement the UART1 Tx pin to check it.
    */

    do
    {
        LIN_TestItem();
        i32Item = getchar();
        if(i32Item == 27) break;
        printf("%c\n", i32Item);
        switch(i32Item)
        {
            case '1':
                LIN_SendHeader(0x30);
                break;
            case '2':
                LIN_MasterTest(0x35, MODE_CLASSIC);
                break;
            case '3':
                LIN_MasterTest(0x12, MODE_ENHANCED);
                break;
            default:
                break;

        }
    }
    while(1);

    UART_Close(UART1);
    
    printf("\nLIN Sample Code End.\n\n");      
    
}

/*---------------------------------------------------------------------------------------------------------*/
/*  LIN Function Test                                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
void LIN_FunctionTestUsingLinCtlReg(void)
{
    int32_t i32Item;

    /* Set UART Configuration, LIN Max Speed is 20K */
    UART_SetLineConfig(UART1, 9600, UART_WORD_LEN_8, UART_PARITY_NONE, UART_STOP_BIT_1);

    /* === CASE 1====
        The sample code will send a LIN header with a 12-bit break field,
        0x55 sync field and ID field is 0x30. Measurement the UART1 Tx pin to check it.
    */

    /* === CASE 2====
        The sample code will send a LIN header with ID is 0x35 and response field.
        The response field with 8 data bytes and checksum without including ID.
        Measurement the UART1 Tx pin to check it.
    */

    /* === CASE 3====
        The sample code will send a LIN header with ID is 0x12 and response field.
        The response field with 8 data bytes and checksum with including ID.
        Measurement the UART1 Tx pin to check it.
    */

    do
    {
        LIN_TestItem();
        i32Item = getchar();
        if(i32Item == 27) break;        
        printf("%c\n", i32Item);
        switch(i32Item)
        {
            case '1':
                LIN_SendHeaderUsingLinCtlReg(0x30, UART_LINCTL_HSEL_BREAK_SYNC_ID);
                break;
            case '2':
                LIN_MasterTestUsingLinCtlReg(0x35, MODE_CLASSIC);
                break;
            case '3':
                LIN_MasterTestUsingLinCtlReg(0x12, MODE_ENHANCED);
                break;
            default:                          
                break;
        }
    }
    while(1);
    
    /* Clear header select setting */
    UART1->LINCTL &= ~UART_LINCTL_HSEL_Msk;    
    
    UART_Close(UART1);
    
    printf("\nLIN Sample Code End.\n\n");    

}

/*---------------------------------------------------------------------------------------------------------*/
/*  Master send header and response                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
void LIN_MasterTest(uint32_t u32id, uint32_t u32ModeSel)
{
    uint32_t au32TestPattern[8] = {0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x8};

    /* Send ID=0x35 Header and Response TestPatten */
    LIN_SendHeader(u32id);
    LIN_SendResponse((int32_t)u32ModeSel, &au32TestPattern[0]);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Master send header and response                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
void LIN_MasterTestUsingLinCtlReg(uint32_t u32id, uint32_t u32ModeSel)
{
    uint8_t au8TestPattern[9] = {0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x8, 0x0}; // 8 data byte + 1 byte checksum
    uint32_t u32Idx;

    if(u32ModeSel == MODE_CLASSIC)
    {
        /* Send break+sync+ID */
        LIN_SendHeaderUsingLinCtlReg(u32id, UART_LINCTL_HSEL_BREAK_SYNC_ID);
        /* Compute checksum without ID and fill checksum value to  au8TestPattern[8] */
        au8TestPattern[8] = ComputeChksumValue(&au8TestPattern[0], 8);
        UART_Write(UART1, &au8TestPattern[0], 9);
    }
    else if(u32ModeSel == MODE_ENHANCED)
    {
        /* Send break+sync+ID and fill ID value to s_au8SendData[0]*/
        LIN_SendHeaderUsingLinCtlReg(u32id, UART_LINCTL_HSEL_BREAK_SYNC);
        /* Fill test pattern to s_au8SendData[1]~ s_au8SendData[8] */
        for(u32Idx = 0; u32Idx < 8; u32Idx++)
            s_au8SendData[s_i32Pointer++] = au8TestPattern[u32Idx];
        /* Compute checksum value with ID and fill checksum value to s_au8SendData[9] */
        s_au8SendData[s_i32Pointer++] = ComputeChksumValue(&s_au8SendData[0], 9) ;
        UART_Write(UART1, &s_au8SendData[1], 9);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/* Compute Parity Value                                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t GetParityValue(uint32_t u32id)
{
    uint32_t u32Res = 0, u32Mask = 0;
    uint32_t au32ID[6], au32ParityBit[2];    

    for(u32Mask = 0; u32Mask < 6; u32Mask++)
        au32ID[u32Mask] = (u32id & (1 << u32Mask)) >> u32Mask;

    au32ParityBit[0] = (au32ID[0] + au32ID[1] + au32ID[2] + au32ID[4]) % 2;
    au32ParityBit[1] = (!((au32ID[1] + au32ID[3] + au32ID[4] + au32ID[5]) % 2));

    u32Res = u32id + (au32ParityBit[0] << 6) + (au32ParityBit[1] << 7);
    return (uint8_t)u32Res;
}

/*---------------------------------------------------------------------------------------------------------*/
/* Compute CheckSum Value , MODE_CLASSIC:(Not Include ID)    MODE_ENHANCED:(Include ID)                    */
/*---------------------------------------------------------------------------------------------------------*/
uint32_t GetCheckSumValue(uint8_t *pu8Buf, uint32_t u32ModeSel)
{
    uint32_t u32Idx, u32CheckSum = 0;

    for(u32Idx = u32ModeSel; u32Idx <= 9; u32Idx++)
    {
        u32CheckSum += pu8Buf[u32Idx];
        if(u32CheckSum >= 256)
            u32CheckSum -= 255;
    }
    return (255 - u32CheckSum);
}

/*---------------------------------------------------------------------------------------------------------*/
/* Compute CheckSum Value                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t ComputeChksumValue(uint8_t *pu8Buf, uint32_t u32ByteCnt)
{
    uint32_t u32Idx, u32CheckSum = 0;

    for(u32Idx = 0 ; u32Idx < u32ByteCnt; u32Idx++)
    {
        u32CheckSum += pu8Buf[u32Idx];
        if(u32CheckSum >= 256)
            u32CheckSum -= 255;
    }
    return (uint8_t)(255 - u32CheckSum);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Send LIN Header Field                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void LIN_SendHeader(uint32_t u32id)
{
    s_i32Pointer = 0 ;

    /* Set LIN operation mode, Tx mode and break field length is 12 bits */
    UART_SelectLINMode(UART1, UART_ALTCTL_LINTXEN_Msk, 11);

    s_au8SendData[s_i32Pointer++] = 0x55 ;                   // SYNC Field
    s_au8SendData[s_i32Pointer++] = GetParityValue(u32id);   // ID+Parity Field
    UART_Write(UART1, s_au8SendData, 2);
}

/*-------------------------------------------------------------------------------------------------------------------------------*/
/*  Send LIN Header Field                                                                                                        */
/*  u32HeaderSel =  UART_LINCTL_HSEL_BREAK/UART_LINCTL_HSEL_BREAK_SYNC/UART_LINCTL_HSEL_BREAK_SYNC_ID                            */
/*-------------------------------------------------------------------------------------------------------------------------------*/
void LIN_SendHeaderUsingLinCtlReg(uint32_t u32id, uint32_t u32HeaderSel)
{
    s_i32Pointer = 0 ;

    /* Switch back to LIN Function */
    UART1->FUNCSEL = UART_FUNCSEL_LIN;

    /* Set LIN 1. PID as 0x30 [UART_LINCTL_PID(0x30)]
               2. Header select as includes "break field", "sync field" and "frame ID field".[UART_LINCTL_HSEL_BREAK_SYNC_ID]
               3. Break/Sync Delimiter Length as 1 bit time [UART_LINCTL_BSL(1)]
               4. Break Field Length as 12 bit time [UART_LINCTL_BRKFL(12)]
               5. ID Parity Enable. Hardware will calculate and fill P0/P1 automatically  [UART_LINCTL_IDPEN_Msk]
    */
    if(u32HeaderSel == UART_LINCTL_HSEL_BREAK_SYNC_ID)
    {
        UART1->LINCTL = UART_LINCTL_PID(u32id) | UART_LINCTL_HSEL_BREAK_SYNC_ID |
                        UART_LINCTL_BSL(1) | UART_LINCTL_BRKFL(12) | UART_LINCTL_IDPEN_Msk;
        /* LIN TX Send Header Enable */
        UART1->LINCTL |= UART_LINCTL_SENDH_Msk;
        /* Wait until break field, sync field and ID field transfer completed */
        while((UART1->LINCTL & UART_LINCTL_SENDH_Msk) == UART_LINCTL_SENDH_Msk);
    }

    /* Set LIN 1. Header select as includes "break field" and "sync field".[UART_LINCTL_HSEL_BREAK_SYNC]
               2. Break/Sync Delimiter Length as 1 bit time [UART_LINCTL_BSL(1)]
               3. Break Field Length as 12 bit time [UART_LINCTL_BRKFL(12)]
    */
    else if(u32HeaderSel == UART_LINCTL_HSEL_BREAK_SYNC)
    {
        UART1->LINCTL = UART_LINCTL_HSEL_BREAK_SYNC | UART_LINCTL_BSL(1) | UART_LINCTL_BRKFL(12);
        /* LIN TX Send Header Enable */
        UART1->LINCTL |= UART_LINCTL_SENDH_Msk;
        /* Wait until break field and sync field transfer completed */
        while((UART1->LINCTL & UART_LINCTL_SENDH_Msk) == UART_LINCTL_SENDH_Msk);

        /* Send ID field, s_au8SendData[0] is ID+parity field*/
        s_au8SendData[s_i32Pointer++] = GetParityValue(u32id);   // ID+Parity Field
        UART_Write(UART1, s_au8SendData, 1);
    }

    /* Set LIN 1. Header select as includes "break field".[UART_LINCTL_HSEL_BREAK]
               2. Break/Sync Delimiter Length as 1 bit time [UART_LINCTL_BSL(1)]
               3. Break Field Length as 12 bit time [UART_LINCTL_BRKFL(12)]
    */
    else if(u32HeaderSel == UART_LINCTL_HSEL_BREAK)
    {
        UART1->LINCTL = UART_LINCTL_HSEL_BREAK | UART_LINCTL_BSL(1) | UART_LINCTL_BRKFL(12);
        /* LIN TX Send Header Enable */
        UART1->LINCTL |= UART_LINCTL_SENDH_Msk;
        /* Wait until break field transfer completed */
        while((UART1->LINCTL & UART_LINCTL_SENDH_Msk) == UART_LINCTL_SENDH_Msk);

        /* Send sync field and ID field*/
        s_au8SendData[s_i32Pointer++] = 0x55 ;                  // SYNC Field
        s_au8SendData[s_i32Pointer++] = GetParityValue(u32id);   // ID+Parity Field
        UART_Write(UART1, s_au8SendData, 2);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Send LIN Response Field                                                                                */
/*---------------------------------------------------------------------------------------------------------*/
void LIN_SendResponse(int32_t checkSumOption, uint32_t *pu32TxBuf)
{
    int32_t i32Idx;

    for(i32Idx = 0; i32Idx < 8; i32Idx++)
        s_au8SendData[s_i32Pointer++] = (uint8_t)pu32TxBuf[i32Idx] ;

    s_au8SendData[s_i32Pointer++] = (uint8_t)GetCheckSumValue(s_au8SendData, (uint32_t)checkSumOption) ; //CheckSum Field

    UART_Write(UART1, s_au8SendData + 2, 9);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Send LIN Response Field                                                                                */
/*---------------------------------------------------------------------------------------------------------*/
void LIN_SendResponseWithByteCnt(int32_t checkSumOption, uint32_t *pu32TxBuf, uint32_t u32ByteCnt)
{
    uint32_t u32Idx;

    /* Prepare data */
    for(u32Idx = 0; u32Idx < u32ByteCnt; u32Idx++)
        s_au8SendData[s_i32Pointer++] = (uint8_t)pu32TxBuf[u32Idx] ;

    /* Prepare check sum */
    if(checkSumOption == MODE_CLASSIC)
        s_au8SendData[s_i32Pointer++] = (uint8_t)GetCheckSumValue(&s_au8SendData[2], u32ByteCnt) ;  //CheckSum Field
    else if(checkSumOption == MODE_ENHANCED)
        s_au8SendData[s_i32Pointer++] = (uint8_t)GetCheckSumValue(&s_au8SendData[1], (u32ByteCnt + 1)) ; //CheckSum Field

    /* Send data and check sum */
    UART_Write(UART1, s_au8SendData + 2, 9);
}


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
    CLK_EnableModuleClock(UART1_MODULE);

    /* Select UART module clock source as HXT and UART module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HXT, CLK_CLKDIV0_UART0(1));
    CLK_SetModuleClock(UART1_MODULE, CLK_CLKSEL1_UART1SEL_HXT, CLK_CLKDIV0_UART1(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & (~(UART0_RXD_PB12_Msk | UART0_TXD_PB13_Msk))) | UART0_RXD_PB12 | UART0_TXD_PB13;

    /* Set PB multi-function pins for UART1 RXD(PB.6) and TXD(PB.7) */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB6MFP_Msk)) | SYS_GPB_MFPL_PB6MFP_UART1_RXD;
    SYS->GPB_MFPL = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB7MFP_Msk)) | SYS_GPB_MFPL_PB7MFP_UART1_TXD;

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

void UART1_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART1 */
    SYS_ResetModule(UART1_RST);

    /* Configure UART1 and set UART1 Baudrate */
    UART_Open(UART1, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{

    int32_t i32Item;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    /* Init UART1 */
    UART1_Init();

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);

    printf("\nUART Sample Program\n");

    /* UART sample LIN function */
    do
    {
        TestItem();
        i32Item = getchar();
        if(i32Item == 27) break;        
        printf("%c\n ", i32Item);
        switch(i32Item)
        {
            case '1':
                LIN_FunctionTest();
                break;
            case '2':
                LIN_FunctionTestUsingLinCtlReg();
                break;
            default:
                break;
        }
    }
    while(1);

    printf("\nUART Sample Program End\n");

    while(1);

}

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
