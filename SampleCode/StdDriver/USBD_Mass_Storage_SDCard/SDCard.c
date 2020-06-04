/****************************************************************************//**
 * @file     sdcard.c
 * @version  V1.00
 * @brief    M2351 series SD Card source file
 *
 * @note
 * Copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "SDCARD.h"
#include "NuMicro.h"


/** @addtogroup M2351_Library M2351 Library
  @{
*/

/** @addtogroup M2351_SDCARD SDCARD Library
  @{
*/



static SPI_T    *g_pSPI = SPI1;




/// @cond HIDDEN_SYMBOLS
static int8_t s_i8IsInitialized = 0, s_i8SDType = 0;
static uint32_t s_u32LogicSector = 0;

// Command table for MMC.  This table contains all commands available in SPI
// mode;  Format of command entries is described above in command structure
// definition;
static COMMAND __I s_CommandList[] =
{
    { 0, NO , 0x95, CMD, R1 , NO }, // CMD0;  GO_IDLE_STATE: reset card;
    { 1, NO , 0xFF, CMD, R1 , NO }, // CMD1;  SEND_OP_COND: initialize card;
    { 8, YES, 0xFF, CMD, R7 , NO }, // CMD8;  SEND_IF_COND
    { 9, NO , 0xFF, RD , R1 , NO }, // CMD9;  SEND_CSD: get card specific data;
    {10, NO , 0xFF, RD , R1 , NO }, // CMD10; SEND_CID: get card identifier;
    {12, NO , 0xFF, CMD, R1b, NO }, // CMD12; STOP_TRANSMISSION: end read;
    {13, NO , 0xFF, CMD, R2 , NO }, // CMD13; SEND_STATUS: read card status;
    {16, YES, 0xFF, CMD, R1 , NO }, // CMD16; SET_BLOCKLEN: set block size;
    {17, YES, 0xFF, RDB , R1 , NO }, // CMD17; READ_SINGLE_BLOCK: read 1 block;
    {18, YES, 0xFF, RD , R1 , YES}, // CMD18; READ_MULTIPLE_BLOCK: read > 1;
    {23, NO , 0xFF, CMD, R1 , NO }, // CMD23; SET_BLOCK_COUNT
    {24, YES, 0xFF, WR , R1 , NO }, // CMD24; WRITE_BLOCK: write 1 block;
    {25, YES, 0xFF, WR , R1 , YES}, // CMD25; WRITE_MULTIPLE_BLOCK: write > 1;
    {27, NO , 0xFF, CMD, R1 , NO }, // CMD27; PROGRAM_CSD: program CSD;
    {28, YES, 0xFF, CMD, R1b, NO }, // CMD28; SET_WRITE_PROT: set wp for group;
    {29, YES, 0xFF, CMD, R1b, NO }, // CMD29; CLR_WRITE_PROT: clear group wp;
    {30, YES, 0xFF, CMD, R1 , NO }, // CMD30; SEND_WRITE_PROT: check wp status;
    {32, YES, 0xFF, CMD, R1 , NO }, // CMD32; TAG_SECTOR_START: tag 1st erase;
    {33, YES, 0xFF, CMD, R1 , NO }, // CMD33; TAG_SECTOR_END: tag end(single);
    {34, YES, 0xFF, CMD, R1 , NO }, // CMD34; UNTAG_SECTOR: deselect for erase;
    {35, YES, 0xFF, CMD, R1 , NO }, // CMD35; TAG_ERASE_GROUP_START;
    {36, YES, 0xFF, CMD, R1 , NO }, // CMD36; TAG_ERASE_GROUP_END;
    {37, YES, 0xFF, CMD, R1 , NO }, // CMD37; UNTAG_ERASE_GROUP;
    {38, YES, 0xFF, CMD, R1b, NO }, // CMD38; ERASE: erase all tagged sectors;
    {42, YES, 0xFF, CMD, R1 , NO }, // CMD42; LOCK_UNLOCK;
    {55, NO , 0xFF, CMD, R1 , NO }, // CMD55; APP_CMD
    {58, NO , 0xFF, CMD, R3 , NO }, // CMD58; READ_OCR: read OCR register;
    {59, YES, 0xFF, CMD, R1 , NO }, // CMD59; CRC_ON_OFF: toggles CRC checking;
    {0x80 + 13, NO , 0xFF, CMD, R2 , NO }, // ACMD13; SD_SEND_STATUS: read card status;
    {0x80 + 23, YES, 0xFF, CMD, R1 , NO }, // ACMD23;SD_SET_WR_BLK_ERASE_COUNT
    {0x80 + 41, YES, 0xFF, CMD, R1 , NO } // ACMD41; SD_SEND_OP_COND: initialize card;
};
/// @endcond HIDDEN_SYMBOLS
/** @addtogroup M2351_SDCARD_EXPORTED_FUNCTIONS SDCARD Library Exported Functions
  @{
*/

void SD_Delay(uint32_t u32Count);
void MMC_FLASH_Init(void);

/**
  * @brief Delay function.
  * @param[in] count loop count.
  * @return none
  */
void SD_Delay(uint32_t u32Count)
{
    uint32_t volatile u32Loop;
    for(u32Loop = 0; u32Loop < u32Count; u32Loop++);
}
/*---------------------------------------------------------------------------------------------------------*/
/* SD CARD Protocol                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
/*
    Transfer Length:48bit
    BIT POSITION       WIDTH(BITS)          VAULE
            [47]                1               0:Start bit
            [46]                1               1:Transmit   0:Receive
         [45:40]                6               CMD8:001000
          [39:8]               32               Reserved
           [7:1]                7               Reserved
             [0]                1               1:End bit

*/

/**
  * @brief This function is used to generate CRC value
  * @param[in] u32Data Input Data
  * @param[in] u16GenPoly CRC7:0x1200, CRC16:0x1021
  * @param[in] u16Accum CRC value
  * @return CRC value
  */
static uint16_t GenerateCRC(uint32_t u32Data, uint16_t u16GenPoly, uint16_t u16Accum)
{
    volatile uint8_t i;

    u32Data <<= 8;
    for(i = 8; i > 0; i--)
    {
        if((u32Data ^ u16Accum) & 0x8000)
            u16Accum = (uint16_t)(u16Accum << 1) ^ u16GenPoly;
        else
            u16Accum <<= 1;
        u32Data <<= 1;
    }
    return u16Accum;
}

/**
  * @brief This function is used to send data though SPI to general clock for SDCARD operation
  * @param[in] u32Data Data to send
  * @return none
  */
static uint32_t SingleWrite(uint32_t u32Data)
{
    while(SPI_IS_BUSY(g_pSPI));
    SPI_WRITE_TX(g_pSPI, u32Data);
    while(SPI_IS_BUSY(g_pSPI));
    return SPI_READ_RX(g_pSPI);
}

/**
  * @brief This function is used to Send SDCARD CMD and Receive Response
  * @param[in] u8Cmd Set command register
  * @param[in] u32Arg Set command argument
  * @param[out] *pu8Char Get register and data
  * @param[out] *pu32Response Get response
  * @retval TRUE get response
  * @retval FALSE 1.SD Card busy, 2.Card moved, 3.Timeout
  */
uint32_t MMC_Command_Exec(uint8_t u8Cmd, uint32_t u32Arg, uint8_t *pu8Char, uint32_t *pu32Response)
{
    uint8_t u8Loopguard;
    COMMAND current_command;                // Local space for the command table
    UINT32 long_arg;                        // Local space for argument
    static uint32_t u32Current_Blklen = 512;
    uint32_t u32Old_Blklen = 512;
    uint32_t u32Counter = 0;                // Byte counter for multi-byte fields;
    UINT16 card_response;                   // Variable for storing card response;
    uint8_t u8Data_Resp;                    // Variable for storing data response;
    UINT16 dummy_CRC;                       // Dummy variable for storing CRC field;
    int32_t i32Count;

    card_response.i = 0;

    current_command = s_CommandList[u8Cmd];// Retrieve desired command table entry
    // from code space;
    if(current_command.command_byte & 0x80)
    {
        // Detect ACMD
        if(MMC_Command_Exec(APP_CMD, EMPTY, EMPTY, pu32Response) == FALSE) //Send APP_CMD
            return FALSE;
    }

    PH10 = 0;//SPI_SET_SS_LOW(g_pSPI);// CS = 0

    SingleWrite(0xFF);
    SingleWrite((current_command.command_byte | 0x40) & 0x7f);
    DBG_PRINTF("CMD:%d,", current_command.command_byte & 0x7f);

    SD_Delay(200);

    long_arg.l = u32Arg;                  // Make argument byte addressable;
    // If current command changes block
    // length, update block length variable
    // to keep track;
    // Command byte = 16 means that a set
    // block length command is taking place
    // and block length variable must be
    // set;
    if(current_command.command_byte == 16)
    {
        u32Current_Blklen = u32Arg;
    }
    // Command byte = 9 or 10 means that a
    // 16-byte register value is being read
    // from the card, block length must be
    // set to 16 bytes, and restored at the
    // end of the transfer;
    if((current_command.command_byte == 9) || (current_command.command_byte == 10))
    {
        u32Old_Blklen = u32Current_Blklen;     // Command is a GET_CSD or GET_CID,
        u32Current_Blklen = 16;             // set block length to 16-bytes;
    }
    // If an argument is required, transmit
    // one, otherwise transmit 4 bytes of
    // 0x00;
    if(current_command.arg_required == YES)
    {
        dummy_CRC.i = GenerateCRC((current_command.command_byte | 0x40), 0x1200, 0);
        for(i32Count = 3; i32Count >= 0; i32Count--)
        {
            SingleWrite(long_arg.b[i32Count]);
            dummy_CRC.i = GenerateCRC(long_arg.b[i32Count], 0x1200, dummy_CRC.i);

        }
        dummy_CRC.i = (dummy_CRC.i >> 8) | 0x01;
        SingleWrite(dummy_CRC.b[0]);
    }
    else
    {
        u32Counter = 0;
        while(u32Counter <= 3)
        {
            SingleWrite(0x00);
            u32Counter++;
        }
        SingleWrite(/*current_command.CRC*/current_command.CRC_SD);
    }

    // The command table entry will indicate
    // what type of response to expect for
    // a given command;  The following
    // conditional handles the MMC response;
    if(current_command.response == R1)
    {
        // Read the R1 response from the card;
        u8Loopguard = 0;
        do
        {
            card_response.b[0] = (uint8_t)SingleWrite(0xFF);
            if(!++u8Loopguard) break;
        }
        while((card_response.b[0] & BUSY_BIT));
        DBG_PRINTF("R1:0x%x, counter:%d\n", card_response.b[0], u8Loopguard);

        if(!u8Loopguard)
        {
            BACK_FROM_ERROR;
        }

//      while((SingleWrite(0xFF)&0xFF) == 0x00);
        *pu32Response = card_response.b[0];
    }
    else if(current_command.response == R1b)   // Read the R1b response;
    {
        u8Loopguard = 0;
        do
        {
            card_response.b[0] =  (uint8_t)SingleWrite(0xFF);
            if(!++u8Loopguard) break;
        }
        while((card_response.b[0] & BUSY_BIT));
        while((SingleWrite(0xFF) & 0xFF) == 0x00);
    }
    else if(current_command.response == R2)
    {
        u8Loopguard = 0;
        do
        {
            card_response.b[0] = (uint8_t)SingleWrite(0xFF);
            if(!++u8Loopguard) break;
        }
        while((card_response.b[0] & BUSY_BIT));
        card_response.b[1] = (uint8_t)SingleWrite(0xFF);
        DBG_PRINTF("R2:0x%x, counter:%d\n", card_response.i, u8Loopguard);
        if(!u8Loopguard)
        {
            BACK_FROM_ERROR;
        }
        *pu32Response = card_response.i;
    }
    else if(current_command.response == R3)
    {
        // Read R3 response;
        u8Loopguard = 0;
        do
        {
            card_response.b[0] = (uint8_t)SingleWrite(0xFF);
            if(!++u8Loopguard) break;
        }
        while((card_response.b[0] & BUSY_BIT));
        DBG_PRINTF("R3:0x%x, counter:%d\n", card_response.b[0], u8Loopguard);
        if(!u8Loopguard)
        {
            BACK_FROM_ERROR;
        }
        u32Counter = 0;
        while(u32Counter <= 3)              // Read next three bytes and store them
        {
            // in local memory;  These bytes make up
            u32Counter++;                    // the Operating Conditions Register
            *pu8Char++ = (uint8_t)SingleWrite(0xFF);
        }
        *pu32Response = card_response.b[0];
    }
    else
    {
        // Read R7 response;
        u8Loopguard = 0;
        do
        {
            card_response.b[0] = (uint8_t)SingleWrite(0xFF);
            if(!++u8Loopguard) break;
        }
        while((card_response.b[0] & BUSY_BIT));
        DBG_PRINTF("R7:0x%x, counter:%d\n", card_response.b[0], u8Loopguard);
        if(!u8Loopguard)
        {
            BACK_FROM_ERROR;
        }
        u32Counter = 0;
        while(u32Counter <= 3)              // Read next three bytes and store them
        {
            // in local memory;  These bytes make up
            u32Counter++;                    // the Operating Conditions Register
            *pu8Char++ = (uint8_t)SingleWrite(0xFF);
        }
        *pu32Response = card_response.b[0];
    }


    switch(current_command.trans_type)   // This conditional handles all data
    {
        // operations;  The command entry
        // determines what type, if any, data
        // operations need to occur;
        case RDB:                         // Read data from the MMC;
            u8Loopguard = 0;

            while((SingleWrite(0xFF) & 0xFF) != START_SBR)
            {
                if(!++u8Loopguard)
                {
                    BACK_FROM_ERROR;
                }
                SD_Delay(119);
            }
            u32Counter = 0;                    // Reset byte counter;
            // Read <u32Current_Blklen> bytes;
            if(pu8Char)
            {
                for(u32Counter = 0; u32Counter < u32Current_Blklen; u32Counter++)
                {
                    SPI_WRITE_TX(g_pSPI, 0xFF);
                    while(SPI_IS_BUSY(g_pSPI));
                    *(pu8Char + u32Counter) = (uint8_t)SPI_READ_RX(g_pSPI);
                }
            }
            else
            {
                for(; u32Counter < u32Current_Blklen; u32Counter++)
                {
                    SPI_WRITE_TX(g_pSPI, 0xFF);
                    while(SPI_IS_BUSY(g_pSPI));
                }
            }
            dummy_CRC.b[1] = (uint8_t)SingleWrite(0xFF); // After all data is read, read the two
            dummy_CRC.b[0] = (uint8_t)SingleWrite(0xFF); // CRC bytes;  These bytes are not used
            // in this mode, but the place holders
            // must be read anyway;
            break;
        case RD:                         // Read data from the MMC;
            u8Loopguard = 0;

            while((SingleWrite(0xFF) & 0xFF) != START_SBR)
            {
                if(!++u8Loopguard)
                {
                    BACK_FROM_ERROR;
                }
            }
            u32Counter = 0;                    // Reset byte counter;
            // Read <u32Current_Blklen> bytes;
            if(pu8Char)
            {
                for(u32Counter = 0; u32Counter < u32Current_Blklen; u32Counter++)
                {
                    SPI_WRITE_TX(g_pSPI, 0xFF);
                    while(SPI_IS_BUSY(g_pSPI));
                    *(pu8Char + u32Counter) = (uint8_t)SPI_READ_RX(g_pSPI);
                }
            }
            else
            {
                for(u32Counter = 0; u32Counter < u32Current_Blklen; u32Counter++)
                {
                    SPI_WRITE_TX(g_pSPI, 0xFF);
                    while(SPI_IS_BUSY(g_pSPI));
                }
            }
            dummy_CRC.b[1] = (uint8_t)SingleWrite(0xFF); // After all data is read, read the two
            dummy_CRC.b[0] = (uint8_t)SingleWrite(0xFF); // CRC bytes;  These bytes are not used
            // in this mode, but the place holders
            // must be read anyway;
            break;

        case WR:
            SingleWrite(0xFF);
            SingleWrite(START_SBW);

            for(u32Counter = 0; u32Counter < u32Current_Blklen; u32Counter++)
            {
                SPI_WRITE_TX(g_pSPI, *(pu8Char + u32Counter));
                dummy_CRC.i = GenerateCRC(*(pu8Char + u32Counter), 0x1021, dummy_CRC.i);
                while(SPI_IS_BUSY(g_pSPI));
            }
            SingleWrite(dummy_CRC.b[1]);
            SingleWrite(dummy_CRC.b[0]);

            u8Loopguard = 0;
            do                            // Read Data Response from card;
            {
                u8Data_Resp = (uint8_t)SingleWrite(0xFF);
                if(!++u8Loopguard) break;
            }
            while((u8Data_Resp & DATA_RESP_MASK) != 0x01);     // When bit 0 of the MMC response
            // is clear, a valid data response
            // has been received;

            if(!u8Loopguard)
            {
                BACK_FROM_ERROR;
            }

            while((SingleWrite(0xFF) & 0xFF) != 0xFF); //Wait for Busy
            SingleWrite(0xFF);
            break;
        default:
            break;
    }

    PH10 = 1;//SPI_SET_SS_HIGH(g_pSPI);// CS = 1

    if((current_command.command_byte == 9) || (current_command.command_byte == 10))
    {
        u32Current_Blklen = u32Old_Blklen;
    }
    return TRUE;
}

/**
  * @brief This function is used to initialize the flash card
  * @return none
  */
void MMC_FLASH_Init(void)
{
    uint32_t u32Response;
    uint16_t u16Loopguard;
    uint32_t i;
    uint8_t u8Counter = 0;
    uint8_t pu8Char[16];              // Data pointer for storing MMC
    uint32_t u32C_Size, u32Bl_Len;
    uint8_t u8C_Mult;


    s_i8IsInitialized = 0;


    PH10 = 1;//SPI_SET_SS_HIGH(g_pSPI);// CS = 1

    SD_Delay(2000);
    //--------------------------------------------------------
    //  Send 74 SD clcok in SD mode for Toshiba SD Card
    //--------------------------------------------------------
    for(u8Counter = 0; u8Counter < 10; u8Counter++)
    {
        SingleWrite(0xFF);
    }
    SD_Delay(2000);

    PH10 = 0;//SPI_SET_SS_LOW(g_pSPI);// CS = 0

    while(MMC_Command_Exec(GO_IDLE_STATE, EMPTY, EMPTY, &u32Response) == FALSE)
        SD_Delay(1000);
    if(u32Response != 0x01)
        return;

    if(MMC_Command_Exec(SEND_IF_COND, 0x15A, pu8Char, &u32Response) && u32Response == 1)
    {
        /* SDC ver 2.00 */
        if(pu8Char[2] == 0x01 && pu8Char[3] == 0x5A)
        {
            /* The card can work at VDD range of 2.7-3.6V */
            u16Loopguard = 0;
            do
            {
                MMC_Command_Exec(SD_SEND_OP_COND, 0x40000000, EMPTY, &u32Response); //Enable HCS(OCR[30])
                if(!++u16Loopguard) break;
                SD_Delay(0x100);
            }
            while(u32Response != 0);
            if(!u16Loopguard)
                return;

            MMC_Command_Exec(READ_OCR, EMPTY, pu8Char, &u32Response);
            s_i8SDType = (pu8Char[0] & 0x40) ? SDv2 | SDBlock : SDv2;
        }
    }
    else
    {
        /* SDv1 or MMCv3 */
        MMC_Command_Exec(SD_SEND_OP_COND, 0x00, EMPTY, &u32Response);
        if(u32Response <= 1)
        {
            u16Loopguard = 0;
            do
            {
                MMC_Command_Exec(SD_SEND_OP_COND, 0x00, EMPTY, &u32Response);
                if(!++u16Loopguard) break;
                SD_Delay(50);
            }
            while(u32Response != 0);
            if(!u16Loopguard)
                return;
            s_i8SDType = SDv1;  /* SDv1 */
        }
        else
        {
            u16Loopguard = 0;
            do
            {
                MMC_Command_Exec(SEND_OP_COND, 0x00, EMPTY, &u32Response);
                if(!++u16Loopguard) break;
                SD_Delay(50);
            }
            while(u32Response != 0);
            if(!u16Loopguard)
                return;
            s_i8SDType = MMCv3; /* MMCv3 */
        }
        MMC_Command_Exec(SET_BLOCKLEN, (uint32_t)PHYSICAL_BLOCK_SIZE, EMPTY, &u32Response);
    }
    if(MMC_Command_Exec(SEND_CSD, EMPTY, pu8Char, &u32Response) == FALSE)
        return;

    if(u32Response == 0)
    {
        DBG_PRINTF("Change speed:");
        for(i = 0; i < 16; i++)
        {
            DBG_PRINTF("0x%X ", pu8Char[i]);
        }

    }
    else
    {
        DBG_PRINTF("CARD STATUS 0x%X:\n", u32Response);
        for(i = 0; i < 16; i++)
        {
            DBG_PRINTF("0x%X ", pu8Char[i]);
        }
        s_u32LogicSector = 0;
        return;
    }

    if(s_i8SDType & SDBlock) // Determine the number of MMC sectors;
    {
        u32Bl_Len = 1 << (pu8Char[5] & 0x0f) ;
        u32C_Size = ((pu8Char[7] & 0x3Fu) << 16) | ((uint32_t)pu8Char[8] << 8) | (pu8Char[9]);
        s_u32LogicSector = u32C_Size * ((512 * 1024) / u32Bl_Len);
    }
    else
    {
        u32Bl_Len = 1 << (pu8Char[5] & 0x0f) ;
        u32C_Size = ((pu8Char[6] & 0x03u) << 10) | ((uint32_t)pu8Char[7] << 2) | ((pu8Char[8] & 0xc0u) >> 6);
        u8C_Mult = (uint8_t)(((pu8Char[9] & 0x03u) << 1) | ((pu8Char[10] & 0x80u) >> 7));
        s_u32LogicSector = (u32C_Size + 1) * (1 << (u8C_Mult + 2)) * (u32Bl_Len / 512);
    }
    DBG_PRINTF("\nLogicSector:%d, PHYSICAL_SIZE:%dMB\n", s_u32LogicSector, (s_u32LogicSector / 2 / 1024));

    u16Loopguard = 0;
    while((MMC_Command_Exec(READ_SINGLE_BLOCK, 0, 0, &u32Response) == FALSE))
    {
        if(!++u16Loopguard) break;
    }
    s_i8IsInitialized = 1;
}

/**
  * @brief This function is used to Open GPIO function and initial SDCARD
  * @retval SD_FAIL Initial SDCARD Failed
  * @retval SD_SUCCESS Success
  */
static volatile uint32_t  s_u32Gfreq;

uint32_t SDCARD_Open(void)
{
    PH9 = 0;//Enable SD Card power
    SD_Delay(100000);

    SPI_ENABLE(g_pSPI);
    /* Configure g_pSPI as a master, 8-bit transaction*/
    SPI_Open(g_pSPI, SPI_MASTER, SPI_MODE_0, 8, 300000);//300Khz for SD initial flow
    DBG_PRINTF("SPI is running at %d Hz\n", SPI_GetBusClock(g_pSPI));
    SPI_DisableAutoSS(g_pSPI);
    PH10 = 1;//SPI_SET_SS_LOW(g_pSPI);
    SingleWrite(0xFFFFFFFF);

    MMC_FLASH_Init();
    SD_Delay(300000);
    if(s_i8IsInitialized)
        DBG_PRINTF("SDCARD INIT OK\n\n");
    else
    {
        DBG_PRINTF("SDCARD INIT FAIL\n\n");
        return SD_FAIL;
    }

    s_u32Gfreq = SPI_SetBusClock(g_pSPI, 16000000);//16Mhz for SD operation speed
    DBG_PRINTF("Now, SPI is running at %d Hz\n", SPI_GetBusClock(g_pSPI));

    return SD_SUCCESS;
}

/**
  * @brief This function is used to close SDCARD
  * @return none
  */
void SDCARD_Close(void)
{
    SPI_Close(g_pSPI);
}

/**
  * @brief This function is used to get card total sector after SDCARD is opened
  * @param[out] *pu32TotSecCnt Get sector count
  * @retval TRUE The size is already saved
  * @retval FALSE The size is zero
  */
uint32_t SDCARD_GetCardSize(uint32_t *pu32TotSecCnt)
{
    if(s_u32LogicSector == 0)
        return FALSE;
    else
        *pu32TotSecCnt = s_u32LogicSector;

    return TRUE;
}

/**
  * @brief This function is used to get logic sector size
  * @return The Logic Sector size
  */
uint32_t GetLogicSector(void)
{
    return s_u32LogicSector;
}

/**
  * @brief This function is used to Get data from SD card
  * @param[in] u32Addr Set start address for LBA
  * @param[in] u32Size Set data size (byte)
  * @param[in] pu8Buffer Set buffer pointer
  * @return none
  */
void SpiRead(uint32_t u32Addr, uint32_t u32Size, uint8_t* pu8Buffer)
{
    /* This is low level read function of USB Mass Storage */
    uint32_t u32Response;
    if(s_i8SDType & SDBlock)
    {
        while(u32Size >= PHYSICAL_BLOCK_SIZE)
        {
            MMC_Command_Exec(READ_SINGLE_BLOCK, u32Addr, pu8Buffer, &u32Response);
            u32Addr   ++;
            pu8Buffer += PHYSICAL_BLOCK_SIZE;
            u32Size  -= PHYSICAL_BLOCK_SIZE;
        }

    }
    else
    {
        u32Addr *= PHYSICAL_BLOCK_SIZE;
        while(u32Size >= PHYSICAL_BLOCK_SIZE)
        {
            MMC_Command_Exec(READ_SINGLE_BLOCK, u32Addr, pu8Buffer, &u32Response);
            u32Addr   += PHYSICAL_BLOCK_SIZE;
            pu8Buffer += PHYSICAL_BLOCK_SIZE;
            u32Size  -= PHYSICAL_BLOCK_SIZE;
        }
    }
}

/**
  * @brief This function is used to store data into SD card
  * @param[in] u32Addr Set start address for LBA
  * @param[in] u32Size Set data size (byte)
  * @param[in] pu8Buffer Set buffer pointer
  * @return none
  */
void SpiWrite(uint32_t u32Addr, uint32_t u32Size, uint8_t* pu8Buffer)
{
    uint32_t u32Response;
    if(s_i8SDType & SDBlock)
    {
        while(u32Size >= PHYSICAL_BLOCK_SIZE)
        {
            MMC_Command_Exec(WRITE_BLOCK, u32Addr, pu8Buffer, &u32Response);
            u32Addr   ++;
            pu8Buffer += PHYSICAL_BLOCK_SIZE;
            u32Size  -= PHYSICAL_BLOCK_SIZE;
        }
    }
    else
    {
        u32Addr *= PHYSICAL_BLOCK_SIZE;
        while(u32Size >= PHYSICAL_BLOCK_SIZE)
        {
            MMC_Command_Exec(WRITE_BLOCK, u32Addr, pu8Buffer, &u32Response);
            u32Addr   += (PHYSICAL_BLOCK_SIZE);
            pu8Buffer += PHYSICAL_BLOCK_SIZE;
            u32Size  -= PHYSICAL_BLOCK_SIZE;
        }
    }
}
/*@}*/ /* end of group M2351_SDCARD_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group M2351_SDCARD_Driver */

/*@}*/ /* end of group M2351_Library */

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
