/***************************************************************************//**
 * @file     isp_user.c
 * @brief    ISP Command source file
 * @version  0x32
 *
 * @note
 * Copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "isp_user.h"

__attribute__((aligned(4))) uint8_t g_au8ResponseBuff[64];
__attribute__((aligned(4))) static uint8_t g_u8ApromBuf[FMC_FLASH_PAGE_SIZE];
uint32_t g_u32ApromSize, g_u32DataFlashAddr, g_u32DataFlashSize;

static uint16_t Checksum(unsigned char *buf, int len)
{
    int i;
    uint16_t c;

    for(c = 0, i = 0 ; i < len; i++)
    {
        c += buf[i];
    }

    return (c);
}

int ParseCmd(uint8_t *pu8Buffer, uint8_t u8len)
{
    static uint32_t u32StartAddress, u32StartAddress_bak, u32TotalLen, u32TotalLen_bak, u32LastDataLen, u32PackNo = 1;
    uint32_t u32PageAddress;
    uint8_t *pu8Response;
    uint16_t u16Lcksum;
    uint32_t u32Lcmd, u32srclen;
    uint8_t *pu8Src;
    static uint32_t u32Gcmd;

    pu8Response = g_au8ResponseBuff;
    pu8Src = pu8Buffer;
    u32srclen = u8len;
    u32Lcmd = inpw((uint32_t)pu8Src);
    outpw((uint32_t)(pu8Response + 4), 0);
    pu8Src += 8;
    u32srclen -= 8;

    ReadData(Config0, Config0 + 16, (uint32_t *)(uint32_t)(pu8Response + 8)); /* Read config */

    if(u32Lcmd == CMD_SYNC_PACKNO)
    {
        u32PackNo = inpw((uint32_t)pu8Src);
    }

    if((u32Lcmd) && (u32Lcmd != CMD_RESEND_PACKET))
    {
        u32Gcmd = u32Lcmd;
    }

    if(u32Lcmd == CMD_GET_FWVER)
    {
        pu8Response[8] = FW_VERSION;
    }
    else if(u32Lcmd == CMD_GET_DEVICEID)
    {
        outpw((uint32_t)(pu8Response + 8), SYS->PDID);
        goto out;
    }
    else if(u32Lcmd == CMD_RUN_APROM)
    {
        FMC_SetVectorPageAddr(FMC_APROM_BASE);
        NVIC_SystemReset();

        /* Trap the CPU */
        while(1);
    }
    else if(u32Lcmd == CMD_CONNECT)
    {
        u32PackNo = 1;
        outpw((uint32_t)(pu8Response + 8), g_u32ApromSize);
        outpw((uint32_t)(pu8Response + 12), g_u32DataFlashAddr);
        goto out;
    }
    else if(u32Lcmd == CMD_ERASE_ALL)
    {
        EraseAP(FMC_APROM_BASE, g_u32ApromSize);
    }

    if((u32Lcmd == CMD_UPDATE_APROM) || (u32Lcmd == CMD_UPDATE_DATAFLASH))
    {
        if(u32Lcmd == CMD_UPDATE_DATAFLASH)
        {
            u32StartAddress = g_u32DataFlashAddr;

            if(g_u32DataFlashSize)    
            {
                EraseAP(g_u32DataFlashAddr, g_u32DataFlashSize);
            }
            else
            {
                goto out;
            }
        }
        else
        {
            u32StartAddress = inpw((uint32_t)pu8Src);
            u32TotalLen = inpw((uint32_t)(pu8Src + 4));
            EraseAP(u32StartAddress, u32TotalLen);
        }

        u32TotalLen = inpw((uint32_t)(pu8Src + 4));
        pu8Src += 8;
        u32srclen -= 8;
        u32StartAddress_bak = u32StartAddress;
        u32TotalLen_bak = u32TotalLen;
    }
    else if(u32Lcmd == CMD_UPDATE_CONFIG)
    {
        UpdateConfig((uint32_t *)(uint32_t)pu8Src, (uint32_t *)(uint32_t)(pu8Response + 8));
        goto out;
    }
    else if(u32Lcmd == CMD_RESEND_PACKET)      /* for APROM and Data flash only */
    {
        u32StartAddress -= u32LastDataLen;
        u32TotalLen += u32LastDataLen;
        u32PageAddress = u32StartAddress & (0x100000 - FMC_FLASH_PAGE_SIZE);

        if(u32PageAddress >= Config0)
        {
            goto out;
        }

        ReadData(u32PageAddress, u32StartAddress, (uint32_t *)g_u8ApromBuf);
        FMC_Erase_User(u32PageAddress);
        WriteData(u32PageAddress, u32StartAddress, (uint32_t *)g_u8ApromBuf);

        if((u32StartAddress % FMC_FLASH_PAGE_SIZE) >= (FMC_FLASH_PAGE_SIZE - u32LastDataLen))
        {
            FMC_Erase_User(u32PageAddress + FMC_FLASH_PAGE_SIZE);
        }

        goto out;
    }

    if((u32Gcmd == CMD_UPDATE_APROM) || (u32Gcmd == CMD_UPDATE_DATAFLASH))
    {
        if(u32TotalLen < u32srclen)
        {
            u32srclen = u32TotalLen; /* prevent last package from over writing */
        }

        u32TotalLen -= u32srclen;
        WriteData(u32StartAddress, u32StartAddress + u32srclen, (uint32_t *)(uint32_t)pu8Src); 
        memset(pu8Src, 0, u32srclen);
        ReadData(u32StartAddress, u32StartAddress + u32srclen, (uint32_t *)(uint32_t)pu8Src);
        u32StartAddress += u32srclen;
        u32LastDataLen =  u32srclen;
    }

out:
    u16Lcksum = Checksum(pu8Buffer, u8len);
    outps((uint32_t)pu8Response, u16Lcksum);
    ++u32PackNo;
    outpw((uint32_t)(pu8Response + 4), u32PackNo);
    u32PackNo++;
    return 0;
}

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
