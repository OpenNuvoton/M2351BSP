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
__attribute__((aligned(4))) static uint8_t s_au8ApromBuf[FMC_FLASH_PAGE_SIZE];
uint32_t g_u32ApromSize, g_u32DataFlashAddr, g_u32DataFlashSize;

static uint16_t Checksum(unsigned char *pucBuf, int i8Len)
{
    int i;
    uint16_t c;

    for(c = 0, i = 0 ; i < i8Len; i++)
    {
        c += pucBuf[i];
    }

    return (c);
}

int ParseCmd(unsigned char *pucBuffer, uint8_t u8len)
{
    static uint32_t u32StartAddress, u32StartAddressBak, u32TotalLen, u32TotalLenBak, u32LastDataLen, s_u32PackNo = 1;
    uint8_t *pu8Response;
    uint16_t u16Lcksum;
    uint32_t u32Lcmd, u32Srclen, u32PageAddress;
    unsigned char *pucSrc;
    static uint32_t s_u32Gcmd;

    pu8Response = g_au8ResponseBuff;
    pucSrc = pucBuffer;
    u32Srclen = u8len;
    u32Lcmd = inpw((uint32_t)pucSrc);
    outpw((uint32_t)(pu8Response + 4), 0);
    pucSrc += 8;
    u32Srclen -= 8;

    /* Read config */
    ReadData(CONFIG0, CONFIG0 + 16, (uint32_t *)((uint32_t)(pu8Response + 8)));

    if(u32Lcmd == CMD_SYNC_PACKNO)
    {
        s_u32PackNo = inpw((uint32_t)(pucSrc));
    }

    if((u32Lcmd) && (u32Lcmd != CMD_RESEND_PACKET))
    {
        s_u32Gcmd = u32Lcmd;
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
        s_u32PackNo = 1;
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
            u32StartAddress = inpw((uint32_t)(pucSrc));
            u32TotalLen = inpw((uint32_t)(pucSrc + 4));
            EraseAP(u32StartAddress, u32TotalLen);
        }

        u32TotalLen = inpw((uint32_t)(pucSrc + 4));
        pucSrc += 8;
        u32Srclen -= 8;
        u32StartAddressBak = u32StartAddress;
        u32TotalLenBak = u32TotalLen;
    }
    else if(u32Lcmd == CMD_UPDATE_CONFIG)
    {
        UpdateConfig((uint32_t *)((uint32_t)pucSrc), (uint32_t *)((uint32_t)(pu8Response + 8)));
        goto out;
    }
    else if(u32Lcmd == CMD_RESEND_PACKET)
    {
        /* For APROM & Data flash only */
        u32StartAddress -= u32LastDataLen;
        u32TotalLen += u32LastDataLen;
        u32PageAddress = u32StartAddress & (0x100000 - FMC_FLASH_PAGE_SIZE);

        if(u32PageAddress >= CONFIG0)
        {
            goto out;
        }

        ReadData(u32PageAddress, u32StartAddress, (uint32_t *)s_au8ApromBuf);
        FMC_Erase_User(u32PageAddress);
        WriteData(u32PageAddress, u32StartAddress, (uint32_t *)s_au8ApromBuf);

        if((u32StartAddress % FMC_FLASH_PAGE_SIZE) >= (FMC_FLASH_PAGE_SIZE - u32LastDataLen))
        {
            FMC_Erase_User(u32PageAddress + FMC_FLASH_PAGE_SIZE);
        }

        goto out;
    }

    if((s_u32Gcmd == CMD_UPDATE_APROM) || (s_u32Gcmd == CMD_UPDATE_DATAFLASH))
    {
        if(u32TotalLen < u32Srclen)
        {
            /* prevent last package from over writing */
            u32Srclen = u32TotalLen;
        }

        u32TotalLen -= u32Srclen;
        WriteData(u32StartAddress, u32StartAddress + u32Srclen, (uint32_t *)((uint32_t)pucSrc));
        memset(pucSrc, 0, u32Srclen);
        ReadData(u32StartAddress, u32StartAddress + u32Srclen, (uint32_t *)((uint32_t)pucSrc));
        u32StartAddress += u32Srclen;
        u32LastDataLen =  u32Srclen;
    }

out:
    u16Lcksum = Checksum(pucBuffer, u8len);
    outps((uint32_t)pu8Response, u16Lcksum);
    ++s_u32PackNo;
    outpw((uint32_t)(pu8Response + 4), s_u32PackNo);
    s_u32PackNo++;
    return 0;
}

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
