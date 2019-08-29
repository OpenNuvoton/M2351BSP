/******************************************************************************
 * @file     isp_user.c
 * @brief    ISP Command source file
 * @version  0x32
 * @date     14, June, 2017
 *
 * @note
 * Copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "isp_user.h"

__attribute__((aligned(4))) uint8_t au8ResponseBuff[64];
__attribute__((aligned(4))) static uint8_t aprom_buf[FMC_FLASH_PAGE_SIZE];
uint32_t g_u32ApromSize, g_u32DataFlashAddr, g_u32DataFlashSize;

static uint16_t Checksum(unsigned char *buf, int len)
{
    int i;
    uint16_t c;

    for (c = 0, i = 0 ; i < len; i++) {
        c += buf[i];
    }

    return (c);
}

int ParseCmd(unsigned char *pucBuffer, uint8_t u8len)
{
    static uint32_t u32StartAddress, u32StartAddress_bak, u32TotalLen, u32TotalLen_bak, u32LastDataLen, u32PackNo = 1;
    uint8_t *pu8Response;
    uint16_t u16Lcksum;
    uint32_t u32Lcmd, srclen, i;
    unsigned char *pucSrc;
    static uint32_t	u32Gcmd;

    pu8Response = au8ResponseBuff;
    pucSrc = pucBuffer;
    srclen = u8len;
    u32Lcmd = inpw(pucSrc);
    outpw(pu8Response + 4, 0);
    pucSrc += 8;
    srclen -= 8;
    /* Read config */
    ReadData(CONFIG0, CONFIG0 + 16, (uint32_t *)(pu8Response + 8));

    if (u32Lcmd == CMD_SYNC_PACKNO) {
        u32PackNo = inpw(pucSrc);
    }

    if ((u32Lcmd) && (u32Lcmd != CMD_RESEND_PACKET)) {
        u32Gcmd = u32Lcmd;
    }

    if (u32Lcmd == CMD_GET_FWVER) {
        pu8Response[8] = FW_VERSION;
    } else if (u32Lcmd == CMD_GET_DEVICEID) {
        outpw(pu8Response + 8, SYS->PDID);
        goto out;
    } else if (u32Lcmd == CMD_RUN_APROM) {
        FMC_SetVectorPageAddr(FMC_APROM_BASE);
        NVIC_SystemReset();

        /* Trap the CPU */
        while (1);
    } else if (u32Lcmd == CMD_CONNECT) {
        u32PackNo = 1;
        outpw(pu8Response + 8, g_u32ApromSize);
        outpw(pu8Response + 12, g_u32DataFlashAddr);
        goto out;
    } else if ((u32Lcmd == CMD_UPDATE_APROM) || (u32Lcmd == CMD_ERASE_ALL)) {
        /* Erase APROM */
        EraseAP(FMC_APROM_BASE, (g_u32ApromSize < g_u32DataFlashAddr) ? g_u32ApromSize : g_u32DataFlashAddr);

        if (u32Lcmd == CMD_ERASE_ALL) {
            /* Erase data flash */
            EraseAP(g_u32DataFlashAddr, g_u32DataFlashSize);
            UpdateConfig((uint32_t *)(pu8Response + 8), NULL);
        }
    }

    if ((u32Lcmd == CMD_UPDATE_APROM) || (u32Lcmd == CMD_UPDATE_DATAFLASH)) {
        if (u32Lcmd == CMD_UPDATE_DATAFLASH) {
            u32StartAddress = g_u32DataFlashAddr;

            if (g_u32DataFlashSize) {
                EraseAP(g_u32DataFlashAddr, g_u32DataFlashSize);
            } else {
                goto out;
            }
        } else {
            u32StartAddress = 0;
        }

        u32TotalLen = inpw(pucSrc + 4);
        pucSrc += 8;
        srclen -= 8;
        u32StartAddress_bak = u32StartAddress;
        u32TotalLen_bak = u32TotalLen;
    } else if (u32Lcmd == CMD_UPDATE_CONFIG) {
        UpdateConfig((uint32_t *)(pucSrc), (uint32_t *)(pu8Response + 8));
        goto out;
    } else if (u32Lcmd == CMD_RESEND_PACKET) {
        /* For APROM&Data flash only */
        uint32_t PageAddress;
        u32StartAddress -= u32LastDataLen;
        u32TotalLen += u32LastDataLen;
        PageAddress = u32StartAddress & (0x100000 - FMC_FLASH_PAGE_SIZE);

        if (PageAddress >= CONFIG0) {
            goto out;
        }

        ReadData(PageAddress, u32StartAddress, (uint32_t *)aprom_buf);
        FMC_Erase_User(PageAddress);
        WriteData(PageAddress, u32StartAddress, (uint32_t *)aprom_buf);

        if ((u32StartAddress % FMC_FLASH_PAGE_SIZE) >= (FMC_FLASH_PAGE_SIZE - u32LastDataLen)) {
            FMC_Erase_User(PageAddress + FMC_FLASH_PAGE_SIZE);
        }

        goto out;
    }

    if ((u32Gcmd == CMD_UPDATE_APROM) || (u32Gcmd == CMD_UPDATE_DATAFLASH)) {
        if (u32TotalLen < srclen) {
            /* prevent last package from over writing */
            srclen = u32TotalLen;
        }

        u32TotalLen -= srclen;
        WriteData(u32StartAddress, u32StartAddress + srclen, (uint32_t *)pucSrc);
        memset(pucSrc, 0, srclen);
        ReadData(u32StartAddress, u32StartAddress + srclen, (uint32_t *)pucSrc);
        u32StartAddress += srclen;
        u32LastDataLen =  srclen;
    }

out:
    u16Lcksum = Checksum(pucBuffer, u8len);
    outps(pu8Response, u16Lcksum);
    ++u32PackNo;
    outpw(pu8Response + 4, u32PackNo);
    u32PackNo++;
    return 0;
}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
