/***************************************************************************//**
 * @file     DataFlashProg.c
 * @brief    M2351 Series Data Flash Access API
 *
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/


/*---------------------------------------------------------------------------------------------------------*/
/* Includes of system headers                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"
#include "dataflashprog.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Macro, type and constant definitions                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
uint32_t g_u32Tag = (uint32_t) - 1;
uint32_t g_sectorBuf[FLASH_PAGE_SIZE / 4];

void DataFlashRead(uint32_t addr, uint32_t size, uint32_t buffer)
{
    /* This is low level read function of USB Mass Storage */
    uint32_t alignAddr, offset, *pu32;
    int32_t i;

    /* Modify the address to MASS_STORAGE_OFFSET */
    addr += MASS_STORAGE_OFFSET;

    /* Get address base on page size alignment */
    alignAddr = addr & (~(FLASH_PAGE_SIZE - 1));

    /* Get the sector offset*/
    offset = (addr & (FLASH_PAGE_SIZE - 1));

    pu32 = (uint32_t *)buffer;

    for(i = 0; i < size / 4; i++)
    {
        /* Read from cache */
        if(((alignAddr + i * 4) >= g_u32Tag) && ((alignAddr + i * 4) < g_u32Tag + FLASH_PAGE_SIZE))
        {
            offset = (addr + i * 4 - g_u32Tag) / 4;
            pu32[i] = g_sectorBuf[offset];
        }
        else
        {
            /* Read from flash */
            pu32[i] = M32(addr + i * 4);
        }
    }
}

void DataFlashWrite(uint32_t addr, uint32_t size, uint32_t buffer)
{
    /* This is low level write function of USB Mass Storage */
    int32_t len, i, offset;
    uint32_t *pu32;
    uint32_t alignAddr;

    /* Modify the address to MASS_STORAGE_OFFSET */
    addr += MASS_STORAGE_OFFSET;

    len = (int32_t)size;

    do
    {
        /* Get address base on page size alignment */
        alignAddr = addr & (~(FLASH_PAGE_SIZE - 1));

        /* Get the sector offset*/
        offset = (addr & (FLASH_PAGE_SIZE - 1));

        /* check cache buffer */
        if(g_u32Tag != alignAddr)
        {
            if(g_u32Tag != (uint32_t) - 1)
            {
                /* We need to flush out cache before update it */
                FMC_Erase(g_u32Tag);

                for(i = 0; i < FLASH_PAGE_SIZE / 4; i++)
                {
                    FMC_Write(g_u32Tag + i * 4, g_sectorBuf[i]);
                }
            }

            /* Load data to cache buffer */
            for(i = 0; i < FLASH_PAGE_SIZE / 4; i++)
            {
                g_sectorBuf[i] = FMC_Read(alignAddr + i * 4);
            }
            g_u32Tag = alignAddr;
        }

        /* Source buffer */
        pu32 = (uint32_t *)buffer;
        /* Get the update length */
        len = FLASH_PAGE_SIZE - offset;
        if(size < len)
            len = size;
        /* Update the destination buffer */
        for(i = 0; i < len / 4; i++)
        {
            g_sectorBuf[offset / 4 + i] = pu32[i];
        }

        size -= len;
        addr += len;
        buffer += len;
    }
    while(size > 0);
}

