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
static uint32_t s_u32Tag = (uint32_t) - 1;
static uint32_t s_au32SectorBuf[FLASH_PAGE_SIZE / 4];

void DataFlashRead(uint32_t u32Addr, uint32_t u32Size, uint32_t u32Buffer)
{
    /* This is low level read function of USB Mass Storage */
    uint32_t u32AlignAddr, u32Offset, *pu32;
    uint32_t i;

    /* Modify the address to MASS_STORAGE_OFFSET */
    u32Addr += MASS_STORAGE_OFFSET;

    /* Get address base on page size alignment */
    u32AlignAddr = u32Addr & (uint32_t)(~(FLASH_PAGE_SIZE - 1));

    /* Get the sector offset*/
    u32Offset = (u32Addr & (FLASH_PAGE_SIZE - 1));

    pu32 = (uint32_t *)u32Buffer;

    for(i = 0; i < u32Size / 4; i++)
    {
        /* Read from cache */
        if(((u32AlignAddr + i * 4) >= s_u32Tag) && ((u32AlignAddr + i * 4) < s_u32Tag + FLASH_PAGE_SIZE))
        {
            u32Offset = (u32Addr + i * 4 - s_u32Tag) / 4;
            pu32[i] = s_au32SectorBuf[u32Offset];
        }
        else
        {
            /* Read from flash */
            pu32[i] = M32(u32Addr + i * 4);
        }
    }
}

void DataFlashWrite(uint32_t u32Addr, uint32_t u32Size, uint32_t u32Buffer)
{
    /* This is low level write function of USB Mass Storage */
    uint32_t u32Len, i, u32Offset;
    uint32_t *pu32;
    uint32_t u32AlignAddr;

    /* Modify the address to MASS_STORAGE_OFFSET */
    u32Addr += MASS_STORAGE_OFFSET;

    u32Len = u32Size;

    do
    {
        /* Get address base on page size alignment */
        u32AlignAddr = u32Addr & (uint32_t)(~(FLASH_PAGE_SIZE - 1));

        /* Get the sector offset*/
        u32Offset = (u32Addr & (FLASH_PAGE_SIZE - 1));

        /* check cache buffer */
        if(s_u32Tag != u32AlignAddr)
        {
            if(s_u32Tag != (uint32_t) - 1)
            {
                /* We need to flush out cache before update it */
                FMC_Erase(s_u32Tag);

                for(i = 0; i < FLASH_PAGE_SIZE / 4; i++)
                {
                    FMC_Write(s_u32Tag + i * 4, s_au32SectorBuf[i]);
                }
            }

            /* Load data to cache buffer */
            for(i = 0; i < FLASH_PAGE_SIZE / 4; i++)
            {
                s_au32SectorBuf[i] = FMC_Read(u32AlignAddr + i * 4);
            }
            s_u32Tag = u32AlignAddr;
        }

        /* Source buffer */
        pu32 = (uint32_t *)u32Buffer;
        /* Get the update length */
        u32Len = FLASH_PAGE_SIZE - u32Offset;
        if(u32Size < u32Len)
            u32Len = u32Size;
        /* Update the destination buffer */
        for(i = 0; i < u32Len / 4; i++)
        {
            s_au32SectorBuf[u32Offset / 4 + i] = pu32[i];
        }

        u32Size -= u32Len;
        u32Addr += u32Len;
        u32Buffer += u32Len;
    }
    while(u32Size > 0);
}

