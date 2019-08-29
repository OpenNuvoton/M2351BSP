/***************************************************************************//**
 * @file     targetdev.c
 * @brief    ISP support function source file
 * @version  0x32
 * @date     14, June, 2017
 *
 * @note
 * Copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include "targetdev.h"
#include "isp_user.h"

uint32_t GetApromSize()
{
    /* The smallest of APROM size is 2K */
    uint32_t size = 0x800, data;
    int result;

    do
    {
        result = FMC_Read_User(size, &data);

        if (result < 0)
        {
            return size;
        }
        else
        {
            size *= 2;
        }
    }
    while (1);
}

/* Data Flash is shared with APROM. */
/* The size and start address are defined in CONFIG1.*/
void GetDataFlashInfo(uint32_t *addr, uint32_t *size)
{
    uint32_t uData;
    *size = 0;
    FMC_Read_User(CONFIG0, &uData);

    /* DFEN enable */
    if ((uData & 0x01) == 0)
    {
        FMC_Read_User(CONFIG1, &uData);

        /* Filter the reserved bits in CONFIG1 */
        uData &= 0x000FFFFF;

        /* avoid CONFIG1 value from error */
        if (uData > g_u32ApromSize || uData & (FMC_FLASH_PAGE_SIZE - 1))
        {
            uData = g_u32ApromSize;
        }

        *addr = uData;
        *size = g_u32ApromSize - uData;
    }
    else
    {
        *addr = g_u32ApromSize;
        *size = 0;
    }
}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
