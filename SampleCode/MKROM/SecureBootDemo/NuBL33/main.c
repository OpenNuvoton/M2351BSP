/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate how to generate the NuBL33 and can be authenticated by NuBL2.
 *
 * @copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"


/*----------------------------------------------------------------------------
  NonSecure Callable Functions from Secure Region
 *----------------------------------------------------------------------------*/
extern uint32_t GetSystemCoreClock(void); /* NuBL32 Non-secure callable function */


/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    printf("\n\nCPU @ %d Hz\n", GetSystemCoreClock());
    printf("+-------------------------------------------+\n");
    printf("|    SecureBootDemo - NuBL33 Sample Code    |\n");
    printf("+-------------------------------------------+\n\n");

    printf("System is executing in NuBL33.\n\n");
    
    while(1) {}
}

/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/
