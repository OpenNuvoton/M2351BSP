/**************************************************************************//**
 * @file     NuBL33_main.c
 * @version  V1.00
 * @brief    Demonstrate NuBL33. (Non-secure code)
 *
 * @copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#include "NuBL_common.h"


/* Non-secure Callable function of NuBL32 */
extern void ShowCountersInNuBL32(uint32_t *in);
extern void BL32_OTA_Start(void);
extern void WdtResetCnt(void);
extern int32_t BL32_GetBL33FwVer(uint32_t * pu32FwVer);

typedef int32_t (*funcptr)(uint32_t);


extern int32_t Secure_func(void);

/*----------------------------------------------------------------------------
  NonSecure Callable Functions from Secure Region
 *----------------------------------------------------------------------------*/
extern int32_t Secure_LED_On_callback(int32_t (*)(uint32_t));
extern int32_t Secure_LED_Off_callback(int32_t (*)(uint32_t));
extern int32_t Secure_LED_On(uint32_t u32Num);
extern int32_t Secure_LED_Off(uint32_t u32Num);

int32_t NonSecure_LED_On(uint32_t u32Num);
int32_t NonSecure_LED_Off(uint32_t u32Num);
void LED_On(uint32_t u32Us);
void LED_Off(uint32_t u32Us);
void SysTick_Handler(void);

/*----------------------------------------------------------------------------
  NonSecure functions used for callbacks
 *----------------------------------------------------------------------------*/
int32_t NonSecure_LED_On(uint32_t u32Num)
{
    (void)u32Num;
    printf("Nonsecure LED On call by Secure\n");
    PC0 = 0;
    return 0;
}

int32_t NonSecure_LED_Off(uint32_t u32Num)
{
    (void)u32Num;
    printf("Nonsecure LED Off call by Secure\n");
    PC0 = 1;
    return 0;
}

/*----------------------------------------------------------------------------
  NonSecure LED control
 *----------------------------------------------------------------------------*/
void LED_On(uint32_t u32Us)
{
    (void)u32Us;
    printf("Nonsecure LED On\n");
    PC1 = 0;
}

void LED_Off(uint32_t u32Us)
{
    (void)u32Us;
    printf("Nonsecure LED Off\n");
    PC1 = 1;
}

/*---------------------------------------------------------------------------------------------------------*/
/* SysTick IRQ Handler                                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
void SysTick_Handler(void)
{
    static uint32_t ticks;
    static uint32_t u32Ticks;

    if(u32Ticks > 800) /* 8s*/
    {
        WdtResetCnt();
        u32Ticks = 0;
    }
    u32Ticks++;

    switch(ticks++)
    {
        case   0:
            LED_On(7u);
            break;
        case 400:
            Secure_LED_On(6u);
            break;
        case 600:
            LED_Off(7u);
            break;
        case 1000:
            Secure_LED_Off(6u);
            break;
        default:
            if(ticks > 1200)
            {
                ticks = 0;
            }
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    uint32_t    u32FwVer = 0;

    printf("\n");
    printf("+--------------------------------------------+\n");
    printf("|    M2351 NuBL33(Non-secure) Sample Code    |\n");
    printf("+--------------------------------------------+\n\n");
{
    extern const FW_INFO_T g_FWinfoInitial;
    uint32_t cfg = g_FWinfoInitial.mData.u32AuthCFGs;
    printf("\n[AuthCFG: 0x%08x]\n", cfg);
}
    
    if (BL32_GetBL33FwVer((uint32_t *)&u32FwVer) == 0)
        printf("NuBL33 Firmware Ver: 0x%08x\n\n", u32FwVer);
    else
        printf("NuBL33 Firmware Ver: N/A\n\n");

    /* Init PC for Nonsecure LED control */
    GPIO_SetMode(PC, BIT1 | BIT0, GPIO_MODE_OUTPUT);

    /* register NonSecure callbacks in Secure application */
    Secure_LED_On_callback(&NonSecure_LED_On);
    Secure_LED_Off_callback(&NonSecure_LED_Off);

    /* Generate Systick interrupt each 10 ms */
//    SystemCoreClockUpdate();
    SysTick_Config(SystemCoreClock / 100);

    BL32_OTA_Start();

    while(1) {}
}

/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/
