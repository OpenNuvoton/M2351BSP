
#include "NuMicro.h"

#include "M2351TouchPanel.h"



int Init_TouchPanel(void)
{
    /* Enable peripheral clock */
    CLK_EnableModuleClock(EADC_MODULE);

    /* Peripheral clock source */
    CLK_SetModuleClock(EADC_MODULE, 0, CLK_CLKDIV0_EADC(8));

    /* Init ADC for TP */
    /* Set input mode as single-end and enable the A/D converter */
    EADC_Open(EADC, EADC_CTL_DIFFEN_SINGLE_END);

    return 1;
}

static volatile    uint32_t    g_u32AdcIntFlag_TP;

/*-----------------------------------------------*/
// ADC01 ISR
//
/*-----------------------------------------------*/
void EADC1_IRQHandler(void)
{
    /* Clear the A/D ADINT1 interrupt flag */
    EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF1_Msk);

    g_u32AdcIntFlag_TP = 1;

}

/*-----------------------------------------------*/
// Get X Position from Touch Panel (ADC input)
//
/*-----------------------------------------------*/
uint16_t Get_TP_X(void)
{
    uint16_t    x_adc_in;

    /*=== Get X from ADC input ===*/
    GPIO_SetMode(PB, BIT8, GPIO_MODE_OUTPUT);   // XR
    GPIO_SetMode(PB, BIT9, GPIO_MODE_INPUT);    // YD
    GPIO_SetMode(PB, BIT10, GPIO_MODE_OUTPUT);   // XL
    PB8 = 1;
    PB10 = 0;

    /* Configure the GPB11 ADC analog input pins.  */
    SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB11MFP_Msk | SYS_GPB_MFPH_PB8MFP_Msk);
    SYS->GPB_MFPH |= SYS_GPB_MFPH_PB11MFP_EADC0_CH11;

    /* Disable the GPB11 digital input path to avoid the leakage current. */
    GPIO_DISABLE_DIGITAL_PATH(PB, BIT11);

    /* Configure the sample module 1 for analog input channel 11 and software trigger source.*/
    EADC_ConfigSampleModule(EADC, 1, EADC_SOFTWARE_TRIGGER, 11); // YU

    /* Clear the A/D ADINT1 interrupt flag for safe */
    EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF1_Msk);

    /* Enable the sample module 1 interrupt.  */
    EADC_ENABLE_INT(EADC, BIT1);    //Enable sample module A/D ADINT1 interrupt.
    EADC_ENABLE_SAMPLE_MODULE_INT(EADC, 1, BIT1);    //Enable sample module 1 interrupt.
    NVIC_EnableIRQ(EADC1_IRQn);

    /* Reset the ADC interrupt indicator and trigger sample module 1 to start A/D conversion */
    g_u32AdcIntFlag_TP = 0;
    EADC_START_CONV(EADC, BIT1);

    /* Wait ADC interrupt (g_u32AdcIntFlag_TP will be set at IRQ_Handler function) */
    while(g_u32AdcIntFlag_TP == 0);
    x_adc_in = EADC_GET_CONV_DATA(EADC, 1);
    return x_adc_in;

}


/*-----------------------------------------------*/
// Get Y Position from Touch Panel (ADC input)
//
/*-----------------------------------------------*/
uint16_t Get_TP_Y(void)
{
    uint16_t    y_adc_in;

    /*=== Get Y from ADC input ===*/
    GPIO_SetMode(PB, BIT11, GPIO_MODE_OUTPUT);   // YU
    GPIO_SetMode(PB, BIT9, GPIO_MODE_OUTPUT);   // YD
    GPIO_SetMode(PB, BIT10, GPIO_MODE_INPUT);    // XL
    PB11 = 1;
    PB9 = 0;

    /* Configure the GPB8 ADC analog input pins.  */
    SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB11MFP_Msk | SYS_GPB_MFPH_PB8MFP_Msk);
    SYS->GPB_MFPH |= SYS_GPB_MFPH_PB8MFP_EADC0_CH8;

    /* Disable the GPB8 digital input path to avoid the leakage current. */
    GPIO_DISABLE_DIGITAL_PATH(PB, BIT8);

    /* Configure the sample module 2 for analog input channel 8 and software trigger source.*/
    EADC_ConfigSampleModule(EADC, 2, EADC_SOFTWARE_TRIGGER, 8); // XR

    /* Clear the A/D ADINT1 interrupt flag for safe */
    EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF1_Msk);

    /* Enable the sample module 2 interrupt.  */
    EADC_ENABLE_INT(EADC, BIT2);    //Enable sample module A/D ADINT1 interrupt.
    EADC_ENABLE_SAMPLE_MODULE_INT(EADC, 1, BIT2);    //Enable sample module 2 interrupt.
    NVIC_EnableIRQ(EADC1_IRQn);

    /* Reset the ADC interrupt indicator and trigger sample module 2 to start A/D conversion */
    g_u32AdcIntFlag_TP = 0;
    EADC_START_CONV(EADC, BIT2);

    /* Wait ADC interrupt (g_u32AdcIntFlag_TP will be set at IRQ_Handler function) */
    while(g_u32AdcIntFlag_TP == 0);
    y_adc_in = EADC_GET_CONV_DATA(EADC, 2);
    return y_adc_in;

}

int Read_TouchPanel(int *x, int *y)
{
    *x = Get_TP_X();
    *y = Get_TP_Y();
    if(((*x & 0x0F00) >= 0x0F00) || ((*y & 0x0F00) >= 0x0F00))
        return 0;
    else
        return 1;
}

int Uninit_TouchPanel(void)
{
    return 1;
}

int Check_TouchPanel(void)
{
    return 0;   //Pen up;
}

