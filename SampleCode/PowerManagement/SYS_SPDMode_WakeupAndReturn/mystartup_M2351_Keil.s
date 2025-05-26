/**************************************************************************//**
 * @file     startup_M2351_Keil.S
 * @version  V3.00
 * @brief    M2351 Series Startup Source File
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2024 Nuvoton Technology Corp. All rights reserved.
 ***************************************************************************
		
***/
/*
//-------- <<< Use Configuration Wizard in Context Menu >>> ------------------
*/


    .section .bss.STACK, "aw", %nobits
    .align 3

    .global __initial_sp
#ifndef Stack_Size
// <o> Stack Size (in Bytes) <0x0-0xFFFFFFFF:8>
    .equ    Stack_Size, 0x00000800
#endif
Stack_Mem:
    .space   Stack_Size
__initial_sp:


    .section .bss.HEAP, "aw", %nobits
    .align  3
    .global Heap_Mem
    .global __heap_base
    .global __heap_limit
#ifndef Heap_Size
// <o>  Heap Size (in Bytes) <0x0-0xFFFFFFFF:8>
    .equ    Heap_Size, 0x00000000
#endif
__heap_base:
Heap_Mem:
    .space  Heap_Size
__heap_limit:

    .eabi_attribute Tag_ABI_align_preserved, 1
    .thumb

//*** <<< end of configuration section >>>    ***


// ; Vector Table Mapped to Address 0 at Reset
    .section RESET, "ax"
    .global     __Vectors
    .global     __Vectors_End
    .global     __Vectors_Size


__Vectors:
    .word     __initial_sp              //; Top of Stack
    .word     Reset_Handler             //; Reset Handler
    .word     NMI_Handler               //; NMI Handler
    .word     HardFault_Handler         //; Hard Fault Handler
    .word     0                         //; Reserved
    .word     0                         //; Reserved
    .word     0                         //; Reserved
    .word     0                         //; Reserved
    .word     0                         //; Reserved
    .word     0                         //; Reserved
    .word     0                         //; Reserved
    .word     SVC_Handler               //; SVCall Handler
    .word     0                         //; Reserved
    .word     0                         //; Reserved
    .word     PendSV_Handler            //; PendSV Handler
    .word     SysTick_Handler           //; SysTick Handler

    //; External Interrupts
    //; maximum of 32 External Interrupts are 
    .word     BOD_IRQHandler		     //; 0
    .word     IRC_IRQHandler             //; 1
    .word     PWRWU_IRQHandler           //; 2
    .word     SRAM_IRQHandler            //; 3
    .word     CLKFAIL_IRQHandler         //; 4
    .word     DEFAULT_IRQHandler         //; 5
    .word     RTC_IRQHandler             //; 6
    .word     TAMPER_IRQHandler          //; 7
    .word     WDT_IRQHandler             //; 8
    .word     WWDT_IRQHandler            //; 9
    .word     EINT0_IRQHandler           //; 10
    .word     EINT1_IRQHandler           //; 11
    .word     EINT2_IRQHandler           //; 12
    .word     EINT3_IRQHandler           //; 13
    .word     EINT4_IRQHandler           //; 14
    .word     EINT5_IRQHandler           //; 15
    .word     GPA_IRQHandler             //; 16
    .word     GPB_IRQHandler             //; 17
    .word     GPC_IRQHandler             //; 18
    .word     GPD_IRQHandler             //; 19
    .word     GPE_IRQHandler             //; 20
    .word     GPF_IRQHandler             //; 21
    .word     QSPI0_IRQHandler           //; 22
    .word     SPI0_IRQHandler            //; 23
    .word     BRAKE0_IRQHandler          //; 24
    .word     EPWM0_P0_IRQHandler        //; 25
    .word     EPWM0_P1_IRQHandler        //; 26
    .word     EPWM0_P2_IRQHandler        //; 27
    .word     BRAKE1_IRQHandler          //; 28
    .word     EPWM1_P0_IRQHandler        //; 29
    .word     EPWM1_P1_IRQHandler        //; 30
    .word     EPWM1_P2_IRQHandler        //; 31
    .word     TMR0_IRQHandler            //; 32
    .word     TMR1_IRQHandler            //; 33
    .word     TMR2_IRQHandler            //; 34
    .word     TMR3_IRQHandler            //; 35
    .word     UART0_IRQHandler           //; 36
    .word     UART1_IRQHandler           //; 37
    .word     I2C0_IRQHandler            //; 38
    .word     I2C1_IRQHandler            //; 39
    .word     PDMA0_IRQHandler           //; 40
    .word     DAC_IRQHandler             //; 41
    .word     EADC0_IRQHandler           //; 42
    .word     EADC1_IRQHandler           //; 43
    .word     ACMP01_IRQHandler          //; 44
    .word     DEFAULT_IRQHandler         //; 45
    .word     EADC2_IRQHandler           //; 46
    .word     EADC3_IRQHandler           //; 47
    .word     UART2_IRQHandler           //; 48
    .word     UART3_IRQHandler           //; 49
    .word     DEFAULT_IRQHandler         //; 50
    .word     SPI1_IRQHandler            //; 51
    .word     SPI2_IRQHandler            //; 52
    .word     USBD_IRQHandler            //; 53
    .word     USBH_IRQHandler            //; 54
    .word     USBOTG_IRQHandler          //; 55
    .word     CAN0_IRQHandler            //; 56
    .word     DEFAULT_IRQHandler         //; 57
    .word     SC0_IRQHandler             //; 58
    .word     SC1_IRQHandler             //; 59
    .word     SC2_IRQHandler             //; 60
    .word     DEFAULT_IRQHandler         //; 61
    .word     SPI3_IRQHandler            //; 62
    .word     DEFAULT_IRQHandler         //; 63
    .word     SDH0_IRQHandler            //; 64
    .word     DEFAULT_IRQHandler         //; 65
    .word     DEFAULT_IRQHandler         //; 66
    .word     DEFAULT_IRQHandler         //; 67
    .word     I2S0_IRQHandler            //; 68
    .word     DEFAULT_IRQHandler         //; 69
    .word     OPA0_IRQHandler            //; 70
    .word     CRPT_IRQHandler            //; 71
    .word     GPG_IRQHandler             //; 72
    .word     EINT6_IRQHandler           //; 73
    .word     UART4_IRQHandler           //; 74
    .word     UART5_IRQHandler           //; 75
    .word     USCI0_IRQHandler           //; 76
    .word     USCI1_IRQHandler           //; 77
    .word     BPWM0_IRQHandler           //; 78
    .word     BPWM1_IRQHandler           //; 79
    .word     DEFAULT_IRQHandler         //; 80
    .word     DEFAULT_IRQHandler         //; 81
    .word     I2C2_IRQHandler            //; 82
    .word     DEFAULT_IRQHandler         //; 83
    .word     QEI0_IRQHandler            //; 84
    .word     QEI1_IRQHandler            //; 85
    .word     ECAP0_IRQHandler           //; 86
    .word     ECAP1_IRQHandler           //; 87
    .word     GPH_IRQHandler             //; 88
    .word     EINT7_IRQHandler           //; 89
    .word     DEFAULT_IRQHandler         //; 90
    .word     DEFAULT_IRQHandler         //; 91
    .word     DEFAULT_IRQHandler         //; 92
    .word     DEFAULT_IRQHandler         //; 93
    .word     DEFAULT_IRQHandler         //; 94
    .word     DEFAULT_IRQHandler         //; 95
    .word     DEFAULT_IRQHandler         //; 96
    .word     DEFAULT_IRQHandler         //; 97
    .word     PDMA1_IRQHandler           //; 98
    .word     SCU_IRQHandler             //; 99
    .word     DEFAULT_IRQHandler         //; 100
    .word     TRNG_IRQHandler            //; 101
					
					
                                                     
__Vectors_End:
    .equ    __Vectors_Size, __Vectors_End - __Vectors

    .section .text, "ax"


// ; Reset Handler

    .global Reset_Handler
    .global  SystemInit
    .global  __main
    .type   Reset_Handler, "function"
Reset_Handler:

                LDR     r0, =0x40000294 /* Check RTC wake-up from SPD flag */
                LDR     r0, [r0, #0]
                MOVS    r1, #4
                ANDS    r0, r0, r1
                BEQ     NORMAL
SPD:                                    /* Wake-up from SPD */
                SUB     sp, sp, #12
                POP     {PC}            /* Execute __SPD_Wakeup */

NORMAL:                                 /* Normal Power-on process */
                MOVS    r0, #0          /* Reserve 3 words stack space to retain data */
                PUSH    {r0}
                PUSH    {r0}
                PUSH    {r0}

                LDR     R0, =SystemInit
                BLX     R0
                LDR     R0, =__main
                BX      R0



// ; Dummy Exception Handlers (infinite loops which can be modified)
    .weak   NMI_Handler
    .type   NMI_Handler, "function"
NMI_Handler:
        B       .

    .weak   HardFault_Handler
    .type   HardFault_Handler, "function"
	
HardFault_Handler:
        MOV     R0, LR
        MRS     R1, MSP
        MRS     R2, PSP
        LDR     R3, =ProcessHardFault
        BLX     R3
        BX      R0

    .weak   SVC_Handler, "function"
SVC_Handler:
        B       .

    .weak   PendSV_Handler, "function"
PendSV_Handler:
        B       .

    .weak   SysTick_Handler, "function"
SysTick_Handler:
        B       .

    .weak  BOD_IRQHandler,"function"//; 0
    .weak  IRC_IRQHandler,"function"//; 1
    .weak  PWRWU_IRQHandler,"function"//; 2
    .weak  SRAM_IRQHandler,"function"//; 3
    .weak  CLKFAIL_IRQHandler,"function"//; 4
    //;.weak  0,"function" ; 5
    .weak  RTC_IRQHandler,"function"//; 6
    .weak  TAMPER_IRQHandler,"function"//; 7
    .weak  WDT_IRQHandler,"function"//; 8
    .weak  WWDT_IRQHandler,"function"//; 9
    .weak  EINT0_IRQHandler,"function"//; 10
    .weak  EINT1_IRQHandler,"function"//; 11
    .weak  EINT2_IRQHandler,"function"//; 12
    .weak  EINT3_IRQHandler,"function"//; 13
    .weak  EINT4_IRQHandler,"function"//; 14
    .weak  EINT5_IRQHandler,"function"//; 15
    .weak  GPA_IRQHandler,"function"//; 16
    .weak  GPB_IRQHandler,"function"//; 17
    .weak  GPC_IRQHandler,"function"//; 18
    .weak  GPD_IRQHandler,"function"//; 19
    .weak  GPE_IRQHandler,"function"//; 20
    .weak  GPF_IRQHandler,"function"//; 21
    .weak  QSPI0_IRQHandler,"function"//; 22
    .weak  SPI0_IRQHandler,"function"//; 23
    .weak  BRAKE0_IRQHandler,"function"//; 24
    .weak  EPWM0_P0_IRQHandler,"function"//; 25
    .weak  EPWM0_P1_IRQHandler,"function"//; 26
    .weak  EPWM0_P2_IRQHandler,"function"//; 27
    .weak  BRAKE1_IRQHandler,"function"//; 28
    .weak  EPWM1_P0_IRQHandler,"function"//; 29
    .weak  EPWM1_P1_IRQHandler,"function"//; 30
    .weak  EPWM1_P2_IRQHandler,"function"//; 31
    .weak  TMR0_IRQHandler,"function"//; 32
    .weak  TMR1_IRQHandler,"function"//; 33
    .weak  TMR2_IRQHandler,"function"//; 34
    .weak  TMR3_IRQHandler,"function"//; 35
    .weak  UART0_IRQHandler,"function"//; 36
    .weak  UART1_IRQHandler,"function"//; 37
    .weak  I2C0_IRQHandler,"function"//; 38
    .weak  I2C1_IRQHandler,"function"//; 39
    .weak  PDMA0_IRQHandler,"function"//; 40
    .weak  DAC_IRQHandler,"function"//; 41
    .weak  EADC0_IRQHandler,"function"//; 42
    .weak  EADC1_IRQHandler,"function"//; 43
    .weak  ACMP01_IRQHandler,"function"//; 44
    //;.weak  0,"function" ; 45
    .weak  EADC2_IRQHandler,"function"//; 46
    .weak  EADC3_IRQHandler,"function"//; 47
    .weak  UART2_IRQHandler,"function"//; 48
    .weak  UART3_IRQHandler,"function"//; 49
    //;.weak  0,"function" ; 50
    .weak  SPI1_IRQHandler,"function"//; 51
    .weak  SPI2_IRQHandler,"function"//; 52
    .weak  USBD_IRQHandler,"function"//; 53
    .weak  USBH_IRQHandler,"function"//; 54
    .weak  USBOTG_IRQHandler,"function"//; 55
    .weak  CAN0_IRQHandler,"function"//; 56
    .weak  CAN1_IRQHandler,"function"//; 57
    .weak  SC0_IRQHandler,"function"//; 58
    .weak  SC1_IRQHandler,"function"//; 59
    .weak  SC2_IRQHandler,"function"//; 60
    .weak  SC3_IRQHandler,"function"//; 61
    .weak  SPI3_IRQHandler,"function"//; 62
    //;.weak  0,"function"; 63
    .weak  SDH0_IRQHandler,"function"//; 64
    //;.weak  0,"function" ; 65
    //;.weak  0,"function" ; 66
    //;.weak  0,"function" ; 67
    .weak  I2S0_IRQHandler,"function"//; 68
    //;.weak  0,"function" ; 69
    .weak  OPA0_IRQHandler,"function"//; 70
    .weak  CRPT_IRQHandler,"function"//; 71
    .weak  GPG_IRQHandler,"function"//; 72
    .weak  EINT6_IRQHandler,"function"//; 73
    .weak  UART4_IRQHandler,"function"//; 74
    .weak  UART5_IRQHandler,"function"//; 75
    .weak  USCI0_IRQHandler,"function"//; 76
    .weak  USCI1_IRQHandler,"function"//; 77
    .weak  BPWM0_IRQHandler,"function"//; 78
    .weak  BPWM1_IRQHandler,"function"//; 79
    //;.weak  0,"function"; 80
    //;.weak  0,"function"; 81
    .weak  I2C2_IRQHandler,"function"//; 82
    //;.weak  0,"function"; 83
    .weak  QEI0_IRQHandler,"function"//; 84
    .weak  QEI1_IRQHandler,"function"//; 85
    .weak  ECAP0_IRQHandler,"function"//; 86
    .weak  ECAP1_IRQHandler,"function"//; 87
    .weak  GPH_IRQHandler,"function"//; 88
    .weak  EINT7_IRQHandler,"function"//; 89
    .weak  SDH1_IRQHandler,"function"//; 90
    //;.weak  0,"function"; 91
    //;.weak  USBH_IRQHandler,"function"; 92
    //;.weak  0,"function"; 93
    //;.weak  0,"function"; 94
    //;.weak  0,"function"; 95
    //;.weak  0,"function"; 96
    //;.weak  0,"function"; 97
    .weak  PDMA1_IRQHandler,"function"//; 98
    //;.weak  SCU_IRQHandler ,"function"; 99
    //;.weak  0,"function"; 100
    .weak  TRNG_IRQHandler,"function"//; 101


BOD_IRQHandler:		        //; 0
IRC_IRQHandler:             //; 1
PWRWU_IRQHandler:           //; 2
SRAM_IRQHandler:            //; 3
CLKFAIL_IRQHandler:         //; 4
//;0                          ; 5
RTC_IRQHandler:             //; 6
TAMPER_IRQHandler:         //; 7
WDT_IRQHandler:             //; 8
WWDT_IRQHandler:           //; 9
EINT0_IRQHandler:           //; 10
EINT1_IRQHandler:           //; 11
EINT2_IRQHandler:           //; 12
EINT3_IRQHandler:           //; 13
EINT4_IRQHandler:           //; 14
EINT5_IRQHandler:           //; 15
GPA_IRQHandler:           //; 16
GPB_IRQHandler:             //; 17
GPC_IRQHandler:             //; 18
GPD_IRQHandler:             //; 19
GPE_IRQHandler:             //; 20
GPF_IRQHandler:             //; 21
QSPI0_IRQHandler:           //; 22
SPI0_IRQHandler:            //; 23
BRAKE0_IRQHandler:          //; 24
EPWM0_P0_IRQHandler:        //; 25
EPWM0_P1_IRQHandler:        //; 26
EPWM0_P2_IRQHandler:        //; 27
BRAKE1_IRQHandler:          //; 28
EPWM1_P0_IRQHandler:        //; 29
EPWM1_P1_IRQHandler:        //; 30
EPWM1_P2_IRQHandler:        //; 31
TMR0_IRQHandler:            //; 32
TMR1_IRQHandler:            //; 33
TMR2_IRQHandler:            //; 34
TMR3_IRQHandler:            //; 35
UART0_IRQHandler:           //; 36
UART1_IRQHandler:           //; 37
I2C0_IRQHandler:            //; 38
I2C1_IRQHandler:            //; 39
PDMA0_IRQHandler:           //; 40
DAC_IRQHandler:             //; 41
EADC0_IRQHandler:           //; 42
EADC1_IRQHandler:           //; 43
ACMP01_IRQHandler:          //; 44
//;0                          ; 45
EADC2_IRQHandler:           //; 46
EADC3_IRQHandler:           //; 47
UART2_IRQHandler:           //; 48
UART3_IRQHandler:           //; 49
//;0                          ; 50
SPI1_IRQHandler:            //; 51
SPI2_IRQHandler:            //; 52
USBD_IRQHandler:            //; 53
//USBH_IRQHandler:            //; 54
USBOTG_IRQHandler:          //; 55
CAN0_IRQHandler:            //; 56
CAN1_IRQHandler:            //; 57
SC0_IRQHandler:             //; 58
SC1_IRQHandler:             //; 59
SC2_IRQHandler:             //; 60
SC3_IRQHandler:             //; 61
SPI3_IRQHandler:            //; 62
//;0                          ; 63
SDH0_IRQHandler:            //; 64
//;0                          ; 65
//;0                          ; 66
//;0                          ; 67
I2S0_IRQHandler:            //; 68
//;0                          ; 69
OPA0_IRQHandler:            //; 70
CRPT_IRQHandler:            //; 71
GPG_IRQHandler:             //; 72
EINT6_IRQHandler:           //; 73
UART4_IRQHandler:           //; 74
UART5_IRQHandler:           //; 75
USCI0_IRQHandler:           //; 76
USCI1_IRQHandler:           //; 77
BPWM0_IRQHandler:           //; 78
BPWM1_IRQHandler:           //; 79
//;0                          ; 80
//;0                          ; 81
I2C2_IRQHandler:            //; 82
//;0                          ; 83
QEI0_IRQHandler:            //; 84
QEI1_IRQHandler:            //; 85
ECAP0_IRQHandler:           //; 86
ECAP1_IRQHandler:           //; 87
GPH_IRQHandler:             //; 88
EINT7_IRQHandler:           //; 89
SDH1_IRQHandler:            //; 90
//;0                          ; 91
//;USBH_IRQHandler:            //; 92
//;0                          ; 93
//;0                          ; 94
//;0                          ; 95
//;0                          ; 96
//;0                          ; 97
PDMA1_IRQHandler:           //; 98
//;SCU_IRQHandler:            ; 99
//;0                         ; 100
TRNG_IRQHandler:            //; 101
DEFAULT_IRQHandler:
         B       .

    .global __PC
    .type   __PC, "function"
__PC: 
                MOV     r0, lr
                BLX     lr

    .global __Enter_SPD
    .type   __Enter_SPD, "function"
__Enter_SPD:                                /* Enter to PD */
                LDR     r0, =__SPD_Wakeup   /* Save SP, LR and __SPD_Wakeup */
                ADDS    r0, #1
                MOV     r1, lr
                MOV     r2, sp
                MOVS    r3, #0
                LDR     r3, [r3]
                MOV     sp, r3
                PUSH    {r0-r2}
                WFI
                POP     {PC}                /* Execute __SPD_Wakeup */
__SPD_Wakeup:                               /* Restore SP and LR */
                POP     {r1,r2}
                MOV     sp, r2
                BX      r1