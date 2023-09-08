/*  Arduino UNO R4 Minima demo code for core RA4M1 peripheral operations:
 *
 * NOTE: Pin allocation is as per R4 Minima, the R4 WiFi has many board-pins with different connections.
 *
 *  Susan Parker - 10th July 2023 - Start.
 *  1. Fast digital pin operation
 *  2. Split ADC operation for non-blocking ADC reads inside GPT timer interrupt
 *
 *  Working on Direct register setups of GPT3, GPT4 , and GPT7
 *
 *  Susan Parker - 19th July 2023.
 *  3. How to attach an interrupt to any RA4M1 peripheral
 *
 *  Susan Parker - 20th July 2023.
 *  4. Direct register setups of GPT3, GPT4 , and GPT7 for synchronised Phase Accurate PWM to buffered level shifting
 *
 *  Susan Parker - 21th July 2023.
 *  5. Add serial SCI2 setup for D0 and D1
 *  6. Start IIC functions << work in progress
 *
 *  Susan Parker - 22nd July 2023.
 *  7. Add DAC12
 *
 *  Susan Parker - 23nd July 2023 > 1st August.
 *  8. SPI1 and SPI0 - SPI1 as Master, SPI0 as Slave - 8-bit to 32-bit transfers
 *     Use of SPI interrupts 
 *     Change GPT3 to GPT1 for trangular phase-accurate PWM on pins D2 and D3
 *     Lots of tidying up and added comments
 *
 * The Minima USB Serial is used for diagnostics and demonstration - would not be used in practise for a FastPWM application.
 *
 * This code is "AS IS" without warranty or liability. 

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
*/

// RA4M1 User’s Manual: Hardware
// This doc has all the register discriptions I use:
// https://www.renesas.com/us/en/document/mah/renesas-ra4m1-group-users-manual-hardware

// Notes:
//  adc_val_A0 = analogRead(analogPin);  // blocking code takes between 22uS to 24uS to complete
//
//  For external aref - ADR4540 - Ultralow Noise, High Accuracy Voltage Reference
//    Using an aref gives c. +- 1 to 2 value in 14 bit reads on USB power
//    https://www.analog.com/media/en/technical-documentation/data-sheets/adr4520_4525_4530_4533_4540_4550.pdf
//


#include "Arduino.h"

/* The following c. 1200 lines of #defines for registers would normally go into a seperate .H file */

// ARM-developer - Accessing memory-mapped peripherals
// https://developer.arm.com/documentation/102618/0100

// ==== Clock Generation ====
#define SYSTEM 0x40010000 // ICU Base - See 13.2.6 page 233
#define SYSTEM_SCKDIVCR  ((volatile unsigned int *)(SYSTEM + 0xE020))  // System Clock Division Control Register
// SYSTEM_SCKDIVCR = 10010100 
//   PCKD[2:0] = 4; 1/16
//   PCKC[2:0] = 1; 1/2


// ==== Interrupt Control Unit ====

/* Default Arduino Startup - see section 13.3.2 for Event Number info
 0 33 USBFS_USBI
 1 34 USBFS_USBR
 2 31 USBFS_D0FIFO
 3 32 USBFS_D1FIFO
 4 1E AGT0_AGTI
 ...
*/

#define ICUBASE 0x40000000 // ICU Base - See 13.2.6 page 233
// 32 bits - 
#define IELSR 0x6300 // ICU Event Link Setting Register n
#define ICU_IELSR00 ((volatile unsigned int *)(ICUBASE + IELSR))            //
#define ICU_IELSR01 ((volatile unsigned int *)(ICUBASE + IELSR + ( 1 * 4))) // 
#define ICU_IELSR02 ((volatile unsigned int *)(ICUBASE + IELSR + ( 2 * 4))) // 
#define ICU_IELSR03 ((volatile unsigned int *)(ICUBASE + IELSR + ( 3 * 4))) // 
#define ICU_IELSR04 ((volatile unsigned int *)(ICUBASE + IELSR + ( 4 * 4))) // 
#define ICU_IELSR05 ((volatile unsigned int *)(ICUBASE + IELSR + ( 5 * 4))) // 
#define ICU_IELSR06 ((volatile unsigned int *)(ICUBASE + IELSR + ( 6 * 4))) // 
#define ICU_IELSR07 ((volatile unsigned int *)(ICUBASE + IELSR + ( 7 * 4))) // 
#define ICU_IELSR08 ((volatile unsigned int *)(ICUBASE + IELSR + ( 8 * 4))) // 
#define ICU_IELSR09 ((volatile unsigned int *)(ICUBASE + IELSR + ( 9 * 4))) // 
#define ICU_IELSR10 ((volatile unsigned int *)(ICUBASE + IELSR + (10 * 4))) // 
#define ICU_IELSR11 ((volatile unsigned int *)(ICUBASE + IELSR + (11 * 4))) // 
#define ICU_IELSR12 ((volatile unsigned int *)(ICUBASE + IELSR + (12 * 4))) // 
#define ICU_IELSR13 ((volatile unsigned int *)(ICUBASE + IELSR + (13 * 4))) // 
#define ICU_IELSR14 ((volatile unsigned int *)(ICUBASE + IELSR + (14 * 4))) // 
#define ICU_IELSR15 ((volatile unsigned int *)(ICUBASE + IELSR + (15 * 4))) // 
#define ICU_IELSR16 ((volatile unsigned int *)(ICUBASE + IELSR + (16 * 4))) // 
#define ICU_IELSR17 ((volatile unsigned int *)(ICUBASE + IELSR + (17 * 4))) // 
#define ICU_IELSR18 ((volatile unsigned int *)(ICUBASE + IELSR + (18 * 4))) // 
#define ICU_IELSR19 ((volatile unsigned int *)(ICUBASE + IELSR + (19 * 4))) // 
#define ICU_IELSR20 ((volatile unsigned int *)(ICUBASE + IELSR + (20 * 4))) // 
#define ICU_IELSR21 ((volatile unsigned int *)(ICUBASE + IELSR + (21 * 4))) // 
#define ICU_IELSR22 ((volatile unsigned int *)(ICUBASE + IELSR + (22 * 4))) // 
#define ICU_IELSR23 ((volatile unsigned int *)(ICUBASE + IELSR + (23 * 4))) // 
#define ICU_IELSR24 ((volatile unsigned int *)(ICUBASE + IELSR + (24 * 4))) // 
#define ICU_IELSR25 ((volatile unsigned int *)(ICUBASE + IELSR + (25 * 4))) // 
#define ICU_IELSR26 ((volatile unsigned int *)(ICUBASE + IELSR + (26 * 4))) // 
#define ICU_IELSR27 ((volatile unsigned int *)(ICUBASE + IELSR + (27 * 4))) // 
#define ICU_IELSR28 ((volatile unsigned int *)(ICUBASE + IELSR + (28 * 4))) // 
#define ICU_IELSR29 ((volatile unsigned int *)(ICUBASE + IELSR + (29 * 4))) // 
#define ICU_IELSR30 ((volatile unsigned int *)(ICUBASE + IELSR + (30 * 4))) // 
#define ICU_IELSR31 ((volatile unsigned int *)(ICUBASE + IELSR + (31 * 4))) // 

// IRQ Event Numbers
#define IRQ_NO_EVENT         0x00
#define IRQ_PORT_IRQ0        0x01
#define IRQ_PORT_IRQ1        0x02
#define IRQ_PORT_IRQ2        0x03
#define IRQ_PORT_IRQ3        0x04
#define IRQ_PORT_IRQ4        0x05
#define IRQ_PORT_IRQ5        0x06
#define IRQ_PORT_IRQ6        0x07
#define IRQ_PORT_IRQ7        0x08
#define IRQ_PORT_IRQ8        0x09
#define IRQ_PORT_IRQ9        0x0A
#define IRQ_PORT_IRQ10       0x0B
#define IRQ_PORT_IRQ11       0x0C
#define IRQ_PORT_IRQ12       0x0D
//      IRQ_PORT_UNUSED      0x0E
#define IRQ_PORT_IRQ14       0x0F
#define IRQ_PORT_IRQ15       0x10
#define IRQ_DMAC0_INT        0x11
#define IRQ_DMAC1_INT        0x12
#define IRQ_DMAC2_INT        0x13
#define IRQ_DMAC3_INT        0x14
#define IRQ_DTC_COMPLETE     0x15
//      IRQ_UNUSED           0x16
#define IRQ_ICU_SNZCANCEL    0x17
#define IRQ_FCU_FRDYI        0x18
#define IRQ_LVD_LVD1         0x19
#define IRQ_LVD_LVD2         0x1A
#define IRQ_VBATT_LVD        0x1B
#define IRQ_MOSC_STOP        0x1C
#define IRQ_SYSTEM_SNZREQ    0x1D
#define IRQ_AGT0_AGTI        0x1E
#define IRQ_AGT0_AGTCMAI     0x1F
#define IRQ_AGT0_AGTCMBI     0x20
#define IRQ_AGT1_AGTI        0x21
#define IRQ_AGT1_AGTCMAI     0x22
#define IRQ_AGT1_AGTCMBI     0x23
#define IRQ_IWDT_NMIUNDF     0x24
#define IRQ_WDT_NMIUNDF      0x25
#define IRQ_RTC_ALM          0x26
#define IRQ_RTC_PRD          0x27
#define IRQ_RTC_CUP          0x28
#define IRQ_ADC140_ADI       0x29
#define IRQ_ADC140_GBADI     0x2A
#define IRQ_ADC140_CMPAI     0x2B
#define IRQ_ADC140_CMPBI     0x2C
#define IRQ_ADC140_WCMPM     0x2D
#define IRQ_ADC140_WCMPUM    0x2E
#define IRQ_ACMP_LP0         0x2F
#define IRQ_ACMP_LP1         0x30
#define IRQ_USBFS_D0FIFO     0x31
#define IRQ_USBFS_D1FIFO     0x32
#define IRQ_USBFS_USBI       0x33
#define IRQ_USBFS_USBR       0x34
#define IRQ_IIC0_RXI         0x35
#define IRQ_IIC0_TXI         0x36
#define IRQ_IIC0_TEI         0x37
#define IRQ_IIC0_EEI         0x38
#define IRQ_IIC0_WUI         0x39
#define IRQ_IIC1_RXI         0x3A
#define IRQ_IIC1_TXI         0x3B
#define IRQ_IIC1_TEI         0x3C
#define IRQ_IIC1_EEI         0x3D
#define IRQ_SSIE0_SSITXI     0x3E
#define IRQ_SSIE0_SSIRXI     0x3F
//      IRQ_UNUSED           0x40
#define IRQ_SSIE0_SSIF       0x41
#define IRQ_CTSU_CTSUWR      0x42
#define IRQ_CTSU_CTSURD      0x43
#define IRQ_CTSU_CTSUFN      0x44
#define IRQ_KEY_INTKR        0x45
#define IRQ_DOC_DOPCI        0x46
#define IRQ_CAC_FERRI        0x47
#define IRQ_CAC_MENDI        0x48
#define IRQ_CAC_OVFI         0x49
#define IRQ_CAN0_ERS         0x4A
#define IRQ_CAN0_RXF         0x4B
#define IRQ_CAN0_TXF         0x4C
#define IRQ_CAN0_RXM         0x4D
#define IRQ_CAN0_TXM         0x4E
#define IRQ_IOPORT_GROUP1    0x4F
#define IRQ_IOPORT_GROUP2    0x50
#define IRQ_IOPORT_GROUP3    0x51
#define IRQ_IOPORT_GROUP4    0x52
#define IRQ_ELC_SWEVT0       0x53
#define IRQ_ELC_SWEVT1       0x54
#define IRQ_POEG_GROUP0      0x55
#define IRQ_POEG_GROUP1      0x56
#define IRQ_GPT0_CCMPA       0x57
#define IRQ_GPT0_CCMPB       0x58
#define IRQ_GPT0_CMPC        0x59
#define IRQ_GPT0_CMPD        0x5A
#define IRQ_GPT0_CMPE        0x5B
#define IRQ_GPT0_CMPF        0x5C
#define IRQ_GPT0_OVF         0x5D
#define IRQ_GPT0_UDF         0x5E
#define IRQ_GPT1_CCMPA       0x5F
#define IRQ_GPT1_CCMPB       0x60
#define IRQ_GPT1_CMPC        0x61
#define IRQ_GPT1_CMPD        0x62
#define IRQ_GPT1_CMPE        0x63
#define IRQ_GPT1_CMPF        0x64
#define IRQ_GPT1_OVF         0x65
#define IRQ_GPT1_UDF         0x66
#define IRQ_GPT2_CCMPA       0x67
#define IRQ_GPT2_CCMPB       0x68
#define IRQ_GPT2_CMPC        0x69
#define IRQ_GPT2_CMPD        0x6A
#define IRQ_GPT2_CMPE        0x6B
#define IRQ_GPT2_CMPF        0x6C
#define IRQ_GPT2_OVF         0x6D
#define IRQ_GPT2_UDF         0x6E
#define IRQ_GPT3_CCMPA       0x6F
#define IRQ_GPT3_CCMPB       0x70
#define IRQ_GPT3_CMPC        0x71
#define IRQ_GPT3_CMPD        0x72
#define IRQ_GPT3_CMPE        0x73
#define IRQ_GPT3_CMPF        0x74
#define IRQ_GPT3_OVF         0x75
#define IRQ_GPT3_UDF         0x76
#define IRQ_GPT4_CCMPA       0x77
#define IRQ_GPT4_CCMPB       0x78
#define IRQ_GPT4_CMPC        0x79
#define IRQ_GPT4_CMPD        0x7A
#define IRQ_GPT4_CMPE        0x7B
#define IRQ_GPT4_CMPF        0x7C
#define IRQ_GPT4_OVF         0x7D
#define IRQ_GPT4_UDF         0x7E
#define IRQ_GPT5_CCMPA       0x7F
#define IRQ_GPT5_CCMPB       0x80
#define IRQ_GPT5_CMPC        0x81
#define IRQ_GPT5_CMPD        0x82
#define IRQ_GPT5_CMPE        0x83
#define IRQ_GPT5_CMPF        0x84
#define IRQ_GPT5_OVF         0x85
#define IRQ_GPT5_UDF         0x86
#define IRQ_GPT6_CCMPA       0x87
#define IRQ_GPT6_CCMPB       0x88
#define IRQ_GPT6_CMPC        0x89
#define IRQ_GPT6_CMPD        0x8A
#define IRQ_GPT6_CMPE        0x8B
#define IRQ_GPT6_CMPF        0x8C
#define IRQ_GPT6_OVF         0x8D
#define IRQ_GPT6_UDF         0x8E
#define IRQ_GPT7_CCMPA       0x8F
#define IRQ_GPT7_CCMPB       0x90
#define IRQ_GPT7_CMPC        0x91
#define IRQ_GPT7_CMPD        0x92
#define IRQ_GPT7_CMPE        0x93
#define IRQ_GPT7_CMPF        0x94
#define IRQ_GPT7_OVF         0x95
#define IRQ_GPT7_UDF         0x96
#define IRQ_GPT_UVWEDGE      0x97
#define IRQ_SCI0_RXI         0x98
#define IRQ_SCI0_TXI         0x99
#define IRQ_SCI0_TEI         0x9A
#define IRQ_SCI0_ERI         0x9B
#define IRQ_SCI0_AM          0x9C
#define IRQ_SCI0_RXI_OR_ERI  0x9D
#define IRQ_SCI1_RXI         0x9E
#define IRQ_SCI1_TXI         0x9F
#define IRQ_SCI1_TEI         0xA0
#define IRQ_SCI1_ERI         0xA1
#define IRQ_SCI1_AM          0xA2
#define IRQ_SCI2_RXI         0xA3
#define IRQ_SCI2_TXI         0xA4
#define IRQ_SCI2_TEI         0xA5
#define IRQ_SCI2_ERI         0xA6
#define IRQ_SCI2_AM          0xA7
#define IRQ_SCI9_RXI         0xA8
#define IRQ_SCI9_TXI         0xA9
#define IRQ_SCI9_TEI         0xAA
#define IRQ_SCI9_ERI         0xAB
#define IRQ_SCI9_AM          0xAC
#define IRQ_SPI0_SPRI        0xAD
#define IRQ_SPI0_SPTI        0xAE
#define IRQ_SPI0_SPII        0xAF
#define IRQ_SPI0_SPEI        0xB0
#define IRQ_SPI0_SPTEND      0xB1
#define IRQ_SPI1_SPRI        0xB2
#define IRQ_SPI1_SPTI        0xB3
#define IRQ_SPI1_SPII        0xB4
#define IRQ_SPI1_SPEI        0xB5
#define IRQ_SPI1_SPTEND      0xB6


// ==== Event Link Controller ====
#define ELCBASE 0x40040000 // Event Link Controller
#define ELC_ELCR     ((volatile unsigned char  *)(ELCBASE + 0x1000))              // Event Link Controller Register
#define ELC_ELSEGR0  ((volatile unsigned char  *)(ELCBASE + 0x1002))              // Event Link Software Event Generation Register 0
#define ELC_ELSEGR1  ((volatile unsigned char  *)(ELCBASE + 0x1004))              // Event Link Software Event Generation Register 1
#define ELSR 0x1010        // Event Link Setting Registers
#define ELC_ELSR00   ((volatile unsigned short *)(ELCBASE + ELSR +( 0 * 4)))      // ELC_GPTA
#define ELC_ELSR01   ((volatile unsigned short *)(ELCBASE + ELSR +( 1 * 4)))      // ELC_GPTB
#define ELC_ELSR02   ((volatile unsigned short *)(ELCBASE + ELSR +( 2 * 4)))      // ELC_GPTC
#define ELC_ELSR03   ((volatile unsigned short *)(ELCBASE + ELSR +( 3 * 4)))      // ELC_GPTD
#define ELC_ELSR04   ((volatile unsigned short *)(ELCBASE + ELSR +( 4 * 4)))      // ELC_GPTE
#define ELC_ELSR05   ((volatile unsigned short *)(ELCBASE + ELSR +( 5 * 4)))      // ELC_GPTF
#define ELC_ELSR06   ((volatile unsigned short *)(ELCBASE + ELSR +( 6 * 4)))      // ELC_GPTG
#define ELC_ELSR07   ((volatile unsigned short *)(ELCBASE + ELSR +( 7 * 4)))      // ELC_GPTH
#define ELC_ELSR08   ((volatile unsigned short *)(ELCBASE + ELSR +( 8 * 4)))      // ELC_AD00 - ADC14A
#define ELC_ELSR09   ((volatile unsigned short *)(ELCBASE + ELSR +( 9 * 4)))      // ELC_AD01 - ADC14B
#define ELC_ELSR12   ((volatile unsigned short *)(ELCBASE + ELSR +(12 * 4)))      // ELC_DA0 - DAC12
#define ELC_ELSR14   ((volatile unsigned short *)(ELCBASE + ELSR +(14 * 4)))      // ELC_PORT1
#define ELC_ELSR15   ((volatile unsigned short *)(ELCBASE + ELSR +(15 * 4)))      // ELC_PORT2
#define ELC_ELSR16   ((volatile unsigned short *)(ELCBASE + ELSR +(16 * 4)))      // ELC_PORT3
#define ELC_ELSR17   ((volatile unsigned short *)(ELCBASE + ELSR +(17 * 4)))      // ELC_PORT4
#define ELC_ELSR18   ((volatile unsigned short *)(ELCBASE + ELSR +(18 * 4)))      // ELC_CTSU


// ==== Low Power Mode Control ====
#define SYSTEM 0x40010000 // System Registers
#define SYSTEM_SBYCR   ((volatile unsigned short *)(SYSTEM + 0xE00C))      // Standby Control Register
#define SYSTEM_MSTPCRA ((volatile unsigned int   *)(SYSTEM + 0xE01C))      // Module Stop Control Register A

#define MSTP 0x40040000 // Module Registers
#define MSTP_MSTPCRB   ((volatile unsigned int   *)(MSTP + 0x7000))      // Module Stop Control Register B
#define MSTPB2   2 // CAN0
#define MSTPB8   8 // IIC1
#define MSTPB9   9 // IIC0
#define MSTPB18 18 // SPI1
#define MSTPB19 19 // SPI0
#define MSTPB22 22 // SCI9
#define MSTPB29 29 // SCI2
#define MSTPB30 30 // SCI1
#define MSTPB31 31 // SCI0

#define MSTP_MSTPCRC   ((volatile unsigned int   *)(MSTP + 0x7004))      // Module Stop Control Register C
#define MSTP_MSTPCRD   ((volatile unsigned int   *)(MSTP + 0x7008))      // Module Stop Control Register D
#define MSTPD2   2 // AGT1   - Asynchronous General Purpose Timer 1 Module
#define MSTPD3   3 // AGT0   - Asynchronous General Purpose Timer 0 Module
#define MSTPD5   5 // GPT320 and GPT321 General 32 bit PWM Timer Module
#define MSTPD6   6 // GPT162 to GPT167 General 16 bit PWM Timer Module
#define MSTPD14 14 // POEG   - Port Output Enable for GPT Module Stop
#define MSTPD16 16 // ADC140 - 14-Bit A/D Converter Module
#define MSTPD19 19 // DAC8   -  8-Bit D/A Converter Module
#define MSTPD20 20 // DAC12  - 12-Bit D/A Converter Module
#define MSTPD29 29 // ACMPLP - Low-Power Analog Comparator Module
#define MSTPD31 31 // OPAMP  - Operational Amplifier Module

// These bits are read as 1, the write value should be 1.
// Bit value 0: Cancel the module-stop state 
// Bit value 1: Enter the module-stop state.


// ==== 14-Bit A/D Converter ====
#define ADCBASE 0x40050000 /* ADC Base */
                                                                           // N/C = Pin not connected; N/A = No pin for LQFP64 package
// 16 bit registers
#define ADC140_ADDR00 ((volatile unsigned short *)(ADCBASE + 0xC020))      // A1 (P000 AN00 AMP+)
#define ADC140_ADDR01 ((volatile unsigned short *)(ADCBASE + 0xC020 +  2)) // A2 (P001 AN01 AMP-) 
#define ADC140_ADDR02 ((volatile unsigned short *)(ADCBASE + 0xC020 +  4)) // A3 (P002 AN02 AMPO) 
#define ADC140_ADDR03 ((volatile unsigned short *)(ADCBASE + 0xC020 +  6)) // N/C
#define ADC140_ADDR04 ((volatile unsigned short *)(ADCBASE + 0xC020 +  8)) // N/C
#define ADC140_ADDR05 ((volatile unsigned short *)(ADCBASE + 0xC020 + 10)) // Aref (P010 AN05 VrefH0)
#define ADC140_ADDR06 ((volatile unsigned short *)(ADCBASE + 0xC020 + 12)) // N/C
#define ADC140_ADDR07 ((volatile unsigned short *)(ADCBASE + 0xC020 + 14)) // TxLED (P012 AN07)
#define ADC140_ADDR08 ((volatile unsigned short *)(ADCBASE + 0xC020 + 16)) // RxLED (P013 AN08)
#define ADC140_ADDR09 ((volatile unsigned short *)(ADCBASE + 0xC020 + 18)) // A0 (P014 AN09 DAC)
#define ADC140_ADDR10 ((volatile unsigned short *)(ADCBASE + 0xC020 + 20)) // N/C
#define ADC140_ADDR11 ((volatile unsigned short *)(ADCBASE + 0xC020 + 22)) // N/A 
#define ADC140_ADDR12 ((volatile unsigned short *)(ADCBASE + 0xC020 + 24)) // N/A 
#define ADC140_ADDR13 ((volatile unsigned short *)(ADCBASE + 0xC020 + 26)) // N/A 
#define ADC140_ADDR14 ((volatile unsigned short *)(ADCBASE + 0xC020 + 28)) // N/A 
#define ADC140_ADDR16 ((volatile unsigned short *)(ADCBASE + 0xC040))      // R10/R26 net (P500 AN16)
#define ADC140_ADDR17 ((volatile unsigned short *)(ADCBASE + 0xC040 +  2)) // SWD-pin8 SCI1_TxD (P501 AN17) 
#define ADC140_ADDR18 ((volatile unsigned short *)(ADCBASE + 0xC040 +  4)) // SWD-pin7 SCI1_RxD (P502 AN18) 
#define ADC140_ADDR19 ((volatile unsigned short *)(ADCBASE + 0xC040 +  6)) // D4 (P103 AN19)
#define ADC140_ADDR20 ((volatile unsigned short *)(ADCBASE + 0xC040 +  8)) // D5 (P102 AN20 ADtrg0)
#define ADC140_ADDR21 ((volatile unsigned short *)(ADCBASE + 0xC040 + 10)) // A4 (P101 AN21 SDA) 
#define ADC140_ADDR22 ((volatile unsigned short *)(ADCBASE + 0xC040 + 12)) // A5 (P100 AN20 SCL) 
#define ADC140_ADDR23 ((volatile unsigned short *)(ADCBASE + 0xC040 + 14)) // N/A
#define ADC140_ADDR24 ((volatile unsigned short *)(ADCBASE + 0xC040 + 16)) // N/A 
#define ADC140_ADDR25 ((volatile unsigned short *)(ADCBASE + 0xC040 + 18)) // N/A
#define ADC140_ADDBLDR  ((volatile unsigned short *)(ADCBASE + 0xC018))    // A/D Data Duplexing Register
#define ADC140_ADTSDR   ((volatile unsigned short *)(ADCBASE + 0xC01A))    // A/D conversion result of temperature sensor output
#define ADC140_ADOCDR   ((volatile unsigned short *)(ADCBASE + 0xC01C))    // A/D result of internal reference voltage
#define ADC140_ADDBLDRA ((volatile unsigned short *)(ADCBASE + 0xC084))    // A/D Data Duplexing Register A 
#define ADC140_ADDBLDRB ((volatile unsigned short *)(ADCBASE + 0xC086))    // A/D Data Duplexing Register B

#define ADC140_ADCSR      ((volatile unsigned short *)(ADCBASE + 0xC000)) // A/D Control Register
#define ADC140_ADANSA0    ((volatile unsigned short *)(ADCBASE + 0xC004)) // A/D Channel Select Register A0
#define ADC140_ADANSA1    ((volatile unsigned short *)(ADCBASE + 0xC006)) // A/D Channel Select Register A1
#define ADC140_ADADS0     ((volatile unsigned short *)(ADCBASE + 0xC008)) // A/D-Converted Value Addition/Average Channel Select Register 0
#define ADC140_ADADS1     ((volatile unsigned short *)(ADCBASE + 0xC00A)) // A/D-Converted Value Addition/Average Channel Select Register 1
#define ADC140_ADCER      ((volatile unsigned short *)(ADCBASE + 0xC00E)) // A/D Control Extended Register 
#define ADC140_ADSTRGR    ((volatile unsigned short *)(ADCBASE + 0xC010)) // A/D Conversion Start Trigger Select Register
#define ADC140_ADEXICR    ((volatile unsigned short *)(ADCBASE + 0xC012)) // A/D Conversion Extended Input Control Register
#define ADC140_ADANSB0    ((volatile unsigned short *)(ADCBASE + 0xC014)) // A/D Channel Select Register B0
#define ADC140_ADANSB1    ((volatile unsigned short *)(ADCBASE + 0xC016)) // A/D Channel Select Register B1
#define ADC140_ADRD       ((volatile unsigned short *)(ADCBASE + 0xC01E)) // A/D Self-Diagnosis Data Register
#define ADC140_ADGSPCR    ((volatile unsigned short *)(ADCBASE + 0xC080)) // A/D Group Scan Priority Control Register
#define ADC140_ADCMPCR    ((volatile unsigned short *)(ADCBASE + 0xC090)) // A/D Compare Function Control Register
#define ADC140_ADCMPANSR0 ((volatile unsigned short *)(ADCBASE + 0xC094)) // A/D Compare Function Window A Channel Select Register 0
#define ADC140_ADCMPANSR1 ((volatile unsigned short *)(ADCBASE + 0xC096)) // A/D Compare Function Window A Channel Select Register 1
#define ADC140_ADCMPLR0   ((volatile unsigned short *)(ADCBASE + 0xC098)) // A/D Compare Function Window A Comparison Condition Setting Register 0
#define ADC140_ADCMPLR1   ((volatile unsigned short *)(ADCBASE + 0xC09A)) // A/D Compare Function Window A Comparison Condition Setting Register 1
#define ADC140_ADCMPDR0   ((volatile unsigned short *)(ADCBASE + 0xC09C)) // A/D Compare Function Window A Lower-Side Level Setting Register
#define ADC140_ADCMPDR1   ((volatile unsigned short *)(ADCBASE + 0xC09E)) // A/D Compare Function Window A Upper-Side Level Setting Register
#define ADC140_ADWINLLB   ((volatile unsigned short *)(ADCBASE + 0xC0A8)) // A/D Compare Function Window B Lower-Side Level Setting Register
#define ADC140_ADWINULB   ((volatile unsigned short *)(ADCBASE + 0xC0AA)) // A/D Compare Function Window B Upper-Side Level Setting Register
#define ADC140_ADCMPSR0   ((volatile unsigned short *)(ADCBASE + 0xC0A0)) // A/D Compare Function Window A Channel Status Register 0
#define ADC140_ADCMPSR1   ((volatile unsigned short *)(ADCBASE + 0xC0A2)) // A/D Compare Function Window A Channel Status Register 1

// 8 bit registers
#define ADC140_ADADC      ((volatile unsigned char  *)(ADCBASE + 0xC00C)) // A/D-Converted Value Addition/Average Count Select Register
#define ADC140_ADDISCR    ((volatile unsigned char  *)(ADCBASE + 0xC07A)) // A/D Disconnection Detection Control Register
#define ADC140_ADHVREFCNT ((volatile unsigned char  *)(ADCBASE + 0xC08A)) // A/D High-Potential/Low-Potential Reference Voltage Control Register
#define ADC140_ADWINMON   ((volatile unsigned char  *)(ADCBASE + 0xC08C)) // A/D Compare Function Window A/B Status Monitor Register
#define ADC140_ADCMPANSER ((volatile unsigned char  *)(ADCBASE + 0xC092)) // A/D Compare Function Window A Extended Input Select Register
#define ADC140_ADCMPLER   ((volatile unsigned char  *)(ADCBASE + 0xC093)) // A/D Compare Function Window A Extended Input Comparison Condition Setting Register
#define ADC140_ADCMPSER   ((volatile unsigned char  *)(ADCBASE + 0xC0A4)) // A/D Compare Function Window A Extended Input Channel Status Register
#define ADC140_ADCMPBNSER ((volatile unsigned char  *)(ADCBASE + 0xC0A6)) // A/D Compare Function Window B Channel Select Register
#define ADC140_ADCMPBSR   ((volatile unsigned char  *)(ADCBASE + 0xC0AC)) // A/D Compare Function Window B Status Register

// 35.2.14 A/D Sampling State Register n (ADSSTRn) (n = 00 to 14, L, T, O)
#define ADC140_ADSSTR00 ((volatile unsigned char *)(ADCBASE + 0xC0E0))      // AN00
#define ADC140_ADSSTR01 ((volatile unsigned char *)(ADCBASE + 0xC0E0 +  1)) // AN01
#define ADC140_ADSSTR02 ((volatile unsigned char *)(ADCBASE + 0xC0E0 +  2)) // AN02
#define ADC140_ADSSTR03 ((volatile unsigned char *)(ADCBASE + 0xC0E0 +  3)) // AN03
#define ADC140_ADSSTR04 ((volatile unsigned char *)(ADCBASE + 0xC0E0 +  4)) // AN04
#define ADC140_ADSSTR05 ((volatile unsigned char *)(ADCBASE + 0xC0E0 +  5)) // AN05
#define ADC140_ADSSTR06 ((volatile unsigned char *)(ADCBASE + 0xC0E0 +  6)) // AN06
#define ADC140_ADSSTR07 ((volatile unsigned char *)(ADCBASE + 0xC0E0 +  7)) // AN07
#define ADC140_ADSSTR08 ((volatile unsigned char *)(ADCBASE + 0xC0E0 +  8)) // AN08
#define ADC140_ADSSTR09 ((volatile unsigned char *)(ADCBASE + 0xC0E0 +  9)) // AN09
#define ADC140_ADSSTR10 ((volatile unsigned char *)(ADCBASE + 0xC0E0 + 10)) // AN10
#define ADC140_ADSSTR11 ((volatile unsigned char *)(ADCBASE + 0xC0E0 + 11)) // AN11
#define ADC140_ADSSTR12 ((volatile unsigned char *)(ADCBASE + 0xC0E0 + 12)) // AN12
#define ADC140_ADSSTR13 ((volatile unsigned char *)(ADCBASE + 0xC0E0 + 13)) // AN13
#define ADC140_ADSSTR14 ((volatile unsigned char *)(ADCBASE + 0xC0E0 + 14)) // AN14
#define ADC140_ADSSTRL  ((volatile unsigned char *)(ADCBASE + 0xC0DD))      // AN16 to AN25 
#define ADC140_ADSSTRT  ((volatile unsigned char *)(ADCBASE + 0xC0DE))      // Temp  
#define ADC140_ADSSTRO  ((volatile unsigned char *)(ADCBASE + 0xC0DF))      // Iref

// ==== 12-Bit D/A Converter ====
#define DACBASE 0x40050000          // DAC Base - DAC output on A0 (P014 AN09 DAC)
#define DAC12_DADR0    ((volatile unsigned short *)(DACBASE + 0xE000))      // D/A Data Register 0 
#define DAC12_DACR     ((volatile unsigned char  *)(DACBASE + 0xE004))      // D/A Control Register
#define DAC12_DADPR    ((volatile unsigned char  *)(DACBASE + 0xE005))      // DADR0 Format Select Register
#define DAC12_DAADSCR  ((volatile unsigned char  *)(DACBASE + 0xE006))      // D/A A/D Synchronous Start Control Register
#define DAC12_DAVREFCR ((volatile unsigned char  *)(DACBASE + 0xE007))      // D/A VREF Control Register


// =========== Ports ============
#define PORTBASE 0x40040000 /* Port Base */

// 19.2.1 Port Control Registers, to control/access multiple pins with a single operation
// Not all options defined for each register, expand as needed

#define PCNTR1 0x0000  // Port n Control Register 1 - 32bit access
#define PORT0_PCNTR1 ((volatile unsigned int *)(PORTBASE + PCNTR1))              // 
#define PORT1_PCNTR1 ((volatile unsigned int *)(PORTBASE + PCNTR1 + (1 * 0x20))) // 
#define PORT2_PCNTR1 ((volatile unsigned int *)(PORTBASE + PCNTR1 + (2 * 0x20))) // 
#define PORT3_PCNTR1 ((volatile unsigned int *)(PORTBASE + PCNTR1 + (3 * 0x20))) // 
#define PORT4_PCNTR1 ((volatile unsigned int *)(PORTBASE + PCNTR1 + (4 * 0x20))) // 
#define PORT5_PCNTR1 ((volatile unsigned int *)(PORTBASE + PCNTR1 + (5 * 0x20))) // 

#define PODR 0x0000  // Port n Control Register 1 - Output Data
#define PORT0_PODR ((volatile unsigned short *)(PORTBASE + PODR))              // 
#define PORT1_PODR ((volatile unsigned short *)(PORTBASE + PODR + (1 * 0x20))) // 
#define PORT2_PODR ((volatile unsigned short *)(PORTBASE + PODR + (2 * 0x20))) // 
#define PORT3_PODR ((volatile unsigned short *)(PORTBASE + PODR + (3 * 0x20))) // 
#define PORT4_PODR ((volatile unsigned short *)(PORTBASE + PODR + (4 * 0x20))) // 
#define PORT5_PODR ((volatile unsigned short *)(PORTBASE + PODR + (5 * 0x20))) // 

#define PDR  0x0002  // Port n Control Register 1 - Pin Direction
#define PORT0_PDR ((volatile unsigned short *)(PORTBASE + PDR))              // 
#define PORT1_PDR ((volatile unsigned short *)(PORTBASE + PDR + (1 * 0x20))) // 
#define PORT2_PDR ((volatile unsigned short *)(PORTBASE + PDR + (2 * 0x20))) // 
#define PORT3_PDR ((volatile unsigned short *)(PORTBASE + PDR + (3 * 0x20))) // 
#define PORT4_PDR ((volatile unsigned short *)(PORTBASE + PDR + (4 * 0x20))) // 
#define PORT5_PDR ((volatile unsigned short *)(PORTBASE + PDR + (5 * 0x20))) // 

#define PIDR  0x0006  // Port n Control Register 2 - Pin State - For reading
#define PORT0_PIDR ((volatile unsigned short *)(PORTBASE + PIDR))              // 
#define PORT1_PIDR ((volatile unsigned short *)(PORTBASE + PIDR + (1 * 0x20))) // 
#define PORT2_PIDR ((volatile unsigned short *)(PORTBASE + PIDR + (2 * 0x20))) // 
#define PORT3_PIDR ((volatile unsigned short *)(PORTBASE + PIDR + (3 * 0x20))) // 
#define PORT4_PIDR ((volatile unsigned short *)(PORTBASE + PIDR + (4 * 0x20))) // 
#define PORT5_PIDR ((volatile unsigned short *)(PORTBASE + PIDR + (5 * 0x20))) // 

#define PORR 0x0008  // Port n Control Register 3 - Pin Reset (to Low Output)
#define PORT0_PORR ((volatile unsigned short *)(PORTBASE + PORR))              // 
#define PORT1_PORR ((volatile unsigned short *)(PORTBASE + PORR + (1 * 0x20))) // 
#define PORT2_PORR ((volatile unsigned short *)(PORTBASE + PORR + (2 * 0x20))) // 
#define PORT3_PORR ((volatile unsigned short *)(PORTBASE + PORR + (3 * 0x20))) // 
#define PORT4_PORR ((volatile unsigned short *)(PORTBASE + PORR + (4 * 0x20))) // 
#define PORT5_PORR ((volatile unsigned short *)(PORTBASE + PORR + (5 * 0x20))) // 

#define POSR  0x000A  // Port n Control Register 3 - Pin Set (to High Output)
#define PORT0_POSR ((volatile unsigned short *)(PORTBASE + POSR))              // 
#define PORT1_POSR ((volatile unsigned short *)(PORTBASE + POSR + (1 * 0x20))) // 
#define PORT2_POSR ((volatile unsigned short *)(PORTBASE + POSR + (2 * 0x20))) // 
#define PORT3_POSR ((volatile unsigned short *)(PORTBASE + POSR + (3 * 0x20))) // 
#define PORT4_POSR ((volatile unsigned short *)(PORTBASE + POSR + (4 * 0x20))) // 
#define PORT5_POSR ((volatile unsigned short *)(PORTBASE + POSR + (5 * 0x20))) // 

// 19.2.5 Port mn Pin Function Select Register (PmnPFS/PmnPFS_HA/PmnPFS_BY) (m = 0 to 9; n = 00 to 15)
// 32 bits - Use for setting pin functions to other than default pin I/O

// Note: These are the pin allocations for the R4 Minima - The WiFI has diferent pin assignments
// Note: Pin IRQxx on R4 Minima pins D0, D1, D2, D3, D8, D12, D13, D15, D16, D17, D18, D19 only

#define P000PFS 0x0800  // Port 0 Pin Function Select Register
#define PFS_P000PFS ((volatile unsigned int *)(PORTBASE + P000PFS))            // A1 / D15 - AN00 - AMP0+ - IRQ6
#define PFS_P001PFS ((volatile unsigned int *)(PORTBASE + P000PFS + ( 1 * 4))) // A2 / D16 - AN01 - AMP0- - IRQ7
#define PFS_P002PFS ((volatile unsigned int *)(PORTBASE + P000PFS + ( 2 * 4))) // A3 / D17 - AN03 - AMP0O - IRQ2
#define PFS_P003PFS ((volatile unsigned int *)(PORTBASE + P000PFS + ( 3 * 4))) // N/C - AN03
#define PFS_P004PFS ((volatile unsigned int *)(PORTBASE + P000PFS + ( 4 * 4))) // N/C - AN04 - IRQ3
#define PFS_P005PFS ((volatile unsigned int *)(PORTBASE + P000PFS + ( 5 * 4))) // N/A - AN11 - IRQ10
#define PFS_P006PFS ((volatile unsigned int *)(PORTBASE + P000PFS + ( 6 * 4))) // N/A - AN12
#define PFS_P007PFS ((volatile unsigned int *)(PORTBASE + P000PFS + ( 7 * 4))) // N/A - AN13
#define PFS_P008PFS ((volatile unsigned int *)(PORTBASE + P000PFS + ( 8 * 4))) // N/A - AN14
// #define PFS_P009PFS ((volatile unsigned int *)(PORTBASE + P000PFS + ( 9 * 4))) // Does not exist
#define PFS_P010PFS ((volatile unsigned int *)(PORTBASE + P000PFS + (10 * 4))) // N/A - AN05
#define PFS_P011PFS ((volatile unsigned int *)(PORTBASE + P000PFS + (11 * 4))) // N/C - AN06 - IRQ15
#define PFS_P012PFS ((volatile unsigned int *)(PORTBASE + P000PFS + (12 * 4))) // TxLED - AN07
#define PFS_P013PFS ((volatile unsigned int *)(PORTBASE + P000PFS + (13 * 4))) // RxLED - AN08
#define PFS_P014PFS ((volatile unsigned int *)(PORTBASE + P000PFS + (14 * 4))) // A0 / D14 - AN09 - DAC0
#define PFS_P015PFS ((volatile unsigned int *)(PORTBASE + P000PFS + (15 * 4))) // N/C - AN10 - IRQ7

#define P100PFS 0x0840  // Port 1 Pin Function Select Register
#define PFS_P100PFS ((volatile unsigned int *)(PORTBASE + P100PFS))            // A5 / D19 - MISOA - SCL1 - IRQ2
#define PFS_P101PFS ((volatile unsigned int *)(PORTBASE + P100PFS + ( 1 * 4))) // A4 / D18 - MOSIA - SDA1 - IRQ1
#define PFS_P102PFS ((volatile unsigned int *)(PORTBASE + P100PFS + ( 2 * 4))) // D5 - RSPCKA
#define PFS_P103PFS ((volatile unsigned int *)(PORTBASE + P100PFS + ( 3 * 4))) // D4 - SSLA0
#define PFS_P104PFS ((volatile unsigned int *)(PORTBASE + P100PFS + ( 4 * 4))) // D3 - IRQ1
#define PFS_P105PFS ((volatile unsigned int *)(PORTBASE + P100PFS + ( 5 * 4))) // D2 - IRQ0
#define PFS_P106PFS ((volatile unsigned int *)(PORTBASE + P100PFS + ( 6 * 4))) // D6
#define PFS_P107PFS ((volatile unsigned int *)(PORTBASE + P100PFS + ( 7 * 4))) // D7
#define PFS_P108PFS ((volatile unsigned int *)(PORTBASE + P100PFS + ( 8 * 4))) // SWD p2 SWDIO
#define PFS_P109PFS ((volatile unsigned int *)(PORTBASE + P100PFS + ( 9 * 4))) // D11 / MOSI
#define PFS_P110PFS ((volatile unsigned int *)(PORTBASE + P100PFS + (10 * 4))) // D12 / MISO - IRQ3
#define PFS_P111PFS ((volatile unsigned int *)(PORTBASE + P100PFS + (11 * 4))) // D13 / SCLK - IRQ4
#define PFS_P112PFS ((volatile unsigned int *)(PORTBASE + P100PFS + (12 * 4))) // D10 / CS
#define PFS_P113PFS ((volatile unsigned int *)(PORTBASE + P100PFS + (13 * 4))) // N/C
#define PFS_P114PFS ((volatile unsigned int *)(PORTBASE + P100PFS + (14 * 4))) // N/A
#define PFS_P115PFS ((volatile unsigned int *)(PORTBASE + P100PFS + (15 * 4))) // N/A

#define P200PFS 0x0880  // Port 2 Pin Function Select Register
#define PFS_P200PFS ((volatile unsigned int *)(PORTBASE + P200PFS))           // NMI
#define PFS_P201PFS ((volatile unsigned int *)(PORTBASE + P200PFS + (1 * 4))) // MD
#define PFS_P202PFS ((volatile unsigned int *)(PORTBASE + P200PFS + (2 * 4))) // N/A
#define PFS_P203PFS ((volatile unsigned int *)(PORTBASE + P200PFS + (3 * 4))) // N/A
#define PFS_P204PFS ((volatile unsigned int *)(PORTBASE + P200PFS + (4 * 4))) // LOVE (Heart Pad on underside of board)
#define PFS_P205PFS ((volatile unsigned int *)(PORTBASE + P200PFS + (5 * 4))) // N/C - IRQ1
#define PFS_P206PFS ((volatile unsigned int *)(PORTBASE + P200PFS + (6 * 4))) // N/C - IRQ0
// Pins P212, P213, P214, and P215 are Crystal functions, or N/C

#define P300PFS 0x08C0  // Port 3 Pin Function Select Register
#define PFS_P300PFS ((volatile unsigned int *)(PORTBASE + P300PFS))            // SWD p4 SWCLK
#define PFS_P301PFS ((volatile unsigned int *)(PORTBASE + P300PFS + ( 1 * 4))) // D0 / RxD - IRQ6
#define PFS_P302PFS ((volatile unsigned int *)(PORTBASE + P300PFS + ( 2 * 4))) // D1 / TxD - IRQ5
#define PFS_P303PFS ((volatile unsigned int *)(PORTBASE + P300PFS + ( 3 * 4))) // D9
#define PFS_P304PFS ((volatile unsigned int *)(PORTBASE + P300PFS + ( 4 * 4))) // D8  - IRQ9
#define PFS_P305PFS ((volatile unsigned int *)(PORTBASE + P300PFS + ( 5 * 4))) // N/C - IRQ8
#define PFS_P306PFS ((volatile unsigned int *)(PORTBASE + P300PFS + ( 6 * 4))) // N/C
#define PFS_P307PFS ((volatile unsigned int *)(PORTBASE + P300PFS + ( 7 * 4))) // N/C

#define P400PFS 0x0900  // Port 4 Pin Function Select Register
#define PFS_P400PFS ((volatile unsigned int *)(PORTBASE + P100PFS))            // N/C - IRQ0
#define PFS_P401PFS ((volatile unsigned int *)(PORTBASE + P100PFS + ( 1 * 4))) // N/C - IRQ5
#define PFS_P402PFS ((volatile unsigned int *)(PORTBASE + P100PFS + ( 2 * 4))) // N/C - IRQ4
#define PFS_P403PFS ((volatile unsigned int *)(PORTBASE + P100PFS + ( 3 * 4))) // N/A
#define PFS_P404PFS ((volatile unsigned int *)(PORTBASE + P100PFS + ( 4 * 4))) // N/A
#define PFS_P405PFS ((volatile unsigned int *)(PORTBASE + P100PFS + ( 5 * 4))) // N/A
#define PFS_P406PFS ((volatile unsigned int *)(PORTBASE + P100PFS + ( 6 * 4))) // N/A
#define PFS_P407PFS ((volatile unsigned int *)(PORTBASE + P100PFS + ( 7 * 4))) // ADTRG0 - R7, R8, and R9 divider.
#define PFS_P408PFS ((volatile unsigned int *)(PORTBASE + P100PFS + ( 8 * 4))) // N/C - IRQ7
#define PFS_P409PFS ((volatile unsigned int *)(PORTBASE + P100PFS + ( 9 * 4))) // N/C - IRQ6
#define PFS_P410PFS ((volatile unsigned int *)(PORTBASE + P100PFS + (10 * 4))) // N/C - IRQ5
#define PFS_P411PFS ((volatile unsigned int *)(PORTBASE + P100PFS + (11 * 4))) // N/C - IRQ4
#define PFS_P412PFS ((volatile unsigned int *)(PORTBASE + P100PFS + (12 * 4))) // N/A
#define PFS_P413PFS ((volatile unsigned int *)(PORTBASE + P100PFS + (13 * 4))) // N/A
#define PFS_P414PFS ((volatile unsigned int *)(PORTBASE + P100PFS + (14 * 4))) // N/A - IRQ9
#define PFS_P415PFS ((volatile unsigned int *)(PORTBASE + P100PFS + (15 * 4))) // N/A - IRQ8

#define P500PFS 0x0940  // Port 5 Pin Function Select Register
#define PFS_P500PFS ((volatile unsigned int *)(PORTBASE + P500PFS))           // AN016 in R10/R26 divider
#define PFS_P501PFS ((volatile unsigned int *)(PORTBASE + P500PFS + (1 * 4))) // SWD p8 TxD - IRQ11
#define PFS_P502PFS ((volatile unsigned int *)(PORTBASE + P500PFS + (2 * 4))) // SWD p7 RxD - IRQ12
#define PFS_P503PFS ((volatile unsigned int *)(PORTBASE + P500PFS + (3 * 4))) // N/A
#define PFS_P504PFS ((volatile unsigned int *)(PORTBASE + P500PFS + (4 * 4))) // N/A
#define PFS_P505PFS ((volatile unsigned int *)(PORTBASE + P500PFS + (5 * 4))) // N/A - IRQ14

#define PFS_PODR   0  // Pin Output Data     - 0: Low output; 1: High output
#define PFS_PIDR   1  // Pin Input State     - Read 0: Low level; 1: High level
#define PFS_PDR    2  // Pin Direction       - 0: Input (input pin); 1: Output (output pin)
#define PFS_PCR    4  // Pull-up Control     - 1: Enable internal pull-up 
#define PFS_NCODR  6  // N-Channel Open Drain Control - 1: NMOS open-drain output.
#define PFS_DSCR  10  // Port Drive Capability - 1: Middle drive; Default 0: Low drive
#define PFS_EOR   12  // Event on Rising     - 1: Detect rising edge  - Set EOR and EOF both to 1
#define PFS_EOF   13  // Event on Falling    - 1: Detect falling edge - ... for Detect both edges
#define PFS_ISEL  14  // IRQ Input Enable    - 1: Used as an IRQn input pin.
#define PFS_ASEL  15  // Analog Input Enable - 1: Used as an analog pin.
#define PFS_PMR   16  // Pin Mode Control    - 1: Used as an I/O port for peripheral functions
#define PFS_PSEL_4_0 24 // Peripheral Function Select

// 16 bit register access
#define PFS_P100PFS_HA ((volatile unsigned short *)(PORTBASE + 0x0842))
#define PFS_P115PFS_HA ((volatile unsigned short *)(PORTBASE + 0x0842 + (15 * 2))) 
#define PFS_P200PFS_HA ((volatile unsigned short *)(PORTBASE + 0x0882))
#define PFS_P206PFS_HA ((volatile unsigned short *)(PORTBASE + 0x0882 + (06 * 2))) //

// 8 bits - Used for fast pin set/clear operations of register lower byte
#define PFS_P000PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x0803))            // A1
#define PFS_P001PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x0803 + ( 1 * 4))) // A2
#define PFS_P002PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x0803 + ( 2 * 4))) // A3
#define PFS_P003PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x0803 + ( 3 * 4))) // N/C
#define PFS_P004PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x0803 + ( 4 * 4))) // N/C
#define PFS_P005PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x0803 + ( 5 * 4))) // N/A
#define PFS_P006PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x0803 + ( 6 * 4))) // N/A
#define PFS_P007PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x0803 + ( 7 * 4))) // N/A
#define PFS_P008PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x0803 + ( 8 * 4))) // N/A
// #define PFS_P009PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x0803 + ( 9 * 4))) // 
#define PFS_P010PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x0803 + (10 * 4))) // N/A
#define PFS_P011PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x0803 + (11 * 4))) // N/C - P58 - VREFL0 << WTF!!!
#define PFS_P012PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x0803 + (12 * 4))) // TxLED - VREFH
#define PFS_P013PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x0803 + (13 * 4))) // RxLED - VREFL
#define PFS_P014PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x0803 + (14 * 4))) // A0 - DAC0
#define PFS_P015PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x0803 + (15 * 4))) // N/C

#define PFS_P100PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x0843))            // A5
#define PFS_P101PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x0843 + ( 1 * 4))) // A4
#define PFS_P102PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x0843 + ( 2 * 4))) // D5
#define PFS_P103PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x0843 + ( 3 * 4))) // D4
#define PFS_P104PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x0843 + ( 4 * 4))) // D3
#define PFS_P105PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x0843 + ( 5 * 4))) // D2
#define PFS_P106PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x0843 + ( 6 * 4))) // D6
#define PFS_P107PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x0843 + ( 7 * 4))) // D7
#define PFS_P108PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x0843 + ( 8 * 4))) // SWDIO
#define PFS_P109PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x0843 + ( 9 * 4))) // D11 / MOSI
#define PFS_P110PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x0843 + (10 * 4))) // D12 / MISO
#define PFS_P111PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x0843 + (11 * 4))) // D13 / SCLK
#define PFS_P112PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x0843 + (12 * 4))) // D10 / CS
#define PFS_P113PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x0843 + (13 * 4))) // N/C
#define PFS_P114PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x0843 + (14 * 4))) // N/A
#define PFS_P115PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x0843 + (15 * 4))) // N/A

#define PFS_P200PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x0883))            // NMI input
#define PFS_P201PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x0883 + (01 * 4))) // MD   
#define PFS_P202PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x0883 + (02 * 4))) // N/A
#define PFS_P203PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x0883 + (03 * 4))) // N/A
#define PFS_P204PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x0883 + (04 * 4))) // LOVE (Heart Pad on underside of board)
#define PFS_P205PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x0883 + (05 * 4))) // N/C
#define PFS_P206PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x0883 + (06 * 4))) // N/C

#define PFS_P212PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x08B3))            // EXTAL
#define PFS_P213PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x08B3 + (01 * 4))) // XTAL
#define PFS_P214PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x08B3 + (02 * 4))) // N/C XCOUT (P214 IN)
#define PFS_P215PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x08B3 + (03 * 4))) // N/C XCIN  (P215 IN)

#define PFS_P300PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x08C3))            // SWCLK (P300)
#define PFS_P301PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x08C3 + (01 * 4))) // D0 / RxD (P301)
#define PFS_P302PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x08C3 + (02 * 4))) // D1 / TxD (P302) 
#define PFS_P303PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x08C3 + (03 * 4))) // D9
#define PFS_P304PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x08C3 + (04 * 4))) // D8
#define PFS_P305PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x08C3 + (05 * 4))) // N/C 
#define PFS_P306PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x08C3 + (06 * 4))) // N/C
#define PFS_P307PFS_BY ((volatile unsigned char  *)(PORTBASE + 0x08C3 + (07 * 4))) // N/C


// ==== General PWM Timer (GPT) ====
#define GPTBASE 0x40070000 /* PWM Base */

#define GTWP 0x8000  // General PWM Timer Write-Protection Register
#define GPT320_GTWP ((volatile unsigned int *)(GPTBASE + GTWP))           
#define GPT321_GTWP ((volatile unsigned int *)(GPTBASE + GTWP + 0x0100))
#define GPT162_GTWP ((volatile unsigned int *)(GPTBASE + GTWP + 0x0200))
#define GPT163_GTWP ((volatile unsigned int *)(GPTBASE + GTWP + 0x0300))
#define GPT164_GTWP ((volatile unsigned int *)(GPTBASE + GTWP + 0x0400))
#define GPT165_GTWP ((volatile unsigned int *)(GPTBASE + GTWP + 0x0500))
#define GPT166_GTWP ((volatile unsigned int *)(GPTBASE + GTWP + 0x0600))
#define GPT167_GTWP ((volatile unsigned int *)(GPTBASE + GTWP + 0x0700))

#define GTSTR 0x8004  // General PWM Timer Software Start Register
#define GPT320_GTSTR ((volatile unsigned int *)(GPTBASE + GTSTR))           
#define GPT321_GTSTR ((volatile unsigned int *)(GPTBASE + GTSTR + 0x0100))
#define GPT162_GTSTR ((volatile unsigned int *)(GPTBASE + GTSTR + 0x0200))
#define GPT163_GTSTR ((volatile unsigned int *)(GPTBASE + GTSTR + 0x0300))
#define GPT164_GTSTR ((volatile unsigned int *)(GPTBASE + GTSTR + 0x0400))
#define GPT165_GTSTR ((volatile unsigned int *)(GPTBASE + GTSTR + 0x0500))
#define GPT166_GTSTR ((volatile unsigned int *)(GPTBASE + GTSTR + 0x0600))
#define GPT167_GTSTR ((volatile unsigned int *)(GPTBASE + GTSTR + 0x0700))

#define GTSTP 0x8008  // General PWM Timer Software Stop Register
#define GPT320_GTSTP ((volatile unsigned int *)(GPTBASE + GTSTP))           
#define GPT321_GTSTP ((volatile unsigned int *)(GPTBASE + GTSTP + 0x0100))
#define GPT162_GTSTP ((volatile unsigned int *)(GPTBASE + GTSTP + 0x0200))
#define GPT163_GTSTP ((volatile unsigned int *)(GPTBASE + GTSTP + 0x0300))
#define GPT164_GTSTP ((volatile unsigned int *)(GPTBASE + GTSTP + 0x0400))
#define GPT165_GTSTP ((volatile unsigned int *)(GPTBASE + GTSTP + 0x0500))
#define GPT166_GTSTP ((volatile unsigned int *)(GPTBASE + GTSTP + 0x0600))
#define GPT167_GTSTP ((volatile unsigned int *)(GPTBASE + GTSTP + 0x0700))

#define GTCLR 0x800C  // General PWM Timer Software Clear Register
#define GPT320_GTCLR ((volatile unsigned int *)(GPTBASE + GTCLR))           
#define GPT321_GTCLR ((volatile unsigned int *)(GPTBASE + GTCLR + 0x0100))
#define GPT162_GTCLR ((volatile unsigned int *)(GPTBASE + GTCLR + 0x0200))
#define GPT163_GTCLR ((volatile unsigned int *)(GPTBASE + GTCLR + 0x0300))
#define GPT164_GTCLR ((volatile unsigned int *)(GPTBASE + GTCLR + 0x0400))
#define GPT165_GTCLR ((volatile unsigned int *)(GPTBASE + GTCLR + 0x0500))
#define GPT166_GTCLR ((volatile unsigned int *)(GPTBASE + GTCLR + 0x0600))
#define GPT167_GTCLR ((volatile unsigned int *)(GPTBASE + GTCLR + 0x0700))

#define GTSSR 0x8010  // General PWM Timer Start Source Select Register
#define GPT320_GTSSR ((volatile unsigned int *)(GPTBASE + GTSSR))           
#define GPT321_GTSSR ((volatile unsigned int *)(GPTBASE + GTSSR + 0x0100))
#define GPT162_GTSSR ((volatile unsigned int *)(GPTBASE + GTSSR + 0x0200))
#define GPT163_GTSSR ((volatile unsigned int *)(GPTBASE + GTSSR + 0x0300))
#define GPT164_GTSSR ((volatile unsigned int *)(GPTBASE + GTSSR + 0x0400))
#define GPT165_GTSSR ((volatile unsigned int *)(GPTBASE + GTSSR + 0x0500))
#define GPT166_GTSSR ((volatile unsigned int *)(GPTBASE + GTSSR + 0x0600))
#define GPT167_GTSSR ((volatile unsigned int *)(GPTBASE + GTSSR + 0x0700))

#define GTPSR 0x8014  // General PWM Timer Stop Source Select Register
#define GPT320_GTPSR ((volatile unsigned int *)(GPTBASE + GTPSR))           
#define GPT321_GTPSR ((volatile unsigned int *)(GPTBASE + GTPSR + 0x0100))
#define GPT162_GTPSR ((volatile unsigned int *)(GPTBASE + GTPSR + 0x0200))
#define GPT163_GTPSR ((volatile unsigned int *)(GPTBASE + GTPSR + 0x0300))
#define GPT164_GTPSR ((volatile unsigned int *)(GPTBASE + GTPSR + 0x0400))
#define GPT165_GTPSR ((volatile unsigned int *)(GPTBASE + GTPSR + 0x0500))
#define GPT166_GTPSR ((volatile unsigned int *)(GPTBASE + GTPSR + 0x0600))
#define GPT167_GTPSR ((volatile unsigned int *)(GPTBASE + GTPSR + 0x0700))

#define GTCSR 0x8018  // General PWM Timer Clear Source Select Register
#define GPT320_GTCSR ((volatile unsigned int *)(GPTBASE + GTCSR))           
#define GPT321_GTCSR ((volatile unsigned int *)(GPTBASE + GTCSR + 0x0100))
#define GPT162_GTCSR ((volatile unsigned int *)(GPTBASE + GTCSR + 0x0200))
#define GPT163_GTCSR ((volatile unsigned int *)(GPTBASE + GTCSR + 0x0300))
#define GPT164_GTCSR ((volatile unsigned int *)(GPTBASE + GTCSR + 0x0400))
#define GPT165_GTCSR ((volatile unsigned int *)(GPTBASE + GTCSR + 0x0500))
#define GPT166_GTCSR ((volatile unsigned int *)(GPTBASE + GTCSR + 0x0600))
#define GPT167_GTCSR ((volatile unsigned int *)(GPTBASE + GTCSR + 0x0700))

#define GTUPSR 0x801C  // General PWM Timer Up Count Source Select Register
#define GPT320_GTUPSR ((volatile unsigned int *)(GPTBASE + GTUPSR))           
#define GPT321_GTUPSR ((volatile unsigned int *)(GPTBASE + GTUPSR + 0x0100))
#define GPT162_GTUPSR ((volatile unsigned int *)(GPTBASE + GTUPSR + 0x0200))
#define GPT163_GTUPSR ((volatile unsigned int *)(GPTBASE + GTUPSR + 0x0300))
#define GPT164_GTUPSR ((volatile unsigned int *)(GPTBASE + GTUPSR + 0x0400))
#define GPT165_GTUPSR ((volatile unsigned int *)(GPTBASE + GTUPSR + 0x0500))
#define GPT166_GTUPSR ((volatile unsigned int *)(GPTBASE + GTUPSR + 0x0600))
#define GPT167_GTUPSR ((volatile unsigned int *)(GPTBASE + GTUPSR + 0x0700))

#define GTDNSR 0x8020  // General PWM Timer Down Count Source Select Register
#define GPT320_GTDNSR ((volatile unsigned int *)(GPTBASE + GTDNSR))           
#define GPT321_GTDNSR ((volatile unsigned int *)(GPTBASE + GTDNSR + 0x0100))
#define GPT162_GTDNSR ((volatile unsigned int *)(GPTBASE + GTDNSR + 0x0200))
#define GPT163_GTDNSR ((volatile unsigned int *)(GPTBASE + GTDNSR + 0x0300))
#define GPT164_GTDNSR ((volatile unsigned int *)(GPTBASE + GTDNSR + 0x0400))
#define GPT165_GTDNSR ((volatile unsigned int *)(GPTBASE + GTDNSR + 0x0500))
#define GPT166_GTDNSR ((volatile unsigned int *)(GPTBASE + GTDNSR + 0x0600))
#define GPT167_GTDNSR ((volatile unsigned int *)(GPTBASE + GTDNSR + 0x0700))

#define GTICASR 0x8024 // General PWM Timer Input Capture Source Select Register A 
#define GTICBSR 0x8028 // General PWM Timer Input Capture Source Select Register B 

#define GTCR 0x802C // General PWM Timer Control Register
#define GPT320_GTCR ((volatile unsigned int *)(GPTBASE + GTCR))           
#define GPT321_GTCR ((volatile unsigned int *)(GPTBASE + GTCR + 0x0100))
#define GPT162_GTCR ((volatile unsigned int *)(GPTBASE + GTCR + 0x0200))
#define GPT163_GTCR ((volatile unsigned int *)(GPTBASE + GTCR + 0x0300))
#define GPT164_GTCR ((volatile unsigned int *)(GPTBASE + GTCR + 0x0400))
#define GPT165_GTCR ((volatile unsigned int *)(GPTBASE + GTCR + 0x0500))
#define GPT166_GTCR ((volatile unsigned int *)(GPTBASE + GTCR + 0x0600))
#define GPT167_GTCR ((volatile unsigned int *)(GPTBASE + GTCR + 0x0700))

#define GTUDDTYC 0x8030 // General PWM Timer Count Direction and Duty Setting Register
#define GPT320_GTUDDTYC ((volatile unsigned int *)(GPTBASE + GTUDDTYC))           
#define GPT321_GTUDDTYC ((volatile unsigned int *)(GPTBASE + GTUDDTYC + 0x0100))
#define GPT162_GTUDDTYC ((volatile unsigned int *)(GPTBASE + GTUDDTYC + 0x0200))
#define GPT163_GTUDDTYC ((volatile unsigned int *)(GPTBASE + GTUDDTYC + 0x0300))
#define GPT164_GTUDDTYC ((volatile unsigned int *)(GPTBASE + GTUDDTYC + 0x0400))
#define GPT165_GTUDDTYC ((volatile unsigned int *)(GPTBASE + GTUDDTYC + 0x0500))
#define GPT166_GTUDDTYC ((volatile unsigned int *)(GPTBASE + GTUDDTYC + 0x0600))
#define GPT167_GTUDDTYC ((volatile unsigned int *)(GPTBASE + GTUDDTYC + 0x0700))

#define GTIOR 0x8034 // General PWM Timer I/O Control Register
#define GPT320_GTIOR ((volatile unsigned int *)(GPTBASE + GTIOR))           
#define GPT321_GTIOR ((volatile unsigned int *)(GPTBASE + GTIOR + 0x0100))
#define GPT162_GTIOR ((volatile unsigned int *)(GPTBASE + GTIOR + 0x0200))
#define GPT163_GTIOR ((volatile unsigned int *)(GPTBASE + GTIOR + 0x0300))
#define GPT164_GTIOR ((volatile unsigned int *)(GPTBASE + GTIOR + 0x0400))
#define GPT165_GTIOR ((volatile unsigned int *)(GPTBASE + GTIOR + 0x0500))
#define GPT166_GTIOR ((volatile unsigned int *)(GPTBASE + GTIOR + 0x0600))
#define GPT167_GTIOR ((volatile unsigned int *)(GPTBASE + GTIOR + 0x0700))

#define GTST 0x803C // General PWM Timer Status Register
#define GPT320_GTST ((volatile unsigned int *)(GPTBASE + GTST))           
#define GPT321_GTST ((volatile unsigned int *)(GPTBASE + GTST + 0x0100))
#define GPT162_GTST ((volatile unsigned int *)(GPTBASE + GTST + 0x0200))
#define GPT163_GTST ((volatile unsigned int *)(GPTBASE + GTST + 0x0300))
#define GPT164_GTST ((volatile unsigned int *)(GPTBASE + GTST + 0x0400))
#define GPT165_GTST ((volatile unsigned int *)(GPTBASE + GTST + 0x0500))
#define GPT166_GTST ((volatile unsigned int *)(GPTBASE + GTST + 0x0600))
#define GPT167_GTST ((volatile unsigned int *)(GPTBASE + GTST + 0x0700))

#define GTBER 0x8040 // General PWM Timer Buffer Enable Register
#define GPT320_GTBER ((volatile unsigned int *)(GPTBASE + GTBER))           
#define GPT321_GTBER ((volatile unsigned int *)(GPTBASE + GTBER + 0x0100))
#define GPT162_GTBER ((volatile unsigned int *)(GPTBASE + GTBER + 0x0200))
#define GPT163_GTBER ((volatile unsigned int *)(GPTBASE + GTBER + 0x0300))
#define GPT164_GTBER ((volatile unsigned int *)(GPTBASE + GTBER + 0x0400))
#define GPT165_GTBER ((volatile unsigned int *)(GPTBASE + GTBER + 0x0500))
#define GPT166_GTBER ((volatile unsigned int *)(GPTBASE + GTBER + 0x0600))
#define GPT167_GTBER ((volatile unsigned int *)(GPTBASE + GTBER + 0x0700))

// Note: GTCNT can only be written to after the counting stops
#define GTCNT 0x8048 // General PWM Timer Counter
#define GPT320_GTCNT ((volatile unsigned int *)(GPTBASE + GTCNT))           
#define GPT321_GTCNT ((volatile unsigned int *)(GPTBASE + GTCNT + 0x0100))
#define GPT162_GTCNT ((volatile unsigned int *)(GPTBASE + GTCNT + 0x0200))
#define GPT163_GTCNT ((volatile unsigned int *)(GPTBASE + GTCNT + 0x0300))
#define GPT164_GTCNT ((volatile unsigned int *)(GPTBASE + GTCNT + 0x0400))
#define GPT165_GTCNT ((volatile unsigned int *)(GPTBASE + GTCNT + 0x0500))
#define GPT166_GTCNT ((volatile unsigned int *)(GPTBASE + GTCNT + 0x0600))
#define GPT167_GTCNT ((volatile unsigned int *)(GPTBASE + GTCNT + 0x0700))

#define GTCCRA 0x804C // General PWM Timer Compare Capture Register A
#define GPT320_GTCCRA ((volatile unsigned int *)(GPTBASE + GTCCRA))           
#define GPT321_GTCCRA ((volatile unsigned int *)(GPTBASE + GTCCRA + 0x0100))
#define GPT162_GTCCRA ((volatile unsigned int *)(GPTBASE + GTCCRA + 0x0200))
#define GPT163_GTCCRA ((volatile unsigned int *)(GPTBASE + GTCCRA + 0x0300))
#define GPT164_GTCCRA ((volatile unsigned int *)(GPTBASE + GTCCRA + 0x0400))
#define GPT165_GTCCRA ((volatile unsigned int *)(GPTBASE + GTCCRA + 0x0500))
#define GPT166_GTCCRA ((volatile unsigned int *)(GPTBASE + GTCCRA + 0x0600))
#define GPT167_GTCCRA ((volatile unsigned int *)(GPTBASE + GTCCRA + 0x0700))

#define GTCCRB 0x8050 // General PWM Timer Compare Capture Register B
#define GPT320_GTCCRB ((volatile unsigned int *)(GPTBASE + GTCCRB))           
#define GPT321_GTCCRB ((volatile unsigned int *)(GPTBASE + GTCCRB + 0x0100))
#define GPT162_GTCCRB ((volatile unsigned int *)(GPTBASE + GTCCRB + 0x0200))
#define GPT163_GTCCRB ((volatile unsigned int *)(GPTBASE + GTCCRB + 0x0300))
#define GPT164_GTCCRB ((volatile unsigned int *)(GPTBASE + GTCCRB + 0x0400))
#define GPT165_GTCCRB ((volatile unsigned int *)(GPTBASE + GTCCRB + 0x0500))
#define GPT166_GTCCRB ((volatile unsigned int *)(GPTBASE + GTCCRB + 0x0600))
#define GPT167_GTCCRB ((volatile unsigned int *)(GPTBASE + GTCCRB + 0x0700))

#define GTCCRC 0x8054 // General PWM Timer Compare Capture Register C
#define GPT320_GTCCRC ((volatile unsigned int *)(GPTBASE + GTCCRC))           
#define GPT321_GTCCRC ((volatile unsigned int *)(GPTBASE + GTCCRC + 0x0100))
#define GPT162_GTCCRC ((volatile unsigned int *)(GPTBASE + GTCCRC + 0x0200))
#define GPT163_GTCCRC ((volatile unsigned int *)(GPTBASE + GTCCRC + 0x0300))
#define GPT164_GTCCRC ((volatile unsigned int *)(GPTBASE + GTCCRC + 0x0400))
#define GPT165_GTCCRC ((volatile unsigned int *)(GPTBASE + GTCCRC + 0x0500))
#define GPT166_GTCCRC ((volatile unsigned int *)(GPTBASE + GTCCRC + 0x0600))
#define GPT167_GTCCRC ((volatile unsigned int *)(GPTBASE + GTCCRC + 0x0700))

//#define GTCCRD 0x805C // General PWM Timer Compare Capture Register D  - See page 425 22.2.19
#define GTCCRD 0x8058 // General PWM Timer Compare Capture Register D
#define GPT320_GTCCRD ((volatile unsigned int *)(GPTBASE + GTCCRD))           
#define GPT321_GTCCRD ((volatile unsigned int *)(GPTBASE + GTCCRD + 0x0100))
#define GPT162_GTCCRD ((volatile unsigned int *)(GPTBASE + GTCCRD + 0x0200))
#define GPT163_GTCCRD ((volatile unsigned int *)(GPTBASE + GTCCRD + 0x0300))
#define GPT164_GTCCRD ((volatile unsigned int *)(GPTBASE + GTCCRD + 0x0400))
#define GPT165_GTCCRD ((volatile unsigned int *)(GPTBASE + GTCCRD + 0x0500))
#define GPT166_GTCCRD ((volatile unsigned int *)(GPTBASE + GTCCRD + 0x0600))
#define GPT167_GTCCRD ((volatile unsigned int *)(GPTBASE + GTCCRD + 0x0700))

//#define GTCCRE 0x8058 // General PWM Timer Compare Capture Register E  - As above...
#define GTCCRE 0x805C // General PWM Timer Compare Capture Register E
#define GPT320_GTCCRE ((volatile unsigned int *)(GPTBASE + GTCCRE))           
#define GPT321_GTCCRE ((volatile unsigned int *)(GPTBASE + GTCCRE + 0x0100))
#define GPT162_GTCCRE ((volatile unsigned int *)(GPTBASE + GTCCRE + 0x0200))
#define GPT163_GTCCRE ((volatile unsigned int *)(GPTBASE + GTCCRE + 0x0300))
#define GPT164_GTCCRE ((volatile unsigned int *)(GPTBASE + GTCCRE + 0x0400))
#define GPT165_GTCCRE ((volatile unsigned int *)(GPTBASE + GTCCRE + 0x0500))
#define GPT166_GTCCRE ((volatile unsigned int *)(GPTBASE + GTCCRE + 0x0600))
#define GPT167_GTCCRE ((volatile unsigned int *)(GPTBASE + GTCCRE + 0x0700))

#define GTCCRF 0x8060 // General PWM Timer Compare Capture Register F
#define GPT320_GTCCRF ((volatile unsigned int *)(GPTBASE + GTCCRF))           
#define GPT321_GTCCRF ((volatile unsigned int *)(GPTBASE + GTCCRF + 0x0100))
#define GPT162_GTCCRF ((volatile unsigned int *)(GPTBASE + GTCCRF + 0x0200))
#define GPT163_GTCCRF ((volatile unsigned int *)(GPTBASE + GTCCRF + 0x0300))
#define GPT164_GTCCRF ((volatile unsigned int *)(GPTBASE + GTCCRF + 0x0400))
#define GPT165_GTCCRF ((volatile unsigned int *)(GPTBASE + GTCCRF + 0x0500))
#define GPT166_GTCCRF ((volatile unsigned int *)(GPTBASE + GTCCRF + 0x0600))
#define GPT167_GTCCRF ((volatile unsigned int *)(GPTBASE + GTCCRF + 0x0700))

#define GTPR 0x8064 // General PWM Timer Cycle Setting Register
#define GPT320_GTPR ((volatile unsigned int *)(GPTBASE + GTPR))           
#define GPT321_GTPR ((volatile unsigned int *)(GPTBASE + GTPR + 0x0100))
#define GPT162_GTPR ((volatile unsigned int *)(GPTBASE + GTPR + 0x0200))
#define GPT163_GTPR ((volatile unsigned int *)(GPTBASE + GTPR + 0x0300))
#define GPT164_GTPR ((volatile unsigned int *)(GPTBASE + GTPR + 0x0400))
#define GPT165_GTPR ((volatile unsigned int *)(GPTBASE + GTPR + 0x0500))
#define GPT166_GTPR ((volatile unsigned int *)(GPTBASE + GTPR + 0x0600))
#define GPT167_GTPR ((volatile unsigned int *)(GPTBASE + GTPR + 0x0700))

#define GTPBR 0x8068 // General PWM Timer Cycle Setting Buffer Register
#define GPT320_GTPBR ((volatile unsigned int *)(GPTBASE + GTPBR))           
#define GPT321_GTPBR ((volatile unsigned int *)(GPTBASE + GTPBR + 0x0100))
#define GPT162_GTPBR ((volatile unsigned int *)(GPTBASE + GTPBR + 0x0200))
#define GPT163_GTPBR ((volatile unsigned int *)(GPTBASE + GTPBR + 0x0300))
#define GPT164_GTPBR ((volatile unsigned int *)(GPTBASE + GTPBR + 0x0400))
#define GPT165_GTPBR ((volatile unsigned int *)(GPTBASE + GTPBR + 0x0500))
#define GPT166_GTPBR ((volatile unsigned int *)(GPTBASE + GTPBR + 0x0600))
#define GPT167_GTPBR ((volatile unsigned int *)(GPTBASE + GTPBR + 0x0700))

// ====  Asynchronous General Purpose Timer (AGT) =====
#define AGTBASE 0x40084000 
#define AGT0_AGT    ((volatile unsigned short *)(AGTBASE))         // AGT Counter Register
#define AGT1_AGT    ((volatile unsigned short *)(AGTBASE + 0x100))
#define AGT0_AGTCMA ((volatile unsigned short *)(AGTBASE + 0x002)) // AGT Compare Match A Register
#define AGT1_AGTCMA ((volatile unsigned short *)(AGTBASE + 0x102))
#define AGT0_AGTCMB ((volatile unsigned short *)(AGTBASE + 0x004)) // AGT Compare Match B Register
#define AGT1_AGTCMB ((volatile unsigned short *)(AGTBASE + 0x104))

// 8 bit registers
#define AGT0_AGTCR    ((volatile unsigned char  *)(AGTBASE + 0x008))  // AGT Control Register
#define AGT1_AGTCR    ((volatile unsigned char  *)(AGTBASE + 0x108))  // 
#define AGT0_AGTMR1   ((volatile unsigned char  *)(AGTBASE + 0x009))  // AGT Mode Register 1
#define AGT1_AGTMR1   ((volatile unsigned char  *)(AGTBASE + 0x109))  // 
#define AGT0_AGTMR2   ((volatile unsigned char  *)(AGTBASE + 0x00A))  // AGT Mode Register 2
#define AGT1_AGTMR2   ((volatile unsigned char  *)(AGTBASE + 0x10A))  // 
#define AGT0_AGTIOC   ((volatile unsigned char  *)(AGTBASE + 0x00C))  // AGT I/O Control Register
#define AGT1_AGTIOC   ((volatile unsigned char  *)(AGTBASE + 0x10C))  // 
#define AGT0_AGTISR   ((volatile unsigned char  *)(AGTBASE + 0x00D))  // AGT Event Pin Select Register
#define AGT1_AGTISR   ((volatile unsigned char  *)(AGTBASE + 0x10D))  // 
#define AGT0_AGTCMSR  ((volatile unsigned char  *)(AGTBASE + 0x00E))  // AGT Compare Match Function Select Register
#define AGT1_AGTCMSR  ((volatile unsigned char  *)(AGTBASE + 0x10E))  // 
#define AGT0_AGTIOSEL ((volatile unsigned char  *)(AGTBASE + 0x00F))  // AGT Pin Select Register
#define AGT1_AGTIOSEL ((volatile unsigned char  *)(AGTBASE + 0x10F))  // 


// ==== 28. Serial Communications Interface (SCI) ====
#define SCIBASE 0x40070000 
// Receive Data Register
#define SCI0_RDR ((volatile unsigned char  *)(SCIBASE + 0x0005)) 
#define SCI1_RDR ((volatile unsigned char  *)(SCIBASE + 0x0025)) 
#define SCI2_RDR ((volatile unsigned char  *)(SCIBASE + 0x0045)) 
#define SCI9_RDR ((volatile unsigned char  *)(SCIBASE + 0x0125))
// Receive 9-bit Data Register
#define SCI0_RDRHL ((volatile unsigned short *)(SCIBASE + 0x0010))
#define SCI1_RDRHL ((volatile unsigned short *)(SCIBASE + 0x0030))
#define SCI2_RDRHL ((volatile unsigned short *)(SCIBASE + 0x0050))
#define SCI9_RDRHL ((volatile unsigned short *)(SCIBASE + 0x0130))
// Receive FIFO Data Register H
#define SCI0_FRDRH ((volatile unsigned char *)(SCIBASE + 0x0010))
#define SCI1_FRDRH ((volatile unsigned char *)(SCIBASE + 0x0030))
// Receive FIFO Data Register L
#define SCI0_FRDRL ((volatile unsigned char *)(SCIBASE + 0x0011))
#define SCI1_FRDRL ((volatile unsigned char *)(SCIBASE + 0x0031))
// Receive FIFO Data Register HL
#define SCI0_FRDRHL ((volatile unsigned short *)(SCIBASE + 0x0010))
#define SCI1_FRDRHL ((volatile unsigned short *)(SCIBASE + 0x0030))
// Transmit Data Register
#define SCI0_TDR ((volatile unsigned char  *)(SCIBASE + 0x0003)) 
#define SCI1_TDR ((volatile unsigned char  *)(SCIBASE + 0x0023)) 
#define SCI2_TDR ((volatile unsigned char  *)(SCIBASE + 0x0043)) 
#define SCI9_TDR ((volatile unsigned char  *)(SCIBASE + 0x0123))
// Transmit 9-bit Data Register
#define SCI0_TDRHL ((volatile unsigned short *)(SCIBASE + 0x000E))
#define SCI1_TDRHL ((volatile unsigned short *)(SCIBASE + 0x002E))
#define SCI2_TDRHL ((volatile unsigned short *)(SCIBASE + 0x004E))
#define SCI9_TDRHL ((volatile unsigned short *)(SCIBASE + 0x012E))
// Transmit FIFO Data Register H
#define SCI0_FTDRH ((volatile unsigned char *)(SCIBASE + 0x000E))
#define SCI1_FTDRH ((volatile unsigned char *)(SCIBASE + 0x002E))
// Transmit FIFO Data Register L
#define SCI0_FTDRL ((volatile unsigned char *)(SCIBASE + 0x000F))
#define SCI1_FTDRL ((volatile unsigned char *)(SCIBASE + 0x002F))
// Transmit FIFO Data Register HL
#define SCI0_FTDRHL ((volatile unsigned short *)(SCIBASE + 0x000E))
#define SCI1_FRDRHL ((volatile unsigned short *)(SCIBASE + 0x002E))
// Serial Mode Register
#define SCI0_SMR ((volatile unsigned char  *)(SCIBASE + 0x0000)) 
#define SCI1_SMR ((volatile unsigned char  *)(SCIBASE + 0x0020)) 
#define SCI2_SMR ((volatile unsigned char  *)(SCIBASE + 0x0040)) 
#define SCI9_SMR ((volatile unsigned char  *)(SCIBASE + 0x0120))
#define SCI_CKS_1_0  0  // Clock Select
#define SCI_MP       2  // Multi-Processor Mode
#define SCI_STOP     3  // Stop Bit Length
#define SCI_PM       4  // Parity Mode
#define SCI_PE       5  // Parity Enable
#define SCI_CHR      6  // Character Length (default 8 bit), in combination with the CHR1 bit in SCMR
#define SCI_CM       7  // Communication Mode
// Serial Control Register
#define SCI0_SCR ((volatile unsigned char  *)(SCIBASE + 0x0002)) 
#define SCI1_SCR ((volatile unsigned char  *)(SCIBASE + 0x0022)) 
#define SCI2_SCR ((volatile unsigned char  *)(SCIBASE + 0x0042)) 
#define SCI9_SCR ((volatile unsigned char  *)(SCIBASE + 0x0122))
#define SCR_CKE_1_0  0  // Clock Enable
#define SCR_TEIE     2  // Transmit End Interrupt Enable
#define SCR_MPIE     3  // Multi-Processor Interrupt Enable
#define SCR_RE       4  // Receive Enable
#define SCR_TE       5  // Transmit Enable
#define SCR_RIE      6  // Receive Interrupt Enable
#define SCR_TIE      7  // Transmit Interrupt Enable
// Serial Status Register
#define SCI0_SSR ((volatile unsigned char  *)(SCIBASE + 0x0004)) 
#define SCI1_SSR ((volatile unsigned char  *)(SCIBASE + 0x0024)) 
#define SCI2_SSR ((volatile unsigned char  *)(SCIBASE + 0x0044)) 
#define SCI9_SSR ((volatile unsigned char  *)(SCIBASE + 0x0124))
#define SSR_TEND      2  // Transmit End Flag
#define SSR_RDRF      6  // Receive Data Full Flag
#define SSR_TDRE      7  // Transmit Data Empty Flag
// Serial Status Register FIFO
#define SCI0_SSR_FIFO ((volatile unsigned char  *)(SCIBASE + 0x0004)) 
#define SCI1_SSR_FIFO ((volatile unsigned char  *)(SCIBASE + 0x0024)) 
#define SSR_DR        0  // Receive Data Ready Flag
#define SSR_TEND      2  // Transmit End Flag
#define SSR_RDF       6  // Receive FIFO Data Full Flag
#define SSR_TDFE      7  // Transmit FIFO Data Empty Flag
// Smart Card Mode Register - and other settings
#define SCI0_SCMR ((volatile unsigned char  *)(SCIBASE + 0x0006)) 
#define SCI1_SCMR ((volatile unsigned char  *)(SCIBASE + 0x0026)) 
#define SCI2_SCMR ((volatile unsigned char  *)(SCIBASE + 0x0046)) 
#define SCI9_SCMR ((volatile unsigned char  *)(SCIBASE + 0x0126))
#define SCMR_SMIF     0  // Smart Card Interface Mode Select; 0: Non-smart card interface mode
#define SCMR_SINV     2  // Transmitted/Received Data Invert
#define SCMR_SDIR     3  // Transmitted/Received Data Transfer Direction
#define SCMR_CHR1     4  // Character Length 1
#define SCMR_BCP2     7  // Base Clock Pulse 2
// Bit Rate Register
#define SCI0_BRR ((volatile unsigned char  *)(SCIBASE + 0x0001)) 
#define SCI1_BRR ((volatile unsigned char  *)(SCIBASE + 0x0021)) 
#define SCI2_BRR ((volatile unsigned char  *)(SCIBASE + 0x0041)) 
#define SCI9_BRR ((volatile unsigned char  *)(SCIBASE + 0x0121))
// Modulation Duty Register
#define SCI0_MDDR ((volatile unsigned char  *)(SCIBASE + 0x0012)) 
#define SCI1_MDDR ((volatile unsigned char  *)(SCIBASE + 0x0032)) 
#define SCI2_MDDR ((volatile unsigned char  *)(SCIBASE + 0x0052)) 
#define SCI9_MDDR ((volatile unsigned char  *)(SCIBASE + 0x0132))
// Serial Extended Mode Register
#define SCI0_SEMR ((volatile unsigned char  *)(SCIBASE + 0x0007)) 
#define SCI1_SEMR ((volatile unsigned char  *)(SCIBASE + 0x0027)) 
#define SCI2_SEMR ((volatile unsigned char  *)(SCIBASE + 0x0047)) 
#define SCI9_SEMR ((volatile unsigned char  *)(SCIBASE + 0x0127))
#define SEMR_BRME     2  // Bit Rate Modulation Enable
#define SEMR_ABCSE    3  // Asynchronous Mode Extended Base Clock Select 1
#define SEMR_ABCS     4  // Asynchronous Mode Base Clock Select
#define SEMR_NFEN     5  // Digital Noise Filter Function Enable
#define SEMR_BGDM     6  // Baud Rate Generator Double-Speed Mode Select
#define SEMR_RXDESEL  7  // Asynchronous Start Bit Edge Detection Select
// Noise Filter Setting Register
#define SCI0_SNFR ((volatile unsigned char  *)(SCIBASE + 0x0008)) 
#define SCI1_SNFR ((volatile unsigned char  *)(SCIBASE + 0x0028)) 
#define SCI2_SNFR ((volatile unsigned char  *)(SCIBASE + 0x0048)) 
#define SCI9_SNFR ((volatile unsigned char  *)(SCIBASE + 0x0128))
// Line Status Register
#define SCI0_LSR ((volatile unsigned short *)(SCIBASE + 0x0018))
#define SCI1_LSR ((volatile unsigned short *)(SCIBASE + 0x0038))
// Serial Port Register
#define SCI0_SPTR ((volatile unsigned char  *)(SCIBASE + 0x001C)) 
#define SCI1_SPTR ((volatile unsigned char  *)(SCIBASE + 0x003C)) 
#define SCI2_SPTR ((volatile unsigned char  *)(SCIBASE + 0x005C)) 
#define SCI9_SPTR ((volatile unsigned char  *)(SCIBASE + 0x013C))



// ==== 29. I2C Bus Interface ====
#define IICBASE 0x40050000 
// I2C Bus Control Register 1
#define IIC0_ICCR1 ((volatile unsigned char  *)(IICBASE + 0x3000)) 
#define IIC1_ICCR1 ((volatile unsigned char  *)(IICBASE + 0x3100)) // 0x9F
// I2C Bus Control Register 2
#define IIC0_ICCR2 ((volatile unsigned char  *)(IICBASE + 0x3001)) 
#define IIC1_ICCR2 ((volatile unsigned char  *)(IICBASE + 0x3101)) // 0x00
// I2C Bus Mode Register 1
#define IIC0_ICMR1 ((volatile unsigned char  *)(IICBASE + 0x3002)) 
#define IIC1_ICMR1 ((volatile unsigned char  *)(IICBASE + 0x3102)) // 0x28
// I2C Bus Mode Register 2
#define IIC0_ICMR2 ((volatile unsigned char  *)(IICBASE + 0x3003)) 
#define IIC1_ICMR2 ((volatile unsigned char  *)(IICBASE + 0x3103)) // 0x05
// I2C Bus Mode Register 3
#define IIC0_ICMR3 ((volatile unsigned char  *)(IICBASE + 0x3004)) 
#define IIC1_ICMR3 ((volatile unsigned char  *)(IICBASE + 0x3104)) // 0x00
// I2C Bus Function Enable Register
#define IIC0_ICFER ((volatile unsigned char  *)(IICBASE + 0x3005)) 
#define IIC1_ICFER ((volatile unsigned char  *)(IICBASE + 0x3105)) // 0x77
// I2C Bus Status Enable Register
#define IIC0_ICSER ((volatile unsigned char  *)(IICBASE + 0x3006)) 
#define IIC1_ICSER ((volatile unsigned char  *)(IICBASE + 0x3106)) // 0x00
// I2C Bus Interupt Enable Register
#define IIC0_ICIER ((volatile unsigned char  *)(IICBASE + 0x3007)) 
#define IIC1_ICIER ((volatile unsigned char  *)(IICBASE + 0x3107)) // 0xB3 Interrupts
// I2C Bus Status Register 1
#define IIC0_ICSR1 ((volatile unsigned char  *)(IICBASE + 0x3008)) 
#define IIC1_ICSR1 ((volatile unsigned char  *)(IICBASE + 0x3108)) // 0x00
// I2C Bus Status Register 2
#define IIC0_ICSR2 ((volatile unsigned char  *)(IICBASE + 0x3009)) 
#define IIC1_ICSR2 ((volatile unsigned char  *)(IICBASE + 0x3109)) // 0x00
// ... 
// I2C Bus Bit Rate Low-Level Register
#define IIC0_ICBRL ((volatile unsigned char  *)(IICBASE + 0x3010)) 
#define IIC1_ICBRL ((volatile unsigned char  *)(IICBASE + 0x3110)) // 0xFB
// I2C Bus Bit Rate High-Level Register
#define IIC0_ICBRH ((volatile unsigned char  *)(IICBASE + 0x3011)) 
#define IIC1_ICBRH ((volatile unsigned char  *)(IICBASE + 0x3111)) // 0xFA
// ...
// I2C Bus Transmit Data Register
#define IIC0_ICDRT ((volatile unsigned char  *)(IICBASE + 0x3012)) 
#define IIC1_ICDRT ((volatile unsigned char  *)(IICBASE + 0x3112)) 
// I2C Bus Receive Data Register
#define IIC0_ICDRR ((volatile unsigned char  *)(IICBASE + 0x3013)) 
#define IIC1_ICDRR ((volatile unsigned char  *)(IICBASE + 0x3113)) 

// Slave Address Register L0-2
#define IIC0_SARL0 ((volatile unsigned char  *)(IICBASE + 0x300A)) 
#define IIC1_SARL0 ((volatile unsigned char  *)(IICBASE + 0x310A)) 
#define IIC0_SARL1 ((volatile unsigned char  *)(IICBASE + 0x300C)) 
#define IIC1_SARL1 ((volatile unsigned char  *)(IICBASE + 0x310C)) 
#define IIC0_SARL2 ((volatile unsigned char  *)(IICBASE + 0x300E)) 
#define IIC1_SARL2 ((volatile unsigned char  *)(IICBASE + 0x310E)) 
// Slave Address Register U0-2
#define IIC0_SARU0 ((volatile unsigned char  *)(IICBASE + 0x300B)) 
#define IIC1_SARU0 ((volatile unsigned char  *)(IICBASE + 0x310B)) 
#define IIC0_SARU1 ((volatile unsigned char  *)(IICBASE + 0x300D)) 
#define IIC1_SARU1 ((volatile unsigned char  *)(IICBASE + 0x310D)) 
#define IIC0_SARU2 ((volatile unsigned char  *)(IICBASE + 0x300F)) 
#define IIC1_SARU2 ((volatile unsigned char  *)(IICBASE + 0x310F)) 

// I2C Bus Wakeup Unit Register
#define IIC0_ICWUR  ((volatile unsigned char  *)(IICBASE + 0x3016)) 
// I2C Bus Wakeup Unit Register
#define IIC0_ICWUR2 ((volatile unsigned char  *)(IICBASE + 0x3017)) 


// ==== 31. Serial Peripheral Interface (SPI) ====
#define SPIBASE 0x40070000 
// SPI Control Register
#define SPI0_SPCR    ((volatile unsigned char  *)(SPIBASE + 0x2000)) 
#define SPI1_SPCR    ((volatile unsigned char  *)(SPIBASE + 0x2100))
#define SPCR_SPMS   0  // SPI Mode Select
#define SPCR_TXMD   1  // Communications Operating Mode
#define SPCR_MODFEN 2  // Mode Fault Error Detection Enable
#define SPCR_MSTR   3  // SPI Master/Slave Mode Select
#define SPCR_SPEIE  4  // SPI Error Interrupt Enable
#define SPCR_SPTIE  5  // Transmit Buffer Empty Interrupt Enable
#define SPCR_SPE    6  // SPI Function Enable
#define SPCR_SPRIE  7  // SPI Receive Buffer Full Interrupt Enable
// SPI Slave Select Polarity Register
#define SPI0_SSLP    ((volatile unsigned char  *)(SPIBASE + 0x2001)) 
#define SPI1_SSLP    ((volatile unsigned char  *)(SPIBASE + 0x2101))
#define SSLP_SSL0P  0  // SSL0 Signal Polarity Setting
#define SSLP_SSL0P  1  // SSL1 Signal Polarity Setting
#define SSLP_SSL0P  2  // SSL2 Signal Polarity Setting
#define SSLP_SSL0P  3  // SSL3 Signal Polarity Setting
// SPI Pin Control Register
#define SPI0_SPPCR   ((volatile unsigned char  *)(SPIBASE + 0x2002)) 
#define SPI1_SPPCR   ((volatile unsigned char  *)(SPIBASE + 0x2102))
#define SPPCR_SPLP   0  // SPI Loopback - data inverted
#define SPPCR_SPLP2  1  // SPI Loopback 2 - data not inverted
#define SPPCR_MOIFV  4  // MOSI Idle Fixed Value
#define SPPCR_MOIFE  5  // MOSI Idle Value Fixing Enable
// SPI Status Register
#define SPI0_SPSR    ((volatile unsigned char  *)(SPIBASE + 0x2003)) 
#define SPI1_SPSR    ((volatile unsigned char  *)(SPIBASE + 0x2103)) 
#define SPSR_OVRF   0  // Overrun Error Flag
#define SPSR_IDLNF  1  // SPI Idle Flag
#define SPSR_MODF   2  // Mode Fault Error Flag
#define SPSR_PERF   3  // Parity Error Flag
#define SPSR_UDRF   4  // Underrun Error Flag
#define SPSR_SPTEF  5  // SPI Transmit Buffer Empty Flag
#define SPSR_SPRF   7  // SPI Receive Buffer Full Flag
// SPI Data Register
#define SPI0_SPDR    ((volatile unsigned int   *)(SPIBASE + 0x2004)) 
#define SPI1_SPDR    ((volatile unsigned int   *)(SPIBASE + 0x2104)) 
#define SPI0_SPDR_HA ((volatile unsigned short *)(SPIBASE + 0x2004)) 
#define SPI1_SPDR_HA ((volatile unsigned short *)(SPIBASE + 0x2104)) 
// SPI Bit Rate Register
#define SPI0_SPBR    ((volatile unsigned char  *)(SPIBASE + 0x200A)) 
#define SPI1_SPBR    ((volatile unsigned char  *)(SPIBASE + 0x210A)) 
#define SPBR_0_7     0  // 0x00 = 24MHz; 0x01 = 12MHz; 0x02 = 8.0MHz; 0x03 = 6.0MHz; 0x04 = 4.8MHz; 0x05 = 4.0MHz
#define SPI_BR_24MHz  0x00
#define SPI_BR_12MHz  0x01
#define SPI_BR_8M0Hz  0x02
#define SPI_BR_6M0Hz  0x03
#define SPI_BR_4M8Hz  0x04
#define SPI_BR_4M0Hz  0x05

// SPI Data Control Register
#define SPI0_SPDCR   ((volatile unsigned char  *)(SPIBASE + 0x200B)) 
#define SPI1_SPDCR   ((volatile unsigned char  *)(SPIBASE + 0x210B))
#define SPDCR_SPRDTD 4  // SPI Receive/Transmit Data Select
#define SPDCR_SPLW   5  // SPI Word Access/Halfword Access Specification
// SPI Clock Delay Register
#define SPI0_SPCKD   ((volatile unsigned char  *)(SPIBASE + 0x200C)) 
#define SPI1_SPCKD   ((volatile unsigned char  *)(SPIBASE + 0x210C)) 
// SPI Slave Select Negation Delay Register
#define SPI0_SSLND   ((volatile unsigned char  *)(SPIBASE + 0x200D)) 
#define SPI1_SSLND   ((volatile unsigned char  *)(SPIBASE + 0x210D)) 
// SPI Next-Access Delay Register
#define SPI0_SPND    ((volatile unsigned char  *)(SPIBASE + 0x200E)) 
#define SPI1_SPND    ((volatile unsigned char  *)(SPIBASE + 0x210E)) 
// SPI Control Register 2
#define SPI0_SPCR2   ((volatile unsigned char  *)(SPIBASE + 0x200F)) 
#define SPI1_SPCR2   ((volatile unsigned char  *)(SPIBASE + 0x210F))
#define SPCR2_SPPE   0  // Parity Enable
#define SPCR2_SPOE   1  // Parity Mode
#define SPCR2_SPIIE  2  // SPI Idle Interrupt Enable
#define SPCR2_PTE    3  // Parity Self-Testing
#define SPCR2_SCKASE 4  // RSPCK Auto-Stop Function Enable
// SPI Command Register 0
#define SPI0_SPCMD0  ((volatile unsigned short *)(SPIBASE + 0x2010)) 
#define SPI1_SPCMD0  ((volatile unsigned short *)(SPIBASE + 0x2110)) 
#define SPCMD0_CPHA      0  // RSPCK Phase Setting
#define SPCMD0_CPOL      1  // RSPCK Polarity Setting
#define SPCMD0_BRDV_0_1  2  // Bit Rate Division Setting
#define SPCMD0_SSLA_0_2  4  // SSL Signal Assertion Setting
#define SPCMD0_SPB_0_3   8  // SPI Data Length Setting
#define SPCMD0_LSBF     12  // SPI LSB First
#define SPCMD0_SPNDEN   13  // SPI Next-Access Delay Enable
#define SPCMD0_SLNDEN   14  // SSL Negation Delay Setting Enable
#define SPCMD0_SCKDEN   15  // RSPCK Delay Setting Enable
#define SPI_BITS_08 0b0100 // 8 bits
#define SPI_BITS_09 0b1000 // 9 bits
#define SPI_BITS_10 0b1001 // 10 bits
#define SPI_BITS_11 0b1010 // 11 bits
#define SPI_BITS_12 0b1011 // 12 bits
#define SPI_BITS_13 0b1100 // 13 bits
#define SPI_BITS_14 0b1101 // 14 bits
#define SPI_BITS_15 0b1110 // 15 bits
#define SPI_BITS_16 0b1111 // 16 bits
#define SPI_BITS_20 0b0000 // 20 bits
#define SPI_BITS_24 0b0001 // 24 bits
#define SPI_BITS_32 0b0010 // 32 bits.
#define SPI_MASTER 1
#define SPI_SLAVE  0
#define SPI_IRQEN  1
#define SPI_POLL   0
#define SPI1       1
#define SPI0       0


// ==== Local Defines ====

#define TRUE 1
#define FALSE 0

#define LOOP_COUNT_VAL 24000  // Counter for interrupt - currently c. 24kHz to replace delay(), etc.


// Note: 
// Pin IRQxx on R4 Minima on digital-pins D0, D1, D2, D3, D8, D12, D13, D15, D16, D17, D18, D19 only
// ... IRQ02 on D19 doesn't attach.

void setup()
  { 
  uint8_t retVal8 = 0;

// These HiJacks should be done first in setup() for first free IRQ slot numbers

  attachInterrupt(13, timer7interrupt, FALLING);        // This IRQ will be asigned to Slot 05 IELSR05 as 0x001 PORT_IRQ0 - Table 13.4
  *ICU_IELSR05 = IRQ_GPT7_OVF;                          // Assign Slot 05 IELSR05 for GPT7_OVF
  *PFS_P111PFS = 0x00000000;                            // Clear D13 ISEL pin assigned Interrupt Enable

//  attachInterrupt(15, adcCompleteInterrupt, FALLING); // This IRQ will be asigned to Slot 06 IELSR06 as 0x002 PORT_IRQ1 - Table 13.4
//  *PFS_P000PFS = 0x00000000;                          // Clear A1/D15 ISEL pin assigned Interrupt Enable
//  *ICU_IELSR06 = IRQ_ADC140_ADI;                      // Assign Slot 06 IELSR06 for ADC140_ADI

  attachInterrupt(15, spiErrorInterrupt, FALLING);      // This IRQ will be asigned to Slot 06 IELSR06 as 0x002 PORT_IRQ1 - Table 13.4
  *PFS_P000PFS = 0x00000000;                            // Clear A1/D15 ISEL pin assigned Interrupt Enable
  *ICU_IELSR06 = IRQ_SPI0_SPEI;                         // Assign Slot 06 IELSR06 for SPI0 Error interrupts (placeholder)

  attachInterrupt(16, spiReceiveInterrupt, FALLING);    // This IRQ will be asigned to Slot 07 IELSR06 as 0x002 PORT_IRQ1 - Table 13.4
  *PFS_P001PFS = 0x00000000;                            // Clear A2/D16 ISEL pin assigned Interrupt Enable
  *ICU_IELSR07 = IRQ_SPI0_SPRI;                         // Assign Slot 07 IELSR07 for SPI0 Rx interrupts (placeholder)

  attachInterrupt(17, spiTransmitInterrupt, FALLING);   // This IRQ will be asigned to Slot 08 IELSR06 as 0x002 PORT_IRQ1 - Table 13.4
  *PFS_P002PFS = 0x00000000;                            // Clear A3/D17 ISEL pin assigned Interrupt Enable
  *ICU_IELSR08 = IRQ_SPI0_SPTI;                         // Assign Slot 08 IELSR08 for SPI0 Tx interrupts (placeholder)


  *PFS_P107PFS_BY = 0x04;                          // Set D7 output low - IRQ time flag pin

  Serial.begin(115200);      // The interrupts for the USB serial are already in place before setup() starts
  while (!Serial){};         // Note: USB serial cannot be used for serial comms when running fast IRQs - diagnostics only.
//  Serial.println(__FILE__);

//  setup_serial_sci2();       // Hardware serial used in fast-poll not with interrupts - use external Ser2USB etc.

#define SPI_DATA_WIDE 1  // <<<< Use for SPI transfers with data widths greater than 16 bits, comment out otherwise !!!

  retVal8 = setup_spi1(SPI_MASTER, SPI_BR_8M0Hz, SPI_BITS_32, SPI_IRQEN);  // SPI_IRQEN or SPI_POLL
//  retVal8 = setup_spi1(SPI_MASTER, SPIBR_8M0Hz, SPI_BITS_16, SPI_IRQEN);  // SPI_IRQEN or SPI_POLL
//  Serial.println(retVal8, HEX); 
//  print_spi_serial_reg(1);

  retVal8 = setup_spi0(SPI_SLAVE, SPI_BR_8M0Hz, SPI_BITS_32, SPI_POLL);
//  retVal8 = setup_spi0(SPI_SLAVE, SPIBR_8M0Hz, SPI_BITS_16, SPI_POLL);
//  Serial.println(retVal8, HEX); 
//  print_spi_serial_reg(0);

  setup_adc();
//  print_adc_sample_state_reg(1);
//  print_adc_registers();

  setup_dac();
//  print_dac_registers();

  setup_timers();
  *AGT0_AGTCR = 0;           // disable Millis counter, delay etc. don't want this cutting into Interrupt response time

//  setup_serial_iic1();     // <<< Still working on this module

//  get_system_info();
//  print_icu_event_links();
//  get_all_timer_status();
//  get_timer_reg_values(1);
//  get_timer_reg_values(3);
//  get_timer_reg_values(4);
//  get_timer_reg_values(7);
//  print_sci_serial_reg(2);
//  print_iic_serial_reg(1);
  }


static uint16_t adc_val_A0;
static uint16_t adc_val_A1;
static uint16_t adc_val_A2;
static uint16_t loop_counter = 0;
static bool loopFlag = FALSE;
static bool actionFlag = FALSE;

// static bool sineFlag = FALSE;
static bool sineFlag = TRUE;
const double refclk = 23460.0;      // <<<< measure and update this GPT rate value
double dfreq0 = 0.00;
volatile unsigned long phaccu0;          // phase accumulator 0
volatile unsigned long tword_m0 = 100;   // dds tuning word m
volatile uint16_t icount0;   // PWM Motor A base index-count
volatile bool directionFlag = FALSE;

#ifdef SPI_DATA_WIDE
volatile uint32_t spi1_received_val = 0;
volatile uint32_t spi1_transmit_val = 0;
volatile uint32_t spi0_received_val = 0;
volatile uint32_t spi0_transmit_val = 0;
#else 
volatile uint16_t spi1_received_val = 0;
volatile uint16_t spi1_transmit_val = 0;
volatile uint16_t spi0_received_val = 0;
volatile uint16_t spi0_transmit_val = 0;
#endif

void loop()
  {
  if(loopFlag == TRUE)
    {
//    *SCI2_TDR = (byte)(adc_val_A1 >> 4); // Transmit Data Register

    dfreq0 = (double)(adc_val_A1) / 16.384;                     // initial output frequency = 0.0 Hz
    tword_m0 = pow(2, 32) * dfreq0 / refclk; // calulate DDS new tuning word
 
    Serial.print(adc_val_A1, HEX);
    Serial.print(", ");
    Serial.print(spi1_received_val, HEX);
    Serial.print(", ");
    Serial.println (spi0_received_val, HEX);
/*
    Serial.print(", ");
    Serial.print(*SPI1_SPSR, HEX);
    Serial.print(", ");
    Serial.println(*SPI0_SPSR, HEX);
*/
//    Serial.print(adc_val_A1);
//    Serial.print(", ");
//     Serial.println(*DAC12_DADR0,HEX);
//    Serial.print(adc_val_A1);
//    Serial.print(", ");
//    Serial.println(adc_val_A2);
//    Serial.println(millis());  // To confirm millis() is disabled.
    loopFlag = FALSE;
    }
  if(actionFlag)
    {
    actionFlag = FALSE;
    }
  }



void timer7interrupt(void)
  {
//  static uint16_t localValA0 = 0;  // Board input ADC A0 used for DAC output
  static uint16_t localValA1 = 0;  // Board input ADC A1
  static uint16_t localValA2 = 0;  // Board input ADC A1
  static uint16_t localCount = 0;
  static uint8_t test = 0;

  *PFS_P107PFS_BY = 0x05;         // D7 start of Timer Interrupt

// ==== ADC ====

//  adc_val_A0 = *ADC140_ADDR09;    // adcValue - internal 16bit register read = c. 123nS 
  adc_val_A1 = *ADC140_ADDR00;    //  
  adc_val_A2 = *ADC140_ADDR01;    //  
  *ADC140_ADCSR |= (0x01 << 15);  // Next ADC conversion = c. 300nS

  *PFS_P107PFS_BY = 0x04;      //  Clear D7 to show time of ADC section

// ==== SPI ====  //  A 32-bit @ 8MHz BR master-slave operation takes 7.0uS

  *PFS_P106PFS_BY = 0x05;         // D6 flag to show start of SPI code

  *SPI0_SPSR &= 0xE2;                      // Clear Underrun, Parity, Mode, and Overrun error Flags
  *SPI0_SPSR |= (0x1 << SPSR_SPTEF);       // Set SPI Transmit Buffer Empty Flag

  if((test = (*SPI0_SPSR & 0x80)) == 0x80)  // If bit 7 is 1 = Valid data found in SPDR/SPDR_HA
    {
#ifdef SPI_DATA_WIDE
    spi0_received_val = *SPI0_SPDR;
#else
    spi0_received_val = *SPI0_SPDR_HA;
#endif
    }
  else                                     // No data found in SPDR/SPDR_HA
    {
    *PFS_P106PFS_BY = 0x05;         // D6 flag
    *PFS_P106PFS_BY = 0x04;         // D6 flag
    }

  if((test = (*SPI0_SPSR & 0x20)) == 0x20)  // If bit 5 is 1 = No data in the transmit buffer
    {
#ifdef SPI_DATA_WIDE
   *SPI0_SPDR = (uint32_t)adc_val_A1;
#else
   *SPI0_SPDR_HA = adc_val_A1;
#endif
    }
  else                                     // Data waiting in the transmit buffer
    {
    *PFS_P106PFS_BY = 0x05;         // D6 flag
    *PFS_P106PFS_BY = 0x04;         // D6 flag
    }

  if((test = (*SPI0_SPSR & 0x04)) == 0x04)  // A mode fault error or an underrun error has occured
    {
    *PFS_P106PFS_BY = 0x05;         // D6 flag
    *SPI0_SPSR &= 0xFB;             // Clear bit 2 Mode Fault Error Flag
    *PFS_P106PFS_BY = 0x04;         // D6 flag
    }

  *SPI1_SPSR &= 0xE2;                      // Clear Underrun, Parity, Mode, and Overrun error Flags
  *SPI1_SPSR |= (0x1 << SPSR_SPTEF);       // Set SPI Transmit Buffer Empty Flag
  *ICU_IELSR07 &= ~0x00010000;             // Make sure IR flag is cleared

#ifdef SPI_DATA_WIDE
      uint32_t local_val = *SPI1_SPDR;     // Make sure register empty
#else
      uint16_t local_val = *SPI1_SPDR_HA;
#endif

#ifdef SPI_DATA_WIDE
  *SPI1_SPDR = (uint32_t)localCount;
#else
  *SPI1_SPDR_HA = localCount;
#endif
  localCount++;

  for(test = 0; test < 16; test++)  // Wait a bit - non-blocking if tests fail, unlike a while() - 32 bit requires 12 or 13 loops
    {
    if((*SPI1_SPSR & (0x01 << SPSR_SPRF)) == (0x01 << SPSR_SPRF))                      // Data valid in receive-register
//    if( ((*SPI1_SPSR & (0x01 << SPSR_SPRF)) == (0x01 << SPSR_SPRF)) && (test > 4) )  // Check after n loops if data Rx valid
      {
      *PFS_P107PFS_BY = 0x05;         // D7 flag
#ifdef SPI_DATA_WIDE
      spi1_received_val = *SPI1_SPDR;
#else
      spi1_received_val = *SPI1_SPDR_HA;
#endif
      *PFS_P107PFS_BY = 0x04;         // D7 flag 
      break;   
      }
    }

  *PFS_P106PFS_BY = 0x04;         // D6 flag

#ifdef SPI_RxWAIT_DIAGS
  for(test = test; test > 0; test--)  // Pulse D7 for number of loops - 1 loop is 374nS
    {
    *PFS_P107PFS_BY = 0x05;         // D7 flag high = 83nS
    *PFS_P107PFS_BY = 0x04;         // D7 flag low  = 83nS; thus pin ops is 166nS from 374nS, therefor loop construct is c. 208nS
    }
#endif

  *PFS_P107PFS_BY = 0x04;         // D7 flag

// ==== DAC ====

  if(sineFlag)
    {
    phaccu0 = phaccu0 + tword_m0;         // soft DDS, phase accu with 32 bits
    icount0 = uint16_t (phaccu0 >> 20);   // use upper 12 bits for phase accu as frequency information
    *DAC12_DADR0 = icount0;               // DAC will update after all ADC conversions have finished
    }
  else
    {
    *DAC12_DADR0 = (adc_val_A1 >> 2);  // DAC will update after all ADC conversions have finished
    }


// ==== GPT PWM ====

  localValA1 = (adc_val_A1 >> 4);
  if(localValA1 == 0)             // Triangular mode PWM "destabilises" if 0x000 or 0x3FF values
    localValA1 = 1;
  else if (localValA1 >= 0x3FF)
    localValA1 = 0x3FE;

  localValA2 = (adc_val_A2 >> 4);
  if(localValA2 == 0)             // Triangular mode PWM "destabilises" if 0x000 or 0x3FF values
    localValA2 = 1;
  else if (localValA2 >= 0x3FF)
    localValA2 = 0x3FE;

//  *GPT163_GTCCRC = (uint32_t)localValA2; 
//  *GPT163_GTCCRD = (uint32_t)localValA1; 
//  *GPT321_GTCCRC = (uint32_t)localValA2; 

  *GPT321_GTCCRC = (uint32_t)(icount0 >> 2); // A output on pin D2
  *GPT321_GTCCRD = (uint32_t)localValA1;     // B output on pin D3

  if(directionFlag)
    {
    *GPT167_GTCCRC = 0x000; 
    *GPT167_GTCCRD = 0x7FD; 
    directionFlag = FALSE;
    }
  else
    {
    *GPT167_GTCCRC = 0x7FD; 
    *GPT167_GTCCRD = 0x000; 
    directionFlag = TRUE;
    }

  *PFS_P107PFS_BY = 0x05;       // D7 flag reassert near end of Interrupt

  loop_counter++;
  if(loop_counter >= LOOP_COUNT_VAL)
    {
    loop_counter = 0;
    loopFlag = TRUE;
    }

  *PFS_P107PFS_BY = 0x04;      // Clear D7 Interrupt Monitor pin
  }                            // Current IRQ time is 10.4uS


void adcCompleteInterrupt(void)
  {
  *PFS_P107PFS_BY = 0x05;      // D7 
  *PFS_P107PFS_BY = 0x04;      //  
  }

void spiErrorInterrupt(void)
  {
  *PFS_P107PFS_BY = 0x05;      // D7 
  *PFS_P107PFS_BY = 0x04;      //  
  }

void spiReceiveInterrupt(void)
  {
  *PFS_P107PFS_BY = 0x05;      // D7 
  *PFS_P107PFS_BY = 0x04;      //  
  *PFS_P107PFS_BY = 0x05;      // D7 - Double Tap to differentiate from Transmit IRQ
  *PFS_P107PFS_BY = 0x04;      //  
  }

void spiTransmitInterrupt(void)
  {
  *PFS_P107PFS_BY = 0x05;      // D7 
  *PFS_P107PFS_BY = 0x04;      //  
  }


void init_regs(void)
  {
  }


// ==== Asynchronous Serial Setup ====
//  These interrupts used in Serial1.begin()
//  7 - A4 - SCI2_TXI
//  8 - A5 - SCI2_TEI
//  9 - A3 - SCI2_RXI
// 10 - A6 - SCI2_ERI
//
// Default settings:
// *SCI2_RDR = 0x00; // Receive Data Register
// *SCI2_TDR = 0xFF; // Transmit Data Register
// *SCI2_SSR = 0x84; // Serial Status Register

void setup_serial_sci2(void)
  {
  *MSTP_MSTPCRB &= (0xFFFFFFFF - (0x01 << MSTPB29));  // Enable SCI2 serial module
  *SCI2_SMR  = 0x00; // Serial Mode Register - 0x00 is default
  *SCI2_SCMR = 0xF2; // Smart Card Mode Register - and other settings
  *SCI2_BRR  = 0x16; // Bit Rate Register
  *SCI2_MDDR = 0xE2; // Modulation Duty Register
  *SCI2_SEMR = 0xC4; // Serial Extended Mode Register
  *SCI2_SNFR = 0x00; // Noise Filter Setting Register
  *SCI2_SPTR = 0x06; // Serial Port Register - 0x06 is default
  *SCI2_SCR  = 0x30; // Serial Control Register - enable Rx and Tx with no interrupts NB Do last
  *PFS_P301PFS   = (0b00100 << 24);  // Select PSEL[4:0] for GTIOC7A - See Table 19.10
  *PFS_P301PFS  |= (0x1 << 16);      // Port Mode Control - Used as an I/O port for peripheral function
  *PFS_P302PFS   = (0b00100 << 24);  // Select PSEL[4:0] for GTIOC7B - See Table 19.10
  *PFS_P302PFS  |= (0x1 << 16);      // Port Mode Control - Used as an I/O port for peripheral function
  }


void setup_serial_iic1(void)
  {
  *MSTP_MSTPCRB &= (0xFFFFFFFF - (0x01 << MSTPB8));  // Enable IIC1 serial module - this module in progress

  *IIC1_ICCR1 = 0x9F; // I2C Bus Control Register 1
  *IIC1_ICCR2 = 0x00; // I2C Bus Control Register 2
  *IIC1_ICMR1 = 0x28; // I2C Bus Mode Register 1
  *IIC1_ICMR2 = 0x05; // I2C Bus Mode Register 2
  *IIC1_ICMR3 = 0x00; // I2C Bus Mode Register 3
  *IIC1_ICFER = 0x77; // I2C Bus Function Enable Register
  *IIC1_ICSER = 0x00; // I2C Bus Status Enable Register
  *IIC1_ICIER = 0x00; // I2C Bus Interupt Enable Register 0xB3
  *IIC1_ICBRL = 0xFB; // I2C Bus Bit Rate Low-Level Register
  *IIC1_ICBRH = 0xFA; // I2C Bus Bit Rate High-Level Register

  *PFS_P100PFS   = (0b00111 << 24);  // Select PSEL[4:0] for SCL1 - See Table 19.6
  *PFS_P100PFS  |= (0x1 << 16);      // Port Mode Control - Used as an I/O port for peripheral function
  *PFS_P101PFS   = (0b00111 << 24);  // Select PSEL[4:0] for SCK1 - See Table 19.6
  *PFS_P101PFS  |= (0x1 << 16);      // Port Mode Control - Used as an I/O port for peripheral function
  }


void setup_timers(void)
  {
  *MSTP_MSTPCRD &= (0xFFFFFFFF - (0x01 << MSTPD6));  // Enable GTP16 timer module
                //   76543210
//  *GPT163_GTSTP  = 0x10011000; // Stop Timers 7, 4 and 3  - Doesn't matter which timer is addressed
  *GPT163_GTSTP  = 0x10010010; // Stop Timers 7, 4 and 1  - Doesn't matter which timer is addressed

// Timer 7 - Sawtooth Mode to provide Buffer Delay for Direction Control to H-bridge 
  *GPT167_GTSSR = 0x80000000; // b31 CSTRT Software Source Counter Start Enable
  *GPT167_GTPSR = 0x80000000; // b31 CSTOP Software Source Counter Stop Enable
  *GPT167_GTCSR = 0x80000000; // b31 CCLR  Software Source Counter Clear Enable

  *GPT167_GTUDDTYC = 0x1;
  *GPT167_GTCNT  = 0x07F6;     // Set Counter value to lineup transition with PWM
  *GPT167_GTCCRA = 0x0000;     // 
  *GPT167_GTCCRB = 0x0000;     // 
  *GPT167_GTCCRC = 0x0000;     // 
  *GPT167_GTCCRD = 0x0000;     // 
  *GPT167_GTPR   = 0x7FD;     // 0X5FA8  24488
  *GPT167_GTPBR  = 0x7FD;     // 0X5FA8  24488
  *GPT167_GTBER  = 0x00150000; // PR, CCRB, and CCRA set to single buffer operation 
  *GPT167_GTCR   = 0x0;        // Set PreScale to zero - Set Sawtooth mode
  *GPT167_GTIOR  = 0x00000000;   // Clear I/O Control Register
  *GPT167_GTIOR |=  0b01001       ; // GTIOA[4:0] = 01001b - Initial output is low; Low Output at GTCCRA compare match; HIGH at end
  *GPT167_GTIOR |= (0b01001 << 16); // GTIOB[4:0] = 01001b - Initial output is low; Low Output at GTCCRB compare match; HIGH at end
  *GPT167_GTIOR |= 0x01000100; // Set OBE and OAE bits 

  *PFS_P304PFS   = (0b00011 << 24);  // Select PSEL[4:0] for GTIOC7A - See Table 19.10
  *PFS_P304PFS  |= (0x1 << 16);      // Port Mode Control - Used as an I/O port for peripheral function
  *PFS_P303PFS   = (0b00011 << 24);  // Select PSEL[4:0] for GTIOC7B - See Table 19.10
  *PFS_P303PFS  |= (0x1 << 16);      // Port Mode Control - Used as an I/O port for peripheral function


// Timer 1 - Trangular-Wave mode for PWM to H-bridge Enables
  *MSTP_MSTPCRD &= (0xFFFFFFFF - (0x01 << MSTPD5));  // Enable GTP32 timer module
  *GPT321_GTSSR = 0x80000000; // b31 CSTRT Software Source Counter Start Enable
  *GPT321_GTPSR = 0x80000000; // b31 CSTOP Software Source Counter Stop  Enable
  *GPT321_GTCSR = 0x80000000; // b31 CCLR  Software Source Counter Clear Enable
//  *GPT321_GTUDDTYC = 0x1;     // Count Direction = UP
  *GPT321_GTUDDTYC = 0x0;     // Count Direction = DOWN
  *GPT321_GTCNT  = 0x03FF;  
  *GPT321_GTCCRA = 0x0000;     // 
  *GPT321_GTCCRB = 0x0000;     // 
  *GPT321_GTCCRC = 0x0000;     // 
  *GPT321_GTCCRD = 0x0000;     // 
  *GPT321_GTPR   = 0x3FF;     // 
  *GPT321_GTPBR  = 0x3FF;     // 
//  *GPT321_GTPR   = 0x400;     // 
//  *GPT321_GTPBR  = 0x400;     // 
  *GPT321_GTBER  = 0x00150000; // PR, CCRB, and CCRA set to single buffer operation 
  *GPT321_GTIOR  = 0x00000000;   // Clear I/O Control Register
  *GPT321_GTIOR |=  0b11011       ; // GTIOA[4:0] = 11011b - Initial output is high; Output toggled at GTCCRA compare match
  *GPT321_GTIOR |= (0b11011 << 16); // GTIOB[4:0] = 11011b - Initial output is high; Output toggled at GTCCRB compare match
  *GPT321_GTIOR |= 0x01000100; // Set OBE and OAE bits 
  *GPT321_GTCR   = 0x0;        // Set PreScale to zero
  *GPT321_GTCR  |= (0x4 << 16);  // Set Triangle-wave PWM mode 1

  *PFS_P104PFS   = (0b00011 << 24);  // Select PSEL[4:0] for GTIOC1B - See Table 19.6
  *PFS_P104PFS  |= (0x1 << 16);      // Port Mode Control - Used as an I/O port for peripheral function
  *PFS_P105PFS   = (0b00011 << 24);  // Select PSEL[4:0] for GTIOC1A - See Table 19.6
  *PFS_P105PFS  |= (0x1 << 16);      // Port Mode Control - Used as an I/O port for peripheral function


/*
// Timer 3 - Trangular-Wave mode for PWM to H-bridge Enables
  *GPT163_GTSSR = 0x80000000; // b31 CSTRT Software Source Counter Start Enable
  *GPT163_GTPSR = 0x80000000; // b31 CSTOP Software Source Counter Stop  Enable
  *GPT163_GTCSR = 0x80000000; // b31 CCLR  Software Source Counter Clear Enable
//  *GPT163_GTUDDTYC = 0x1;     // Count Direction = UP
  *GPT163_GTUDDTYC = 0x0;     // Count Direction = DOWN
  *GPT163_GTCNT  = 0x03FF;  
  *GPT163_GTCCRA = 0x0000;     // 
  *GPT163_GTCCRB = 0x0000;     // 
  *GPT163_GTCCRC = 0x0000;     // 
  *GPT163_GTCCRD = 0x0000;     // 
  *GPT163_GTPR   = 0x3FF;     // 
  *GPT163_GTPBR  = 0x3FF;     // 
//  *GPT163_GTPR   = 0x400;     // 
//  *GPT163_GTPBR  = 0x400;     // 
  *GPT163_GTBER  = 0x00150000; // PR, CCRB, and CCRA set to single buffer operation 
  *GPT163_GTIOR  = 0x00000000;   // Clear I/O Control Register
  *GPT163_GTIOR |=  0b11011       ; // GTIOA[4:0] = 11011b - Initial output is high; Output toggled at GTCCRA compare match
  *GPT163_GTIOR |= (0b11011 << 16); // GTIOB[4:0] = 11011b - Initial output is high; Output toggled at GTCCRB compare match
  *GPT163_GTIOR |= 0x01000100; // Set OBE and OAE bits 
  *GPT163_GTCR   = 0x0;        // Set PreScale to zero
  *GPT163_GTCR  |= (0x4 << 16);  // Set Triangle-wave PWM mode 1

  *PFS_P111PFS   = (0b00011 << 24);  // Select PSEL[4:0] for GTIOC3A - See Table 19.7
  *PFS_P111PFS  |= (0x1 << 16);      // Port Mode Control - Used as an I/O port for peripheral function
  *PFS_P112PFS   = (0b00011 << 24);  // Select PSEL[4:0] for GTIOC3B - See Table 19.7
  *PFS_P112PFS  |= (0x1 << 16);      // Port Mode Control - Used as an I/O port for peripheral function
*/

// Timer 4 - Basic Timing with Interrupt - This is the Minima PCB underside HEART shaped pad connected timer.
  *GPT164_GTSSR = 0x80000000;        // b31 CSTRT Software Source Counter Start Enable
  *GPT164_GTPSR = 0x80000000;        // b31 CSTOP Software Source Counter Stop  Enable
  *GPT164_GTCSR = 0x80000000;        // b31 CCLR  Software Source Counter Clear Enable
  *GPT164_GTUDDTYC = 0x1;            // Count Direction = UP
  *GPT164_GTCNT  = 0x000;  
  *GPT164_GTCCRB = 0x1FF; 
  *GPT164_GTCCRD = 0x1FF;            // Set for 25/75 M/S Clock
  *GPT164_GTPR   = 0x7FD;            // 
  *GPT164_GTPBR  = 0x7FD;            // Set for overall period 
  *GPT164_GTIOR  = 0x00090000;       // B = High at End, Low at Match 
  *GPT164_GTIOR |= 0x01000000;       // Set OBE bit 
  *GPT164_GTCR   = 0x0;              // Set PreScale to zero - Set Sawtooth mode
  *PFS_P204PFS   = (0b00011 << 24);  // Select PSEL[4:0] for GTIOC4B - See Table 19.8
  *PFS_P204PFS  |= (0x1 << 16);      // Port Mode Control - Used as an I/O port for peripheral function

// All timers
                //   76543210
//  *GPT164_GTSTR  = 0x10011000;  // Start Timers 7, 3 and 4  - Doesn't matter which timer is addressed
  *GPT164_GTSTR  = 0x10010010;  // Start Timers 7, 3 and 4  - Doesn't matter which timer is addressed
  *GPT167_GTCR   |= 0x1;        // Start Timer 7 with a PreScale of zero
  *GPT321_GTCR   |= 0x1;        // Start Timer 1 with a PreScale of zero
//  *GPT163_GTCR   |= 0x1;        // Start Timer 3 with a PreScale of zero
  *GPT164_GTCR   |= 0x1;        // Start Timer 4 with a PreScale of zero
//  *GPT163_GTCLR  = 0x10011000;  // Clear Timers 7, 3 and 4  - Doesn't matter which timer is addressed
  *GPT164_GTCLR  = 0x10010010;  // Clear Timers 7, 1 and 4  - Doesn't matter which timer is addressed
  }


void get_system_info(void)
  {
  Serial.print("SYSTEM_SCKDIVCR ");
  Serial.println(*SYSTEM_SCKDIVCR, HEX); 
  Serial.print("SYSTEM_SBYCR    ");
  Serial.println(*SYSTEM_SBYCR, HEX);
  Serial.print("SYSTEM_MSTPCRA ");
  Serial.println(*SYSTEM_MSTPCRA, HEX);
  Serial.print("MSTP_MSTPCRB ");
  Serial.println(*MSTP_MSTPCRB, HEX);
  Serial.print("MSTP_MSTPCRC ");
  Serial.println(*MSTP_MSTPCRC, HEX);
  Serial.print("MSTP_MSTPCRD ");
  Serial.println(*MSTP_MSTPCRD, HEX);
  Serial.println(" ");
  }


void get_all_timer_status(void)
  {
  Serial.print("AGT0_AGTCR   ");
  Serial.println(*AGT0_AGTCR, HEX);
  Serial.print("AGT1_AGTCR   ");
  Serial.println(*AGT1_AGTCR, HEX);
  Serial.print("GPT320_GTST  ");
  Serial.println(*GPT320_GTST, HEX);
  Serial.print("GPT321_GTST  ");
  Serial.println(*GPT321_GTST, HEX);
  Serial.print("GPT162_GTST  ");
  Serial.println(*GPT162_GTST, HEX);
  Serial.print("GPT163_GTST  ");
  Serial.println(*GPT163_GTST, HEX);
  Serial.print("GPT164_GTST  "); 
  Serial.println(*GPT164_GTST, HEX); 
  Serial.print("GPT165_GTST  ");
  Serial.println(*GPT165_GTST, HEX);
  Serial.print("GPT166_GTST  ");
  Serial.println(*GPT166_GTST, HEX); 
  Serial.print("GPT167_GTST  ");  
  Serial.println(*GPT167_GTST, HEX);
  }


void get_timer_reg_values(int timer_number)
  {
  unsigned int local_gpt_val = 0;
  Serial.print("Registers for GPT"); // 
  Serial.println(timer_number);      //

  Serial.print("GTWP   ");   // 
  local_gpt_val = *((volatile unsigned int *)(GPTBASE + GTWP + (timer_number * 0x100)));
  Serial.println(local_gpt_val, HEX);

  Serial.print("GTSSR  ");   // 
  local_gpt_val = *((volatile unsigned int *)(GPTBASE + GTSSR + (timer_number * 0x100)));
  Serial.println(local_gpt_val, HEX);

  Serial.print("GTPSR  ");   // 
  local_gpt_val = *((volatile unsigned int *)(GPTBASE + GTPSR + (timer_number * 0x100)));
  Serial.println(local_gpt_val, HEX);

  Serial.print("GTCSR  ");   // 
  local_gpt_val = *((volatile unsigned int *)(GPTBASE + GTCSR + (timer_number * 0x100)));
  Serial.println(local_gpt_val, HEX);

  Serial.print("GTIOR  ");   // 
  local_gpt_val = *((volatile unsigned int *)(GPTBASE + GTIOR + (timer_number * 0x100)));
  Serial.println(local_gpt_val, HEX);

  Serial.print("GTBER  ");   // 
  local_gpt_val = *((volatile unsigned int *)(GPTBASE + GTBER + (timer_number * 0x100)));
  Serial.println(local_gpt_val, HEX);

  Serial.print("GTUPSR ");   // 
  local_gpt_val = *((volatile unsigned int *)(GPTBASE + GTUPSR + (timer_number * 0x100)));
  Serial.println(local_gpt_val, HEX);

  Serial.print("GTDNSR ");   // 
  local_gpt_val = *((volatile unsigned int *)(GPTBASE + GTDNSR + (timer_number * 0x100)));
  Serial.println(local_gpt_val, HEX);

  Serial.print("GTCR   ");   // 
  local_gpt_val = *((volatile unsigned int *)(GPTBASE + GTCR + (timer_number * 0x100)));
  Serial.println(local_gpt_val, HEX);

  Serial.print("GTUDDTYC ");   // 
  local_gpt_val = *((volatile unsigned int *)(GPTBASE + GTUDDTYC + (timer_number * 0x100)));
  Serial.println(local_gpt_val, HEX);

  Serial.print("GTST   ");   // 
  local_gpt_val = *((volatile unsigned int *)(GPTBASE + GTST + (timer_number * 0x100)));
  Serial.print(local_gpt_val, HEX);
  Serial.println(" Flags");   // 

  Serial.print("GTCCRA ");   // 
  local_gpt_val = *((volatile unsigned int *)(GPTBASE + GTCCRA + (timer_number * 0x100)));
  Serial.println(local_gpt_val, HEX);

  Serial.print("GTCCRB ");   // 
  local_gpt_val = *((volatile unsigned int *)(GPTBASE + GTCCRB + (timer_number * 0x100)));
  Serial.println(local_gpt_val, HEX);

  Serial.print("GTCCRC ");   // 
  local_gpt_val = *((volatile unsigned int *)(GPTBASE + GTCCRC + (timer_number * 0x100)));
  Serial.println(local_gpt_val, HEX);

  Serial.print("GTCCRD ");   // 
  local_gpt_val = *((volatile unsigned int *)(GPTBASE + GTCCRD + (timer_number * 0x100)));
  Serial.println(local_gpt_val, HEX);

  Serial.print("GTPR   ");   // 
  local_gpt_val = *((volatile unsigned int *)(GPTBASE + GTPR + (timer_number * 0x100)));
  Serial.println(local_gpt_val, HEX);

  Serial.print("GTPBR  ");   // 
  local_gpt_val = *((volatile unsigned int *)(GPTBASE + GTPBR + (timer_number * 0x100)));
  Serial.println(local_gpt_val, HEX);

  Serial.println(" ");   
  }


void setup_dac(void)       // Note make sure ADC is stopped before setup DAC
  {
  *MSTP_MSTPCRD &= (0xFFFFFFFF - (0x01 << MSTPD20));  // Enable DAC12 module
  *DAC12_DADPR    = 0x00;        // DADR0 Format Select Register - Set right-justified format
  *ELC_ELSR12     = 0x0000;      // See 36.2.4 for Synchronous Conversion mode
  *DAC12_DAADSCR  = 0x80;        // D/A A/D Synchronous Start Control Register - Enable
//  *DAC12_DAADSCR  = 0x00;        // D/A A/D Synchronous Start Control Register - Default
// 36.3.2 Notes on Using the Internal Reference Voltage as the Reference Voltage
  *DAC12_DAVREFCR = 0x00;        // D/A VREF Control Register - Write 0x00 first - see 36.2.5
  *DAC12_DADR0    = 0x0000;      // D/A Data Register 0 
   delayMicroseconds(10);        
  *DAC12_DAVREFCR = 0x01;        // D/A VREF Control Register - Select AVCC0/AVSS0 for Vref
//  *DAC12_DAVREFCR = 0x03;        // D/A VREF Control Register - Select Internal reference voltage/AVSS0
//  *DAC12_DAVREFCR = 0x06;        // D/A VREF Control Register - Select External Vref; set VREFH&L pins used for LEDs
//  *PFS_P012PFS_BY = 0x05;        // Set VREFH pin high
//  *PFS_P013PFS_BY = 0x04;        // Set VREFL pin low 
  *DAC12_DACR     = 0x5F;        // D/A Control Register - 
   delayMicroseconds(5);         // 
  *DAC12_DADR0    = 0x0800;      // D/A Data Register 0 
  *PFS_P014PFS   = 0x00000000;   // Make sure all cleared
  *PFS_P014PFS  |= (0x1 << 15);  // Port Mode Control - Used as an analog pin
  }


void print_dac_registers(void)
  {
  Serial.print("DAC12_DADR0    ");
  Serial.print(*DAC12_DADR0,HEX);
  Serial.println(" - D/A Data Register 0");
  Serial.print("DAC12_DACR     ");
  Serial.print(*DAC12_DACR,HEX);
  Serial.println(" - D/A Control Register");
  Serial.print("DAC12_DADPR    ");
  Serial.print(*DAC12_DADPR,HEX);
  Serial.println(" - D/A Format Select Register");
  Serial.print("DAC12_DAADSCR  ");
  Serial.print(*DAC12_DAADSCR,HEX);
  Serial.println(" - D/A A/D Synchronous Start Control Register");
  Serial.print("DAC12_DAVREFCR ");
  Serial.print(*DAC12_DAVREFCR,HEX);
  Serial.println(" - D/A VREF Control Register");
  Serial.println(" ");
  }

/*
from print_adc_registers();   Get IDE analogRead() setups
  ADC140_ADCSR    64 A/D Control Register = b6 GBADIE Group B Scan End Interrupt Enable
  ADC140_ADANSA0 512 A/D Channel Select Register A0 = Selected ANSA09
  ADC140_ADANSA1   1 A/D Channel Select Register A1 = Selected ANSA16 net R10(+5V>100K)/R26(13.7K>GND) (P500) ???
  ADC140_ADCER    38 A/D Control Extended Register = b5 ACE A/D Data Register Auto Clear Enable; b2, b1 ADPRC[1:0] A/D Conversion 14bit
A/D Control Extended Register (ADCER)
b5 ACE A/D Data Register Automatic Clearing Enable
  0: Automatic clearing disabled
  1: Automatic clearing enabled.  << This is set in analogRead()
*/

void setup_adc(void)
  {
  *MSTP_MSTPCRD &= (0xFFFFFFFF - (0x01 << MSTPD16));  // Enable ADC140 module

  *ADC140_ADHVREFCNT = 0x01;         // Set External Aref = analogReference(AR_EXTERNAL);      
  *ADC140_ADCER    = 0x06;           // 14 bit mode, clear ACE bit 5
  *ADC140_ADCSR   |= (0x01 << 6);    // Set b6 - GBADIE Group B Scan End Interrupt Enable
//  *ADC140_ADANSA0 |= (0x01 << 9);    // Selected ANSA09
  *ADC140_ADANSA0 |= (0x01 << 1);    // Selected ANSA01
  *ADC140_ADANSA0 |= (0x01 << 0);    // Selected ANSA00
  *ADC140_ADADC    = 0x83;           // Average mode - 4x
//  *ADC140_ADADS0  |= (0x01 << 9);    // Enable Averaging for ANSA09
  *ADC140_ADADS0  |= (0x01 << 1);    // Enable Averaging for ANSA01
  *ADC140_ADADS0  |= (0x01 << 0);    // Enable Averaging for ANSA00
//  *ADC140_ADCSR   |= (0x01 << 15);   // Start a conversion
  }


void print_adc_registers(void)
  {
  Serial.print("ADC140_ADCSR ");
  Serial.print(*ADC140_ADCSR);
  Serial.println(" A/D Control Register");
  Serial.print("ADC140_ADANSA0 ");
  Serial.print(*ADC140_ADANSA0);
  Serial.println(" A/D Channel Select Register A0");
  Serial.print("ADC140_ADANSA1 ");
  Serial.print(*ADC140_ADANSA1);
  Serial.println(" A/D Channel Select Register A1");
  Serial.print("ADC140_ADADS0 ");
  Serial.print(*ADC140_ADADS0); 
  Serial.println(" A/D-Converted Value Addition/Average Channel Select Register 0");
  Serial.print("ADC140_ADADS1 ");
  Serial.print(*ADC140_ADADS1);
  Serial.println(" A/D-Converted Value Addition/Average Channel Select Register 1");
  Serial.print("ADC140_ADCER ");
  Serial.print(*ADC140_ADCER);
  Serial.println(" A/D Control Extended Register");
  Serial.print("ADC140_ADSTRGR ");
  Serial.print(*ADC140_ADSTRGR);
  Serial.println(" A/D Conversion Start Trigger Select Register");
  Serial.print("ADC140_ADEXICR ");
  Serial.print(*ADC140_ADEXICR);
  Serial.println(" A/D Conversion Extended Input Control Register");
  Serial.print("ADC140_ADANSB0 ");
  Serial.print(*ADC140_ADANSB0);
  Serial.println(" A/D Channel Select Register B0");
  Serial.print("ADC140_ADANSB1 ");
  Serial.print(*ADC140_ADANSB1);
  Serial.println(" A/D Channel Select Register B1");
  Serial.print("ADC140_ADRD ");
  Serial.print(*ADC140_ADRD);
  Serial.println(" A/D Self-Diagnosis Data Register");
  Serial.print("ADC140_ADGSPCR ");
  Serial.print(*ADC140_ADGSPCR);
  Serial.println(" A/D Group Scan Priority Control Register");
  Serial.print("ADC140_ADCMPCR ");
  Serial.print(*ADC140_ADCMPCR);
  Serial.println(" A/D Compare Function Control Register");
  }


void print_adc_sample_state_reg(int ssregval)
  {
  Serial.print("ADC140_ADSSTR");
  Serial.print(ssregval);
  Serial.print(" = ");
  Serial.println(*((volatile unsigned char *)(ADCBASE + 0xC0E0 + ssregval))); 
  }


void print_iic_serial_reg(int iicregval)
  {
  unsigned int local_iic_val = 0;
  unsigned char iic_val_index = 0;
  Serial.print("IIC");
  Serial.print(iicregval);
  Serial.println(" Registers");

  for(iic_val_index = 0; iic_val_index <= 0x13; iic_val_index++)
    {
    Serial.print(iic_val_index,HEX);  
    Serial.print(" = ");  
    local_iic_val = *((volatile unsigned char *)(IICBASE + 0x3000 + iic_val_index + (0x100 * iicregval)));            //
    Serial.println(local_iic_val, HEX);
    }
  Serial.println(" ");  
  }


uint8_t setup_spi0(bool MSmode, uint8_t bitRate, uint8_t numBits, bool IRQmode)
  {
  *MSTP_MSTPCRB &= (0xFFFFFFFF - (0x01 << MSTPB19));  // Enable SPI0 module
  *SPI0_SPCR   = 0x00;  // Make sure SPI is disabled for register initialisation
  *SPI0_SSLP   = 0x00;  // Set SPI Slave Select Polarity Register - default active low

  if(MSmode)
    {
    *SPI0_SPBR   = bitRate;  // 0x00 = 24MHz; 0x01 = 12MHz; 0x02 = 8.0MHz; 0x03 = 6.0MHz; 0x04 = 4.8MHz; 0x05 = 4.0MHz  
    *SPI0_SPPCR  = 0x30;  // MOIFV and MOIFE
    *SPI0_SPCR2  = 0x10; 
    *SPI0_SPCMD0 = 0xE001;  // 
    }
  else
    {
    *SPI0_SPBR   = 0x00;
    *SPI0_SPCMD0 = 0x0001;  // 
    *SPI0_SPCKD  = 0x00;  // SPI Clock Delay Register - set to 0 for slave-mode
    *SPI0_SSLND  = 0x00;  // SPI Slave Select Negation Delay Register - set to 0 for slave-mode
    *SPI0_SPND   = 0x00;  // SPI Next-Access Delay Register - set to 0 for slave-mode
    *SPI0_SPCR2  = 0x00;  // Auto-stop funtion, b4 = 0 disabled  
    }

  if(numBits >= 4)
    {
    *SPI0_SPDCR  = 0x00;  // b5 = 0 SPLW SPI Halfword Access, default for 16 bit or smaller transfers
    *SPI0_SPDR_HA = 0xAAAA;  // Some initial data
    }
  else
    {
    *SPI0_SPDCR |= (0x01 << SPDCR_SPLW);  // b5 = 1 SPLW SPI Word Access for 20, 24, and 32 bit transfers
    *SPI0_SPDR = 0x55555555;  // Some initial data
    }

  *SPI0_SPCMD0 |= (numBits << SPCMD0_SPB_0_3);  

  *PFS_P103PFS   = (0b00110 << 24);  // Select PSEL[4:0] for SSLA0 - See Table 19.7
  *PFS_P103PFS  |= (0x1 << 16);      // Port Mode Control - Used as an I/O port for peripheral function
  *PFS_P102PFS   = (0b00110 << 24);  // Select PSEL[4:0] for RSPCKA
  *PFS_P102PFS  |= (0x1 << 16);      // Port Mode Control - Used as an I/O port for peripheral function
  *PFS_P101PFS   = (0b00110 << 24);  // Select PSEL[4:0] for MOS1A
  *PFS_P101PFS  |= (0x1 << 16);      // Port Mode Control - Used as an I/O port for peripheral function
  *PFS_P100PFS   = (0b00110 << 24);  // Select PSEL[4:0] for MISOA
  *PFS_P100PFS  |= (0x1 << 16);      // Port Mode Control - Used as an I/O port for peripheral function
  *PFS_P100PFS  |= (0x1 << 4);       // Pull-up Control - enable internal pullup
  if(IRQmode)
    {
    *ICU_IELSR06 = IRQ_SPI0_SPEI;         // HiJack Slot 06 IELSR06 for SPI0 interrupts
    *ICU_IELSR07 = IRQ_SPI0_SPRI;         // HiJack Slot 07 IELSR07 for SPI0 interrupts
    *ICU_IELSR08 = IRQ_SPI0_SPTI;         // HiJack Slot 08 IELSR08 for SPI0 interrupts
    *SPI0_SPCR   |= (0x1 << SPCR_SPTIE);  // Enable SPI Transmit Buffer Empty interrupt
    *SPI0_SPCR   |= (0x1 << SPCR_SPRIE);  // Enable SPI Receive Buffer Full interrupt
    *SPI0_SPCR   |= (0x1 << SPCR_SPEIE);  // Enable SPI Error interrupt
    }

  if(MSmode)
    {
    *SPI0_SPCR   |= 0x48;  // Enable SPI 4-wire method - master-mode
//  *SPI1_SPCR   |= 0x49;  // Enable SPI 3-wire method (IDE default)
    }
  else
    {
    *SPI0_SPCR   |= 0x40;  // Enable SPI 4-wire method - slave-mode
//  *SPI1_SPCR   |= 0x41;  // Enable SPI 3-wire method - slave-mode
    }
  return(*SPI0_SPCR);
  }


uint8_t setup_spi1(bool MSmode, uint8_t bitRate, uint8_t numBits, bool IRQmode)
  {
  *MSTP_MSTPCRB &= (0xFFFFFFFF - (0x01 << MSTPB18));  // Enable SPI1 module
  *SPI1_SPCR   = 0x00;  // Make sure SPI is disabled for register initialisation
  *SPI1_SSLP   = 0x00;  // Set SPI Slave Select Polarity Register - default active low
  if(MSmode)
    {
    *SPI1_SPBR   = bitRate;  // 0x00 = 24MHz; 0x01 = 12MHz; 0x02 = 8.0MHz; 0x03 = 6.0MHz; 0x04 = 4.8MHz; 0x05 = 4.0MHz  
    *SPI1_SPPCR  = 0x30;  // MOIFV and MOIFE
    *SPI1_SPCR2  = 0x10; 
    *SPI1_SPCMD0 = 0xE001;  // 
    }
  else
    {
    *SPI1_SPCMD0 = 0x0001;  // 
    }

  if(numBits >= 4)
    {
    *SPI1_SPDCR  = 0x00;  // b5 = 0 SPLW SPI Halfword Access, default for 16 bit or smaller transfers
    *SPI1_SPDR_HA = 0xAAAA;  // Some initial data
    }
  else
    {
    *SPI1_SPDCR |= (0x01 << SPDCR_SPLW);  // b5 = 1 SPLW SPI Word Access for 20, 24, and 32 bit transfers
    *SPI1_SPDR = 0x55555555;  // Some initial data
    }
  *SPI1_SPCMD0 |= (numBits << SPCMD0_SPB_0_3);  
  *PFS_P112PFS   = (0b00110 << 24);  // Select PSEL[4:0] for SSLB0 - See Table 19.7
  *PFS_P112PFS  |= (0x1 << 16);      // Port Mode Control - Used as an I/O port for peripheral function
  *PFS_P111PFS   = (0b00110 << 24);  // Select PSEL[4:0] for RSPCKB
//  *PFS_P111PFS  |= (0x1 << 10);      // Port Drive Capability - Middle drive (LED on D13), no visible effect :(
  *PFS_P111PFS  |= (0x1 << 16);      // Port Mode Control - Used as an I/O port for peripheral function
  *PFS_P110PFS   = (0b00110 << 24);  // Select PSEL[4:0] for MISOB
  *PFS_P110PFS  |= (0x1 << 16);      // Port Mode Control - Used as an I/O port for peripheral function
  *PFS_P109PFS   = (0b00110 << 24);  // Select PSEL[4:0] for MOSIB
  *PFS_P109PFS  |= (0x1 << 16);      // Port Mode Control - Used as an I/O port for peripheral function
  if(IRQmode)
    {
    *ICU_IELSR06 = IRQ_SPI1_SPEI;         // HiJack Slot 06 IELSR05 for SPI1 interrupts
    *ICU_IELSR07 = IRQ_SPI1_SPRI;         // HiJack Slot 07 IELSR05 for SPI1 interrupts
    *ICU_IELSR08 = IRQ_SPI1_SPTI;         // HiJack Slot 08 IELSR05 for SPI1 interrupts
    *SPI1_SPCR   |= (0x1 << SPCR_SPTIE);  // Enable SPI Transmit Buffer Empty interrupt
    *SPI1_SPCR   |= (0x1 << SPCR_SPRIE);  // Enable SPI Receive Buffer Full interrupt
    *SPI1_SPCR   |= (0x1 << SPCR_SPEIE);  // Enable SPI Error interrupt
    }
  if(MSmode)
    {
    *SPI1_SPCR   |= 0x48;  // Enable SPI 4-wire method - master-mode
//  *SPI1_SPCR   |= 0x49;  // Enable SPI 3-wire method (IDE default)
    }
  else
    {
    *SPI1_SPCR   |= 0x40;  // Enable SPI 4-wire method - slave-mode
//  *SPI1_SPCR   |= 0x41;  // Enable SPI 3-wire method - slave-mode
    }
  return(*SPI1_SPCR);
  }


void print_spi_serial_reg(int spiregval)
  {
  unsigned int local_spi_val = 0;
  unsigned char spi_val_index = 0;
  Serial.print("SPI");
  Serial.print(spiregval);
  Serial.println(" Registers");
  Serial.println(*SPI0_SPDR_HA,HEX);
  for(spi_val_index = 0; spi_val_index <= 0x3; spi_val_index++)
    {
    Serial.print(spi_val_index,HEX);  
    Serial.print(" = ");  
    local_spi_val = *((volatile unsigned char *)(SPIBASE + 0x2000 + spi_val_index + (0x100 * spiregval)));            //
    Serial.println(local_spi_val, HEX);
    }
  for(spi_val_index = 0x0A; spi_val_index <= 0x0F; spi_val_index++)
    {
    Serial.print(spi_val_index,HEX);  
    Serial.print(" = ");  
    local_spi_val = *((volatile unsigned char *)(SPIBASE + 0x2000 + spi_val_index + (0x100 * spiregval)));            //
    Serial.println(local_spi_val, HEX);
    }
  Serial.print(spi_val_index,HEX);  
  Serial.print(" = ");  
  local_spi_val = *((volatile unsigned short *)(SPIBASE + 0x2000 + spi_val_index + (0x100 * spiregval)));            //
  Serial.println(local_spi_val, HEX);

  Serial.println(" ");  
  }

void print_sci_serial_reg(int sciregval)
  {
  Serial.print("SCI");
  Serial.print(sciregval);
  Serial.println(" Registers");
 
  Serial.print("*SCI");
  Serial.print(sciregval);
  Serial.print("_RDR");
  Serial.print(" = 0x");
  Serial.print(*((volatile unsigned char  *)(SCIBASE + 0x0005 + (0x20 * sciregval))),HEX); 
  Serial.println("; // Receive Data Register");
  Serial.print("*SCI");
  Serial.print(sciregval);
  Serial.print("_TDR");
  Serial.print(" = 0x");
  Serial.print(*((volatile unsigned char  *)(SCIBASE + 0x0003 + (0x20 * sciregval))),HEX); 
  Serial.println("; // Transmit Data Register");
  Serial.print("*SCI");
  Serial.print(sciregval);
  Serial.print("_SMR");
  Serial.print(" = 0x");
  Serial.print(*((volatile unsigned char  *)(SCIBASE + 0x0000 + (0x20 * sciregval))),HEX); 
  Serial.println("; // Serial Mode Register");
  Serial.print("*SCI");
  Serial.print(sciregval);
  Serial.print("_SSR");
  Serial.print(" = 0x");
  Serial.print(*((volatile unsigned char  *)(SCIBASE + 0x0004 + (0x20 * sciregval))),HEX); 
  Serial.println("; // Serial Status Register");
  Serial.print("*SCI");
  Serial.print(sciregval);
  Serial.print("_SCMR");
  Serial.print(" = 0x");
  Serial.print(*((volatile unsigned char  *)(SCIBASE + 0x0006 + (0x20 * sciregval))),HEX); 
  Serial.println("; // Smart Card Mode Register - and other settings");
  Serial.print("*SCI");
  Serial.print(sciregval);
  Serial.print("_BRR");
  Serial.print(" = 0x");
  Serial.print(*((volatile unsigned char  *)(SCIBASE + 0x0001 + (0x20 * sciregval))),HEX); 
  Serial.println("; // Bit Rate Register");
  Serial.print("*SCI");
  Serial.print(sciregval);
  Serial.print("_MDDR");
  Serial.print(" = 0x");
  Serial.print(*((volatile unsigned char  *)(SCIBASE + 0x0012 + (0x20 * sciregval))),HEX); 
  Serial.println("; // Modulation Duty Register");
  Serial.print("*SCI");
  Serial.print(sciregval);
  Serial.print("_SEMR");
  Serial.print(" = 0x");
  Serial.print(*((volatile unsigned char  *)(SCIBASE + 0x0007 + (0x20 * sciregval))),HEX); 
  Serial.println("; // Serial Extended Mode Register");
  Serial.print("*SCI");
  Serial.print(sciregval);
  Serial.print("_SNFR");
  Serial.print(" = 0x");
  Serial.print(*((volatile unsigned char  *)(SCIBASE + 0x0008 + (0x20 * sciregval))),HEX); 
  Serial.println("; // Noise Filter Setting Register");
  Serial.print("*SCI");
  Serial.print(sciregval);
  Serial.print("_SPTR");
  Serial.print(" = 0x");
  Serial.print(*((volatile unsigned char  *)(SCIBASE + 0x001C + (0x20 * sciregval))),HEX); 
  Serial.println("; // Serial Port Register");
  Serial.print("*SCI");
  Serial.print(sciregval);
  Serial.print("_SCR");
  Serial.print(" = 0x");
  Serial.print(*((volatile unsigned char  *)(SCIBASE + 0x0002 + (0x20 * sciregval))),HEX); 
  Serial.println("; // Serial Control Register");
  }


// Function: print_icu_event_links();
// The following is to determin which interrupts are in use
// RA4M1 Group ICU Event Number Table 13.4 
//
// Slot - Event Number - Name of the Interrupt
// 0 - 33 - USBFS_USBI
// 1 - 34 - USBFS_USBR
// 2 - 31 - USBFS_D0FIFO
// 3 - 32 - USBFS_D1FIFO
// 4 - 1E - AGT0_AGTI
//
// The above 5 entries are always present before the code gets to setup()

// Use PROGMEM structures to place strings into program memory 
// https://www.arduino.cc/reference/en/language/variables/utilities/progmem/

const char string_00[] PROGMEM = "No_Event";
const char string_01[] PROGMEM = "PORT_IRQ0";
const char string_02[] PROGMEM = "PORT_IRQ1";
const char string_03[] PROGMEM = "PORT_IRQ2";
const char string_04[] PROGMEM = "PORT_IRQ3";
const char string_05[] PROGMEM = "PORT_IRQ4";
const char string_06[] PROGMEM = "PORT_IRQ5";
const char string_07[] PROGMEM = "PORT_IRQ6";
const char string_08[] PROGMEM = "PORT_IRQ7";
const char string_09[] PROGMEM = "PORT_IRQ8";
const char string_0A[] PROGMEM = "PORT_IRQ9";
const char string_0B[] PROGMEM = "PORT_IRQ10";
const char string_0C[] PROGMEM = "PORT_IRQ11";
const char string_0D[] PROGMEM = "PORT_IRQ12";
const char string_0E[] PROGMEM = "PORT_UNUSED";
const char string_0F[] PROGMEM = "PORT_IRQ14";
const char string_10[] PROGMEM = "PORT_IRQ15";
const char string_11[] PROGMEM = "DMAC0_INT";
const char string_12[] PROGMEM = "DMAC1_INT";
const char string_13[] PROGMEM = "DMAC2_INT";
const char string_14[] PROGMEM = "DMAC3_INT";
const char string_15[] PROGMEM = "DTC_COMPLETE";
const char string_16[] PROGMEM = "UNUSED";
const char string_17[] PROGMEM = "ICU_SNZCANCEL";
const char string_18[] PROGMEM = "FCU_FRDYI";
const char string_19[] PROGMEM = "LVD_LVD1";
const char string_1A[] PROGMEM = "LVD_LVD2";
const char string_1B[] PROGMEM = "VBATT_LVD";
const char string_1C[] PROGMEM = "MOSC_STOP";
const char string_1D[] PROGMEM = "SYSTEM_SNZREQ";
const char string_1E[] PROGMEM = "AGT0_AGTI";
const char string_1F[] PROGMEM = "AGT0_AGTCMAI";
const char string_20[] PROGMEM = "AGT0_AGTCMBI";
const char string_21[] PROGMEM = "AGT1_AGTI";
const char string_22[] PROGMEM = "AGT1_AGTCMAI";
const char string_23[] PROGMEM = "AGT1_AGTCMBI";
const char string_24[] PROGMEM = "IWDT_NMIUNDF";
const char string_25[] PROGMEM = "WDT_NMIUNDF";
const char string_26[] PROGMEM = "RTC_ALM";
const char string_27[] PROGMEM = "RTC_PRD";
const char string_28[] PROGMEM = "RTC_CUP";
const char string_29[] PROGMEM = "ADC140_ADI";
const char string_2A[] PROGMEM = "ADC140_GBADI";
const char string_2B[] PROGMEM = "ADC140_CMPAI";
const char string_2C[] PROGMEM = "ADC140_CMPBI";
const char string_2D[] PROGMEM = "ADC140_WCMPM";
const char string_2E[] PROGMEM = "ADC140_WCMPUM";
const char string_2F[] PROGMEM = "ACMP_LP0";
const char string_30[] PROGMEM = "ACMP_LP1";
const char string_31[] PROGMEM = "USBFS_D0FIFO";
const char string_32[] PROGMEM = "USBFS_D1FIFO";
const char string_33[] PROGMEM = "USBFS_USBI";
const char string_34[] PROGMEM = "USBFS_USBR";
const char string_35[] PROGMEM = "IIC0_RXI";
const char string_36[] PROGMEM = "IIC0_TXI";
const char string_37[] PROGMEM = "IIC0_TEI";
const char string_38[] PROGMEM = "IIC0_EEI";
const char string_39[] PROGMEM = "IIC0_WUI";
const char string_3A[] PROGMEM = "IIC1_RXI";
const char string_3B[] PROGMEM = "IIC1_TXI";
const char string_3C[] PROGMEM = "IIC1_TEI";
const char string_3D[] PROGMEM = "IIC1_EEI";
const char string_3E[] PROGMEM = "SSIE0_SSITXI";
const char string_3F[] PROGMEM = "SSIE0_SSIRXI";
const char string_40[] PROGMEM = "UNUSED";
const char string_41[] PROGMEM = "SSIE0_SSIF";
const char string_42[] PROGMEM = "CTSU_CTSUWR";
const char string_43[] PROGMEM = "CTSU_CTSURD";
const char string_44[] PROGMEM = "CTSU_CTSUFN";
const char string_45[] PROGMEM = "KEY_INTKR";
const char string_46[] PROGMEM = "DOC_DOPCI";
const char string_47[] PROGMEM = "CAC_FERRI";
const char string_48[] PROGMEM = "CAC_MENDI";
const char string_49[] PROGMEM = "CAC_OVFI";
const char string_4A[] PROGMEM = "CAN0_ERS";
const char string_4B[] PROGMEM = "CAN0_RXF";
const char string_4C[] PROGMEM = "CAN0_TXF";
const char string_4D[] PROGMEM = "CAN0_RXM";
const char string_4E[] PROGMEM = "CAN0_TXM";
const char string_4F[] PROGMEM = "IOPORT_GROUP1";
const char string_50[] PROGMEM = "IOPORT_GROUP2";
const char string_51[] PROGMEM = "IOPORT_GROUP3";
const char string_52[] PROGMEM = "IOPORT_GROUP4";
const char string_53[] PROGMEM = "ELC_SWEVT0";
const char string_54[] PROGMEM = "ELC_SWEVT1";
const char string_55[] PROGMEM = "POEG_GROUP0";
const char string_56[] PROGMEM = "POEG_GROUP1";
const char string_57[] PROGMEM = "GPT0_CCMPA";
const char string_58[] PROGMEM = "GPT0_CCMPB";
const char string_59[] PROGMEM = "GPT0_CMPC";
const char string_5A[] PROGMEM = "GPT0_CMPD";
const char string_5B[] PROGMEM = "GPT0_CMPE";
const char string_5C[] PROGMEM = "GPT0_CMPF";
const char string_5D[] PROGMEM = "GPT0_OVF";
const char string_5E[] PROGMEM = "GPT0_UDF";
const char string_5F[] PROGMEM = "GPT1_CCMPA";
const char string_60[] PROGMEM = "GPT1_CCMPB";
const char string_61[] PROGMEM = "GPT1_CMPC";
const char string_62[] PROGMEM = "GPT1_CMPD";
const char string_63[] PROGMEM = "GPT1_CMPE";
const char string_64[] PROGMEM = "GPT1_CMPF";
const char string_65[] PROGMEM = "GPT1_OVF";
const char string_66[] PROGMEM = "GPT1_UDF";
const char string_67[] PROGMEM = "GPT2_CCMPA";
const char string_68[] PROGMEM = "GPT2_CCMPB";
const char string_69[] PROGMEM = "GPT2_CMPC";
const char string_6A[] PROGMEM = "GPT2_CMPD";
const char string_6B[] PROGMEM = "GPT2_CMPE";
const char string_6C[] PROGMEM = "GPT2_CMPF";
const char string_6D[] PROGMEM = "GPT2_OVF";
const char string_6E[] PROGMEM = "GPT2_UDF";
const char string_6F[] PROGMEM = "GPT3_CCMPA";
const char string_70[] PROGMEM = "GPT3_CCMPB";
const char string_71[] PROGMEM = "GPT3_CMPC";
const char string_72[] PROGMEM = "GPT3_CMPD";
const char string_73[] PROGMEM = "GPT3_CMPE";
const char string_74[] PROGMEM = "GPT3_CMPF";
const char string_75[] PROGMEM = "GPT3_OVF";
const char string_76[] PROGMEM = "GPT3_UDF";
const char string_77[] PROGMEM = "GPT4_CCMPA";
const char string_78[] PROGMEM = "GPT4_CCMPB";
const char string_79[] PROGMEM = "GPT4_CMPC";
const char string_7A[] PROGMEM = "GPT4_CMPD";
const char string_7B[] PROGMEM = "GPT4_CMPE";
const char string_7C[] PROGMEM = "GPT4_CMPF";
const char string_7D[] PROGMEM = "GPT4_OVF";
const char string_7E[] PROGMEM = "GPT4_UDF";
const char string_7F[] PROGMEM = "GPT5_CCMPA";
const char string_80[] PROGMEM = "GPT5_CCMPB";
const char string_81[] PROGMEM = "GPT5_CMPC";
const char string_82[] PROGMEM = "GPT5_CMPD";
const char string_83[] PROGMEM = "GPT5_CMPE";
const char string_84[] PROGMEM = "GPT5_CMPF";
const char string_85[] PROGMEM = "GPT5_OVF";
const char string_86[] PROGMEM = "GPT5_UDF";
const char string_87[] PROGMEM = "GPT6_CCMPA";
const char string_88[] PROGMEM = "GPT6_CCMPB";
const char string_89[] PROGMEM = "GPT6_CMPC";
const char string_8A[] PROGMEM = "GPT6_CMPD";
const char string_8B[] PROGMEM = "GPT6_CMPE";
const char string_8C[] PROGMEM = "GPT6_CMPF";
const char string_8D[] PROGMEM = "GPT6_OVF";
const char string_8E[] PROGMEM = "GPT6_UDF";
const char string_8F[] PROGMEM = "GPT7_CCMPA";
const char string_90[] PROGMEM = "GPT7_CCMPB";
const char string_91[] PROGMEM = "GPT7_CMPC";
const char string_92[] PROGMEM = "GPT7_CMPD";
const char string_93[] PROGMEM = "GPT7_CMPE";
const char string_94[] PROGMEM = "GPT7_CMPF";
const char string_95[] PROGMEM = "GPT7_OVF";
const char string_96[] PROGMEM = "GPT7_UDF";
const char string_97[] PROGMEM = "GPT_UVWEDGE";
const char string_98[] PROGMEM = "SCI0_RXI";
const char string_99[] PROGMEM = "SCI0_TXI";
const char string_9A[] PROGMEM = "SCI0_TEI";
const char string_9B[] PROGMEM = "SCI0_ERI";
const char string_9C[] PROGMEM = "SCI0_AM";
const char string_9D[] PROGMEM = "SCI0_RXI_OR_ERI";
const char string_9E[] PROGMEM = "SCI1_RXI";
const char string_9F[] PROGMEM = "SCI1_TXI";
const char string_A0[] PROGMEM = "SCI1_TEI";
const char string_A1[] PROGMEM = "SCI1_ERI";
const char string_A2[] PROGMEM = "SCI1_AM";
const char string_A3[] PROGMEM = "SCI2_RXI";
const char string_A4[] PROGMEM = "SCI2_TXI";
const char string_A5[] PROGMEM = "SCI2_TEI";
const char string_A6[] PROGMEM = "SCI2_ERI";
const char string_A7[] PROGMEM = "SCI2_AM";
const char string_A8[] PROGMEM = "SCI9_RXI";
const char string_A9[] PROGMEM = "SCI9_TXI";
const char string_AA[] PROGMEM = "SCI9_TEI";
const char string_AB[] PROGMEM = "SCI9_ERI";
const char string_AC[] PROGMEM = "SCI9_AM";
const char string_AD[] PROGMEM = "SPI0_SPRI";
const char string_AE[] PROGMEM = "SPI0_SPTI";
const char string_AF[] PROGMEM = "SPI0_SPII";
const char string_B0[] PROGMEM = "SPI0_SPEI";
const char string_B1[] PROGMEM = "SPI0_SPTEND";
const char string_B2[] PROGMEM = "SPI1_SPRI";
const char string_B3[] PROGMEM = "SPI1_SPTI";
const char string_B4[] PROGMEM = "SPI1_SPII";
const char string_B5[] PROGMEM = "SPI1_SPEI";
const char string_B6[] PROGMEM = "SPI1_SPTEND";
const char string_B7[] PROGMEM = "UNUSED";

const char *const string_table[] PROGMEM = {
string_00, string_01, string_02, string_03, string_04, string_05, string_06, string_07,
string_08, string_09, string_0A, string_0B, string_0C, string_0D, string_0E, string_0F,
string_10, string_11, string_12, string_13, string_14, string_15, string_16, string_17,
string_18, string_19, string_1A, string_1B, string_1C, string_1D, string_1E, string_1F,
string_20, string_21, string_22, string_23, string_24, string_25, string_26, string_27,
string_28, string_29, string_2A, string_2B, string_2C, string_2D, string_2E, string_2F,
string_30, string_31, string_32, string_33, string_34, string_35, string_36, string_37,
string_38, string_39, string_3A, string_3B, string_3C, string_3D, string_3E, string_3F,
string_40, string_41, string_42, string_43, string_44, string_45, string_46, string_47,
string_48, string_49, string_4A, string_4B, string_4C, string_4D, string_4E, string_4F,
string_50, string_51, string_52, string_53, string_54, string_55, string_56, string_57,
string_58, string_59, string_5A, string_5B, string_5C, string_5D, string_5E, string_5F,
string_60, string_61, string_62, string_63, string_64, string_65, string_66, string_67,
string_68, string_69, string_6A, string_6B, string_6C, string_6D, string_6E, string_6F,
string_70, string_71, string_72, string_73, string_74, string_75, string_76, string_77,
string_78, string_79, string_7A, string_7B, string_7C, string_7D, string_7E, string_7F,
string_80, string_81, string_82, string_83, string_84, string_85, string_86, string_87,
string_88, string_89, string_8A, string_8B, string_8C, string_8D, string_8E, string_8F,
string_90, string_91, string_92, string_93, string_94, string_95, string_96, string_97,
string_98, string_99, string_9A, string_9B, string_9C, string_9D, string_9E, string_9F,
string_A0, string_A1, string_A2, string_A3, string_A4, string_A5, string_A6, string_A7,
string_A8, string_A9, string_AA, string_AB, string_AC, string_AD, string_AE, string_AF,
string_B0, string_B1, string_B2, string_B3, string_B4, string_B5, string_B6, string_B7
};

char message_buffer[30];  // 

void print_icu_event_links(void)
  {
  unsigned int local_icu_val = 0;
  unsigned char icu_val_index = 0;

  for(icu_val_index = 0; icu_val_index < 32; icu_val_index++)
    {
    Serial.print(icu_val_index);  
    Serial.print(" - ");  
    local_icu_val = *((volatile unsigned int *)(ICUBASE + IELSR + (icu_val_index * 4)));            //
    Serial.print(local_icu_val, HEX);
    strcpy_P(message_buffer, (char *)pgm_read_word(&(string_table[local_icu_val])));  // 
    Serial.print(" - ");  
    Serial.println(message_buffer);
    if(local_icu_val == IRQ_NO_EVENT) break;      // Only print active allocations - these are always contigious from 0
    }
  Serial.println(" ");  
  }
