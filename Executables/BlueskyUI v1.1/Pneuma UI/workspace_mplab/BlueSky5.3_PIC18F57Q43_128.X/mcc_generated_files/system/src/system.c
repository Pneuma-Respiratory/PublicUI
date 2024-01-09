/**
 * System Driver Source File
 * 
 * @file system.c
 * 
 * @ingroup systemdriver
 * 
 * @brief This file contains the API implementation for the System driver.
 *
 * @version Driver Version 2.0.2
*/

/*
© [2024] Microchip Technology Inc. and its subsidiaries.

    Subject to your compliance with these terms, you may use Microchip 
    software and any derivatives exclusively with Microchip products. 
    You are responsible for complying with 3rd party license terms  
    applicable to your use of 3rd party software (including open source  
    software) that may accompany Microchip software. SOFTWARE IS ?AS IS.? 
    NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS 
    SOFTWARE, INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT,  
    MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT 
    WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY 
    KIND WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF 
    MICROCHIP HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE 
    FORESEEABLE. TO THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP?S 
    TOTAL LIABILITY ON ALL CLAIMS RELATED TO THE SOFTWARE WILL NOT 
    EXCEED AMOUNT OF FEES, IF ANY, YOU PAID DIRECTLY TO MICROCHIP FOR 
    THIS SOFTWARE.
*/

#include "../system.h"

void PMD_Initialize(void);

void SYSTEM_Initialize(void)
{
    CLOCK_Initialize();
    PIN_MANAGER_Initialize();
    ADCC_Initialize();
    CLKREF_Initialize();
    CWG1_Initialize();
    FVR_Initialize();
    NCO1_Initialize();
    NVM_Initialize();
    PMD_Initialize();
    Timer1_Initialize();
    UART2_Initialize();
    UART3_Initialize();
    WWDT_Initialize();
    INTERRUPT_Initialize();
}

void PMD_Initialize(void)
{
    //IOCMD IOC enabled; CLKRMD CLKR enabled; SCANMD SCANNER disabled; CRCMD CRC enabled; FVRMD FVR enabled; SYSCMD SYSCLK enabled; HLVDMD HLVD enabled; 
    PMD0 = 0x8;
    //TMR0MD TMR0 enabled; TMR1MD TMR1 enabled; TMR2MD TMR2 disabled; TMR3MD TMR3 disabled; TMR4MD TMR4 disabled; TMR5MD TMR5 disabled; TMR6MD TMR6 disabled; SMT1MD SMT1 enabled; 
    PMD1 = 0x7C;
    //ZCDMD ZCD disabled; CM1MD CM1 disabled; CM2MD CM2 disabled; ACTMD ACT enabled; ADCMD ADC enabled; DAC1MD DAC1 disabled; 
    PMD3 = 0x47;
    //CWG1MD CWG1 enabled; CWG2MD CWG2 enabled; CWG3MD CWG3 enabled; NCO1MD NCO1 enabled; NCO2MD NCO2 enabled; NCO3MD NCO3 enabled; DSM1MD DSM1 disabled; 
    PMD4 = 0x8;
    //CCP1MD CCP1 disabled; CCP2MD CCP2 disabled; CCP3MD CCP3 enabled; PWM1MD PWM1 enabled; PWM2MD PWM2 enabled; PWM3MD PWM3 enabled; 
    PMD5 = 0x3;
    //U1MD UART1 enabled; U2MD UART2 enabled; U3MD UART3 enabled; U4MD UART4 disabled; U5MD UART5 disabled; SPI1MD SPI1 disabled; SPI2MD SPI2 disabled; I2C1MD I2C1 disabled; 
    PMD6 = 0xC7;
    //CLC1MD CLC1 enabled; CLC2MD CLC2 enabled; CLC3MD CLC3 disabled; CLC4MD CLC4 disabled; CLC5MD CLC5 disabled; CLC6MD CLC6 disabled; CLC7MD CLC7 disabled; CLC8MD CLC8 disabled; 
    PMD7 = 0xFC;
    //DMA1MD DMA1 enabled; DMA2MD DMA2 enabled; DMA6MD DMA6 disabled; DMA3MD DMA3 disabled; DMA4MD DMA4 disabled; DMA5MD DMA5 disabled; 
    PMD8 = 0x3C;
}


