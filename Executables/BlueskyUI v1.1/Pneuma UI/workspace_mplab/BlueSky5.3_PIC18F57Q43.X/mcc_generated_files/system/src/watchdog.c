/**
 * WWDT Generated Driver File.
 * 
 * @file wwdt.c
 * 
 * @ingroup  wwdt
 * 
 * @brief This file contains the API implementation for the WWDT driver.
 *
 * @version WWDT Driver Version 2.0.0
*/
/*
? [2023] Microchip Technology Inc. and its subsidiaries.

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

#include "../watchdog.h"

void WWDT_Initialize(void)
{
    WDTCON0 = WDTCPS;
    WDTCON1 = WDTCWS;     // WDTCWS|WDTCCS;    
}

void WWDT_SoftEnable(void)
{
    // WWDT software enable. 
    WDTCON0bits.SEN=1;
}

void WWDT_SoftDisable(void)
{
    // WWDT software disable.
    WDTCON0bits.SEN=0;
}

bool WWDT_TimeOutStatusGet(void)
{
    return (0);
}

bool WWDT_WindowViolationStatusGet(void)
{
    return (0);
}  

void WWDT_TimerClear(void)
{
    // Disable the interrupt,read back the WDTCON0 reg for arming, 
    // clearing the WWDT and enable the interrupt.
    uint8_t readBack=0;
    
    bool state = GIE;
    GIE = 0;
    readBack = WDTCON0;
    CLRWDT();
    GIE = state;
}

/**
 End of File
*/