 /*
 * MAIN Generated Driver File
 * 
 * @file main.c
 * 
 * @defgroup main MAIN
 * 
 * @brief This is the generated driver implementation file for the MAIN driver.
 *
 * @version MAIN Driver Version 1.0.0
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
#include "mcc_generated_files/system/system.h"
#include "application/hal_board_cfg.h"
#include "application/app_task.h"


void gpio_Init( void )              
{                               
    // Power on                         
    // VBAT_CN                          
    IO_RA2_SetDigitalOutput();              
    IO_RA2_SetPushPull();               
    IO_RA2_SetHigh();               
    
    // SW_TEST, INPUT,PULL UP ENABLE
    IO_RF0_SetDigitalInput(); 
    IO_RF0_SetPullup();
    IO_RF0_SetHigh();
    
    // ULN-IN1 --- ULN-IN4 PushPull Output
    IO_RB3_SetDigitalMode();
    IO_RB3_SetDigitalOutput();   //IN4
    IO_RB3_SetPushPull();
    IO_RB3_SetLow();
    
    IO_RF4_SetDigitalMode();
    IO_RF4_SetDigitalOutput();   //IN3
    IO_RF4_SetPushPull();
    IO_RF4_SetLow();    
    
    IO_RF5_SetDigitalMode();
    IO_RF5_SetDigitalOutput();   //IN2
    IO_RF5_SetPushPull();
    IO_RF5_SetLow();
    
    IO_RF6_SetDigitalMode();
    IO_RF6_SetDigitalOutput();   //IN1
    IO_RF6_SetPushPull();
    IO_RF6_SetLow();  
    
    IO_RF7_SetDigitalMode();
    IO_RF7_SetDigitalOutput();   //IN0
    IO_RF7_SetPushPull();
    IO_RF7_SetLow();  
    
 // DC24V_EN   RA0 
    IO_RA0_SetDigitalMode();
    IO_RA0_SetDigitalOutput();   // DC24V_EN
    IO_RA0_SetPushPull();
    IO_RA0_SetLow();
    
 // DC8V_EN   RA1   
    IO_RA1_SetDigitalMode();
    IO_RA1_SetDigitalOutput();   // DC8V_EN
    IO_RA1_SetPushPull();        
    IO_RA1_SetLow(); 
    
 // PVCC_CON
    IO_RC2_SetDigitalMode();
    IO_RC2_SetDigitalOutput();   // PVCC_CN
    IO_RC2_SetPushPull();        
    IO_RC2_SetHigh();             
    
 // 5VIN_TEST, INPUT, PULL UP DISABLE 
    IO_RD4_SetDigitalMode();    
    IO_RD4_SetDigitalInput(); 
    IO_RD4_ResetPullup();
    IO_RD4_SetHigh();
    
 // CHRG_TEST, INPUT, PULL UP ENABLE
    IO_RD5_SetDigitalMode();
    IO_RD5_SetDigitalInput(); 
    IO_RD5_SetPullup();  
    IO_RD5_SetHigh();

    // ANC3--RC3-ADC-AD_PDI
    IO_RC3_SetDigitalInput();
    IO_RC3_ResetPullup();
    IO_RC3_SetAnalogMode();
    IO_RC3_SetLow();  
    
    // ANC6--RC6-ADC-AD_CUT
    IO_RC6_SetDigitalInput();
    IO_RC6_ResetPullup();
    IO_RC6_SetAnalogMode();
    IO_RC6_SetLow();  
    
   // ANC7--RC7-ADC-AD_BOUT
    IO_RC7_SetDigitalInput();
    IO_RC7_ResetPullup();
    IO_RC7_SetAnalogMode();
    IO_RC7_SetLow();
    
}

/*
    Main application
*/

int main(void)
{
    SYSTEM_Initialize();

    // If using interrupts in PIC18 High/Low Priority Mode you need to enable the Global High and Low Interrupts 
    // If using interrupts in PIC Mid-Range Compatibility Mode you need to enable the Global Interrupts 
    // Use the following macros to: 

    // Enable the Global High Interrupts 
    INTERRUPT_GlobalInterruptHighEnable(); 

    // Disable the Global High Interrupts 
    //INTERRUPT_GlobalInterruptHighDisable(); 

    // Enable the Global Low Interrupts 
    //INTERRUPT_GlobalInterruptLowEnable(); 

    // Disable the Global Low Interrupts 
    //INTERRUPT_GlobalInterruptLowDisable(); 
    VBAT_CN_ON(); 
    Timer1_Start();
    gpio_Init();    
    app_task_Init();
    WWDT_SoftEnable();
    while(1)
    {
        app_task_Callback();
    }    
}