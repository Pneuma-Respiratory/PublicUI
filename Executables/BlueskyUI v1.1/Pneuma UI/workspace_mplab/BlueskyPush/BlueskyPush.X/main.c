/**
  Generated Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This is the main file generated using PIC10 / PIC12 / PIC16 / PIC18 MCUs

  Description:
    This header file provides implementations for driver APIs for all modules selected in the GUI.
    Generation Information :
        Product Revision  :  PIC10 / PIC12 / PIC16 / PIC18 MCUs - 1.81.8
        Device            :  PIC16F18877
        Driver Version    :  2.00
*/

/*
    (c) 2018 Microchip Technology Inc. and its subsidiaries. 
    
    Subject to your compliance with these terms, you may use Microchip software and any 
    derivatives exclusively with Microchip products. It is your responsibility to comply with third party 
    license terms applicable to your use of third party software (including open source software) that 
    may accompany Microchip software.
    
    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER 
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY 
    IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS 
    FOR A PARTICULAR PURPOSE.
    
    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP 
    HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO 
    THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL 
    CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT 
    OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS 
    SOFTWARE.
 * 
*/

#include "mcc_generated_files/mcc.h"
#include "application/app_task.h"

extern uint16_t count_200us;

void gpio_Init( void )
{
    // Power on
    // RA2--VBAT_CN
    IO_RA2_SetDigitalOutput();  
    IO_RA2_SetPushPull();  
    IO_RA2_SetHigh(); 
    
    // RC1--SW_TEST ,RC1 AS INPUT,PULL UP ENABLE
    IO_RC1_SetDigitalInput(); 
    IO_RC1_SetPullup();
    IO_RC1_SetHigh();
    
    // ULN-IN1 --- ULN-IN4 (RB3--RB0) PushPull Output
    IO_RB0_SetDigitalMode();
    IO_RB0_SetDigitalOutput();   //IN4
    IO_RB0_SetPushPull();
    IO_RB0_SetLow();
    
    IO_RB1_SetDigitalMode();
    IO_RB1_SetDigitalOutput();   //IN3
    IO_RB1_SetPushPull();
    IO_RB1_SetLow();    
    
    IO_RB2_SetDigitalMode();
    IO_RB2_SetDigitalOutput();   //IN2
    IO_RB2_SetPushPull();
    IO_RB2_SetLow();
    
    IO_RB3_SetDigitalMode();
    IO_RB3_SetDigitalOutput();   //IN1
    IO_RB3_SetPushPull();
    IO_RB3_SetLow();  
    
 // DC24V_EN   RA0 
    IO_RA0_SetDigitalMode();
    IO_RA0_SetDigitalOutput();   //RA0--DC24V_EN
    IO_RA0_SetPushPull();
    IO_RA0_SetLow();
    
 // DC8V_EN   RA1   
    IO_RA1_SetDigitalMode();
    IO_RA1_SetDigitalOutput();   //RA1--DC8V_EN
    IO_RA1_SetPushPull();        
    IO_RA1_SetLow(); 
    
 // PVCC_CON   RC0
    IO_RC0_SetDigitalMode();
    IO_RC0_SetDigitalOutput();   //RC0--PVCC_CN
    IO_RC0_SetPushPull();        
    IO_RC0_SetLow();    
    
 // 5VIN_TEST  RD6 AS INPUT, PULL UP DISABLE 
    IO_RD6_SetDigitalMode();    
    IO_RD6_SetDigitalInput(); 
    IO_RD6_ResetPullup();
    IO_RD6_SetHigh();
    
 // CHRG_TEST  RD7 AS INPUT, PULL UP ENABLE
    IO_RD7_SetDigitalMode();
    IO_RD7_SetDigitalInput(); 
    IO_RD7_SetPullup();  
    IO_RD7_SetHigh();
// AND4--AD_CUT      
    channel_AND4_SetDigitalInput();
    channel_AND4_ResetPullup();
    channel_AND4_SetAnalogMode();
    channel_AND4_SetLow();   

    // AND5--AD_BOUT
    channel_AND5_SetDigitalInput();
    channel_AND5_ResetPullup();
    channel_AND5_SetAnalogMode();
    channel_AND5_SetLow();
    
   // ANC2--PD2-ADC
    channel_ANC2_SetDigitalInput();
    channel_ANC2_ResetPullup();
    channel_ANC2_SetAnalogMode();
    channel_ANC2_SetLow();  
    
  // ANC3--PD1-ADC   
    channel_ANC3_SetDigitalInput();
    channel_ANC3_ResetPullup();
    channel_ANC3_SetAnalogMode();
    channel_ANC3_SetLow();
    
}

/*
                         Main application
 */
void main(void)
{
    uint16_t ADC_value3;
    // initialize the device
    SYSTEM_Initialize();

    gpio_Init();
    // When using interrupts, you need to set the Global and Peripheral Interrupt Enable bits
    // Use the following macros to:

    // Enable the Global Interrupts
    INTERRUPT_GlobalInterruptEnable();

    // Enable the Peripheral Interrupts
    INTERRUPT_PeripheralInterruptEnable();

    // Disable the Global Interrupts
    //INTERRUPT_GlobalInterruptDisable();

    // Disable the Peripheral Interrupts
    //INTERRUPT_PeripheralInterruptDisable();

    app_task_Init();
    
//    app_task_Test();
    
    while (1)
    {
#if 0
        // Add your application code
        if (count_200us>5000)
        {
            count_200us=0;
            ADC_value3=ADCC_GetSingleConversion(channel_ANA3);
            EUSART_Write(0x5a);
            EUSART_Write( (uint8_t)((ADC_value3&0xff00)>>8) );
            EUSART_Write( (uint8_t)((ADC_value3&0x00ff))    );
            EUSART_Write(0xa5);
        }
#endif
        app_task_Callback();
    }
}
/**
 End of File
*/