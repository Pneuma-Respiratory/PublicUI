/**
 * Interrupt Manager Generated Driver File.
 *
 * @file interrupt.c
 * 
 * @ingroup interrupt 
 * 
 * @brief This file contains the API implementation for the Interrupt Manager driver.
 * 
 * @version Interrupt Manager Driver Version 2.1.3
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

#include "../../system/interrupt.h"
#include "../../system/system.h"
#include "../pins.h"

void (*INT0_InterruptHandler)(void);
void (*INT1_InterruptHandler)(void);
void (*INT2_InterruptHandler)(void);

void  INTERRUPT_Initialize (void)
{
    // Enable Interrupt Priority Vectors (16CXXX Compatibility Mode)
    INTCON0bits.IPEN = 1;

    // Assign peripheral interrupt priority vectors

    // TMRI - high priority
    IPR3bits.TMR1IP = 1;

    // URXI - high priority
    IPR9bits.U3RXIP = 1;

    // URXI - high priority
    IPR8bits.U2RXIP = 1;

    // ADI - high priority
    IPR1bits.ADIP = 1;


    // IOCI - low priority
    IPR0bits.IOCIP = 0;    

    // UTXI - low priority
    IPR9bits.U3TXIP = 0;    

    // UEI - low priority
    IPR9bits.U3EIP = 0;    

    // UTXI - low priority
    IPR8bits.U2TXIP = 0;    

    // UEI - low priority
    IPR8bits.U2EIP = 0;    


    // Clear the interrupt flag
    // Set the external interrupt edge detect
    EXT_INT0_InterruptFlagClear();   
    EXT_INT0_risingEdgeSet();    
    // Set Default Interrupt Handler
    INT0_SetInterruptHandler(INT0_DefaultInterruptHandler);
    // EXT_INT0_InterruptEnable();

    // Clear the interrupt flag
    // Set the external interrupt edge detect
    EXT_INT1_InterruptFlagClear();   
    EXT_INT1_risingEdgeSet();    
    // Set Default Interrupt Handler
    INT1_SetInterruptHandler(INT1_DefaultInterruptHandler);
    // EXT_INT1_InterruptEnable();

    // Clear the interrupt flag
    // Set the external interrupt edge detect
    EXT_INT2_InterruptFlagClear();   
    EXT_INT2_risingEdgeSet();    
    // Set Default Interrupt Handler
    INT2_SetInterruptHandler(INT2_DefaultInterruptHandler);
    // EXT_INT2_InterruptEnable();

}

/**
 * @ingroup interrupt
 * @brief Executes whenever a high-priority interrupt is triggered. This routine checks the source of the interrupt and calls the relevant interrupt function.
 * @pre INTERRUPT_Initialize() is already called.
 * @param None.
 * @return None.
 */
void __interrupt() INTERRUPT_InterruptManagerHigh (void)
{
    // interrupt handler
    if(PIE3bits.TMR1IE == 1 && PIR3bits.TMR1IF == 1)
    {
        Timer1_OverflowISR();
    }
    if(PIE9bits.U3RXIE == 1 && PIR9bits.U3RXIF == 1)
    {
        UART3_RxInterruptHandler();
    }
    if(PIE8bits.U2RXIE == 1 && PIR8bits.U2RXIF == 1)
    {
        UART2_RxInterruptHandler();
    }
    if(PIE1bits.ADIE == 1 && PIR1bits.ADIF == 1)
    {
        ADCC_ISR();
    }
}

/**
 * @ingroup interrupt
 * @brief Executes whenever a low-priority interrupt is triggered. This routine checks the source of the interrupt and calls the relevant interrupt function.
 * @pre INTERRUPT_Initialize() is already called.
 * @param None.
 * @return None.
 */
void __interrupt(low_priority) INTERRUPT_InterruptManagerLow (void)
{
    // interrupt handler
    if(PIE0bits.IOCIE == 1 && PIR0bits.IOCIF == 1)
    {
        PIN_MANAGER_IOC();
    }
    if(PIE9bits.U3TXIE == 1 && PIR9bits.U3TXIF == 1)
    {
        UART3_TxInterruptHandler();
    }
    if(PIE9bits.U3EIE == 1 && PIR9bits.U3EIF == 1)
    {
        UART3_ErrorInterruptHandler();
    }
    if(PIE8bits.U2TXIE == 1 && PIR8bits.U2TXIF == 1)
    {
        UART2_TxInterruptHandler();
    }
    if(PIE8bits.U2EIE == 1 && PIR8bits.U2EIF == 1)
    {
        UART2_ErrorInterruptHandler();
    }
}

void INT0_ISR(void)
{
    EXT_INT0_InterruptFlagClear();

    // Callback function gets called everytime this ISR executes
    INT0_CallBack();    
}


void INT0_CallBack(void)
{
    // Add your custom callback code here
    if(INT0_InterruptHandler)
    {
        INT0_InterruptHandler();
    }
}

void INT0_SetInterruptHandler(void (* InterruptHandler)(void)){
    INT0_InterruptHandler = InterruptHandler;
}

void INT0_DefaultInterruptHandler(void){
    // add your INT0 interrupt custom code
    // or set custom function using INT0_SetInterruptHandler()
}
void INT1_ISR(void)
{
    EXT_INT1_InterruptFlagClear();

    // Callback function gets called everytime this ISR executes
    INT1_CallBack();    
}


void INT1_CallBack(void)
{
    // Add your custom callback code here
    if(INT1_InterruptHandler)
    {
        INT1_InterruptHandler();
    }
}

void INT1_SetInterruptHandler(void (* InterruptHandler)(void)){
    INT1_InterruptHandler = InterruptHandler;
}

void INT1_DefaultInterruptHandler(void){
    // add your INT1 interrupt custom code
    // or set custom function using INT1_SetInterruptHandler()
}
void INT2_ISR(void)
{
    EXT_INT2_InterruptFlagClear();

    // Callback function gets called everytime this ISR executes
    INT2_CallBack();    
}


void INT2_CallBack(void)
{
    // Add your custom callback code here
    if(INT2_InterruptHandler)
    {
        INT2_InterruptHandler();
    }
}

void INT2_SetInterruptHandler(void (* InterruptHandler)(void)){
    INT2_InterruptHandler = InterruptHandler;
}

void INT2_DefaultInterruptHandler(void){
    // add your INT2 interrupt custom code
    // or set custom function using INT2_SetInterruptHandler()
}

/**
 End of File
*/
