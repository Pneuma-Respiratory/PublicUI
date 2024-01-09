/***********************************************************************************
  Filename:     adapter_gpio.c

  Description:  
                

***********************************************************************************/

/***********************************************************************************
* INCLUDES
*/

#include "../mcc_generated_files/system/pins.h"

#include "hal_board_cfg.h"
#include "hal_led.h"

/***********************************************************************************
 * CONSTANTS
 */

/***********************************************************************************
 * MACROS
 */
 

/***********************************************************************************
* GLOBAL VARIABLES
*/
extern uint8_t led_AllOff;
/***********************************************************************************
* LOCAL VARIABLES
*/
#if 0
static HalLedStatus_t HalLedStatusCtl[HAL_MAX_LEDS];
static HalLedStatus_t *pHalLedStatusCtl = HalLedStatusCtl;
static uint16_t HalLedState;              // LED state at last set/clr/blink update
static uint16_t preBlinkState;            // Original State before going to blink mode
                                          // bit 0, 1, 2, 3 represent led 0, 1, 2, 3
static HalLedStatus_t HalLedStatusControl;
static uint8_t LedFlash = 1;
static uint8_t blink_flag = false;
#endif

/***********************************************************************************
* LOCAL FUNCTIONS
*/

/***********************************************************************************
* GLOBAL FUNCTIONS
*/


/***********************************************************************************
* LOCAL FUNCTIONS
*/


/***********************************************************************************
*  @fn   
*  @brief Initialize LED Service
*/

void HalLed_Init( void )
{        
    IO_RA3_SetHigh();      IO_RA3_SetDigitalOutput();      IO_RA3_SetOpenDrain();
    IO_RA4_SetHigh();      IO_RA4_SetDigitalOutput();      IO_RA4_SetOpenDrain();
    IO_RA5_SetHigh();      IO_RA5_SetDigitalOutput();      IO_RA5_SetOpenDrain();
    
    IO_RE0_SetHigh();      IO_RE0_SetDigitalOutput();      IO_RE0_SetOpenDrain();
    IO_RE1_SetHigh();      IO_RE1_SetDigitalOutput();      IO_RE1_SetOpenDrain();
    IO_RE2_SetHigh();      IO_RE2_SetDigitalOutput();      IO_RE2_SetOpenDrain();
    
    IO_RC5_SetHigh();      IO_RC5_SetDigitalOutput();      IO_RC5_SetOpenDrain();
    IO_RC4_SetHigh();      IO_RC4_SetDigitalOutput();      IO_RC4_SetOpenDrain();
    IO_RD3_SetHigh();      IO_RD3_SetDigitalOutput();      IO_RD3_SetOpenDrain();
    IO_RD2_SetHigh();      IO_RD2_SetDigitalOutput();      IO_RD2_SetOpenDrain();
}

/***********************************************************************************
*  @fn   
*  @brief Turns specified LED ON or OFF
*/
void HalLed_AllOnOff( uint8_t mode )
{
    if ( mode == HAL_LED_MODE_ON ){
        LED1R_ON();
        LED1G_ON();
        LED1B_ON();
        LED2_ON();
        LED3_ON();
        LED4_ON();
        LED5_ON();
        LED6_ON();
        LED7_ON();
        LED8_ON();
    }
    else if ( mode == HAL_LED_MODE_OFF ){
        LED1R_OFF();
        LED1G_OFF();
        LED1B_OFF();
        LED2_OFF();
        LED3_OFF();
        LED4_OFF();
        LED5_OFF();
        LED6_OFF();
        LED7_OFF();
        LED8_OFF();
    }
}

void HalLed_StatusOnOff( uint8_t type, uint8_t mode )
{
    if ( mode == HAL_LED_MODE_ON ){
        if ( (type == 1) || (type == 5) ){
            LED5_ON();
            LED6_ON();
            LED7_ON();
            LED8_ON();
        }
        else if ( type == 2 ){
            LED5_ON();
            LED6_ON();
            LED7_ON();
        }
        else if ( type == 3 ){
            LED5_ON();
            LED6_ON();
        }
        else if ( type == 4 ){
            LED5_ON();
        }
        else if ( type == 5 ){
            //GPIO_setOutputLowOnPin( LED8_PORT, LED8_PIN );
        }
    }
    else if ( mode == HAL_LED_MODE_OFF ){        
        if ( (type == 1) || (type == 5) ){
            LED5_OFF();
            LED6_OFF();
            LED7_OFF();
            LED8_OFF();
        }
        else if ( type == 2 ){
            LED5_OFF();
            LED6_OFF();
            LED7_OFF();
        }
        else if ( type == 3 ){
            LED5_OFF();
            LED6_OFF();
        }
        else if ( type == 4 ){
            LED5_OFF();
        }
    }
}
