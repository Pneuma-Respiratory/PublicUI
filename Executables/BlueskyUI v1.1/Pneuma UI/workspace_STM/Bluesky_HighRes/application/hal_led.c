/***********************************************************************************
  Filename:     hal_led.c

  Description:  
                

***********************************************************************************/

/***********************************************************************************
* INCLUDES
*/
#include "cmsis_os.h"
#include "stm32l4xx.h"
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_gpio.h"

#include "main.h"
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
    GPIO_setOutputHighOnPin( LED1R_PORT, LED1R_PIN );
    GPIO_setOutputHighOnPin( LED1G_PORT, LED1G_PIN );
    GPIO_setOutputHighOnPin( LED1B_PORT, LED1B_PIN );
    GPIO_setOutputHighOnPin( LED2_PORT, LED2_PIN );
    GPIO_setOutputHighOnPin( LED3_PORT, LED3_PIN );
    GPIO_setOutputHighOnPin( LED4_PORT, LED4_PIN );
    GPIO_setOutputHighOnPin( LED5_PORT, LED5_PIN );
    GPIO_setOutputHighOnPin( LED6_PORT, LED6_PIN );
    GPIO_setOutputHighOnPin( LED7_PORT, LED7_PIN );
    GPIO_setOutputHighOnPin( LED8_PORT, LED8_PIN );
    /*
    GPIO_setAsOutputPin( LED1R_PORT, LED1R_PIN );
    GPIO_setAsOutputPin( LED1G_PORT, LED1G_PIN );
    GPIO_setAsOutputPin( LED1B_PORT, LED1B_PIN );
    GPIO_setAsOutputPin( LED2_PORT, LED2_PIN );
    GPIO_setAsOutputPin( LED3_PORT, LED3_PIN );
    GPIO_setAsOutputPin( LED4_PORT, LED4_PIN );
    GPIO_setAsOutputPin( LED5_PORT, LED5_PIN );
    GPIO_setAsOutputPin( LED6_PORT, LED6_PIN );
    GPIO_setAsOutputPin( LED7_PORT, LED7_PIN );
    GPIO_setAsOutputPin( LED8_PORT, LED8_PIN );
    */
}

/***********************************************************************************
*  @fn   
*  @brief Turns specified LED ON or OFF
*/
void HalLed_AllOnOff( uint8_t mode )
{
    if ( mode == HAL_LED_MODE_ON ){
    }
    else if ( mode == HAL_LED_MODE_OFF ){
        GPIO_setOutputHighOnPin( LED1R_PORT, LED1R_PIN );
        GPIO_setOutputHighOnPin( LED1G_PORT, LED1G_PIN );
        GPIO_setOutputHighOnPin( LED1B_PORT, LED1B_PIN );
        GPIO_setOutputHighOnPin( LED2_PORT, LED2_PIN );
        GPIO_setOutputHighOnPin( LED3_PORT, LED3_PIN );
        GPIO_setOutputHighOnPin( LED4_PORT, LED4_PIN );
        GPIO_setOutputHighOnPin( LED5_PORT, LED5_PIN );
        GPIO_setOutputHighOnPin( LED6_PORT, LED6_PIN );
        GPIO_setOutputHighOnPin( LED7_PORT, LED7_PIN );
        GPIO_setOutputHighOnPin( LED8_PORT, LED8_PIN );
    }
}

void HalLed_StatusOnOff( uint8_t type, uint8_t mode )
{
    if ( mode == HAL_LED_MODE_ON ){
        if ( (type == 1) || (type == 5) ){
            GPIO_setOutputLowOnPin( LED5_PORT, LED5_PIN );
            GPIO_setOutputLowOnPin( LED6_PORT, LED6_PIN );
            GPIO_setOutputLowOnPin( LED7_PORT, LED7_PIN );
            GPIO_setOutputLowOnPin( LED8_PORT, LED8_PIN );
        }
        else if ( type == 2 ){
            GPIO_setOutputLowOnPin( LED6_PORT, LED6_PIN );
            GPIO_setOutputLowOnPin( LED7_PORT, LED7_PIN );
            GPIO_setOutputLowOnPin( LED8_PORT, LED8_PIN );
        }
        else if ( type == 3 ){
            GPIO_setOutputLowOnPin( LED7_PORT, LED7_PIN );
            GPIO_setOutputLowOnPin( LED8_PORT, LED8_PIN );
        }
        else if ( type == 4 ){
            GPIO_setOutputLowOnPin( LED8_PORT, LED8_PIN );
        }
        else if ( type == 5 ){
            //GPIO_setOutputLowOnPin( LED8_PORT, LED8_PIN );
        }
        
    }
    else if ( mode == HAL_LED_MODE_OFF ){        
        if ( (type == 1) || (type == 5) ){
            GPIO_setOutputHighOnPin( LED5_PORT, LED5_PIN );
            GPIO_setOutputHighOnPin( LED6_PORT, LED6_PIN );
            GPIO_setOutputHighOnPin( LED7_PORT, LED7_PIN );
            GPIO_setOutputHighOnPin( LED8_PORT, LED8_PIN );
        }
        else if ( type == 2 ){
            GPIO_setOutputHighOnPin( LED6_PORT, LED6_PIN );
            GPIO_setOutputHighOnPin( LED7_PORT, LED7_PIN );
            GPIO_setOutputHighOnPin( LED8_PORT, LED8_PIN );
        }
        else if ( type == 3 ){
            GPIO_setOutputHighOnPin( LED7_PORT, LED7_PIN );
            GPIO_setOutputHighOnPin( LED8_PORT, LED8_PIN );
        }
        else if ( type == 4 ){
            GPIO_setOutputHighOnPin( LED8_PORT, LED8_PIN );
        }
    }
}
