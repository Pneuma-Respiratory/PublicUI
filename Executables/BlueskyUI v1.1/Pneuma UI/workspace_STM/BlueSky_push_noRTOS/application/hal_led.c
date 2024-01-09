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
#define led_count			8
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
int ports[8] = {LED8_PORT, LED7_PORT, LED6_PORT, LED5_PORT, LED4_PORT, LED3_PORT, LED2_PORT, LED1G_PORT};
int leds[8] = {LED8_PIN, LED7_PIN, LED6_PIN, LED5_PIN, LED4_PIN, LED3_PIN, LED2_PIN, LED1G_PIN};

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
}

/***********************************************************************************
*  @fn
*  @brief Turns on specified led at input position
*/
void HalLed_Spec( uint8_t pos )
{
	HalLed_AllOnOff(HAL_LED_MODE_OFF);
	GPIO_setOutputLowOnPin(ports[pos], leds[pos]);
}

/***********************************************************************************
*  @fn   
*  @brief Turns all LED's on/off
*/
void HalLed_AllOnOff( uint8_t mode )
{
    uint8_t i;
    if ( mode == HAL_LED_MODE_ON ){
		GPIO_setOutputLowOnPin( LED1R_PORT, LED1R_PIN );
		GPIO_setOutputLowOnPin( LED1G_PORT, LED1G_PIN );
		GPIO_setOutputLowOnPin( LED1B_PORT, LED1B_PIN );
		GPIO_setOutputLowOnPin( LED2_PORT, LED2_PIN );
		GPIO_setOutputLowOnPin( LED3_PORT, LED3_PIN );
		GPIO_setOutputLowOnPin( LED4_PORT, LED4_PIN );
		GPIO_setOutputLowOnPin( LED5_PORT, LED5_PIN );
		GPIO_setOutputLowOnPin( LED6_PORT, LED6_PIN );
		GPIO_setOutputLowOnPin( LED7_PORT, LED7_PIN );
		GPIO_setOutputLowOnPin( LED8_PORT, LED8_PIN );
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
