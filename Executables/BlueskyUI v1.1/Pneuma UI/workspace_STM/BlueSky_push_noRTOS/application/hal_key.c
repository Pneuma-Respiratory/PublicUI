/***********************************************************************************
  Filename:     cc_debugger.c

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
#include "hal_key.h"



/***********************************************************************************
 * CONSTANTS
 */

/***********************************************************************************
 * MACROS
 */

/***********************************************************************************
* GLOBAL VARIABLES
*/

/***********************************************************************************
* LOCAL VARIABLES
*/
keyState_t KeyOn  = { 0x00, 0x00, 0x00 };

/***********************************************************************************
* LOCAL FUNCTIONS
*/

/***********************************************************************************
* GLOBAL FUNCTIONS
*/


/***********************************************************************************
* LOCAL FUNCTIONS
*/



/***************************************************************************************************
 * @fn      halKeyScan()
 *
 * @brief   
 *
 * @param   port  -- key port data
 *          pin   -- pin of the          // eg. HAL_KEY_SW_6_BIT    BV(1)
 *          key   -- key structure
 * 
 * @return  Key.flag.BIT.event
 */
#if PROG_TYPE != 7
uint8_t halKeyScan( void )
{
    uint8_t pin, rslt;
    keyState_t *pKey;
    
    pKey = &KeyOn;
    pin = HAL_GPIO_ReadPin( SW_ON_PORT, SW_ON_PIN );

    if ( pin == pKey->state ){
        if ( ++pKey->count >= 3 ){
            //pKey->count = 3;
        }
    }
    pKey->state = pin;
 
    if ( pin ){
        if ( (pKey->count >= 3) && (pKey->count <= 50) ){
            pKey->count = 0;
            rslt = KEY_SPRESS;
        }
        else{
            pKey->count = 0;
            rslt = KEY_RLS;
        }
    }
    else{
        if ( pKey->count > KEY_LP_TIME ){
            rslt = KEY_LPRESS;
        }
        else{
            rslt = KEY_ON_HOLD;
        }
    }
    
    return rslt;
}
#else
uint8_t halKeyScan( void )
{
    uint8_t pin, rslt;
    keyState_t *pKey;

    pKey = &KeyOn;
    pin = HAL_GPIO_ReadPin( SW_ON_PORT, SW_ON_PIN );

    if ( pin == pKey->state ){
        if ( ++pKey->count >= 3 ){
            pKey->count = 3;
        }
    }
    else
    {
    		pKey->state = pin;
    		pKey->count = 0;
    }
    if(pKey->count >= 3)
    {
    	if(pin)
    	{
    		rslt = KEY_RLS;
    	}
    	else
    	{
    		rslt = KEY_SPRESS;
    	}
    }
    return rslt;
}
#endif
