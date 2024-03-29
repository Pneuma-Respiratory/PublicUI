/***********************************************************************************
  Filename:     hal_pwm.c

  Description:  
                

***********************************************************************************/

/***********************************************************************************
* INCLUDES
*/
#include "tim.h"
#include "stm32l4xx_hal_tim.h"
#include "hal_board_cfg.h"

#include "hal_pwm.h"

/***********************************************************************************
 * CONSTANTS
 */

/***********************************************************************************
 * MACROS
 */

/***********************************************************************************
* GLOBAL VARIABLES
*/
extern uint32_t tim3_per_value;
/***********************************************************************************
* LOCAL VARIABLES
*/

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
 *  @brief    
 *  @param    freq     -   108  (108��2.5 KHz)
 *            percent  -   50
 *            value = 20000/freq = [180, 189]
 *  @return   
 */
//void PWM_Setup( uint8 freq, uint8 percent )
void PWM_Setup( uint32_t value )
{
	tim3_per_value = value;
	MX_TIM3_Init();
	HAL_TIM3_PWM_Start(TIM_CHANNEL_1);
}

void PWM_Start( uint32_t pwm_ch )
{
	HAL_TIM3_PWM_Start( pwm_ch );
}

void PWM_Stop( uint32_t pwm_ch )
{
	HAL_TIM3_PWM_Stop( pwm_ch );
}

void PWM2_Setup( uint32_t value )
{

}
