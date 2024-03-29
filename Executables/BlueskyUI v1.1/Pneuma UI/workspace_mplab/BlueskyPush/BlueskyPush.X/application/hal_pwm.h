
/**************************************************************************************************
 *                                        - hal_pwm.h -
 *
 *
 **************************************************************************************************
 */

#ifndef HAL_PWM_H
#define HAL_PWM_H


/***********************************************************************************
* INCLUDES
*/

#include "hal_pwm.h"

/***********************************************************************************
 * CONSTANTS
 */

/***********************************************************************************
 * MACROS
 */
#define PWM_183K        0x01

/***********************************************************************************
* TYPEDEFS
*/

/***********************************************************************************
* Global Variable
*/

/*******************************************************************************
*  Global Function
*/

/*
 *  @fn   
 *  @brief
 */

void PWM_Setup( uint8_t freq, uint8_t idx );
void PWM_Start( void );
void PWM_Stop( void );

//void PWM2_Setup( uint8 value );

#endif
