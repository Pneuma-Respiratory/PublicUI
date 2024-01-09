
/**************************************************************************************************
 *                                       
 *
 * 
 *
 **************************************************************************************************
 */

#ifndef HAL_KEY_H
#define HAL_KEY_H

#include "stdint.h"

/***********************************************************************************
* INCLUDES
*/


/***********************************************************************************
 * CONSTANTS
 */
//-------------------------------------------------------------------------------------------------

#define KEY_ST0               0x00
#define KEY_ST1               0x01   // key state 1
#define KEY_ST2               0x02
#define KEY_ST3               0x03
     
#define CLICK_MIN_TIME        10
#define DCLICK_IDLE_TIME      10      
#define HOLD_TIME_NORMAL      100
#define HOLD_TIME_LP          60        // Low power mode Long press time

#define KEY_ON_INVALID        0x00
#define KEY_RLS               0x01
#define KEY_ON_HOLD           0x02
#define KEY_OFF_HOLD          0x03
#define KEY_LPRESS            0x04
#define KEY_SPRESS            0x05
#define KEY_POWER_ON          0xA0
     
    
typedef struct{
    uint8_t state;
    uint8_t event;
    uint16_t count;
}keyState_t;

/***********************************************************************************
 * MACROS
 */

/***********************************************************************************
* TYPEDEFS
*/

/***********************************************************************************
* CONSTANTS
*/
// Long press: 160x10ms
// Short press: 20x10ms ~ 150x10ms
// 20x10ms ~ 20x10ms delay ~ 20x10ms

#define KEY_ST0                             0x00
#define KEY_ST1                             0x01   // key state 1
#define KEY_ST2                             0x02
#define KEY_ST3                             0x03
     
#define CLICK_MIN_TIME                 10
#define DCLICK_IDLE_TIME              10      
#define HOLD_TIME_NORMAL          100
#define HOLD_TIME_LP                    60        // Low power mode Long press time

#define KEY_ON_INVALID                 0x00
#define KEY_RLS                              0x01
#define KEY_ON_HOLD                     0x02
#define KEY_OFF_HOLD                   0x03
#define KEY_LPRESS                       0x04
#define KEY_SPRESS                       0x05
#define KEY_POWER_ON                 0xA0

/***********************************************************************************
* Global Variable
*/

/*******************************************************************************
*  Global Function
*/
uint8_t halKeyScan( void );

#endif
