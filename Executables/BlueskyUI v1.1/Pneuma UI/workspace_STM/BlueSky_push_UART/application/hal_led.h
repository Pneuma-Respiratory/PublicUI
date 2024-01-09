
/**************************************************************************************************
 *                                        - hal_led.h -
 *
 *
 **************************************************************************************************
 */

#ifndef HAL_LED_H
#define HAL_LED_H


/***********************************************************************************
* INCLUDES
*/
#include "stdint.h"
#include "hal_led.h"

/***********************************************************************************
 * CONSTANTS
 */

/***********************************************************************************
 * MACROS
 */
#define HAL_LED_1          LED3_R
#define LED3_R             0x0001   // P1.0
#define LED3_G             0x0002   // P1.1
#define LED3_B             0x0004   // P1.2
#define LED2_R             0x0008   // P1.3
#define LED2_G             0x0010   // P1.4
#define LED2_B             0x0020   // P1.5
#define LED1_R             0x0040   // P1.6
#define LED1_G             0x0080   // P1.7
#define LED1_B             0x0100   // P2.7
//@@   
#define LED4_R             0x0200   // PJ.1
#define LED4_B             0x0400   // PJ.2   

#define HAL_LED_ALL        0x07FF
// Y = R + G
#define HAL_LED3_R         0x00
#define HAL_LED3_G         0x01   // P1.1
#define HAL_LED3_B         0x02   // P1.2
#define HAL_LED2_R         0x03   // P1.3
#define HAL_LED2_G         0x04   // P1.4
#define HAL_LED2_B         0x05   // P1.5
#define HAL_LED1_R         0x06   // P1.6
#define HAL_LED1_G         0x07   // P1.7
#define HAL_LED1_B         0x08   // P2.7
     
#define HAL_STA_LEDS       0x09
//@@   
#define HAL_LED4_R         0x09   // PJ.1
#define HAL_LED4_B         0x0A   // PJ.2
#define HAL_BATT_LEDS      0x0B
     
#define HAL_MAX_LEDS       0x0B

/* LED Modes */
#define HAL_LED_MODE_OFF     0x00
#define HAL_LED_MODE_ON      0x01
#define HAL_LED_MODE_BLINK   0x02
#define HAL_LED_MODE_TOGGLE  0x08
     
#define HAL_BATT_LED_OFF     0x10
#define HAL_BATT_LED_ON      0x20

/***********************************************************************************
* TYPEDEFS
*/
#if 0
typedef struct{
	uint8_t mode;            // Operation mode
    uint8_t onPct;           // On cycle percentage
	uint16_t left;            // Blink cycles left
	uint16_t time;           // On/off cycle time (2msec)
    uint32_t next;           // Time for next change
}HalLedStatus_t;
#endif
typedef struct{
	uint8_t mode;            // Operation mode
    uint8_t onPct;           // On cycle percentage
	uint16_t left;            // Blink cycles left
	uint16_t time;           // On/off cycle time (2msec)
	uint32_t next;           // Time for next change
}HalLedStatus_t;

/*
typedef struct
{
  HalLedControl_t HalLedControlTable[HAL_LED_DEFAULT_MAX_LEDS];
  uint8_t           sleepActive;
} HalLedStatus_t;
*/
/***********************************************************************************
* Global Variable
*/

/*******************************************************************************
*  Global Function
*/

/***********************************************************************************
*  @fn   
*  @brief    
*/
void HalLed_Init( void );

uint8_t HalLed_Set( uint16_t leds, uint8_t mode );

void HalLed_Blink(uint16_t leds, uint16_t numBlinks );

uint8_t HalLed_Blink_Update( void );

void HalLed_AllOnOff( uint8_t mode );
void HalLed_StatusOnOff( uint8_t type, uint8_t mode );

void HalLed_SingleOnOff( uint16_t leds, uint8_t mode );


#endif
