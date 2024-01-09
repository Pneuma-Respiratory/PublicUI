/************************************************************************************

 * **********************************************************************************/

#ifndef HAL_BOARD_CFG_H
#define HAL_BOARD_CFG_H


/***********************************************************************************
* INCLUDES
*/


#include <xc.h>
#include <string.h>
#include "../mcc_generated_files/pin_manager.h"


#include "hal_led.h"
#include "hal_adc.h"

#include "hal_sdp.h"
//#include "hal_flash.h"
#include "hal_key.h"
#include "hal_pwm.h"


/*********************************************************************
 * TYPEDEFS
 */
typedef uint32_t UTCTime;


typedef enum
{
    GPIO_PIN_RESET = 0,
    GPIO_PIN_SET
}GPIO_PinState;

typedef enum
{
  SUCCESS = 0,
  ERROR = !SUCCESS
} ErrorStatus;

/***********************************************************************************
 * CONSTANTS
 */
#define BIT7                                          0x80
#define BIT6                                          0x40
#define BIT5                                          0x20
#define BIT4                                          0x10
#define BIT3                                          0x08
#define BIT2                                          0x04
#define BIT1                                          0x02
#define BIT0                                          0x01

#define WDT_EN                                        0
#define USE_EXTAL2                                    1

#define WTD_ENABLE                                    0
#define POWER_KEY_LOW_LEVEL                           false

#define UART_USED                                     1
#define PRESSURE_EN                                   1

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Failure informations

// I2C
#define I2C_ERROR_DEVICE              0x10
#define ADC_FAIL                      0x51
#define ADC_BUSY                      0x01
#define ADC_IDLE                      0x00
     
// Key
#define SW_ON                         0x01
#define SW_OFF                        0x02

// Battery, 10bit ADC, Vref = 2.048V --> 1024 -- 2.048V
#define BATT_THR_1                   987     // 3.95V --> 1.975V --> 987
#define BATT_THR_2                   937     // 3.75V --> 1.875V --> 937
#define BATT_THR_3                   900     // 3.6V  --> 1.8V   --> 900
#define BATT_THR_4                   825     // 3.3V  --> 1.65V  --> 825

#define BATT_THR_3V9                 975     // 3.9V  --> 1.95V  --> 975
#define BATT_THR_3V7                 925     // 3.7V  --> 1.85V  --> 925
#define BATT_THR_3V5                 875     // 3.5V  --> 1.75V  --> 875

#define BAT_VOL_FULL                 0x05
#define BAT_VOL_NORMAL               0x04
#define BAT_VOL_LOW                  0x03
#define BAT_VOL_BAD                  0x02
#define BAT_VOL_OFF                  0x01

#define BAT_CHARGE_OFF               0x00
#define BAT_CHARGE_ON                0x01
#define BAT_CHARGE_FULL              0x02
     
#define SDP_START_THR                 50
#define SDP_STOP_THR                 10
#define SDP_RESTAR_THR               30
                                            
#define CURRENT_FOSC_INI         300		    // INI FOSC CURRENT COMPARE

#define CURRENT_THR1             350	        // current adc > THR1  ,stop ejection
#define ALARM1_COUNT             3              // TOTAL TIMES = ALARM1_COUNT

#define CURRENT_THR2             330            // current adc > THR2  ,stop ejection
#define ALARM2_COUNT             5              //  Continuous Times = ALARM2_COUNT

#define CURT_OFFSET              120
#define ALARM3_COUNT             5


//#define SET_ADC_VAL              450	      // set current value  4V  0.68A-0.72A
#define PWM_DUTY_MIN             15          


/***********************************************************************************
 * MACROS
 */
//#define GPIO_setOutputHighOnPin( port, pin )   HAL_GPIO_WritePin( port, pin, GPIO_PIN_SET )
//#define GPIO_setOutputLowOnPin( port, pin )    HAL_GPIO_WritePin( port, pin, GPIO_PIN_RESET )

// ADC
#define ADC_CUT_CH               channel_AND4
#define ADC_BAT_CH               channel_AND5
#define ADC_PD1_AD               channel_ANC3
#define ADC_PD2_AD               channel_ANC2
// 5VIN_TST
#define VBUS_GetValue()          IO_RD6_GetValue()
// CHRG
#define CHRG_GetValue()          IO_RD7_GetValue()
// IN1~IN4
#define ULN_IN1_ON()             IO_RB3_SetHigh()
#define ULN_IN1_OFF()            IO_RB3_SetLow()
#define ULN_IN2_ON()             IO_RB2_SetHigh()
#define ULN_IN2_OFF()            IO_RB2_SetLow()
#define ULN_IN3_ON()             IO_RB1_SetHigh()
#define ULN_IN3_OFF()            IO_RB1_SetLow()
#define ULN_IN4_ON()             IO_RB0_SetHigh()
#define ULN_IN4_OFF()            IO_RB0_SetLow()

#define ADI_OE_OFF()             PWM_Stop()
#define ADI_OE_ON()              PWM_Start()

// EN1, MP3429
#define DC24V_EN_ON()	 	     IO_RA0_SetHigh()
#define DC24V_EN_OFF()		     IO_RA0_SetLow()
// EN0, TPS55340
#define DC8V_EN_ON()	 	     IO_RA1_SetHigh()
#define DC8V_EN_OFF()		     IO_RA1_SetLow()
// VBAT_CN
#define VBAT_CN_ON()             IO_RA2_SetHigh()
#define VBAT_CN_OFF()            IO_RA2_SetLow()
// PVCC
#define PVCC_ON()                IO_RC0_SetHigh()
#define PVCC_OFF()               IO_RC0_SetLow()
// SW_TEST
#define SW_GetValue()            IO_RC1_GetValue()
// I2C_SDA, I2C_SCL
#define SDA_PIN_OUT_H()          IO_RD0_SetHigh()
#define SDA_PIN_OUT_L()          IO_RD0_SetLow()
#define SDA_PIN_GET()            IO_RD0_GetValue()
#define SCL_PIN_OUT_H()          IO_RD1_SetHigh()
#define SCL_PIN_OUT_L()          IO_RD1_SetLow()
#define SCL_PIN_GET()            IO_RD1_GetValue()

// LED
#define LED1R_ON()               IO_RA5_SetLow()
#define LED1R_OFF()              IO_RA5_SetHigh()
#define LED1B_ON()               IO_RA4_SetLow()
#define LED1B_OFF()              IO_RA4_SetHigh()
#define LED1G_ON()               IO_RA3_SetLow()
#define LED1G_OFF()              IO_RA3_SetHigh()
#define LED2_ON()                IO_RE0_SetLow()
#define LED2_OFF()               IO_RE0_SetHigh()
#define LED3_ON()                IO_RE1_SetLow()
#define LED3_OFF()               IO_RE1_SetHigh()
#define LED4_ON()                IO_RE2_SetLow()
#define LED4_OFF()               IO_RE2_SetHigh()
#define LED5_ON()                IO_RC5_SetLow()
#define LED5_OFF()               IO_RC5_SetHigh()
#define LED5_TOG()               IO_RC5_Toggle()
#define LED6_ON()                IO_RC4_SetLow()
#define LED6_OFF()               IO_RC4_SetHigh()
#define LED7_ON()                IO_RD3_SetLow()
#define LED7_OFF()               IO_RD3_SetHigh()
#define LED8_ON()                IO_RD2_SetLow()
#define LED8_OFF()               IO_RD2_SetHigh()

/*
Prog Types 
 * 0 = "Button and Pressure Actuation"
 * 1 = "Button Actuation"
 * 2 = "Pressure Actuation"
 * 3 = "Multiple Shots"
 * 4 = "Smoke Machine"
 * 5 = "Pressure Sensor Check"
 * 6 = "Timed Breath"
*/
#define BUTT_AND_PRESS          0
#define BUTTON                  1
#define PRESS                   2
#define MULTI_SHOT              3
#define SMOKE_MC                4
#define PRESSURE_CHECK          5
#define TIMED_BREATH            6
#define CONSUMER_STUDY          7

/*******************************************************************************
*  Global Function
*/

/***********************************************************************************
* Global Variable
*/

/*******************************************************************************
*  Global Function
*/


#endif
