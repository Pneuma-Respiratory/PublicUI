
/**************************************************************************************************
 *                                        - app_task.h -
 *
 *
 **************************************************************************************************
 */

#ifndef APP_TASK_H
#define APP_TASK_H


/***********************************************************************************
* INCLUDES
*/
#include "stdint.h"
#include "../mcc_generated_files/nco1.h"

/***********************************************************************************
 * CONSTANTS
 */
#define TASK_ACTIVE_EVT                  0x8000
#define TASK_IDLE_EVT                    0x0000
     
#define NCO_CLK                         16000000    // 16MHz system clock source

#define KEY_SCAN_EVT                     0x0001
#define KEY_PROCESS_EVT                  0x0002
// LED task events
#define LED_BLINK_EVT                    0x0001
#define LED_STATE_UPDATE_EVT             0x0002
#define LED_STATE_BLINK_EVT              0x0004
#define LED_BAT_BLINK_EVT                0x0008
#define LED_YELLOW_BLINK_EVT             0x0010
#define LED_BLUE_BLINK_EVT               0x0020
#define LED_OFF                          0x00

#define LED_BAT_FULL                     0x0f
#define LED_BAT_NORMAL                   0x07
#define LED_BAT_LOW                      0x03
#define LED_BAT_BAD                      0x01
#define LED1_GREEN                       0x01
#define LED1_YELLOW                      0x03
#define LED1_RED                         0x02
#define LED1_BLUE                        0x04
#define LED_PWMH                         1
#define LED_PWML                         30

// Spray task events and commands
#define SPRAY_READY_EVT                  0x0001
#define SPRAY_ACTIVE_EVT                 0x0002
#define SPRAY_INHALATION_EVT             0x0004
#define SPRAY_HOLE_BREATH_EVT            0x0008
#define SPRAY_COMPLETE_EVT               0x0010
#define SPRAY_TERMINATE_EVT              0x0020
     
// Flash task events
#define FLASH_READ_RECORD_EVT            0x0001
#define FLASH_WRITE_RECORD_EVT           0x0002
#define FLASH_READ_INFO_EVT              0x0004
#define FLASH_WRITE_INFO_EVT             0x0008
#define RAM_WRITE_RECORD_EVT             0x0010
#define FLASH_SHA_SLOT2_WR_EVT           0x0020
#define FLASH_SHA_SLOT3_WR_EVT           0x0040

// Cartridge task events
#define CART_DETECT_EVT                  0x0001
#define CART_READ_EVT                    0x0002
#define CART_WRITE1_EVT                  0x0004
#define CART_WRITE2_EVT                  0x0008

// ADC task events
#define ADC_BOT_EVT                      0x0001
#define ADC_BAT_EVT                      0x0002

// UART task events
#define UART_IDLE_EVT                    0x0000
#define UART_TX_EVT                      0x0001
#define UART_TXW_EVT                     0x0002
#define UART_RX_EVT                      0x0004
#define UART_RXW_EVT                     0x0008
     
#define POWER_OFF_KEY                      0x01
#define POWER_OFF_TIMEOUT                  0x02
     
#define WAKEUP_RESET                       0x00
#define WAKEUP_ON_KEY                      0x01
#define WAKEUP_PMU                         0x02
     
// BLE
#define SOF_RX                             0xFF
#define SOF_TX                             0xFD
#define EOF_RX                             0xAB
#define EOF_TX                             0xAB
     
    
#define BLE_CMD1                           0x01
#define BLE_CMD2                           0x02
#define BLE_CMD3                           0x03
#define BLE_CMD4                           0x04
#define BLE_CMD5                           0x05
#define BLE_CMD1_CFM                       0x0B
#define BLE_CMD4_CFM                       0x0A
     
#define BLE_CMD_RTC                        0xA0

#define NOTI_CMD1                          0x02
#define NOTI_CMD2                          0x03
#define NOTI_CMD3                          0x0A
#define NOTI_CMD4                          0x01
#define NOTI_CMD5                          0x05
#define NOTI_CMD_CFM                       0x0A
     
#define NOTI_CMD_RTC                       0xA0

#define RCD_NORMAL_CMD                     0x0001
#define RT_NOTI_CMD                        0x0002
#define RCD_NOTI_CMD                       0x0004
#define PRAC_NOTI_CMD                      0x0080
     
#define PWM_MODE_OFF                       0x00
#define PWM_MODE_A                         0x01
#define PWM_MODE_B                         0x02
#define PWM_MODE_AB                        0x03
     
#define PWM_CHAN_A                         0x01
#define PWM_CHAN_B                         0x02

     
#define TRIGER_SDP                         0x01
#define TRIGER_KEY                         0x02
#define TRIGER_AUTO                        0x03
#define TRIGER_HOLD                        0x04

#define TIMER0_EN                          T0CON0bits.T0EN
//Ticket Time
#define TICKET_RELOAD                      2
//autotune repeat
#define AUTO_REPEAT_CD                     AUTOTUNE_RATE/5         //countdown to autotune repeat init (based on tmr2 @5ms rollover)

/***********************************************************************************
 * MACROS
 */
//#define ADC_CURRENT_MEASURE {\
//    ADC_Start( ADC_CUT_CH );\
//    while( adc_state == ADC_BUSY );\
//    pwm_adc_val = ADC_Average( ADC_CUT_CH );\
//    }
///***********************************************************************************
//*  @fn     
//*  @brief  converts frequency into nco equivilant
//*  @param  desired frequency    
//*            
//*  @return  Nco frequency equivilant
//*/
//#define FREQ_CONVERT(freq)      (uint24_t) (freq-2250) * 2.0/ NCO_CLK  * 1048576
///***********************************************************************************
//*  @fn     
//*  @brief  Macro sets NCO based on parameter x
//*  @param  x: NCO frequency calculated from FREQ_CONVERT
//*            
//*  @return   none
//*/
//#define SET_NCO(x)  {\
//    NCO1INCU = (uint8_t)(x >> 16);\
//    NCO1INCH = (uint8_t)(x >> 8);\
//    NCO1INCL = (uint8_t)(x);\
//    }
//#define NCO_OFF ADI_OE_OFF(); DC8V_EN_OFF(); DC24V_EN_OFF()
/***********************************************************************************
* TYPEDEFS
*/

typedef struct
{
    uint16_t event;
    uint16_t interval;
    uint16_t param1;
    uint16_t param2;
}task_event_t;

typedef struct
{
    task_event_t task_ticket;
    task_event_t task_autotune;
    task_event_t task_key;
    task_event_t task_led;
    task_event_t task_charge;
    task_event_t task_spray;
    task_event_t task_sdp3x;
    task_event_t task_battery;
    task_event_t task_wr_flash;
    task_event_t task_adc;
    task_event_t task_power_off;
    task_event_t task_uart;
    task_event_t task_debug;
}app_task_t;

// LED
typedef struct{
    uint8_t led_array;
    uint8_t led1_color;
}led_state_t;

typedef enum {
	EVENT_APP_NONE=0,
	EVENT_APP_TASK_ACTIVE,
	// ERROR
	EVENT_APP_UNKNOWN_ERROR,
} APP_THREAD_Event;

/***********************************************************************************
* Global Variable
*/
app_task_t app_Task;
led_state_t led_state;
led_state_t prev_led_state;

static uint24_t current_frc;
static uint16_t ticket_count = TICKET_RELOAD;

//LED PWM
uint8_t led_pwm_flag = 1;
static uint16_t led_pwm_high = LED_PWMH;
static uint16_t led_pwm_low = LED_PWML;
/*******************************************************************************
*  Global Function
*/

/***********************************************************************************
*  @fn   
*  @brief    
*/

void app_task_Init( void );
void task_ticket_update( void );
void app_task_Callback( void );
void app_task_Test( void );
void measure_pressure();
void save_pressure();
void save_shot();
void AutotuneCallBack(void);
void SprayTaskInit(void);
void SprayTaskCallback(void);
void SprayTaskEnd(void);
void app_set_frq( uint32_t frq );
uint24_t FreqConversion(uint32_t freq);

#endif
