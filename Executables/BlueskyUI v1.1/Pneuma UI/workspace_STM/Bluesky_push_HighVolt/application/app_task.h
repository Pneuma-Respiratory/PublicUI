
/**************************************************************************************************
 *                                        - task.h -
 *
 * 
 *
 **************************************************************************************************
 */

#ifndef __app_task_h
#define __app_task_h

/***********************************************************************************
* INCLUDES
*/

#include "hal_board_cfg.h"

/*********************************************************************
 * MACROS
 */

    
/*********************************************************************
 * CONSTANTS
 */
#define TASK_ACTIVE_EVT                  0x8000
#define TASK_IDLE_EVT                    0x0000
     
#define KEY_SCAN_EVT                     0x0001
#define KEY_PROCESS_EVT                  0x0002
// LED task events
#define LED_BLINK_EVT                    0x0001
#define LED_STATE_UPDATE_EVT             0x0002
#define LED_STATE_BLINK_EVT              0x0004
#define LED_BAT_BLINK_EVT                0x0008
#define LED_YELLOW_BLINK_EVT             0x0010
#define LED_BLUE_BLINK_EVT               0x0020
#define LED_STATE_FREQ_EVT               0x0040
     
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

// Voice task events and commands
#define VOICE_LOW_BATT_EVT               0x0001
#define VOICE_BOTTLE_EMPTY_EVT           0x0002
#define VOICE_BOTTLE_AEMPTY_EVT          0x0004
#define VOICE_BOTTLE_EXIPRE_EVT          0x0008
#define VOICE_BOTTLE_NULL_EVT            0x0010
#define VOICE_EX_INHALE_EVT              0x0020
#define VOICE_BAD_BATT_EVT               0x0040
#define VOICE_REP_CART_EVT               0x0080
#define VOICE_REP_CART_SOON_EVT          0x0100

#define VOICE_HOLD_BREATH_EVT            0x0400
#define VOICE_TURN_OFF_EVT               0x0800
#define VOICE_RASPS_EVT                  0x1000
#define VOICE_DING_EVT                   0x2000
#define VOICE_AMP_EVT                    0x4000                // voice chip AMP is open

#define VOICE_ALL_EVT                    0x3FFF

#define VOICE_BOTTLE_AEMPTY_CMD            0x01
#define VOICE_BOTTLE_EMPTY_CMD             0x02
#define VOICE_BOTTLE_EXIPRE_CMD            0x03
#define VOICE_BOTTLE_NULL_CMD              0x04
//#define VOICE_LOW_BATT_CMD                 0x05
#define VOICE_EX_INHALE_CMD                0x06

#define VOICE_HOLD_BREATH_CMD              0x0A
#define VOICE_TURN_OFF_CMD                 0x0B
#define VOICE_RASPS_CMD                    0x0C
#define VOICE_DING_CMD                     0x0D

#define VOICE_CLOSE_CAP_CMD                0x0E
#define VOICE_LOW_BATT_CMD                 0x10
#define VOICE_BAD_BATT_CMD                 0x11
#define VOICE_REP_CART_CMD                 0x12
#define VOICE_REP_CART_SOON_CMD            0x13

/*********************************************************************
 * TYPEDEFS
 */
#pragma pack(1)
typedef struct
{
    uint16_t event;
    uint16_t interval;
    uint16_t param1;
    uint16_t param2;
}task_event_t;

typedef struct
{
    task_event_t task_key;
    task_event_t task_led;
    task_event_t task_charge;
    task_event_t task_spray;
    task_event_t task_sdp3x;
    task_event_t task_battery;
    task_event_t task_wr_flash;
    task_event_t task_cartridge;
    task_event_t task_interval;
    task_event_t task_adc;
    task_event_t task_power_off;
    task_event_t task_uart;
    task_event_t task_voice;
    task_event_t task_debug;
}app_task_t;

// LED
typedef struct{
    uint8_t mode;
    uint16_t cnt;
}led_state_t;

typedef enum {
	EVENT_APP_NONE=0,
	EVENT_APP_TASK_ACTIVE,
	// ERROR
	EVENT_APP_UNKNOWN_ERROR,
} APP_THREAD_Event;

typedef struct{
       uint16_t shot_num;
       uint16_t freq_idx;
       uint16_t used_freq;
       uint16_t total_time;
       uint16_t batt_lvl;
       uint16_t current;
       uint16_t pressure;
}info_shot_t;

#pragma pack()

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * FUNCTIONS
 */

void app_Queue_Init(void);
void app_Queue_Send(uint32_t d);
int app_Queue_Receive(void);


void task_ticket_update( void );

#endif
