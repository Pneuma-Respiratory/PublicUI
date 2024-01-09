    /***********************************************************************************
  Filename:     task.c

  Description:
***********************************************************************************/
/******************************************************
 * MCU: PIC18F57Q43,UQFN48  PCB Version: BlueSky5.3_NEW2_DRV 20230810  
 * change data: 2023-08-10 
 * 5PCS Version,2023-08-18 
 * change data  2023-08-18  debug autotune etc.
 * change data  2023-08-31  debug watchdog reset
 * change data  2023-09-11  debug autotune program
 * change data  2023-09-15  debug transducer protect current
 * change data  2023-09-20  debug transducer protect current again.       
 
 * change data  2023-09-25  
 * PCB Version: BlueSky5.3_New3_DRV REVA  20230915  
 * changed as 7 level Voltage adjust( 2PCS ULN2003)
 * added IN5,IN6       
 * Verify data 20231006 ?change protect current AD Value.  
 * change data:20231030
 * 1.change uln_level_con as switch case mode. VT_set_value();
 * 2.modify autotune program,total 40(100HZ step) + 24(6.7HZ step).
 *   chang max_adc value
 * 3.plan to modify transducer protect alarm program.
 * 2023-11-1
 * Debug Protect current of Spray Process And Remove Mesh Model. 
 * BlueSky5.3_PIC18F57Q43_New3_TW_130_0
 * change data:2023-11-15
 * File name:BlueSky5.3_PIC18F57Q43_New3_TW_130_0_uln0.X
 * Alarm1-Alarm6 Protect will stop Spray(flash 2 times)        
********************************************************/

/***********************************************************************************
* INCLUDES
*/

#include "string.h"                                                             
#include "stdlib.h"
#include "../mcc_generated_files/timer/tmr1.h"
#include "../mcc_generated_files/adc/adcc.h"
#include "../mcc_generated_files/system/watchdog.h"
#include "hal_board_cfg.h"
#include "hal_i2c.h"
#include "app_task.h"
#include "../hal_flash.h"
/***********************************************************************************
 * TYEPDEF
 */
#pragma pack(1)
typedef enum
{
    STATE_SYSTEM_WAIT = 0,
    STATE_SYSTEM_CHECK,   
    STATE_SPRAY_READY,
    STATE_SPRAY_ACTIVE_DLY,
    STATE_SPRAY_SCAN,
    STATE_SPRAY_READY_TIMEOUT,
    STATE_SPRAY_ACTIVE,
    STATE_SPRAY_INHALATION,
    STATE_HOLD_BREATH,
    STATE_DOSE_COMPLETE,
    STATE_CART_ALL_EMP,
    STATE_TERMINATE
} spray_control_state_t;

// 64 bytes  
typedef struct{
       uint16_t write_cnt;

       uint16_t total_cnt;
       uint16_t used_cnt;
       uint32_t total_time;
       uint32_t used_time;
       uint16_t per_sdp_time;
       uint16_t per_key_time;
       UTCTime fill_time;
       UTCTime expire_time;

       uint32_t unused_pct;
       uint8_t state;
       uint8_t reserved[31];
       uint16_t info_crc;
}info_mem_t;

typedef struct{
    uint16_t timeA;
    uint16_t timeB;
    uint8_t cnt;
}bt_pwm_t;

#if 0
typedef struct
{
    // Slot0
    uint32_t bt_id;
    uint16_t bt_type;
    uint8_t per_dose;
    uint16_t amount_dose;
    uint8_t spray_cnt;
    uint8_t perday_cnt;
    uint16_t total_count;            // 总次数
    uint8_t total_rescue_cnt;
    uint16_t spray_time;
    uint8_t reserve_dat0[14];
    uint8_t slot0_crc[2];
    // Slot1
    uint32_t produce_Clock;
    uint32_t expire_Clock;
    uint8_t pwm_duty;
    uint8_t ejctor_cal1;
    uint16_t ejctor_cal2;
    uint16_t ejctor_cal3;
    uint16_t ejctor_cal4;
    uint8_t reserve_dat1[14];
    uint8_t slot1_crc[2];
    // Slot2
    uint32_t openClock;
    uint16_t openFlag;
    uint8_t reserve_dat2[24];
    uint8_t slot2_crc[2];
    // Slot3
    uint16_t remain_cnt;             // 剩余次数
    uint8_t rescue_cnt;
    uint32_t volTime;
    uint32_t updateClock;
    uint8_t day_cnt;
    uint8_t cart_state;
    uint8_t close_action;

    uint8_t reserve_dat3[16];
    uint8_t slot3_crc[2];
}new_bt_attr_t;
#endif
#pragma pack()
/***********************************************************************************
 * CONSTANTS
 */
//#define LOOP_TEST_MODE         
//#define BUTTON_TEST_MODE      
#define START_DELAY_TIME        1500
#define PWM_ON_TIME             750
#define PWM_OFF_TIME            750

#define TRIGER_OFF_STATE        0x01
#define KEY_TRIGER_ON           0x02
#define KEY_TRIGER_OFF          0x03

#define LCD_FLASH_INTV          310

// Work flags
#define DIS_CART_SCAN           0x01      // Disable cartridge scan function
#define ALM_CART_EMP
     
#define WORK_MODE_NORMAL        0x0100    // normal work mode
#define WORK_MODE_RT            0x0200    // monitor mode
#define WORK_MODE_PRAC          0x0400    // practise mode
#define WORK_MODE_HISTORY       0x0800    // read history
#define BLE_WORK_MODE_ALL       0x0F00   
#define CART_ALARM_TRG_NUM      16
#define SPRAY_LIMIT_NUM         200       //0626 4

#define SPRAY_ON_DLY            1         // Delay 1ms
#define SPRAY_ON_TIME           3000      // 3s
//#define SPRAY_ON_TIME           4000    // 4s
//#define SPRAY_ON_TIME           5000    // 5s
//#define SPRAY_ON_TIME           6000    // 6s

#define BREATH_ON_TIME          6000      // 6s
#define SPRAY_TURN_OFF_DLY      300       // unit: 100ms; turn off after 32s = 320*100ms  150 = 30s

#define SCAN_DLY_1              20       	     // 100*1ms=100ms ,delay 100ms --scan frequency
#define SCAN_DLY_2              1         		 // 1ms
#define RE_TRIGER_TIME          500       		 // delay enable trigger time 500*1ms=0.5S
#define SDP_STOP_PERCENT        25        		 // stop spray pressure value percent = pressure_value_max * 25%

#define TEST                    0
#define UNUSED_CNT              100
     
#define PWM_AB_SWITCH_CNT       0                // 2,4,6,8.....
#define PWM_A_SWITCH_TIME       0                // 50 * 2ms
#define PWM_B_SWITCH_TIME       0

#define PRES_SAMPLE_VAL         10		         // pressure sensor sampling interval = 10*1ms

/***********************************************************************************
 * MACROS
 */

#define VOICE_OFF_IMME()                        do{ Voice_Command( VOICE_AMP_OFF, volume ); cur_voice = 0; } while(0)

#define portCONVERT_MS_2_TICKS( x )             ( ( (x) * configTICK_RATE_HZ ) / 1000 )
#define portCONVERT_TICKS_2_MS( x )             ( ( (x) * 1000 ) / configTICK_RATE_HZ )

/***********************************************************************************
* GLOBAL VARIABLES
*/

uint8_t machine_state = TRIGER_OFF_STATE;               // Button triger ON/OFF, PMU ON/OFF
uint32_t voice_para = 0;
app_task_t app_Task;
app_task_t *pApp_Task = &app_Task;
uint16_t spray_intv = 500;
uint16_t cart_intv = 100;

led_state_t led_state[HAL_MAX_LEDS];
uint8_t led_AllOff = true;
uint16_t lcd_disp = 0;
uint16_t lcd_disp_backup = 0;
uint8_t key_value = KEY_RLS;
uint16_t key_cnt = 0;
uint8_t key_state_backup = KEY_RLS;
info_mem_t user_info, *p_user_info = &user_info;
uint32_t spray_time;

uint8_t sdp_thr = 0;
// cartidge_attr_t cart_attr, *p_cart_attr = &cart_attr, *p_bt_state = NULL;

#if 0
bt_pwm_t bt_ab_pwm;           // NOT USED

uint8_t atcab_flag = false;
uint8_t slot_data[128];
uint8_t atcab_sn[9];
new_bt_attr_t new_cart_attr, *p_new_cart_attr = &new_cart_attr;
#endif

/***********************************************************************************
* LOCAL VARIABLES
*/
volatile spray_control_state_t s_spray_state = STATE_SYSTEM_WAIT;
volatile uint8_t sys_ticket_bool = false;
volatile uint8_t sys_ticket_fg1 = false;
volatile uint32_t sys_ticket = 0;
volatile uint32_t lt_en_time = 0;
volatile uint32_t lt_sta2_time = 0;
volatile uint32_t lt_en_start = 0;
volatile uint32_t lt_en_stop = 0;
volatile uint8_t wdt_ticket = 0;
volatile uint16_t bottle_detect_time;
volatile uint16_t spray_ready_time;
volatile uint16_t spray_ready_time_total;
volatile uint8_t alarm_cnt = 0;
volatile uint8_t wdt_wakeup_time;
volatile uint8_t wakeup_type = WAKEUP_RESET;
volatile uint8_t rasps_repeat = 0;
volatile uint8_t alarm_repeat = 0;
static uint8_t spray_triger = 0;
static uint8_t spray_key_en = false;
static uint8_t flag_led_red = 0;
info_shot_t shot_info;

//180.00KHZ + 43*100HZ = 184.3KHZ
#define DEVICE_FRC         FRC_180KHZ + 45*FRC_100HZ  // current device tranducer frequency

// MCLK = 4MHZ                            
#define SCAN_CNT1               40        // SCAN FREQUENCY QTY1 
#define SCAN_CNT2               40        // SCAN FREQUENCY QTY2
#define FRC_10KHZ               1311      
#define FRC_1KHZ                131        
#define FRC_100HZ               13        // 100HZ(AUTO FRC)  
#define FRC_7HZ6                1         // 7.63HZ (AUTO FRC)  
                                           
#define PWM_SCAN_FOSC_INI       39325      //300.00KHZ
#define FRC_170KHZ              22282      //170.00KHZ  //570A
#define FRC_180KHZ              23593      //180.00KHZ  //frequency compensation
#define FRC_190KHZ              24904      //190.00KHZ  // 

static uint32_t current_frc;
static uint32_t max_frc;

static uint32_t bk_frc;

static uint16_t pwm_adc_max;
static uint16_t bk_adc_max;
static uint16_t pwm_adc_val;
static uint16_t before_pwm_adc;
static uint8_t  duty_set_val = 0;
static uint8_t sdp_cnt = 0;

static uint8_t pwm_list_offset = 0;
static uint8_t pwm_adc_offset = 0;

static uint32_t sdp_value_max;            
static uint16_t sdp_value;
static uint16_t bat_adc = 0;
static uint8_t bat_vol = 0;
static uint8_t bat_flag = BAT_CHARGE_OFF;
static uint8_t bat_cnt = 0;
static uint8_t bat_led_flash = 0;

static uint8_t sw_status = 0;
static uint8_t fg_stop_spray;

static uint16_t cnt_adc_10 = 0;
static uint16_t pwm_adc_sum = 0;
static uint16_t pwm_adc_evg = 0;
static uint8_t  curt_adc_offset1 = 0;
static uint8_t  curt_adc_offset2 = 0;
static uint16_t pwm_set_adc = 0;    
static uint16_t pwm_offset_max = 0;
static uint16_t adc_error = 0;  

static uint8_t  uln_level = 0;			// VT_LEVEL SET VALUE
static uint8_t  before_level = 0;      
static uint16_t  uln_offset = 0;

static uint8_t  cnt_adc_LT = 0;			// CURRENT need ADD
static uint8_t  cnt_adc_GT = 0;         // CURRENT need SUB

static uint8_t  current_slope = 0;
static uint8_t  cnt_slope1 = 0;
static uint8_t  cnt_slope2 = 0;

static uint8_t  cnt_alarm1A = 0;
static uint8_t  cnt_alarm1B = 0;
static uint8_t  cnt_alarm2 = 0;
static uint8_t  cnt_alarm3 = 0;
static uint8_t  cnt_alarm4 = 0;
static uint8_t  cnt_alarm5 = 0;

static uint8_t cnt_flash = 0;
static uint8_t flash_cnt = 0;
static uint8_t cnt_3S = 20;

static uint16_t cnt_delay_ini = 0;		// spray initiale preheating delay time
                                        
static uint8_t cnt_scan_1ms = 0;		// 1ms count time
                                        
static uint8_t eventActive = false;     
                                        
extern volatile uint8_t uart2RxCount;   
static uint8_t uart_rx_delay = false;   
static uint8_t uart_rx_cnt;             
                                        
static uint16_t adc_tx_val;             // uart print used
uint16_t pwm_tx_adc[400];               // uart print used                                 
uint16_t pwm_freq_adc[400];             
static uint16_t pwm_adc_cnt;

static uint8_t pwm_duty[350];           
static uint8_t uln_vol[350];
static uint16_t vol_duty_cnt;           

#if 1
/***********************************************************************************
* EXTERNAL VARIABLES
*/
extern volatile uint8_t adc_state;

uint16_t aTxBuffer[200];
uint16_t aRxBuffer[20];
/***********************************************************************************
* GLOBAL FUNCTIONS
*/
//extern uint8_t uartTest( void );
//extern uint8_t HAL_UART_Send( uint8_t *data, uint8_t length );

/***********************************************************************************
* LOCAL FUNCTIONS
*/
static void KeyTask_Process( void );

static uint8_t state_spray_ready_init( void );

static void power_off( void );
static void power_on( uint8_t type );
void LedTask_Process( void );
void ChargeTask_Process( void );
void BatTask_Process( void );
void SprayTask_Process( void );
void cart_Detect( void );
static void CartTask_Process( void );
void save_shot();

void uart_rx_process( void );
//uint8_t uartTask_Process( void );

void app_task_Init( void );
void app_task_Callback( void );
void VT_value_set(void);

/***********************************************************************************
*  @fn     
*  @brief  Reset the state machine of SprayTask
*  @param      
*            
*  @return   none
*/
void state_machine_init(void)
{
	app_task_Init();
}

uint16_t swap_16bits( uint16_t data )
{
	uint16_t tmp;
	tmp = ((uint8_t)(data >> 8)) & 0x00FF;
	tmp |= ((data <<8) & 0xFF00);

	return tmp;
}

void app_set_frq( uint32_t frq )
{
    NCO1INCU = (uint8_t)(frq >> 16);
    NCO1INCH = (uint8_t)(frq >> 8);
    NCO1INCL = (uint8_t)(frq);
}
void PWM_Duty_Set( uint8_t cnt )
{
    uint8_t cwg1con0 = 0;
    
    cwg1con0 = CWG1CON0;
    CWG1CON0 &= 0x3F;
    CWG1DBR = cnt;
    CWG1DBF = cnt;
    CWG1CON0 = cwg1con0 | 0x40;
}
void bat_init( void )
{
    uint8_t bat_vol_bk;
    
    bat_cnt = 0;
bat_s:
    bat_vol_bk = bat_vol;
    ADC_Start( ADC_BAT_CH );
    while( adc_state == ADC_BUSY );
    bat_adc = ADC_Average( ADC_BAT_CH );

    if ( bat_adc >= BATT_THR_1 ){
        bat_vol = BAT_VOL_FULL;
    }
    else if( (bat_adc < BATT_THR_1) && (bat_adc >= BATT_THR_2) ){
        bat_vol = BAT_VOL_NORMAL;
    }
    else if( (bat_adc < BATT_THR_2) && (bat_adc >= BATT_THR_3) ){
        bat_vol = BAT_VOL_LOW;
    }
    else if( (bat_adc < BATT_THR_3) && (bat_adc >= BATT_THR_4) ){  
        bat_vol = BAT_VOL_BAD;
    }
    else{
        bat_vol = BAT_VOL_OFF;
    }
    
    if ( bat_vol_bk == bat_vol ){
        if ( ++bat_cnt < 3 ){
            goto bat_s;
        }
    }
    else{
        bat_cnt = 0;
        goto bat_s;
    }
    
    bat_cnt = 0;
    // All off
    LED1R_OFF();   LED1G_OFF();   LED1B_OFF();
    LED2_OFF();    LED3_OFF();    LED4_OFF();
    
    if ( bat_vol == BAT_VOL_FULL ){
        LED1G_ON();
        LED2_ON();
        LED3_ON();
        LED4_ON();
    }
    else if ( bat_vol == BAT_VOL_NORMAL ){
        LED1G_ON();
        LED2_ON();
        LED3_ON();
    }
    else if ( bat_vol == BAT_VOL_LOW ){
        LED1G_ON();
        LED2_ON();
    }
    else if ( bat_vol == BAT_VOL_BAD ){
        LED1R_ON();
        LED1G_ON();
    }
    else if ( bat_vol == BAT_VOL_OFF ){
        LED1R_ON();
    }

}

/***********************************************************************************
*  @fn     
*  @brief  Set the environment variables, check the cartridge information
*  @param      
*            
*  @return   none
*/
void app_task_Init( void )
{
    pApp_Task->task_charge.interval = 10;
    pApp_Task->task_battery.interval = 100;
    pApp_Task->task_key.interval = 20;
    pApp_Task->task_led.interval = 75;
    pApp_Task->task_spray.interval = 20;
    pApp_Task->task_wr_flash.interval = 2000;
    pApp_Task->task_cartridge.interval = 100;
    pApp_Task->task_uart.interval = 1000;
    
    pApp_Task->task_battery.event = TASK_ACTIVE_EVT;
    pApp_Task->task_led.event = TASK_ACTIVE_EVT | LED_STATE_UPDATE_EVT;
    pApp_Task->task_key.event = TASK_IDLE_EVT;
    pApp_Task->task_spray.event = TASK_IDLE_EVT;
    pApp_Task->task_wr_flash.event = TASK_IDLE_EVT;
    pApp_Task->task_cartridge.event = TASK_IDLE_EVT | CART_DETECT_EVT;
    pApp_Task->task_uart.event = UART_RX_EVT;

    wakeup_type |= WAKEUP_ON_KEY;

 //???   memset( (uint8_t *)led_state, 0x00, sizeof(led_state) );

//???    power_on(wakeup_type);
    spray_ready_time_total = SPRAY_TURN_OFF_DLY + 20;
    
    s_spray_state = STATE_SPRAY_READY;
    sw_status = KEY_POWER_ON;     // KEY_SPRESS;
    
    // Read flash, Init parameters
    p_user_info = &user_info;
    p_user_info->total_cnt = 100;
    p_user_info->used_cnt = 0;
    p_user_info->unused_pct = 100;
    
    ///cart_Detect();
    HalSdp_Init();
    
    ADI_OE_OFF();
    DC8V_EN_OFF();
    DC24V_EN_OFF();;
    // PIC TEST
    HalLed_Init();
///    HalLed_AllOnOff( HAL_LED_MODE_ON );
    bat_init();
    shot_info.shot_num = flash_init();

}

/***********************************************************************************
*  @fn     
*  @brief  All tasks run in this function
*  @param      
*            
*  @return   none
*/

void app_task_Callback( void )
{
	if ( pApp_Task->task_charge.event & TASK_ACTIVE_EVT ){
             pApp_Task->task_charge.event ^= TASK_ACTIVE_EVT;
             pApp_Task->task_charge.interval = 75*2;				//0.15S
             ChargeTask_Process();
	}
#if 1
	if ( pApp_Task->task_battery.event & TASK_ACTIVE_EVT ){
		pApp_Task->task_battery.event ^= TASK_ACTIVE_EVT;
        if ( VBUS_GetValue() == GPIO_PIN_SET ){
            pApp_Task->task_battery.interval = 100;
        }
        else {
            pApp_Task->task_battery.interval = 1000;				//1S
            BatTask_Process();
        }
	}

	if ( pApp_Task->task_led.event & TASK_ACTIVE_EVT ){
		pApp_Task->task_led.event ^= TASK_ACTIVE_EVT;
		LedTask_Process();
	}

	if ( pApp_Task->task_key.event & TASK_ACTIVE_EVT ){
		pApp_Task->task_key.event ^= TASK_ACTIVE_EVT;
		KeyTask_Process();
        WWDT_TimerClear();
	}

	if ( pApp_Task->task_spray.event & TASK_ACTIVE_EVT ){
		pApp_Task->task_spray.event ^= TASK_ACTIVE_EVT;
		SprayTask_Process();
		pApp_Task->task_spray.interval = spray_intv;
	}

    if ( pApp_Task->task_cartridge.event & TASK_ACTIVE_EVT ){
        pApp_Task->task_cartridge.event ^= TASK_ACTIVE_EVT;
        pApp_Task->task_cartridge.interval = cart_intv;
        CartTask_Process();
    }
	if ( pApp_Task->task_power_off.event & TASK_ACTIVE_EVT ){
        pApp_Task->task_power_off.event ^= TASK_ACTIVE_EVT;
        power_off();
	}

    if ( pApp_Task->task_uart.event & TASK_ACTIVE_EVT ){
		pApp_Task->task_uart.event ^= TASK_ACTIVE_EVT;
		pApp_Task->task_uart.interval = 100;
		uart_rx_process();
		//uartTask_Process();
	}
#endif
}

void VT_value_set( void )                                                                                                                                                                     
{
    switch( uln_level )
    {
        case 0:
            ULN_IN0_OFF();
        	ULN_IN1_OFF();
        	ULN_IN2_OFF();
        	ULN_IN3_OFF();
        	ULN_IN4_OFF();
            ULN_IN5_OFF();
            ULN_IN6_OFF();
            break;
        case 1:
            ULN_IN0_ON();
        	ULN_IN1_OFF();
       	    ULN_IN2_OFF();
       	    ULN_IN3_OFF();
       	    ULN_IN4_OFF();
            ULN_IN5_OFF();
            ULN_IN6_OFF();
            break;
        case 2:
        	ULN_IN0_OFF();
      	    ULN_IN1_ON();
      	    ULN_IN2_OFF();
      	    ULN_IN3_OFF();
            ULN_IN4_OFF();
            ULN_IN5_OFF();
            ULN_IN6_OFF();
            break;
        case 3:
        	ULN_IN0_ON();
     	    ULN_IN1_ON();
     	    ULN_IN2_OFF();
     	    ULN_IN3_OFF();
            ULN_IN4_OFF();
            ULN_IN5_OFF();
            ULN_IN6_OFF();
            break;
        case 4:
        	ULN_IN0_OFF();
    	    ULN_IN1_OFF();
    	    ULN_IN2_ON();
    	    ULN_IN3_OFF();
            ULN_IN4_OFF();
            ULN_IN5_OFF();
            ULN_IN6_OFF();
            break;
        case 5:
        	ULN_IN0_ON();
       	    ULN_IN1_OFF();
       	    ULN_IN2_ON();
       	    ULN_IN3_OFF();
            ULN_IN4_OFF();
            ULN_IN5_OFF();
            ULN_IN6_OFF();
            break;
        case 6:
        	ULN_IN0_OFF();
      	    ULN_IN1_ON();
      	    ULN_IN2_ON();
      	    ULN_IN3_OFF();
            ULN_IN4_OFF();
            ULN_IN5_OFF();
            ULN_IN6_OFF();
            break;
        case 7:
        	ULN_IN0_ON();
     	    ULN_IN1_ON();
     	    ULN_IN2_ON();
     	    ULN_IN3_OFF();
            ULN_IN4_OFF();
            ULN_IN5_OFF();
            ULN_IN6_OFF();
            break;
        case 8:
        	ULN_IN0_OFF();
    	    ULN_IN1_OFF();
    	    ULN_IN2_OFF();
    	    ULN_IN3_ON();
            ULN_IN4_OFF();
            ULN_IN5_OFF();
            ULN_IN6_OFF();
            break;
        case 9:
        	ULN_IN0_ON();
       	    ULN_IN1_OFF();
       	    ULN_IN2_OFF();
       	    ULN_IN3_ON();
            ULN_IN4_OFF();
            ULN_IN5_OFF();
            ULN_IN6_OFF();
            break;
        case 10:
        	ULN_IN0_OFF();
      	    ULN_IN1_ON();
      	    ULN_IN2_OFF();
      	    ULN_IN3_ON();
            ULN_IN4_OFF();
            ULN_IN5_OFF();
            ULN_IN6_OFF();
            break;
        case 11:
        	ULN_IN0_ON();
     	    ULN_IN1_ON();
     	    ULN_IN2_OFF();
     	    ULN_IN3_ON();
            ULN_IN4_OFF();
            ULN_IN5_OFF();
            ULN_IN6_OFF();
            break;
        case 12:
        	ULN_IN0_OFF();
    	    ULN_IN1_OFF();
    	    ULN_IN2_ON();
    	    ULN_IN3_ON();
            ULN_IN4_OFF();
            ULN_IN5_OFF();
            ULN_IN6_OFF();
            break;
        case 13:
        	ULN_IN0_ON();
       	    ULN_IN1_OFF();
       	    ULN_IN2_ON();
       	    ULN_IN3_ON();
            ULN_IN4_OFF();
            ULN_IN5_OFF();
            ULN_IN6_OFF();
            break;
        case 14:
        	ULN_IN0_OFF();
      	    ULN_IN1_ON();
      	    ULN_IN2_ON();
      	    ULN_IN3_ON();
            ULN_IN4_OFF();
            ULN_IN5_OFF();
            ULN_IN6_OFF();
            break;
        case 15:
        	ULN_IN0_ON();
     	    ULN_IN1_ON();
     	    ULN_IN2_ON();
     	    ULN_IN3_ON();
            ULN_IN4_OFF();
            ULN_IN5_OFF();
            ULN_IN6_OFF();
            break;
         case 16:
            ULN_IN0_OFF();
        	ULN_IN1_OFF();
        	ULN_IN2_OFF();
        	ULN_IN3_OFF();
        	ULN_IN4_ON();  
            ULN_IN5_OFF();
            ULN_IN6_OFF();
            break;
        case 17:
            ULN_IN0_ON();
        	ULN_IN1_OFF();
       	    ULN_IN2_OFF();
       	    ULN_IN3_OFF();
       	    ULN_IN4_ON();
            ULN_IN5_OFF();
            ULN_IN6_OFF();            
            break;
        case 18:
        	ULN_IN0_OFF();
      	    ULN_IN1_ON();
      	    ULN_IN2_OFF();
      	    ULN_IN3_OFF();
            ULN_IN4_ON();
            ULN_IN5_OFF();
            ULN_IN6_OFF();   
            break;
        case 19:
        	ULN_IN0_ON();
     	    ULN_IN1_ON();
     	    ULN_IN2_OFF();
     	    ULN_IN3_OFF();
            ULN_IN4_ON();
            ULN_IN5_OFF();
            ULN_IN6_OFF(); 
            break;
        case 20:
        	ULN_IN0_OFF();
    	    ULN_IN1_OFF();
    	    ULN_IN2_ON();
    	    ULN_IN3_OFF();
            ULN_IN4_ON();
            ULN_IN5_OFF();
            ULN_IN6_OFF(); 
            break;
        case 21:
        	ULN_IN0_ON();
       	    ULN_IN1_OFF();
       	    ULN_IN2_ON();
       	    ULN_IN3_OFF();
            ULN_IN4_ON();
            ULN_IN5_OFF();
            ULN_IN6_OFF();            
            break;
        case 22:
        	ULN_IN0_OFF();
      	    ULN_IN1_ON();
      	    ULN_IN2_ON();
      	    ULN_IN3_OFF();
            ULN_IN4_ON();
            ULN_IN5_OFF();
            ULN_IN6_OFF();             
            break;
        case 23:
        	ULN_IN0_ON();
     	    ULN_IN1_ON();
     	    ULN_IN2_ON();
     	    ULN_IN3_OFF();
            ULN_IN4_ON();
            ULN_IN5_OFF();
            ULN_IN6_OFF(); 
            break;
        case 24:
        	ULN_IN0_OFF();
    	    ULN_IN1_OFF();
    	    ULN_IN2_OFF();
    	    ULN_IN3_ON();
            ULN_IN4_ON();
            ULN_IN5_OFF();
            ULN_IN6_OFF(); 
            break;
        case 25:
        	ULN_IN0_ON();
       	    ULN_IN1_OFF();
       	    ULN_IN2_OFF();
       	    ULN_IN3_ON();
            ULN_IN4_ON();
            ULN_IN5_OFF();
            ULN_IN6_OFF(); 
            break;
        case 26:
        	ULN_IN0_OFF();
      	    ULN_IN1_ON();
      	    ULN_IN2_OFF();
      	    ULN_IN3_ON();
            ULN_IN4_ON();
            ULN_IN5_OFF();
            ULN_IN6_OFF(); 
            break;
        case 27:
        	ULN_IN0_ON();
     	    ULN_IN1_ON();
     	    ULN_IN2_OFF();
     	    ULN_IN3_ON();
            ULN_IN4_ON();
            ULN_IN5_OFF();
            ULN_IN6_OFF(); 
            break;
        case 28:
        	ULN_IN0_OFF();
    	    ULN_IN1_OFF();
    	    ULN_IN2_ON();
    	    ULN_IN3_ON();
            ULN_IN4_ON();
            ULN_IN5_OFF();
            ULN_IN6_OFF(); 
            break;
        case 29:
        	ULN_IN0_ON();
       	    ULN_IN1_OFF();
       	    ULN_IN2_ON();
       	    ULN_IN3_ON();
            ULN_IN4_ON();
            ULN_IN5_OFF();
            ULN_IN6_OFF(); 
            break;
        case 30:
        	ULN_IN0_OFF();
      	    ULN_IN1_ON();
      	    ULN_IN2_ON();
      	    ULN_IN3_ON();
            ULN_IN4_ON();     
            ULN_IN5_OFF();
            ULN_IN6_OFF();            
            break;
        case 31:
        	ULN_IN0_ON();
     	    ULN_IN1_ON();
     	    ULN_IN2_ON();
     	    ULN_IN3_ON();
            ULN_IN4_ON();
            ULN_IN5_OFF();
            ULN_IN6_OFF();            
            break; 
        case 32:
            ULN_IN0_OFF();
        	ULN_IN1_OFF();
        	ULN_IN2_OFF();
        	ULN_IN3_OFF();
        	ULN_IN4_OFF();
            ULN_IN5_ON();
            ULN_IN6_OFF();
            break;
        case 33:
            ULN_IN0_ON();
        	ULN_IN1_OFF();
       	    ULN_IN2_OFF();
       	    ULN_IN3_OFF();
       	    ULN_IN4_OFF();
            ULN_IN5_ON();
            ULN_IN6_OFF();
            break;
        case 34:
        	ULN_IN0_OFF();
      	    ULN_IN1_ON();
      	    ULN_IN2_OFF();
      	    ULN_IN3_OFF();
            ULN_IN4_OFF();
            ULN_IN5_ON();
            ULN_IN6_OFF();
            break;
        case 35:
        	ULN_IN0_ON();
     	    ULN_IN1_ON();
     	    ULN_IN2_OFF();
     	    ULN_IN3_OFF();
            ULN_IN4_OFF();
            ULN_IN5_ON();
            ULN_IN6_OFF();
            break;
        case 36:
        	ULN_IN0_OFF();
    	    ULN_IN1_OFF();
    	    ULN_IN2_ON();
    	    ULN_IN3_OFF();
            ULN_IN4_OFF();
            ULN_IN5_ON();
            ULN_IN6_OFF();
            break;
        case 37:
        	ULN_IN0_ON();
       	    ULN_IN1_OFF();
       	    ULN_IN2_ON();
       	    ULN_IN3_OFF();
            ULN_IN4_OFF();
            ULN_IN5_ON();
            ULN_IN6_OFF();
            break;
        case 38:
        	ULN_IN0_OFF();
      	    ULN_IN1_ON();
      	    ULN_IN2_ON();
      	    ULN_IN3_OFF();
            ULN_IN4_OFF();
            ULN_IN5_ON();
            ULN_IN6_OFF();
            break;
        case 39:
        	ULN_IN0_ON();
     	    ULN_IN1_ON();
     	    ULN_IN2_ON();
     	    ULN_IN3_OFF();
            ULN_IN4_OFF();
            ULN_IN5_ON();
            ULN_IN6_OFF();
            break;
        case 40:
        	ULN_IN0_OFF();
    	    ULN_IN1_OFF();
    	    ULN_IN2_OFF();
    	    ULN_IN3_ON();
            ULN_IN4_OFF();
            ULN_IN5_ON();
            ULN_IN6_OFF();
            break;
        case 41:
        	ULN_IN0_ON();
       	    ULN_IN1_OFF();
       	    ULN_IN2_OFF();
       	    ULN_IN3_ON();
            ULN_IN4_OFF();
            ULN_IN5_ON();
            ULN_IN6_OFF();
            break;
        case 42:
        	ULN_IN0_OFF();
      	    ULN_IN1_ON();
      	    ULN_IN2_OFF();
      	    ULN_IN3_ON();
            ULN_IN4_OFF();
            ULN_IN5_ON();
            ULN_IN6_OFF();
            break;
        case 43:
        	ULN_IN0_ON();
     	    ULN_IN1_ON();
     	    ULN_IN2_OFF();
     	    ULN_IN3_ON();
            ULN_IN4_OFF();
            ULN_IN5_ON();
            ULN_IN6_OFF();
            break;
        case 44:
        	ULN_IN0_OFF();
    	    ULN_IN1_OFF();
    	    ULN_IN2_ON();
    	    ULN_IN3_ON();
            ULN_IN4_OFF();
            ULN_IN5_ON();
            ULN_IN6_OFF();
            break;
        case 45:
        	ULN_IN0_ON();
       	    ULN_IN1_OFF();
       	    ULN_IN2_ON();
       	    ULN_IN3_ON();
            ULN_IN4_OFF();
            ULN_IN5_ON();
            ULN_IN6_OFF();
            break;
        case 46:
        	ULN_IN0_OFF();
      	    ULN_IN1_ON();
      	    ULN_IN2_ON();
      	    ULN_IN3_ON();
            ULN_IN4_OFF();
            ULN_IN5_ON();
            ULN_IN6_OFF();
            break;
        case 47:
        	ULN_IN0_ON();
     	    ULN_IN1_ON();
     	    ULN_IN2_ON();
     	    ULN_IN3_ON();
            ULN_IN4_OFF();
            ULN_IN5_ON();
            ULN_IN6_OFF();
            break;
         case 48:
            ULN_IN0_OFF();
        	ULN_IN1_OFF();
        	ULN_IN2_OFF();
        	ULN_IN3_OFF();
        	ULN_IN4_ON();  
            ULN_IN5_ON();
            ULN_IN6_OFF();
            break;
        case 49:
            ULN_IN0_ON();
        	ULN_IN1_OFF();
       	    ULN_IN2_OFF();
       	    ULN_IN3_OFF();
       	    ULN_IN4_ON();
            ULN_IN5_ON();
            ULN_IN6_OFF();            
            break;
        case 50:
        	ULN_IN0_OFF();
      	    ULN_IN1_ON();
      	    ULN_IN2_OFF();
      	    ULN_IN3_OFF();
            ULN_IN4_ON();
            ULN_IN5_ON();
            ULN_IN6_OFF();   
            break;
        case 51:
        	ULN_IN0_ON();
     	    ULN_IN1_ON();
     	    ULN_IN2_OFF();
     	    ULN_IN3_OFF();
            ULN_IN4_ON();
            ULN_IN5_ON();
            ULN_IN6_OFF(); 
            break;
        case 52:
        	ULN_IN0_OFF();
    	    ULN_IN1_OFF();
    	    ULN_IN2_ON();
    	    ULN_IN3_OFF();
            ULN_IN4_ON();
            ULN_IN5_ON();
            ULN_IN6_OFF(); 
            break;
        case 53:
        	ULN_IN0_ON();
       	    ULN_IN1_OFF();
       	    ULN_IN2_ON();
       	    ULN_IN3_OFF();
            ULN_IN4_ON();
            ULN_IN5_ON();
            ULN_IN6_OFF();            
            break;
        case 54:
        	ULN_IN0_OFF();
      	    ULN_IN1_ON();
      	    ULN_IN2_ON();
      	    ULN_IN3_OFF();
            ULN_IN4_ON();
            ULN_IN5_ON();
            ULN_IN6_OFF();             
            break;
        case 55:
        	ULN_IN0_ON();
     	    ULN_IN1_ON();
     	    ULN_IN2_ON();
     	    ULN_IN3_OFF();
            ULN_IN4_ON();
            ULN_IN5_ON();
            ULN_IN6_OFF(); 
            break;
        case 56:
        	ULN_IN0_OFF();
    	    ULN_IN1_OFF();
    	    ULN_IN2_OFF();
    	    ULN_IN3_ON();
            ULN_IN4_ON();
            ULN_IN5_ON();
            ULN_IN6_OFF(); 
            break;
        case 57:
        	ULN_IN0_ON();
       	    ULN_IN1_OFF();
       	    ULN_IN2_OFF();
       	    ULN_IN3_ON();
            ULN_IN4_ON();
            ULN_IN5_ON();
            ULN_IN6_OFF(); 
            break;
        case 58:
        	ULN_IN0_OFF();
      	    ULN_IN1_ON();
      	    ULN_IN2_OFF();
      	    ULN_IN3_ON();
            ULN_IN4_ON();
            ULN_IN5_ON();
            ULN_IN6_OFF(); 
            break;
        case 59:
        	ULN_IN0_ON();
     	    ULN_IN1_ON();
     	    ULN_IN2_OFF();
     	    ULN_IN3_ON();
            ULN_IN4_ON();
            ULN_IN5_ON();
            ULN_IN6_OFF(); 
            break;
        case 60:
        	ULN_IN0_OFF();
    	    ULN_IN1_OFF();
    	    ULN_IN2_ON();
    	    ULN_IN3_ON();
            ULN_IN4_ON();
            ULN_IN5_ON();
            ULN_IN6_OFF(); 
            break;
        case 61:
        	ULN_IN0_ON();
       	    ULN_IN1_OFF();
       	    ULN_IN2_ON();
       	    ULN_IN3_ON();
            ULN_IN4_ON();
            ULN_IN5_ON();
            ULN_IN6_OFF(); 
            break;
        case 62:
        	ULN_IN0_OFF();
      	    ULN_IN1_ON();
      	    ULN_IN2_ON();
      	    ULN_IN3_ON();
            ULN_IN4_ON();     
            ULN_IN5_ON();
            ULN_IN6_OFF();            
            break;
        case 63:
        	ULN_IN0_ON();
     	    ULN_IN1_ON();
     	    ULN_IN2_ON();
     	    ULN_IN3_ON();
            ULN_IN4_ON();
            ULN_IN5_ON();
            ULN_IN6_OFF();            
            break; 
        case 64:
            ULN_IN0_OFF();
        	ULN_IN1_OFF();
        	ULN_IN2_OFF();
        	ULN_IN3_OFF();
        	ULN_IN4_OFF();
            ULN_IN5_OFF();
            ULN_IN6_ON();
            break;
        case 65:
            ULN_IN0_ON();
        	ULN_IN1_OFF();
       	    ULN_IN2_OFF();
       	    ULN_IN3_OFF();
       	    ULN_IN4_OFF();
            ULN_IN5_OFF();
            ULN_IN6_ON();
            break;
        case 66:
        	ULN_IN0_OFF();
      	    ULN_IN1_ON();
      	    ULN_IN2_OFF();
      	    ULN_IN3_OFF();
            ULN_IN4_OFF();
            ULN_IN5_OFF();
            ULN_IN6_ON();
            break;
        case 67:
        	ULN_IN0_ON();
     	    ULN_IN1_ON();
     	    ULN_IN2_OFF();
     	    ULN_IN3_OFF();
            ULN_IN4_OFF();
            ULN_IN5_OFF();
            ULN_IN6_ON();
            break;
        case 68:
        	ULN_IN0_OFF();
    	    ULN_IN1_OFF();
    	    ULN_IN2_ON();
    	    ULN_IN3_OFF();
            ULN_IN4_OFF();
            ULN_IN5_OFF();
            ULN_IN6_ON();
            break;
        case 69:
        	ULN_IN0_ON();
       	    ULN_IN1_OFF();
       	    ULN_IN2_ON();
       	    ULN_IN3_OFF();
            ULN_IN4_OFF();
            ULN_IN5_OFF();
            ULN_IN6_ON();
            break;
        case 70:
        	ULN_IN0_OFF();
      	    ULN_IN1_ON();
      	    ULN_IN2_ON();
      	    ULN_IN3_OFF();
            ULN_IN4_OFF();
            ULN_IN5_OFF();
            ULN_IN6_ON();
            break;
        case 71:
        	ULN_IN0_ON();
     	    ULN_IN1_ON();
     	    ULN_IN2_ON();
     	    ULN_IN3_OFF();
            ULN_IN4_OFF();
            ULN_IN5_OFF();
            ULN_IN6_ON();
            break;
        case 72:
        	ULN_IN0_OFF();
    	    ULN_IN1_OFF();
    	    ULN_IN2_OFF();
    	    ULN_IN3_ON();
            ULN_IN4_OFF();
            ULN_IN5_OFF();
            ULN_IN6_ON();
            break;
        case 73:
        	ULN_IN0_ON();
       	    ULN_IN1_OFF();
       	    ULN_IN2_OFF();
       	    ULN_IN3_ON();
            ULN_IN4_OFF();
            ULN_IN5_OFF();
            ULN_IN6_ON();
            break;
        case 74:
        	ULN_IN0_OFF();
      	    ULN_IN1_ON();
      	    ULN_IN2_OFF();
      	    ULN_IN3_ON();
            ULN_IN4_OFF();
            ULN_IN5_OFF();
            ULN_IN6_ON();
            break;
        case 75:
        	ULN_IN0_ON();
     	    ULN_IN1_ON();
     	    ULN_IN2_OFF();
     	    ULN_IN3_ON();
            ULN_IN4_OFF();
            ULN_IN5_OFF();
            ULN_IN6_ON();
            break;
        case 76:
        	ULN_IN0_OFF();
    	    ULN_IN1_OFF();
    	    ULN_IN2_ON();
    	    ULN_IN3_ON();
            ULN_IN4_OFF();
            ULN_IN5_OFF();
            ULN_IN6_ON();
            break;
        case 77:
        	ULN_IN0_ON();
       	    ULN_IN1_OFF();
       	    ULN_IN2_ON();
       	    ULN_IN3_ON();
            ULN_IN4_OFF();
            ULN_IN5_OFF();
            ULN_IN6_ON();
            break;
        case 78:
        	ULN_IN0_OFF();
      	    ULN_IN1_ON();
      	    ULN_IN2_ON();
      	    ULN_IN3_ON();
            ULN_IN4_OFF();
            ULN_IN5_OFF();
            ULN_IN6_ON();
            break;
        case 79:
        	ULN_IN0_ON();
     	    ULN_IN1_ON();
     	    ULN_IN2_ON();
     	    ULN_IN3_ON();
            ULN_IN4_OFF();
            ULN_IN5_OFF();
            ULN_IN6_ON();
            break;
         case 80:
            ULN_IN0_OFF();
        	ULN_IN1_OFF();
        	ULN_IN2_OFF();
        	ULN_IN3_OFF();
        	ULN_IN4_ON();  
            ULN_IN5_OFF();
            ULN_IN6_ON();
            break;
        case 81:
            ULN_IN0_ON();
        	ULN_IN1_OFF();
       	    ULN_IN2_OFF();
       	    ULN_IN3_OFF();
       	    ULN_IN4_ON();
            ULN_IN5_OFF();
            ULN_IN6_ON();           
            break;
        case 82:
        	ULN_IN0_OFF();
      	    ULN_IN1_ON();
      	    ULN_IN2_OFF();
      	    ULN_IN3_OFF();
            ULN_IN4_ON();
            ULN_IN5_OFF();
            ULN_IN6_ON();   
            break;
        case 83:
        	ULN_IN0_ON();
     	    ULN_IN1_ON();
     	    ULN_IN2_OFF();
     	    ULN_IN3_OFF();
            ULN_IN4_ON();
            ULN_IN5_OFF();
            ULN_IN6_ON();
            break;
        case 84:
        	ULN_IN0_OFF();
    	    ULN_IN1_OFF();
    	    ULN_IN2_ON();
    	    ULN_IN3_OFF();
            ULN_IN4_ON();
            ULN_IN5_OFF();
            ULN_IN6_ON();
            break;
        case 85:
        	ULN_IN0_ON();
       	    ULN_IN1_OFF();
       	    ULN_IN2_ON();
       	    ULN_IN3_OFF();
            ULN_IN4_ON();
            ULN_IN5_OFF();
            ULN_IN6_ON();           
            break;
        case 86:
        	ULN_IN0_OFF();
      	    ULN_IN1_ON();
      	    ULN_IN2_ON();
      	    ULN_IN3_OFF();
            ULN_IN4_ON();
            ULN_IN5_OFF();
            ULN_IN6_ON();            
            break;
        case 87:
        	ULN_IN0_ON();
     	    ULN_IN1_ON();
     	    ULN_IN2_ON();
     	    ULN_IN3_OFF();
            ULN_IN4_ON();
            ULN_IN5_OFF();
            ULN_IN6_ON();
            break;
        case 88:
        	ULN_IN0_OFF();
    	    ULN_IN1_OFF();
    	    ULN_IN2_OFF();
    	    ULN_IN3_ON();
            ULN_IN4_ON();
            ULN_IN5_OFF();
            ULN_IN6_ON();
            break;
        case 89:
        	ULN_IN0_ON();
       	    ULN_IN1_OFF();
       	    ULN_IN2_OFF();
       	    ULN_IN3_ON();
            ULN_IN4_ON();
            ULN_IN5_OFF();
            ULN_IN6_ON();
            break;
        case 90:
        	ULN_IN0_OFF();
      	    ULN_IN1_ON();
      	    ULN_IN2_OFF();
      	    ULN_IN3_ON();
            ULN_IN4_ON();
            ULN_IN5_OFF();
            ULN_IN6_ON();
            break;
        case 91:
        	ULN_IN0_ON();
     	    ULN_IN1_ON();
     	    ULN_IN2_OFF();
     	    ULN_IN3_ON();
            ULN_IN4_ON();
            ULN_IN5_OFF();
            ULN_IN6_ON();
            break;
        case 92:
        	ULN_IN0_OFF();
    	    ULN_IN1_OFF();
    	    ULN_IN2_ON();
    	    ULN_IN3_ON();
            ULN_IN4_ON();
            ULN_IN5_OFF();
            ULN_IN6_ON();
            break;
        case 93:
        	ULN_IN0_ON();
       	    ULN_IN1_OFF();
       	    ULN_IN2_ON();
       	    ULN_IN3_ON();
            ULN_IN4_ON();
            ULN_IN5_OFF();
            ULN_IN6_ON();
            break;
        case 94:
        	ULN_IN0_OFF();
      	    ULN_IN1_ON();
      	    ULN_IN2_ON();
      	    ULN_IN3_ON();
            ULN_IN4_ON();     
            ULN_IN5_OFF();
            ULN_IN6_ON();          
            break;
        case 95:
        	ULN_IN0_ON();
     	    ULN_IN1_ON();
     	    ULN_IN2_ON();
     	    ULN_IN3_ON();
            ULN_IN4_ON();
            ULN_IN5_OFF();
            ULN_IN6_ON();            
            break; 
        case 96:
            ULN_IN0_OFF();
        	ULN_IN1_OFF();
        	ULN_IN2_OFF();
        	ULN_IN3_OFF();
        	ULN_IN4_OFF();
            ULN_IN5_ON();
            ULN_IN6_ON();
            break;
        case 97:
            ULN_IN0_ON();
        	ULN_IN1_OFF();
       	    ULN_IN2_OFF();
       	    ULN_IN3_OFF();
       	    ULN_IN4_OFF();
            ULN_IN5_ON();
            ULN_IN6_ON();
            break;
        case 98:
        	ULN_IN0_OFF();
      	    ULN_IN1_ON();
      	    ULN_IN2_OFF();
      	    ULN_IN3_OFF();
            ULN_IN4_OFF();
            ULN_IN5_ON();
            ULN_IN6_ON();
            break;
        case 99:
        	ULN_IN0_ON();
     	    ULN_IN1_ON();
     	    ULN_IN2_OFF();
     	    ULN_IN3_OFF();
            ULN_IN4_OFF();
            ULN_IN5_ON();
            ULN_IN6_ON();
            break;
        case 100:
        	ULN_IN0_OFF();
    	    ULN_IN1_OFF();
    	    ULN_IN2_ON();
    	    ULN_IN3_OFF();
            ULN_IN4_OFF();
            ULN_IN5_ON();
            ULN_IN6_ON();
            break;
        case 101:
        	ULN_IN0_ON();
       	    ULN_IN1_OFF();
       	    ULN_IN2_ON();
       	    ULN_IN3_OFF();
            ULN_IN4_OFF();
            ULN_IN5_ON();
            ULN_IN6_ON();
            break;
        case 102:
        	ULN_IN0_OFF();
      	    ULN_IN1_ON();
      	    ULN_IN2_ON();
      	    ULN_IN3_OFF();
            ULN_IN4_OFF();
            ULN_IN5_ON();
            ULN_IN6_ON();
            break;
        case 103:
        	ULN_IN0_ON();
     	    ULN_IN1_ON();
     	    ULN_IN2_ON();
     	    ULN_IN3_OFF();
            ULN_IN4_OFF();
            ULN_IN5_ON();
            ULN_IN6_ON();
            break;
        case 104:
        	ULN_IN0_OFF();
    	    ULN_IN1_OFF();
    	    ULN_IN2_OFF();
    	    ULN_IN3_ON();
            ULN_IN4_OFF();
            ULN_IN5_ON();
            ULN_IN6_ON();
            break;
        case 105:
        	ULN_IN0_ON();
       	    ULN_IN1_OFF();
       	    ULN_IN2_OFF();
       	    ULN_IN3_ON();
            ULN_IN4_OFF();
            ULN_IN5_ON();
            ULN_IN6_ON();
            break;
        case 106:
        	ULN_IN0_OFF();
      	    ULN_IN1_ON();
      	    ULN_IN2_OFF();
      	    ULN_IN3_ON();
            ULN_IN4_OFF();
            ULN_IN5_ON();
            ULN_IN6_ON();
            break;
        case 107:
        	ULN_IN0_ON();
     	    ULN_IN1_ON();
     	    ULN_IN2_OFF();
     	    ULN_IN3_ON();
            ULN_IN4_OFF();
            ULN_IN5_ON();
            ULN_IN6_ON();
            break;
        case 108:
        	ULN_IN0_OFF();
    	    ULN_IN1_OFF();
    	    ULN_IN2_ON();
    	    ULN_IN3_ON();
            ULN_IN4_OFF();
            ULN_IN5_ON();
            ULN_IN6_ON();
            break;
        case 109:
        	ULN_IN0_ON();
       	    ULN_IN1_OFF();
       	    ULN_IN2_ON();
       	    ULN_IN3_ON();
            ULN_IN4_OFF();
            ULN_IN5_ON();
            ULN_IN6_ON();
            break;
        case 110:
        	ULN_IN0_OFF();
      	    ULN_IN1_ON();
      	    ULN_IN2_ON();
      	    ULN_IN3_ON();
            ULN_IN4_OFF();
            ULN_IN5_ON();
            ULN_IN6_ON();
            break;
        case 111:
        	ULN_IN0_ON();
     	    ULN_IN1_ON();
     	    ULN_IN2_ON();
     	    ULN_IN3_ON();
            ULN_IN4_OFF();
            ULN_IN5_ON();
            ULN_IN6_ON();
            break;
         case 112:
            ULN_IN0_OFF();
        	ULN_IN1_OFF();
        	ULN_IN2_OFF();
        	ULN_IN3_OFF();
        	ULN_IN4_ON();  
            ULN_IN5_ON();
            ULN_IN6_ON();
            break;
        case 113:
            ULN_IN0_ON();
        	ULN_IN1_OFF();
       	    ULN_IN2_OFF();
       	    ULN_IN3_OFF();
       	    ULN_IN4_ON();
            ULN_IN5_ON();
            ULN_IN6_ON();    
            break;
        case 114:
        	ULN_IN0_OFF();
      	    ULN_IN1_ON();
      	    ULN_IN2_OFF();
      	    ULN_IN3_OFF();
            ULN_IN4_ON();
            ULN_IN5_ON();
            ULN_IN6_ON(); 
            break;
        case 115:
        	ULN_IN0_ON();
     	    ULN_IN1_ON();
     	    ULN_IN2_OFF();
     	    ULN_IN3_OFF();
            ULN_IN4_ON();
            ULN_IN5_ON();
            ULN_IN6_ON();
            break;
        case 116:
        	ULN_IN0_OFF();
    	    ULN_IN1_OFF();
    	    ULN_IN2_ON();
    	    ULN_IN3_OFF();
            ULN_IN4_ON();
            ULN_IN5_ON();
            ULN_IN6_ON();
            break;
        case 117:
        	ULN_IN0_ON();
       	    ULN_IN1_OFF();
       	    ULN_IN2_ON();
       	    ULN_IN3_OFF();
            ULN_IN4_ON();
            ULN_IN5_ON();
            ULN_IN6_ON();           
            break;
        case 118:
        	ULN_IN0_OFF();
      	    ULN_IN1_ON();
      	    ULN_IN2_ON();
      	    ULN_IN3_OFF();
            ULN_IN4_ON();
            ULN_IN5_ON();
            ULN_IN6_ON();          
            break;
        case 119:
        	ULN_IN0_ON();
     	    ULN_IN1_ON();
     	    ULN_IN2_ON();
     	    ULN_IN3_OFF();
            ULN_IN4_ON();
            ULN_IN5_ON();
            ULN_IN6_ON();
            break;
        case 120:
        	ULN_IN0_OFF();
    	    ULN_IN1_OFF();
    	    ULN_IN2_OFF();
    	    ULN_IN3_ON();
            ULN_IN4_ON();
            ULN_IN5_ON();
            ULN_IN6_ON();
            break;
        case 121:
        	ULN_IN0_ON();
       	    ULN_IN1_OFF();
       	    ULN_IN2_OFF();
       	    ULN_IN3_ON();
            ULN_IN4_ON();
            ULN_IN5_ON();
            ULN_IN6_ON();
            break;
        case 122:
        	ULN_IN0_OFF();
      	    ULN_IN1_ON();
      	    ULN_IN2_OFF();
      	    ULN_IN3_ON();
            ULN_IN4_ON();
            ULN_IN5_ON();
            ULN_IN6_ON();
            break;
        case 123:
        	ULN_IN0_ON();
     	    ULN_IN1_ON();
     	    ULN_IN2_OFF();
     	    ULN_IN3_ON();
            ULN_IN4_ON();
            ULN_IN5_ON();
            ULN_IN6_ON();
            break;
        case 124:
        	ULN_IN0_OFF();
    	    ULN_IN1_OFF();
    	    ULN_IN2_ON();
    	    ULN_IN3_ON();
            ULN_IN4_ON();
            ULN_IN5_ON();
            ULN_IN6_ON(); 
            break;
        case 125:
        	ULN_IN0_ON();
       	    ULN_IN1_OFF();
       	    ULN_IN2_ON();
       	    ULN_IN3_ON();
            ULN_IN4_ON();
            ULN_IN5_ON();
            ULN_IN6_ON(); 
            break;
        case 126:
        	ULN_IN0_OFF();
      	    ULN_IN1_ON();
      	    ULN_IN2_ON();
      	    ULN_IN3_ON();
            ULN_IN4_ON();     
            ULN_IN5_ON();
            ULN_IN6_ON();            
            break;
        case 127:
        	ULN_IN0_ON();
     	    ULN_IN1_ON();
     	    ULN_IN2_ON();
     	    ULN_IN3_ON();
            ULN_IN4_ON();
            ULN_IN5_ON();
            ULN_IN6_ON();           
            break; 
        default:
        	uln_level = 0;
            ULN_IN0_OFF(); 
        	ULN_IN1_OFF();
        	ULN_IN2_OFF();
        	ULN_IN3_OFF();
        	ULN_IN4_OFF();
            ULN_IN5_OFF();
            ULN_IN6_OFF();             
            break;
    }
}
void KeyTask_Process( void )
{
    uint8_t key;
    
     if ( lt_en_time ) {
      return;
    }

    key = key_value;   // halKeyScan();
    key_value = KEY_RLS;
    
    if ( (sw_status == KEY_POWER_ON) && (key != KEY_RLS) ){
        return;
    }
    
    sw_status = key;
    if ( sw_status == KEY_ON_HOLD ){
        spray_ready_time = 0;
    }
    else if ( sw_status == KEY_SPRESS ){
        if ( s_spray_state == STATE_SPRAY_READY ){
            spray_key_en = true;
        }
        else
        {
            spray_key_en = false;
        }
    }
    else if ( sw_status == KEY_LPRESS ){
        power_off();
    }
}

void ChargeTask_Process( void )                                                                                                     
{   
    uint8_t vbus_pin, charge_pin;
    uint8_t bat_flag_bk;

    bat_flag_bk = bat_flag;
    bat_flag = BAT_CHARGE_OFF;

    if ( VBUS_GetValue() == GPIO_PIN_SET )
    {                                                
        delay_ms(3);                            
        vbus_pin = VBUS_GetValue();
        charge_pin = CHRG_GetValue();
        if ( vbus_pin == GPIO_PIN_SET )
        {
            pApp_Task->task_power_off.interval = 0;
            pApp_Task->task_power_off.event = 0;
            if ( charge_pin == GPIO_PIN_RESET )
            {
                bat_flag = BAT_CHARGE_ON;
            }
            else
            {
                bat_flag = BAT_CHARGE_FULL;
            }
        }
    }
    else 
    {
        bat_led_flash = 0;
    } 
    if ( bat_flag != bat_flag_bk )
    {
            pApp_Task->task_led.interval = 5;
            pApp_Task->task_led.event |= LED_STATE_UPDATE_EVT;
    }
    if ( bat_flag == BAT_CHARGE_ON )
    {
        pApp_Task->task_charge.interval = 250*2;				//0.5S
        
        if ( ++bat_led_flash > 5 ){
            bat_led_flash = 1;
        }
        switch ( bat_led_flash ){
            case 1:
                LED1R_OFF();
                LED1G_OFF();
                LED1B_OFF();
                LED2_OFF();
                LED3_OFF();
                LED4_OFF();
                
                LED1G_ON();
                break;
            case 2:
                 LED2_ON();
                 break;
            case 3:
                 LED3_ON();
                 break;
            case 4:
                 LED4_ON();
                 break;
            default:
                 break;
        }
    }
    else if ( bat_flag == BAT_CHARGE_FULL ){
        bat_led_flash = 0;
        LED1R_OFF();
        LED1B_OFF();
        LED1G_ON();
        LED2_ON();
        LED3_ON();
        LED4_ON();
    }
}
#if 1                                                               
void BatTask_Process( void )                                                                                                                        
{                                                   
    uint8_t bat_vol_bk;

    if ( s_spray_state != STATE_SPRAY_READY ){
        return;
    }
    bat_vol_bk = bat_vol;
    ADC_Start( ADC_BAT_CH );
    while( adc_state == ADC_BUSY );
    bat_adc = ADC_Average( ADC_BAT_CH );
    
    if ( bat_adc >= BATT_THR_1 ){           
        bat_vol = BAT_VOL_FULL;             
    }
    else if( (bat_adc < BATT_THR_1) && (bat_adc >= BATT_THR_2) ){
        bat_vol = BAT_VOL_NORMAL;
    }
    else if( (bat_adc < BATT_THR_2) && (bat_adc >= BATT_THR_3) ){
        bat_vol = BAT_VOL_LOW;
    }
    else if( (bat_adc < BATT_THR_3) && (bat_adc >= BATT_THR_4) ){  
        bat_vol = BAT_VOL_BAD;
    }
    else{
        bat_vol = BAT_VOL_OFF;
    }
    if ( bat_vol_bk == bat_vol ){
        bat_cnt = 0;
    }
    else
    {
        if ( (++bat_cnt < 3) && (bat_vol_bk != 0) ){
            bat_vol = bat_vol_bk;
            return;
        }
    }
    bat_cnt = 0;    
    if ( bat_flag != BAT_CHARGE_OFF )
    {
            pApp_Task->task_battery.interval = 1000;		//1S
            return;
     }   
        // All off
        LED1R_OFF();
        LED1G_OFF();
        LED1B_OFF();
        LED2_OFF();
        LED3_OFF();
        LED4_OFF();
        
        if ( bat_vol == BAT_VOL_FULL ){
            LED1G_ON();
            LED2_ON();
            LED3_ON();
            LED4_ON();
        }
        else if ( bat_vol == BAT_VOL_NORMAL ){
            LED1G_ON();
            LED2_ON();
            LED3_ON();
        }
        else if ( bat_vol == BAT_VOL_LOW ){
            LED1G_ON();
            LED2_ON();
        }
        else if ( bat_vol == BAT_VOL_BAD ){
            LED1R_ON();
            LED1G_ON();
        }
        else if ( bat_vol == BAT_VOL_OFF ){
            LED1R_ON();
        }
}

/***********************************************************************************
*  @fn     
*  @brief  LED process task
*  @param      
*            
*  @return   none
*/
void LedTask_Process( void )
{
    uint8_t blink, percent, type;

    blink = (uint8_t)(pApp_Task->task_led.param1);
    percent = (uint8_t)(p_user_info->unused_pct);
    //percent = 100;
    if ( percent >= 75 ){
        type = 1;
    }
    else if ( (percent <= 75) && (percent > 50)){
        type = 2;
    }
    else if ( (percent <= 50) && (percent > 25)){
        type = 3;
    }
    else if ( (percent <= 25) && (percent > 0)){
		type = 4;
	}
    else{
        type = 5;
    }
    if ( pApp_Task->task_led.event & LED_STATE_BLINK_EVT ){
        HalLed_StatusOnOff( 1, HAL_LED_MODE_OFF );
        if ( blink % 2 ){
            if ( ((type == 5) && (blink == 1)) == false ){
                HalLed_StatusOnOff( type, HAL_LED_MODE_ON );
            }
        }
        else{
            HalLed_StatusOnOff( type, HAL_LED_MODE_OFF );
        }
        
        if ( blink ){
            if ( --blink == 0 ){
                pApp_Task->task_led.event ^= LED_STATE_BLINK_EVT;
                pApp_Task->task_led.interval = 0;
                pApp_Task->task_led.param1 = 0;
                pApp_Task->task_led.param2 = 0;
            }
            else{
                pApp_Task->task_led.interval = pApp_Task->task_led.param2;
            }
        }
        pApp_Task->task_led.param1 = (uint16_t)blink;
    }
    else if ( pApp_Task->task_led.event & LED_STATE_UPDATE_EVT ){
        pApp_Task->task_led.event ^= LED_STATE_UPDATE_EVT;
        HalLed_StatusOnOff( 1, HAL_LED_MODE_OFF );
        switch ( type ){
            case 1:
            case 2:
            case 3:
            case 4:
                     HalLed_StatusOnOff( type, HAL_LED_MODE_ON );
                     break;
            case 5:  break;
        }
    }    
}            
void SprayTask_Process( void )                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    
{                                                                             
    static uint8_t pressure_index = 0;
    static uint8_t current_index = 0;
    static uint8_t pressure_inc = 0;
    static uint8_t current_inc = 0;
    const uint8_t current_rate = 25;         // save current every 50 ms
    const uint8_t pressure_rate = 25;        // save pressure every 50 ms
    switch( s_spray_state )                                                                                                             
    {       
        case STATE_SYSTEM_WAIT:
            spray_intv = 1;
            s_spray_state = STATE_SPRAY_READY;
            break;
        case STATE_SYSTEM_CHECK:
            spray_intv = 1;
            s_spray_state = STATE_SPRAY_READY;
            break;
        case STATE_SPRAY_READY:                                                 
            if ( state_spray_ready_init() )
            {
                if ( sdp_thr++ > 0 )
                {   
                    sdp_thr = 0;
                    pApp_Task->task_led.interval = 2;
                    pApp_Task->task_led.event = LED_STATE_BLINK_EVT;   
                    pApp_Task->task_led.param1 = 2;                     
                    pApp_Task->task_led.param2 = 100*2;                // 100*2*1ms = 0.2S
                    pApp_Task->task_power_off.interval = 0;            
                    pApp_Task->task_power_off.event = 0;                
                    s_spray_state = STATE_SPRAY_ACTIVE_DLY;            
                    lt_en_time = 0;                                      
                    app_set_frq( PWM_SCAN_FOSC_INI );                   
                    duty_set_val = PWM_DUTY_MAX;                                   
                    PWM_Duty_Set( duty_set_val );                      
                    uln_level = LEVEL_INI;                             
                    before_level = uln_level;
                    VT_value_set();								       // set output voltage     
                    pwm_set_adc = CURRENT_TARGET;
                    ADI_OE_ON();                					   // Start 183KHZ Transducer work
                    DC8V_EN_ON();
                    DC24V_EN_ON();        
                    cnt_alarm1A = 0;
                    cnt_alarm1B = 0;
                    spray_intv = 2;         
                    adc_error = 0;
                    cnt_delay_ini = 0;                                
                    fg_stop_spray = 0;                                // clear fg_stop_spray
                    spray_ready_time = 0;                             
                    HalSdp_Start_Measure();                           
                }
            }
            else
            {
                if( VBUS_GetValue() == GPIO_PIN_SET )
                {
                    spray_ready_time =0;
                }
                if ( ++spray_ready_time > spray_ready_time_total )
                {
                    s_spray_state = STATE_SPRAY_READY_TIMEOUT;
                    spray_intv = 5;
                }
                else
                {
                    spray_intv = 50*2;                           		//100ms
                }                                                       
            }                                                                   
            break;                                                              
        case STATE_SPRAY_ACTIVE_DLY:                                                                                                                                                                                                                                                                                                                                                
        	spray_intv = 1;                                                                                                                                                                         
        	cnt_delay_ini++;                                                    
        	if(cnt_delay_ini >= SCAN_DLY_1)                                       
        	{                                                                   
                pwm_adc_offset = 0;                   
                pwm_list_offset = 0;                
                pwm_adc_cnt = 0;        
                pwm_adc_max = 0;                                                
                spray_intv = SCAN_DLY_2;                                                
                current_frc = DEVICE_FRC + ((SCAN_CNT1 >> 1) - 5)*FRC_100HZ;    //read device frquency
                max_frc = current_frc;                                                
                app_set_frq( current_frc );                                     //set autotune first frequency
                s_spray_state = STATE_SPRAY_SCAN;                               
                lt_en_time = sys_ticket + SPRAY_ON_TIME;                        
                if( spray_triger == TRIGER_SDP )                          
                {                                                                       
                    lt_en_time = sys_ticket + BREATH_ON_TIME;             	    //breath spray time limited
                }                                                               
        	}                                                                   
        	else
        	{                                      
        		if(cnt_delay_ini >= 10)			//  10ms                                                                 
        		{         
            		ADC_Start( ADC_CUT_CH );
                    while( adc_state == ADC_BUSY );
                    pwm_adc_val = ADC_Average( ADC_CUT_CH );
                    if(cnt_delay_ini == 10)
                    {
                        pwm_freq_adc[pwm_adc_cnt] = pwm_adc_val;
                        pwm_adc_cnt++;
                    }
                    if(pwm_adc_val > 1500)          // GT 1.5A 
                    {
                        adc_error++;
                    }
                    else
                    {
#if CURRENT_PROTECTION_EN
                        if((pwm_adc_val >= CURRENT_FOSC_INI) || (adc_error >= 3 ))
                        {                   
                            ADI_OE_OFF();    
                            DC8V_EN_OFF();   
                            DC24V_EN_OFF();                
                            lt_en_time = 0;                
                            s_spray_state = STATE_SPRAY_ACTIVE;
                            if(fg_stop_spray == 0)
                            {    
                                fg_stop_spray = 3;
                                pApp_Task->task_led.interval = 2;
                                pApp_Task->task_led.event = LED_STATE_BLINK_EVT;
                                pApp_Task->task_led.param1 = 4;
                                pApp_Task->task_led.param2 = 50*2;			//50*2*1ms=100ms
                            }                                   // Initial ALARM VALUE = 1
                        }
                        else
                        {
                            adc_error = 0;
                        }
#endif
                    }
        		}
        	}
            break;                                                      
        case STATE_SPRAY_SCAN                                                                                                                                                                                                                                                                                                                                                               :                                                                        
            spray_intv = SCAN_DLY_2;                                                                                                                                                                                            
            ADC_Start( ADC_CUT_CH );                                    
            while( adc_state == ADC_BUSY );                             
            pwm_adc_val = ADC_Average( ADC_CUT_CH );
            if(pwm_adc_val > 1500)          // GT 1.5A 
            {
                adc_error++;
#if CURRENT_PROTECTION_EN

                if(adc_error >= 5)
                {
                    ADI_OE_OFF();   
                    DC8V_EN_OFF();  
                    DC24V_EN_OFF();  
                    lt_en_time = 0;                
                    s_spray_state = STATE_SPRAY_ACTIVE;
                    if(fg_stop_spray == 0)
                    {    
                        fg_stop_spray = 3;
                        pApp_Task->task_led.interval = 2;
                        pApp_Task->task_led.event = LED_STATE_BLINK_EVT;
                        pApp_Task->task_led.param1 = 4;
                        pApp_Task->task_led.param2 = 50*2;			//50*2*1ms=100ms
                    }   
                }
                break;
#endif
            }
            else
            {
                adc_error = 0;
                pwm_freq_adc[pwm_adc_cnt] = pwm_adc_val;                    
                pwm_adc_cnt++;                                              
#if CURRENT_PROTECTION_EN

                if(pwm_adc_val >= CURRENT_THR1B)                             
                {
                    cnt_alarm1B++;
                    if(pwm_adc_val >= CURRENT_THR1A)
                    {
                        cnt_alarm1A++;
                    }
                    if((cnt_alarm1A >= ALARM1A_COUNT ) || (cnt_alarm1B >= ALARM1B_COUNT))
                    {
                        ADI_OE_OFF();   
                        DC8V_EN_OFF();  
                        DC24V_EN_OFF();  
                        lt_en_time = 0;                
                        flag_led_red++; 
                        s_spray_state = STATE_SPRAY_ACTIVE;
                        if(fg_stop_spray == 0)
                        {    
                            fg_stop_spray = 3;
                            pApp_Task->task_led.interval = 2;
                            pApp_Task->task_led.event = LED_STATE_BLINK_EVT;
                            pApp_Task->task_led.param1 = 4;
                            pApp_Task->task_led.param2 = 50*2;			//50*2*1ms=100ms
                        }                              
                    }
                }
#endif
                if(pwm_adc_val > pwm_adc_max)                                
                {                                              
                    max_frc = current_frc;
                    pwm_adc_max = pwm_adc_val;
                }                                                                                            
                pwm_adc_offset++;                                   
                if ( pwm_adc_offset < SCAN_CNT1 )                            
                {                                                           
                    if (pwm_adc_offset == 1) shot_info.init_freq = current_frc;
                    current_frc -= FRC_100HZ;                            //autotune (as 100HZ)
                    app_set_frq( current_frc );
                }
                else
                {
                    if(pwm_adc_offset == SCAN_CNT1)
                    {
                        current_frc = max_frc + FRC_100HZ + (SCAN_CNT2 >> 1)*FRC_7HZ6; //autotune(as 7.6HZ)
                        //current_frc = max_frc + (SCAN_CNT2 >> 1)*FRC_7HZ6; //autotune(as 7.6HZ)
                        app_set_frq( current_frc );
                        pwm_adc_max = 0;
                    }       
                    else                                                     
                    {                                                          
                        pwm_adc_offset = SCAN_CNT1 + 1;                     
                        pwm_list_offset++;                                  
                        if(pwm_list_offset <= SCAN_CNT2)                     
                        {                                                    
                            current_frc -= FRC_7HZ6;					    //autotune (as 10HZ)
                            app_set_frq( current_frc );                     
                        }                                                      
                        else                                               
                        {                                                               
                            //current_frc = max_frc - FRC_7HZ6;     
                            current_frc = max_frc; 
                            app_set_frq( current_frc );                        
                            bk_frc = current_frc;							// save current_frc
                            bk_adc_max = pwm_adc_max;			            // save max adc value
                            pwm_list_offset = SCAN_CNT2;				    
                            sdp_cnt = 50;									// 0.1S reset
                            spray_intv = 5;								    // delay 10ms-- test first ADC
                            cnt_alarm2 = 0;
                            cnt_alarm3 = 0;
                            cnt_alarm4 = 0;
                            cnt_alarm5 = 0;
                            pwm_adc_sum = 0;
                            cnt_adc_10 = 0;
                            vol_duty_cnt = 0;                               
                            before_pwm_adc = 220*4;
                            uln_offset = 0;   
                            if ( spray_triger == TRIGER_SDP )                  
                            {
                                s_spray_state = STATE_SPRAY_INHALATION;
                            }
                            else
                            {
                                s_spray_state = STATE_SPRAY_ACTIVE;
                            }
                        }
                    }
                }
                break;  
            }
        case STATE_SPRAY_READY_TIMEOUT:
            if ( bat_flag != BAT_CHARGE_OFF )
            {
            	spray_intv = 5;
                spray_ready_time = 0;
                s_spray_state = STATE_SPRAY_READY;
            }
            power_off();
            break;

        case STATE_SPRAY_INHALATION:                                
            spray_intv = 1;                                         //1ms
         	ADC_Start( ADC_CUT_CH );                                                                
        	while( adc_state == ADC_BUSY );                         
        	pwm_adc_val = ADC_Average( ADC_CUT_CH ); 
            if((current_index < CURRENT_SIZE) && (current_inc++ % current_rate == 0))
            {
                shot_info.current[current_index++] = pwm_adc_val;
            } 
            if(pwm_adc_val > 1500)                                  // GT 1.5A 
            {
                adc_error++;
#if CURRENT_PROTECTION_EN
                if(adc_error >= 5)                                  //
                {
                    ADI_OE_OFF();   
                    DC8V_EN_OFF();  
                    DC24V_EN_OFF();  
                    lt_en_time = 0;                 
                    s_spray_state = STATE_SPRAY_ACTIVE;
                    if(fg_stop_spray == 0)
                    {    
                        fg_stop_spray = 3;
                        pApp_Task->task_led.interval = 2;
                        pApp_Task->task_led.event = LED_STATE_BLINK_EVT;
                        pApp_Task->task_led.param1 = 4;
                        pApp_Task->task_led.param2 = 50*2;			//50*2*1ms=100ms
                    }   
                }
                break;
#endif
            }
            else
            {
                adc_error = 0;
                pwm_adc_sum += pwm_adc_val;                             
                cnt_adc_10++;                                           
                if(cnt_adc_10 >= 10)                                                                                                        
                {                                                       
                    cnt_adc_10 = 0;                                 //10ms
                    pwm_adc_evg = pwm_adc_sum/10;                       
                    pwm_adc_sum = 0;                                                                                 
                    pwm_freq_adc[pwm_adc_cnt] = pwm_adc_evg;            
                    if ( pwm_adc_cnt < 400 )                            
                    {                                                   
                        pwm_adc_cnt++;                                               
                    }  
                    if(uln_level == before_level)
                    {
                        uln_offset++;
                    }
                    if((abs(pwm_adc_evg - pwm_set_adc) >= ALARM5_OFFSET) && (uln_offset > 5))
                    {
                        cnt_alarm5++;
                        if(pwm_offset_max < abs(pwm_adc_evg - pwm_set_adc))
                        {
                            pwm_offset_max = abs(pwm_adc_evg - pwm_set_adc);
                        }
                    }
                    else
                    {
                        cnt_alarm5 = 0;
                        pwm_offset_max = 0;
                    }
                    if(uln_level > before_level)                         
                    {
                        if(pwm_adc_evg > pwm_set_adc + (uln_level - before_level)*7 + UP_OFFSET)
                        {
                            cnt_alarm2++;
                        }
                    }
                    else
                    {
                        if((before_level - uln_level) > 3)
                        {
                            if(pwm_adc_evg > pwm_set_adc - (before_level - uln_level)*5 + DOWN_OFFSET1)
                            {
                                cnt_alarm2++;
                            } 
                        }
                        else
                        {
                            if(pwm_adc_evg > pwm_set_adc - (before_level - uln_level)*4 + DOWN_OFFSET2)
                            {
                                cnt_alarm2++;
                            }                        
                        }
                    }
                    if(pwm_adc_evg >= CURRENT_THR2)                                                 
                    {                                               
                        cnt_alarm3++;                               
                    }                                               
                    before_level = uln_level;                                       
                    before_pwm_adc = pwm_adc_evg;   
                    if(pwm_adc_evg >= CURRENT_TARGET + 5)                                                                                                                                               
                    {                                                               
                        cnt_adc_GT = 0;
                        cnt_adc_LT++;
                        if((pwm_adc_evg >= CURRENT_TARGET + 8) || (cnt_adc_LT >= 5))                                                              
                        {                                               
                            cnt_adc_LT = 0; 
                            curt_adc_offset1 = (pwm_adc_evg - CURRENT_TARGET)/CURT_L_OFFSET_STEP; //Sub Voltage Quickly
                            if(curt_adc_offset1 < 1)
                            {
                                curt_adc_offset1 = 1;
                            }
                            if(uln_level != 0)                                      
                            {                                                       
                                cnt_alarm4 = 0;                                                     
                                if(uln_level >= curt_adc_offset1)                   
                                {
                                    uln_level = uln_level - curt_adc_offset1;
                                }
                                else
                                {
                                    uln_level = 0;
                                }
                                VT_value_set();    
                            }
                            else
                            {   
                                if(duty_set_val <= PWM_DUTY_MIN)
                                {
                                    duty_set_val = duty_set_val + curt_adc_offset1;
                                    if(duty_set_val >= PWM_DUTY_MIN)
                                    {
                                        duty_set_val = PWM_DUTY_MIN;
                                    }
                                    PWM_Duty_Set(duty_set_val);
                                    cnt_alarm4 = 0;
                                }       
                                else    
                                {       
                                    if((pwm_adc_evg >= OVER_CURRENT) || (pwm_adc_evg >= CURRENT_TARGET + 50))
                                    {          
                                        cnt_alarm4++;
                                    }
                                    else
                                    {
                                        cnt_alarm4 = 0;
                                    }
                                }
                            }
                        }
                    }
                    else    
                    {       
                        if(pwm_adc_evg <= CURRENT_TARGET - 3)  
                        {   
                            cnt_adc_LT = 0;
                            cnt_adc_GT++;
                            if((pwm_adc_evg <= CURRENT_TARGET - 5) || (cnt_adc_GT >= 5))
                            {
                                cnt_adc_GT = 0;
                                pwm_set_adc = CURRENT_TARGET;
                                curt_adc_offset1 = (pwm_set_adc - pwm_adc_evg)/CURT_H_OFFSET_STEP;    //Add Voltage Slowly
                                if(curt_adc_offset1 < 1)
                                {
                                    curt_adc_offset1 = 1;
                                }
                                if(uln_level < 127)
                                {   
                                    uln_level = uln_level + curt_adc_offset1;
                                    if(uln_level >= 127)
                                    {
                                        uln_level = 127;
                                    }
                                    VT_value_set();
                                }
                                else
                                {
                                    if(duty_set_val > PWM_DUTY_MAX)
                                    {   
                                        if(duty_set_val  < curt_adc_offset1)
                                        {       
                                            duty_set_val = PWM_DUTY_MAX;
                                        }   
                                        else
                                        {
                                            duty_set_val = duty_set_val - curt_adc_offset1;
                                        }
                                        PWM_Duty_Set(duty_set_val);
                                    }
                                }  
                            }
                        }
                    }
                    uln_vol[vol_duty_cnt] = uln_level;
                    pwm_duty[vol_duty_cnt] = duty_set_val;
                    if(vol_duty_cnt < 300)
                    {
                        vol_duty_cnt++;
                    }  
#if CURRENT_PROTECTION_EN
                    if((cnt_alarm2 >= ALARM2_COUNT)||(cnt_alarm3 >= ALARM3_COUNT)||(cnt_alarm4 >= ALARM4_COUNT)||(cnt_alarm5 >= ALARM5_COUNT))                                                                                             
                    {                                                               
                        ADI_OE_OFF();
                        DC8V_EN_OFF();
                        DC24V_EN_OFF();
                        lt_en_time = 0;
                        s_spray_state = STATE_SPRAY_ACTIVE;
                        if(fg_stop_spray == 0)
                        {
                            fg_stop_spray = 3;
                            pApp_Task->task_led.interval = 2;
                            pApp_Task->task_led.event = LED_STATE_BLINK_EVT;
                            pApp_Task->task_led.param1 = 4;
                            pApp_Task->task_led.param2 = 50*2;			//50*2*1ms=100ms
                        }
                    }                                     
                
#endif
                }    
            }
            sdp_cnt++;                                      
            if(sdp_cnt >= PRES_SAMPLE_VAL)					// pressure sensor sampling interval 0.01S
            {                           
                if (pressure_index < PRESSURE_SIZE && pressure_inc++ % pressure_rate == 0)
                {
                   shot_info.pressure[pressure_index++] = sdp_value;
                }
            	sdp_cnt = 0;                                
            	sdp_value = 0;
            	HalSdp_GetValue( &sdp_value, SDP_STOP_THR );
            	if( sdp_value > sdp_value_max){
            		sdp_value_max = sdp_value;
            	}
            	if( sdp_value < sdp_value_max*SDP_STOP_PERCENT/100 )
            	{
                    if(++sdp_thr > 0)
                    {
                    	sdp_thr = 0;
                        ADI_OE_OFF();
                       	DC8V_EN_OFF();
                       	DC24V_EN_OFF();
                        lt_en_time = 0;
                        s_spray_state = STATE_SPRAY_ACTIVE;
                        if(fg_stop_spray == 0)
                        {
                        	fg_stop_spray = 3;
                            pApp_Task->task_led.interval = 2;
                            pApp_Task->task_led.event = LED_STATE_BLINK_EVT;
                            pApp_Task->task_led.param1 = 4;
                            pApp_Task->task_led.param2 = 50*2;                          // 0.1S
                        }
                    }
            	}                                                   
            }
            break;                                                  
                                                                    
        case STATE_SPRAY_ACTIVE:                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              
                                                                    
       	spray_intv = 1;                                             //1ms  
       	if(lt_en_time != 0)                                                                                                                                                                                                                                                                    
       	{                                                                    
         	ADC_Start( ADC_CUT_CH );                                                                
        	while( adc_state == ADC_BUSY );                         
        	pwm_adc_val = ADC_Average( ADC_CUT_CH ); 
            if((current_index < CURRENT_SIZE) && (current_inc++ % current_rate == 0))
            {
                shot_info.current[current_index++] = pwm_adc_val;
            } 
            if(pwm_adc_val > 1500)                                  // GT 1.5A 
            {
                adc_error++;
#if CURRENT_PROTECTION_EN
                if(adc_error >= 5)                                  //
                {
                    ADI_OE_OFF();   
                    DC8V_EN_OFF();  
                    DC24V_EN_OFF();  
                    lt_en_time = 0;                 
                    s_spray_state = STATE_SPRAY_ACTIVE;
                    if(fg_stop_spray == 0)
                    {    
                        fg_stop_spray = 3;
                        pApp_Task->task_led.interval = 2;
                        pApp_Task->task_led.event = LED_STATE_BLINK_EVT;
                        pApp_Task->task_led.param1 = 4;
                        pApp_Task->task_led.param2 = 50*2;			//50*2*1ms=100ms
                    }   
                }
                break;
#endif
            }
            else
            {
                adc_error = 0;
                pwm_adc_sum += pwm_adc_val;                             
                cnt_adc_10++;                                           
                if(cnt_adc_10 >= 10)                                                                                                        
                {                                                       
                    cnt_adc_10 = 0;                                 //10ms
                    pwm_adc_evg = pwm_adc_sum/10;                       
                    pwm_adc_sum = 0;                                                                                 
                    pwm_freq_adc[pwm_adc_cnt] = pwm_adc_evg;            
                    if ( pwm_adc_cnt < 400 )                            
                    {                                                   
                        pwm_adc_cnt++;                                               
                    }  
                    if(uln_level == before_level)
                    {
                        uln_offset++;
                    }
                    if((abs(pwm_adc_evg - pwm_set_adc) >= ALARM5_OFFSET) && (uln_offset > 5))
                    {
                        cnt_alarm5++;
                        if(pwm_offset_max < abs(pwm_adc_evg - pwm_set_adc))
                        {
                            pwm_offset_max = abs(pwm_adc_evg - pwm_set_adc);
                        }
                    }
                    else
                    {
                        cnt_alarm5 = 0;
                        pwm_offset_max = 0;
                    }
                    if(uln_level > before_level)                         
                    {
                        if(pwm_adc_evg > pwm_set_adc + (uln_level - before_level)*7 + UP_OFFSET)
                        {
                            cnt_alarm2++;
                        }
                    }
                    else
                    {
                        if((before_level - uln_level) > 3)
                        {
                            if(pwm_adc_evg > pwm_set_adc - (before_level - uln_level)*5 + DOWN_OFFSET1)
                            {
                                cnt_alarm2++;
                            } 
                        }
                        else
                        {
                            if(pwm_adc_evg > pwm_set_adc - (before_level - uln_level)*4 + DOWN_OFFSET2)
                            {
                                cnt_alarm2++;
                            }                        
                        }
                    }
                    if(pwm_adc_evg >= CURRENT_THR2)                                                 
                    {                                               
                        cnt_alarm3++;                               
                    }                                               
                    before_level = uln_level;                                       
                    before_pwm_adc = pwm_adc_evg;   
                    if(pwm_adc_evg >= CURRENT_TARGET + 5)                                                                                                                                               
                    {                                                               
                        cnt_adc_GT = 0;
                        cnt_adc_LT++;
                        if((pwm_adc_evg >= CURRENT_TARGET + 8) || (cnt_adc_LT >= 5))                                                              
                        {                                               
                            cnt_adc_LT = 0; 
                            curt_adc_offset1 = (pwm_adc_evg - CURRENT_TARGET)/CURT_L_OFFSET_STEP; //Sub Voltage Quickly
                            if(curt_adc_offset1 < 1)
                            {
                                curt_adc_offset1 = 1;
                            }
                            if(uln_level != 0)                                      
                            {                                                       
                                cnt_alarm4 = 0;                                                     
                                if(uln_level >= curt_adc_offset1)                   
                                {
                                    uln_level = uln_level - curt_adc_offset1;
                                }
                                else
                                {
                                    uln_level = 0;
                                }
                                VT_value_set();    
                            }
                            else
                            {   
                                if(duty_set_val <= PWM_DUTY_MIN)
                                {
                                    duty_set_val = duty_set_val + curt_adc_offset1;
                                    if(duty_set_val >= PWM_DUTY_MIN)
                                    {
                                        duty_set_val = PWM_DUTY_MIN;
                                    }
                                    PWM_Duty_Set(duty_set_val);
                                    cnt_alarm4 = 0;
                                }       
                                else    
                                {       
                                    if((pwm_adc_evg >= OVER_CURRENT) || (pwm_adc_evg >= CURRENT_TARGET + 50))
                                    {          
                                        cnt_alarm4++;
                                    }
                                    else
                                    {
                                        cnt_alarm4 = 0;
                                    }
                                }
                            }
                        }
                    }
                    else    
                    {       
                        if(pwm_adc_evg <= CURRENT_TARGET - 3)  
                        {   
                            cnt_adc_LT = 0;
                            cnt_adc_GT++;
                            if((pwm_adc_evg <= CURRENT_TARGET - 5) || (cnt_adc_GT >= 5))
                            {
                                cnt_adc_GT = 0;
                                pwm_set_adc = CURRENT_TARGET;
                                curt_adc_offset1 = (pwm_set_adc - pwm_adc_evg)/CURT_H_OFFSET_STEP;    //Add Voltage Slowly
                                if(curt_adc_offset1 < 1)
                                {
                                    curt_adc_offset1 = 1;
                                }
                                if(uln_level < 127)
                                {   
                                    uln_level = uln_level + curt_adc_offset1;
                                    if(uln_level >= 127)
                                    {
                                        uln_level = 127;
                                    }
                                    VT_value_set();
                                }
                                else
                                {
                                    if(duty_set_val > PWM_DUTY_MAX)
                                    {   
                                        if(duty_set_val  < curt_adc_offset1)
                                        {       
                                            duty_set_val = PWM_DUTY_MAX;
                                        }   
                                        else
                                        {
                                            duty_set_val = duty_set_val - curt_adc_offset1;
                                        }
                                        PWM_Duty_Set(duty_set_val);
                                    }
                                }  
                            }
                        }
                    }
                    uln_vol[vol_duty_cnt] = uln_level;
                    pwm_duty[vol_duty_cnt] = duty_set_val;
                    if(vol_duty_cnt < 300)
                    {
                        vol_duty_cnt++;
                    }  
#if CURRENT_PROTECTION_EN
                    if((cnt_alarm2 >= ALARM2_COUNT)||(cnt_alarm3 >= ALARM3_COUNT)||(cnt_alarm4 >= ALARM4_COUNT)||(cnt_alarm5 >= ALARM5_COUNT))                                                                                             
                    {                                                               
                        ADI_OE_OFF();
                        DC8V_EN_OFF();
                        DC24V_EN_OFF();
                        lt_en_time = 0;
                        s_spray_state = STATE_SPRAY_ACTIVE;
                        if(fg_stop_spray == 0)
                        {
                            fg_stop_spray = 3;
                            pApp_Task->task_led.interval = 2;
                            pApp_Task->task_led.event = LED_STATE_BLINK_EVT;
                            pApp_Task->task_led.param1 = 4;
                            pApp_Task->task_led.param2 = 50*2;			//50*2*1ms=100ms
                        }
                    }       
#endif
                }    
            }
       	}
       	else
        {
       		s_spray_state = STATE_DOSE_COMPLETE;
       		spray_intv = RE_TRIGER_TIME;
            ADI_OE_OFF();
            DC8V_EN_OFF();
            DC24V_EN_OFF();
       	}
        break;
             
        case STATE_DOSE_COMPLETE:
             
            HalSdp_Stop_Measure();
            if (!is_mem_full()) save_shot();
            spray_triger = 0;
            spray_intv = 50;
            s_spray_state = STATE_SPRAY_READY;
            spray_ready_time = 0;
            pApp_Task->task_led.interval = 10;
            pApp_Task->task_led.event |= LED_STATE_UPDATE_EVT;
            pressure_index = 0;
            current_index = 0;
            pressure_inc = 0;
            current_inc = 0;
            break;
             
        default:
        	break;
    }
}


static uint8_t state_spray_ready_init( void )
{
    uint16_t value;
    uint8_t rst = false;
    
    if ( bat_vol <= BAT_VOL_OFF ){
        return rst;
    }

#if PRESSURE_EN
    value = HalSdp_GetThr();
    if ( value > SDP_TRIGER_THR ){
        spray_triger = TRIGER_SDP;
        rst = true;
        sdp_value_max = value;
    }
#endif

    //if ( sw_status == KEY_SPRESS ){
    if ( spray_key_en ){
        spray_key_en = false;
        sdp_thr = 2;
        spray_triger = TRIGER_KEY;
        rst = true;
    }
    
    return rst;
}

void save_shot(){
 	shot_info.shot_num++;
    shot_info.used_freq = current_frc;
 	shot_info.batt_lvl = bat_adc; 
    shot_info.sys_ticket = sys_ticket;
    shot_info.spray_time = spray_time;
 	for (uint8_t i = 0; i < AUTOTUNE_SIZE; i++){
        if (i < sizeof(pwm_freq_adc) / sizeof(pwm_freq_adc[0]))
            shot_info.autotune[i] = pwm_freq_adc[i];
 	}
	write_shot(&shot_info);
 
    spray_time = 0;
    shot_info.spray_time = 0;
    memset(&shot_info.pressure, 0, sizeof(shot_info.pressure));
    memset(&shot_info.current, 0, sizeof(shot_info.current));
    memset(&shot_info.autotune, 0, sizeof(shot_info.autotune));
}

static void CartTask_Process( void )
{
    
#if 1        // Test,
    p_user_info->unused_pct = UNUSED_CNT;
    return;
#endif
    
#if 0
    
    cartidge_attr_t *state_bk = p_bt_state;
    
    if ( bat_flag != BAT_CHARGE_OFF ){
		return;
	}

    if ( pApp_Task->task_cartridge.event & CART_DETECT_EVT ){
        if ( sha_detect() ){
            cart_intv = 250;
            if ( state_bk == NULL ){
                Hal_I2C_Select( SHA204A );
                if ( ATCA_SUCCESS == atcab_read_all((uint8_t *)p_cart_attr, 3) ){
                    p_bt_state = p_cart_attr;
                    p_user_info->total_cnt = p_cart_attr->total_eje_dose;
                    p_user_info->used_cnt = p_cart_attr->used_eje_dose;
                    p_user_info->total_time = p_cart_attr->total_eje_time;
                    p_user_info->used_time = p_cart_attr->used_eje_time;
                    p_user_info->per_sdp_time = p_cart_attr->per_sdp_time;
                    p_user_info->per_key_time = p_cart_attr->per_key_time;
                    p_user_info->fill_time = p_cart_attr->fill_time;
                    p_user_info->expire_time = p_cart_attr->expire_time;
                    if ( p_user_info->used_time < p_user_info->total_time ){
						p_user_info->unused_pct = 100 - p_user_info->used_time * 100 / p_user_info->total_time;
					}
					else{
						p_user_info->unused_pct = 0;
					}
#if TEST
                    p_user_info->unused_pct = UNUSED_CNT;
#endif
                }
                else{
                    p_bt_state = NULL;
                }
                Hal_I2C_Release( SHA204A );
            }
        }
        else {
            p_bt_state = NULL;
            cart_intv = 150;
        }

        if ( state_bk != p_bt_state ){
            pApp_Task->task_led.interval = 10;
            pApp_Task->task_led.event |= LED_STATE_UPDATE_EVT;
        }
    }
    if ( pApp_Task->task_cartridge.event & CART_WRITE2_EVT ){
        //if ( (p_bt_state != NULL) && (p_user_info->used_time < p_user_info->total_time) ){   // 会导致最�?�一次喷雾时间无法写入芯片
        if ( p_bt_state != NULL ){
            if ( ATCA_SUCCESS == sha_update_slot2( p_user_info->used_cnt, p_user_info->used_time ) ){
                pApp_Task->task_cartridge.event ^= CART_WRITE2_EVT;
            }
            else {
                cart_intv = 150;
            }
        }
        else {
            pApp_Task->task_cartridge.event ^= CART_WRITE2_EVT;
        }
    }
#endif
}

void cart_Detect( void )
{
#if 0    // Test
    p_bt_state = p_cart_attr;
	p_user_info->total_cnt = p_cart_attr->total_eje_dose = 200;
	p_user_info->used_cnt = p_cart_attr->used_eje_dose = 0;
	p_user_info->total_time = p_cart_attr->total_eje_time = 3000000;
	p_user_info->used_time = p_cart_attr->used_eje_time = 0;
	p_user_info->per_sdp_time = p_cart_attr->per_sdp_time = 2500;
	p_user_info->per_key_time = p_cart_attr->per_key_time = 1500;
	p_user_info->fill_time = p_cart_attr->fill_time = 0;
	p_user_info->expire_time = p_cart_attr->expire_time = 1550735087;
	if ( p_user_info->used_time < p_user_info->total_time ){
		p_user_info->unused_pct = 100 - p_user_info->used_time * 100 / p_user_info->total_time;
	}
	else{
		p_user_info->unused_pct = 0;
	}
    return;
#endif

#if 0
    p_bt_state = NULL;

    if ( sha_detect() == false ){
            return;
    }
    else{
        Hal_I2C_Select( SHA204A );
        if ( ATCA_SUCCESS == atcab_read_all((uint8_t *)p_cart_attr, 3) ){
            p_bt_state = p_cart_attr;
            //pApp_Task->task_cartridge.event ^= CART_READ_EVT;
            p_user_info->total_cnt = p_cart_attr->total_eje_dose;
            p_user_info->used_cnt = p_cart_attr->used_eje_dose;
            p_user_info->total_time = p_cart_attr->total_eje_time;
            p_user_info->used_time = p_cart_attr->used_eje_time;
            p_user_info->per_sdp_time = p_cart_attr->per_sdp_time;
            p_user_info->per_key_time = p_cart_attr->per_key_time;
            p_user_info->fill_time = p_cart_attr->fill_time;
            p_user_info->expire_time = p_cart_attr->expire_time;
            if ( p_user_info->used_time < p_user_info->total_time ){
            	p_user_info->unused_pct = 100 - p_user_info->used_time * 100 / p_user_info->total_time;
            }
            else{
            	p_user_info->unused_pct = 0;
            }
#if TEST
            p_user_info->unused_pct = UNUSED_CNT;
#endif
        }
        Hal_I2C_Release( SHA204A );
    }
#endif
}

static void power_on( uint8_t type )
{
    if ( type == WAKEUP_RESET ){
        //VBAT_CN_OFF();
        VBAT_CN_ON();
    }
    else{
        VBAT_CN_ON();
    }
}

static void power_off( void )
{    
    
    if ( bat_flag != BAT_CHARGE_OFF ){
        return;
    }
    HalLed_AllOnOff( HAL_LED_MODE_OFF );
    VBAT_CN_OFF();
    while(1);
}

void uart_rx_process( void )
{
#if 0
	uint8_t i;

	if ( (aRxBuffer[0] == 0xFF) && (aRxBuffer[1] == 0x01) &&  \
		 (aRxBuffer[2] == 0x04) && (aRxBuffer[7] == 0xAB) ){

		for ( i = 0; i < 60; i++ ){
			aTxBuffer[i] = 0;
			aRxBuffer[i] = 0;
		}
		aTxBuffer[0] = 0xFD;  aTxBuffer[1] = 0x02;  aTxBuffer[2] = 0x38;
		aTxBuffer[3] = (uint8_t)(((p_user_info->total_cnt) - (p_user_info->used_cnt)) >> 8);
		aTxBuffer[4] = (uint8_t)((p_user_info->total_cnt) - (p_user_info->used_cnt));
		aTxBuffer[59] = 0xAB;
	}
#else
    if ( uart2RxCount > 0 ){
        if ( uart_rx_delay == false ){
            uart_rx_delay = true;
            pApp_Task->task_uart.interval = 50;
            return;
        }
        uart_rx_delay = false;
//---        uart_rx_cnt = EUSART_Get( (uint8_t *)aRxBuffer, uart2RxCount );
        memcpy( aTxBuffer, aRxBuffer, uart_rx_cnt );
//---        EUSART_Send( (uint8_t *)aTxBuffer, uart_rx_cnt );
    }
#endif
}

#if 0
uint8_t uartTask_Process( void )
{
	HAL_StatusTypeDef rslt = HAL_OK;

	if ( pApp_Task->task_uart.event == UART_RX_EVT ){
		if(HAL_UART_Receive_IT(&Uart1Handle, aRxBuffer, 8) != HAL_OK){
			rslt = HAL_ERROR;
		}
		else{
			// Reset transmission eventFlag
			UartReady = RESET;
			pApp_Task->task_uart.event ^= UART_RX_EVT;
			pApp_Task->task_uart.event |= UART_RXW_EVT;
		}
	}
	else if ( pApp_Task->task_uart.event == UART_RXW_EVT ){
		if ( UartReady != SET ){
			rslt = HAL_BUSY;
		}
		else {
			UartReady = RESET;
			pApp_Task->task_uart.event ^= UART_RXW_EVT;
			// process the received data
            uart_rx_process();
            pApp_Task->task_uart.event |= UART_TX_EVT;
		}
	}
	else if ( pApp_Task->task_uart.event == UART_TX_EVT ){
		if( HAL_UART_Transmit_IT(&Uart1Handle, (uint8_t*)aTxBuffer, 60)!= HAL_OK ){
			rslt = HAL_ERROR;
		}
		else{
			// Reset transmission eventFlag
			UartReady = RESET;
			pApp_Task->task_uart.event ^= UART_TX_EVT;
			pApp_Task->task_uart.event |= UART_TXW_EVT;
		}
	}
	else if ( pApp_Task->task_uart.event == UART_TXW_EVT ){
		if ( UartReady != SET ){
			rslt = HAL_BUSY;
		}
		else {
			UartReady = RESET;
			pApp_Task->task_uart.event ^= UART_TXW_EVT;
			pApp_Task->task_uart.event |= UART_RX_EVT;
		}
	}

	return rslt;
}
#endif

#endif   // for debug

#endif
/***********************************************************************************
*  @fn     
*  @brief  Update system ticket every for all tasks
*  @param      
*            
*  @return   none
*/
void task_ticket_update( void )
{
    //uint32_t time;
    uint8_t eventFlag = false;
    
    sys_ticket++;                               
    wdt_ticket = 0;                             
    if ( lt_en_time )                                           
    {              
        spray_time++;
        if ( sys_ticket >= lt_en_time )            
        {
            ADI_OE_OFF();
            DC8V_EN_OFF();
            DC24V_EN_OFF();
            if ( fg_stop_spray == 0 )
            {
                fg_stop_spray = 3;
                pApp_Task->task_led.interval = 2;
                pApp_Task->task_led.event = LED_STATE_BLINK_EVT;
                pApp_Task->task_led.param1 = 4;
                pApp_Task->task_led.param2 = 50*2;
            }
            lt_en_time = 0;
        }
     }
    if ( pApp_Task->task_charge.interval > 0 ){
    	if ( --pApp_Task->task_charge.interval == 0 ){
    		pApp_Task->task_charge.event |=  TASK_ACTIVE_EVT;
    		eventFlag = true;
    	}
    }

    if ( pApp_Task->task_battery.interval > 0 ){
    	if ( --pApp_Task->task_battery.interval == 0 ){
    		pApp_Task->task_battery.event |=  TASK_ACTIVE_EVT;
    		eventFlag = true;
    	}
    }
        
    if ( pApp_Task->task_key.interval > 0 ){
        if ( --pApp_Task->task_key.interval == 0 ){
        	pApp_Task->task_key.event |=  TASK_ACTIVE_EVT;
        	eventFlag = true;
        	pApp_Task->task_key.interval = 20;					//20MS
			if ( lt_en_time == 0 ) {
				if ( key_value == KEY_RLS ){
					key_value = halKeyScan();
				}
			}
        }
    }
    if ( pApp_Task->task_led.interval > 0 ){
		if ( --pApp_Task->task_led.interval ==0 ){
			pApp_Task->task_led.event |=  TASK_ACTIVE_EVT;
			eventFlag = true;
		}
	}
    if ( pApp_Task->task_spray.interval > 0 ){
        if ( --pApp_Task->task_spray.interval ==0 ){
            pApp_Task->task_spray.event |=  TASK_ACTIVE_EVT;
            eventFlag = true;
        }
    }
    if ( pApp_Task->task_wr_flash.interval > 0 ){
        if ( --pApp_Task->task_wr_flash.interval == 0 ){
            pApp_Task->task_wr_flash.event |= TASK_ACTIVE_EVT;
            eventFlag = true;
        }
    }

    if ( pApp_Task->task_adc.interval > 0 ){
        if ( --pApp_Task->task_adc.interval == 0 ){
            pApp_Task->task_adc.event |= TASK_ACTIVE_EVT;
            eventFlag = true;
        }
    }
    
    if ( pApp_Task->task_power_off.interval > 0 ){
        if ( --pApp_Task->task_power_off.interval == 0 ){
            pApp_Task->task_power_off.event |= TASK_ACTIVE_EVT;
            eventFlag = true;
        }
    }

    if ( pApp_Task->task_cartridge.interval > 0 ){
            if ( --pApp_Task->task_cartridge.interval == 0 ){
                pApp_Task->task_cartridge.event |= TASK_ACTIVE_EVT;
                eventFlag = true;
            }
        }

	if ( pApp_Task->task_sdp3x.interval > 0 ){
		if ( --pApp_Task->task_sdp3x.interval == 0 ){
			pApp_Task->task_sdp3x.event |= TASK_ACTIVE_EVT;
			eventFlag = true;
		}
	}
#if 0
    if ( pApp_Task->task_uart.interval > 0 ){
		if ( --pApp_Task->task_uart.interval == 0 ){
			pApp_Task->task_uart.event |= TASK_ACTIVE_EVT;
			eventFlag = true;
		}
	}
#endif
    
}
