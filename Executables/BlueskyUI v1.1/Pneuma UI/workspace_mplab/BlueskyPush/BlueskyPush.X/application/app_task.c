    /***********************************************************************************
  Filename:     task.c

  Description:
***********************************************************************************/
/******************************************************
 * MCU: PIC16F18877,UQFN40 PCB Version: BlueSky AD9833 REVA 20230516 
 * change data: 2023-06-12 
AMOUNT_OF_SHOTS
********************************************************/

/***********************************************************************************
* INCLUDES
*/
#include "string.h"
#include "stdlib.h"
#include "../mcc_generated_files/tmr1.h"
#include "../mcc_generated_files/adcc.h"
#include "hal_board_cfg.h"
#include "hal_i2c.h"
#include "app_task.h"
#include "hal_flash.h"
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

typedef struct{
    uint16_t timeA;
    uint16_t timeB;
    uint8_t cnt;
}bt_pwm_t;
#pragma pack()
/***********************************************************************************
 * CONSTANTS
*/
#define LED_METHOD              1
/* Macro for frequency conversion and autotune implementation */
#define FREQ_TO_NCO(freq)       freq * 1048576 / 16000000 * 2
//#define DEVICE_FRC         FRC_180KHZ + 43*FRC_100HZ  // current device tranducer frequency

// MCLK = 4MHZ                            
#define SCAN_CNT1               36        // SCAN FREQUENCY QTY1 
#define SCAN_CNT2               14        // SCAN FREQUENCY QTY2
#define FRC_10KHZ               1311      
#define FRC_1KHZ                131        
#define FRC_100HZ               13        // 100HZ(AUTO FRC)  
#define FRC_7HZ6                1         // 7.63HZ (AUTO FRC)  
                                           
#define PWM_SCAN_FOSC_INI       39325      //300.00KHZ
#define FRC_170KHZ              22282      //170.00KHZ  //570A
#define FRC_180KHZ              23593 - 2*FRC_1KHZ     //180.00KHZ  //frequency compensation
#define FRC_190KHZ              24904      //190.00KHZ  // 

#define TRIGER_OFF_STATE        0x01
#define KEY_TRIGER_ON           0x02
#define KEY_TRIGER_OFF          0x03

#define LCD_FLASH_INTV          310

// Work flags
#define DIS_CART_SCAN           0x01      // Disable cartridge scan function
#define ALM_CART_EMP
     
#define SCAN_DLY_1              20       	     // 100*1ms=100ms ,delay 100ms --scan frequency
#define SCAN_DLY_2              1         		 // 1ms
#define SDP_STOP_PERCENT        25        		 // stop spray pressure value percent = pressure_value_max * 25%

#define TEST                    0
#define UNUSED_CNT              100

#define PRES_SAMPLE_VAL         1		         // pressure sensor sampling interval = 10*1ms

/***********************************************************************************
 * MACROS
 */
/***********************************************************************************
* GLOBAL VARIABLES
*/

app_task_t *pApp_Task = &app_Task;
uint16_t spray_intv = 500;
uint16_t cart_intv = 100;
uint8_t led_AllOff = true;
uint16_t lcd_disp = 0;
uint16_t lcd_disp_backup = 0;
uint8_t key_value = KEY_RLS;
uint16_t key_cnt = 0;
uint8_t key_state_backup = KEY_RLS;
uint8_t shot_mem_full;
info_shot_t shot_info;
uint16_t curPress = 0;
uint16_t pressBuf[200];
uint32_t spray_time;
uint8_t sdp_thr = 0;
uint16_t avg_current = 0;
uint16_t queued_shots = 0;

/***********************************************************************************
* LOCAL VARIABLES
*/
volatile spray_control_state_t s_spray_state = STATE_SYSTEM_WAIT;
volatile uint8_t sys_ticket_bool = false;
volatile uint8_t sys_ticket_fg1 = false;
volatile uint32_t sys_ticket = 0;
volatile uint32_t lt_en_time = 0;
volatile uint8_t wdt_ticket = 0;
volatile uint16_t spray_ready_time;
volatile uint16_t spray_ready_time_total;
volatile uint8_t wdt_wakeup_time;
volatile uint8_t wakeup_type = WAKEUP_RESET;
volatile uint8_t alarm_repeat = 0;
static uint8_t spray_triger = 0;
static uint8_t spray_key_en = false;
static uint8_t flag_led_red = 0;
static uint24_t max_frc;
static uint32_t last_freq_used;

static uint16_t autotuneCount=0;
static bool autotuneFlag=0;                                 //if 0 runs initial autotune, if 1 runs repeat autotune
static bool fineAutoFlag=0;
static uint32_t bk_frc;
static uint8_t blink_shot_flag=0;
static uint8_t chrg_flag = 1;

static uint16_t pwm_adc_max;
static uint16_t bk_adc_max;
static uint16_t pwm_adc_val;
static uint8_t  duty_set_val = 0;

static uint8_t sdp_cnt = 0;
static uint8_t currentIndex = 0;
uint16_t pwm_freq_adc[120];
static uint8_t pwm_adc_cnt;

static uint8_t pwm_list_offset = 0;
static uint8_t pwm_adc_offset = 0;

static uint32_t sdp_value_max;            //????? ?????
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

static uint8_t  uln_level = BRIDGE_VT;			// VT_LEVEL SET VALUE
static uint8_t  before_level = 0;

static uint8_t  cnt_alarm1 = 0;
static uint8_t  cnt_alarm2 = 0;
static uint8_t  cnt_alarm3 = 0;

static uint8_t cnt_flash = 0;
static uint8_t flash_cnt = 0;
static uint8_t cnt_3S = 20;

static uint16_t cnt_delay_ini = 0;		// spray initiale preheating delay time
static uint8_t cnt_scan_1ms = 0;		// 1ms count time
static uint8_t eventActive = false;

/***********************************************************************************
* EXTERNAL VARIABLES
*/
extern volatile uint8_t adc_state;

//extern uint8_t aTxBuffer[];
//extern uint8_t aRxBuffer[];
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

// uint8_t uartTask_Process( void );

void app_task_Init( void );
void VT_value_set(void);
void save_shot(void);

uint8_t adc_cart_check(void);
uint8_t adc_bat_check(void);                //takes adc measuremnt of battery and returns a 
uint24_t FreqConversion(uint32_t freq);

void led_bat_update(void);              //updates LEDs based on battery measurement
void blink_setup(uint8_t param1,uint8_t param2); //blink the leds
void app_set_frq( uint32_t frq );

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
        bat_vol_bk = adc_bat_check();
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
        led_bat_update();
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
#if LED_METHOD
    pApp_Task->task_led.interval = 75;
#else
    pApp_Task->task_led.interval = LED_PWMH;
#endif
    pApp_Task->task_spray.interval = 20;
    pApp_Task->task_wr_flash.interval = 2000;
    pApp_Task->task_uart.interval = 1000;
    
    pApp_Task->task_battery.event = TASK_ACTIVE_EVT;
    pApp_Task->task_led.event = TASK_ACTIVE_EVT | LED_STATE_UPDATE_EVT;
    pApp_Task->task_key.event = TASK_IDLE_EVT;
    pApp_Task->task_spray.event = TASK_IDLE_EVT;
    pApp_Task->task_wr_flash.event = TASK_IDLE_EVT;
    pApp_Task->task_uart.event = UART_RX_EVT;
    led_state.led_array = 0xff;
    led_state.led1_color = 0;
    wakeup_type |= WAKEUP_ON_KEY;
    spray_ready_time_total = IDLE_TIMEOUT;
    
    s_spray_state = STATE_SPRAY_READY;
    sw_status = KEY_POWER_ON;     // KEY_SPRESS;

    ///cart_Detect();
    HalSdp_Init();
    ADI_OE_OFF();
    HalLed_Init();
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
    if(pApp_Task->task_ticket.event & TASK_ACTIVE_EVT){
        pApp_Task->task_ticket.event ^= TASK_ACTIVE_EVT;
        task_ticket_update();
    }

    if ( pApp_Task->task_wr_flash.event & TASK_ACTIVE_EVT ){
        pApp_Task->task_wr_flash.event ^= TASK_ACTIVE_EVT;
        save_shot();
    }
	if ( pApp_Task->task_charge.event & TASK_ACTIVE_EVT ){
             pApp_Task->task_charge.event ^= TASK_ACTIVE_EVT;
             pApp_Task->task_charge.interval = 75*2;				//0.15S
             ChargeTask_Process();
        #if WTD_ENABLE
        // Refresh IWDG: reload counter
        if(HAL_IWDG_Refresh(&IwdgHandle) != HAL_OK){
	    //Error_Handler();
        }
        #endif
	}
	if ( pApp_Task->task_battery.event & TASK_ACTIVE_EVT ){
		pApp_Task->task_battery.event ^= TASK_ACTIVE_EVT;
        if ( VBUS_GetValue() == GPIO_PIN_SET ){
//            pApp_Task->task_battery.interval = 100;
            pApp_Task->task_battery.interval = 1000;				//1S
            BatTask_Process();
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
	}

	if ( pApp_Task->task_spray.event & TASK_ACTIVE_EVT ){
		pApp_Task->task_spray.event ^= TASK_ACTIVE_EVT;
		SprayTask_Process();
		pApp_Task->task_spray.interval = spray_intv;
	}
    
	if ( pApp_Task->task_power_off.event & TASK_ACTIVE_EVT ){
        pApp_Task->task_power_off.event ^= TASK_ACTIVE_EVT;
        power_off();
	}

    if ( pApp_Task->task_uart.event & TASK_ACTIVE_EVT ){
		pApp_Task->task_uart.event ^= TASK_ACTIVE_EVT;
		pApp_Task->task_uart.interval = 100;
	}

}

void VT_value_set( void )
{
    switch( uln_level )
    {
        case 0:
        	 ULN_IN1_OFF();
        	 ULN_IN2_OFF();
        	 ULN_IN3_OFF();
        	 ULN_IN4_OFF();
             break;
        case 1:
        	 ULN_IN1_ON();
       	     ULN_IN2_OFF();
       	     ULN_IN3_OFF();
       	     ULN_IN4_OFF();
             break;
        case 2:
        	 ULN_IN1_OFF();
      	     ULN_IN2_ON();
      	     ULN_IN3_OFF();
      	     ULN_IN4_OFF();
             break;
        case 3:
        	 ULN_IN1_ON();
     	     ULN_IN2_ON();
     	     ULN_IN3_OFF();
     	     ULN_IN4_OFF();
             break;
        case 4:
        	 ULN_IN1_OFF();
    	     ULN_IN2_OFF();
    	     ULN_IN3_ON();
    	     ULN_IN4_OFF();
             break;
        case 5:
        	 ULN_IN1_ON();
       	     ULN_IN2_OFF();
       	     ULN_IN3_ON();
       	     ULN_IN4_OFF();
             break;
        case 6:
        	 ULN_IN1_OFF();
      	     ULN_IN2_ON();
      	     ULN_IN3_ON();
      	     ULN_IN4_OFF();
             break;
        case 7:
        	 ULN_IN1_ON();
     	     ULN_IN2_ON();
     	     ULN_IN3_ON();
     	     ULN_IN4_OFF();
             break;
        case 8:
        	 ULN_IN1_OFF();
    	     ULN_IN2_OFF();
    	     ULN_IN3_OFF();
    	     ULN_IN4_ON();
             break;
        case 9:
        	 ULN_IN1_ON();
       	     ULN_IN2_OFF();
       	     ULN_IN3_OFF();
       	     ULN_IN4_ON();
             break;
        case 10:
        	 ULN_IN1_OFF();
      	     ULN_IN2_ON();
      	     ULN_IN3_OFF();
      	     ULN_IN4_ON();
             break;
        case 11:
        	 ULN_IN1_ON();
     	     ULN_IN2_ON();
     	     ULN_IN3_OFF();
     	     ULN_IN4_ON();
             break;
        case 12:
        	 ULN_IN1_OFF();
    	     ULN_IN2_OFF();
    	     ULN_IN3_ON();
    	     ULN_IN4_ON();
             break;
        case 13:
        	 ULN_IN1_ON();
       	     ULN_IN2_OFF();
       	     ULN_IN3_ON();
       	     ULN_IN4_ON();
             break;
        case 14:
        	 ULN_IN1_OFF();
      	     ULN_IN2_ON();
      	     ULN_IN3_ON();
      	     ULN_IN4_ON();
             break;
        case 15:
        	 ULN_IN1_ON();
     	     ULN_IN2_ON();
     	     ULN_IN3_ON();
     	     ULN_IN4_ON();
             break;
        default:
        	 uln_level = 0;
        	 ULN_IN1_OFF();
        	 ULN_IN2_OFF();
        	 ULN_IN3_OFF();
        	 ULN_IN4_OFF();
             break;
    }
}

void KeyTask_Process( void )
{
    uint8_t key;   
     if ( lt_en_time ) return;
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
#if PROG_TYPE != CONSUMER_STUDY
        if ( s_spray_state == STATE_SPRAY_READY ){
            spray_key_en = true;
        }
        else
        {
            spray_key_en = false;
        }
#endif
    }
    else if ( sw_status == KEY_LPRESS ){
#if PROG_TYPE == MULTI_SHOT
    	queued_shots = AMOUNT_OF_SHOTS;
#elif PROG_TYPE == CONSUMER_STUDY
        if ( s_spray_state == STATE_SPRAY_READY ){
            spray_key_en = true;
        }
        else
        {
            spray_key_en = false;
        }
#else
        power_off();
#endif    
    }
}

void ChargeTask_Process( void )                                                                                                     
{
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
#if LED_METHOD
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
#else
            if ( charge_pin == GPIO_PIN_RESET )
            {

                if(chrg_flag==1)
                {
                    chrg_flag=0;
                }
                else bat_flag = BAT_CHARGE_ON;
            }
            else
            {
                bat_flag = BAT_CHARGE_FULL;
                chrg_flag=1;
            }
#endif
        }
        else {
            //s_spray_state = STATE_SPRAY_READY;
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
#if LED_METHOD
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
#else
    if ( bat_flag == BAT_CHARGE_ON )
    {
        pApp_Task->task_charge.interval = 500;				//0.5S
        if ( ++bat_led_flash > 5 ){
            bat_led_flash = 1;
        }
        switch ( bat_led_flash ){
            case 1:
                led_state.led_array &= 0xf0;
                led_state.led_array |= LED_BAT_BAD;
                led_state.led1_color = LED1_GREEN;
                break;
            case 2:
                led_state.led_array |= LED_BAT_LOW;
                 break;
            case 3:
                led_state.led_array |= LED_BAT_NORMAL;
                 break;
            case 4:
                 led_state.led_array |= LED_BAT_FULL;
                 break;
            default:
                 break;
        }
    }
    else if ( bat_flag == BAT_CHARGE_FULL ){
        //bat_led_flash = 0;
        led_state.led_array |= LED_BAT_FULL;
        led_state.led1_color = LED1_GREEN;
    }
#endif
}
}

void BatTask_Process( void )                                                                                                                        
{                                                   
    uint8_t bat_vol_bk;
    if ( s_spray_state != STATE_SPRAY_READY ){
        return;
    }
    bat_vol_bk = adc_bat_check();
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
    led_bat_update();
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
#if LED_METHOD
    uint8_t blink, percent, type;

    blink = (uint8_t)(pApp_Task->task_led.param1);
    percent = 100;
    type = 1;
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
                if(blink_shot_flag==1){
                    blink_shot_flag=0;
                    pApp_Task->task_wr_flash.event |= TASK_ACTIVE_EVT;
                }
            }
            else{
                pApp_Task->task_led.interval = pApp_Task->task_led.param2;
            }
        }
        pApp_Task->task_led.param1 = (uint16_t)blink;
    }
    else if ( pApp_Task->task_led.event & LED_STATE_UPDATE_EVT ){
        pApp_Task->task_led.event ^= LED_STATE_UPDATE_EVT;
        //HalLed_StatusOnOff( 1, HAL_LED_MODE_OFF );
        HalLed_StatusOnOff( type, HAL_LED_MODE_ON );
//        switch ( type ){
//            case 1:
//            case 2:
//            case 3:
//            case 4:
//                     HalLed_StatusOnOff( type, HAL_LED_MODE_ON );
//                     break;
//            case 5:  break;
//        }
    }
#else
    uint8_t blink,i,val;
    blink = (uint8_t)(pApp_Task->task_led.param1);
    if ( pApp_Task->task_led.event & LED_STATE_BLINK_EVT ){
        if ( blink % 2 ){
            HalLed_AllOnOff(HAL_LED_MODE_OFF);
            led_pwm_flag = 0;
        }
        else{
            HalLed_AllOnOff(HAL_LED_MODE_ON);
            led_pwm_flag = 0;
        }
        
        if ( blink ){
            if ( --blink == 0 ){
                led_pwm_flag = 0;
                pApp_Task->task_led.event ^= LED_STATE_BLINK_EVT;
                pApp_Task->task_led.interval = LED_PWMH;
                led_pwm_flag = 1;
                pApp_Task->task_led.param1 = 0;
                pApp_Task->task_led.param2 = 0;
            }
            else{
                pApp_Task->task_led.interval = 100;//pApp_Task->task_led.param2;
            }
        }
        pApp_Task->task_led.param1 = (uint16_t)blink;
    }
    else if ( pApp_Task->task_led.event & LED_STATE_UPDATE_EVT ){
        pApp_Task->task_led.event ^= LED_STATE_UPDATE_EVT;
    }
    
    //update output
    if(led_pwm_flag)
    {
        for(i=1;i<=8;i++)
        {
            val = (led_state.led_array >> (i-1));
            val &= 0x01;
            hal_led_onoffCase(i,val);
        }
        led_pwm_flag = 0;
        pApp_Task->task_led.interval = led_pwm_high;
    }
    else
    {
        HalLed_AllOnOff(HAL_LED_MODE_OFF);
        led_pwm_flag = 1;
        pApp_Task->task_led.interval = led_pwm_low;
    }
#endif
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
                     pApp_Task->task_led.param2 = 100*2;               // 100*2*1ms = 0.2S
                     pApp_Task->task_power_off.interval = 0;
                     pApp_Task->task_power_off.event = 0;
                     s_spray_state = STATE_SPRAY_ACTIVE_DLY;
                     lt_en_time = 0;
                     app_set_frq( PWM_SCAN_FOSC_INI );            
                     duty_set_val = 0;
                     PWM_Duty_Set( duty_set_val );
                     ADI_OE_ON();                					   // Start 183KHZ Transducer work
                     DC8V_EN_ON();
                     DC24V_EN_ON();
                     cnt_alarm1 = 0;
                     if (VOLTAGE_REG_EN == 1) uln_level = 0;
                     else uln_level = BRIDGE_VT;
                     VT_value_set();								   // set output voltage
                     if (SPRAY_DELAY > 0) spray_intv = SPRAY_DELAY;
                     else spray_intv = 1;
                     cnt_delay_ini = 0;
                     fg_stop_spray = 0;                                // clear fg_stop_spray
                     spray_ready_time = 0;                             
                     HalSdp_Start_Measure();
                 }
             }
             else
             {
                  if( SW_GetValue() == GPIO_PIN_RESET )
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
                     spray_intv = 5*2;                           		//10ms
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
                current_frc = RES_FREQ + (SCAN_CNT1 >> 1) * FRC_100HZ;    //read device frquency
                max_frc = current_frc;
                app_set_frq( current_frc );	                                 //set autotune first frequency
                shot_info.init_freq = current_frc;
                s_spray_state = STATE_SPRAY_SCAN;
                lt_en_time = sys_ticket + ON_TIME;
                if( spray_triger == TRIGER_SDP )
                {
                    lt_en_time = sys_ticket + BREATH_ON_TIME;             	     //breath spray time limited
                }
            }
            else
            {                                      
                if(cnt_delay_ini >= 10)			// > 10ms                       
                {         
                    ADC_Start( ADC_CUT_CH );
                    while( adc_state == ADC_BUSY );
                    pwm_adc_val = ADC_Average( ADC_CUT_CH );
                    if(pwm_adc_val >= CURRENT_FOSC_INI)
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
            }
            break;
        case STATE_SPRAY_SCAN:                                                                        
             
             spray_intv = SCAN_DLY_2;
             ADC_Start( ADC_CUT_CH );
             while( adc_state == ADC_BUSY );
             pwm_adc_val = ADC_Average( ADC_CUT_CH );

             pwm_freq_adc[pwm_adc_cnt] = pwm_adc_val;
             pwm_adc_cnt++;
#if CURRENT_PROTECTION_EN == 1
             if(pwm_adc_val >= CURRENT_THR1)
             {
            	  cnt_alarm1++;
            	  if(cnt_alarm1 >= ALARM1_COUNT )
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
                      break;
            	  }
             }
#endif
             if(pwm_adc_val >= pwm_adc_max)                 
             {                                              
            	 max_frc = current_frc;
            	 pwm_adc_max = pwm_adc_val;
             }
             pwm_adc_offset++;                          
             if ( pwm_adc_offset < SCAN_CNT1 )
             {
                 if (pwm_adc_offset == 0) shot_info.init_freq = current_frc;
                 current_frc -= FRC_100HZ;                 //autotune (as 100HZ)
                 app_set_frq( current_frc );
             }
             else
             {
            	 if(pwm_adc_offset == SCAN_CNT1)
            	 {
            		 current_frc = max_frc + (SCAN_CNT2 >> 1)*FRC_7HZ6; //autotune(as 7.6HZ)
            		 app_set_frq( current_frc );
            		 pwm_adc_max = 0;
            	 }
            	 else
            	 {                                                      
            		 pwm_adc_offset = SCAN_CNT1 + 1;                    
            		 pwm_list_offset++;                                 
            		 if(pwm_list_offset <= SCAN_CNT2)
            		 {   
            			 current_frc -= FRC_7HZ6;					   //autotune (as 10HZ)
            			 app_set_frq( current_frc );                   
            		 }                                                     
            		 else                                              
            		 {                                                               
            			 current_frc = max_frc;                                            
            			 app_set_frq( current_frc );                      
            			 bk_frc = current_frc;							// save current_frc
            			 bk_adc_max = pwm_adc_max;			            // save max adc value
            			 pwm_list_offset = SCAN_CNT2;				    //
            			 sdp_cnt = 50;									// 0.1S reset
            			 spray_intv = 5;								// delay 10ms-- test first ADC
            			 cnt_alarm2 = 0;
            			 cnt_alarm3 = 0;
                         pwm_adc_sum = 0;
                         cnt_adc_10 = 0;
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
            
            spray_intv = 1;				      			//1ms
         	ADC_Start( ADC_CUT_CH );
        	while( adc_state == ADC_BUSY );
        	pwm_adc_val = ADC_Average( ADC_CUT_CH );
            // prevent out of range index and set current measurement rate
            if((currentIndex < CURRENT_SIZE) && (current_inc++ % current_rate == 0))
            {
                shot_info.current[currentIndex++] = pwm_adc_val;
            } 
#if VOLTAGE_REG_EN == 1
            pwm_adc_sum += pwm_adc_val;                       
       		cnt_adc_10++;
       		if(cnt_adc_10 >= 10)
       		{
       		    cnt_adc_10 = 0;                             //10ms
       		    pwm_adc_evg = pwm_adc_sum/10;
       		    pwm_adc_sum = 0;
                pwm_freq_adc[pwm_adc_cnt] = pwm_adc_evg;           
                if ( pwm_adc_cnt < 120 )                           
                {
                   pwm_adc_cnt++;
                }                                              
        	    if(pwm_adc_evg >= CURRENT_TARGET + 2)         
	    	    {                                          
                    if(pwm_adc_evg >= CURRENT_TARGET + 6) 
                    {
                        if(uln_level != 0)
                        {
                            uln_level = uln_level - 1;
                            VT_value_set();                            
                        }
                        else
                        {
                            if(duty_set_val <= PWM_DUTY_MIN)
                            {
                                duty_set_val++;
                                PWM_Duty_Set(duty_set_val);
                            }
                        }
                    }
                    else
                    {
                        if(duty_set_val <= PWM_DUTY_MIN)
                        {
                            duty_set_val++;
                            PWM_Duty_Set(duty_set_val);
                        }
                        else
                        {
                            if(uln_level != 0)
                            {
                                uln_level = uln_level - 1;
                                VT_value_set();
                            }
                        }
                    }
                }
                else
                {
                    if(pwm_adc_evg <= CURRENT_TARGET - 2)
                    {
                        if(pwm_adc_evg <= CURRENT_TARGET - 6)
                        {
                            if(uln_level < 15)
                            {
                                uln_level++;
                                VT_value_set();
                            }
                            else
                            {
                                if(duty_set_val != 0)
                                {   
                                    duty_set_val -= 1;
                                    PWM_Duty_Set(duty_set_val);
                                }
                            }  
                        }
                        else
                        {
                            if(duty_set_val != 0)
                            {   
                                duty_set_val -= 1;
                                PWM_Duty_Set(duty_set_val);
                            }
                            else
                            {
                                uln_level++;
                                if(uln_level >= 15)
                                {
                                    uln_level = 15;
                                }
                                VT_value_set();                        
                            }
                        }
                    }
                }                         
            }                        
#endif
#if CURRENT_PROTECTION_EN == 1
            if(pwm_adc_val >= CURRENT_THR2)
            {
                cnt_alarm2++;
                if(cnt_alarm2 >= ALARM2_COUNT )
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
                    break;
                }
            }
            else
            {
                cnt_alarm2 = 0;
            }
            if(pwm_adc_val >= bk_adc_max + CURT_OFFSET)
            {
                cnt_alarm3++;
                if(cnt_alarm3 >= ALARM3_COUNT)
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
                    break;
                }
            }
            else
            {
                cnt_alarm3 = 0;
            }               
#endif
            sdp_cnt++;
            if(sdp_cnt >= PRES_SAMPLE_VAL)					// pressure sensor sampling interval 0.01S
            {
                sdp_cnt = 0;
                sdp_value = 0;
                HalSdp_GetValue( &sdp_value, SDP_STOP_THR );
                // save pressure sensor reading into shot_info
                if (pressure_index < PRESSURE_SIZE && pressure_inc++ % pressure_rate == 0)
                {
                   shot_info.pressure[pressure_index++] = sdp_value;
                }
                if( sdp_value > sdp_value_max){
                    sdp_value_max = sdp_value;
                }
                if( sdp_value < SDP_STOP_THR )
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

       	spray_intv = 1;										// 1ms
       	if(lt_en_time != 0)
       	{
         	ADC_Start( ADC_CUT_CH );
        	while( adc_state == ADC_BUSY );
        	pwm_adc_val = ADC_Average( ADC_CUT_CH );
            // prevent out of range index and set current measurement rate
            if((currentIndex < CURRENT_SIZE) && (current_inc++ % current_rate == 0))
            {
                shot_info.current[currentIndex++] = pwm_adc_val;
            } 
#if VOLTAGE_REG_EN == 1
            pwm_adc_sum += pwm_adc_val;                       
       		cnt_adc_10++;
       		if(cnt_adc_10 >= 10)
       		{
       		    cnt_adc_10 = 0;                             //10ms
       		    pwm_adc_evg = pwm_adc_sum/10;
       		    pwm_adc_sum = 0;
//                pwm_freq_adc[pwm_adc_cnt] = pwm_adc_evg;           
                if ( pwm_adc_cnt < 120 )                           
                {
                   pwm_adc_cnt++;
                }                                              
        	    if(pwm_adc_evg >= CURRENT_TARGET + 2)         
	    	    {                                          
                    if(pwm_adc_evg >= CURRENT_TARGET + 6) 
                    {
                        if(uln_level != 0)
                        {
                            uln_level = uln_level - 1;
                            VT_value_set();                            
                        }
                        else
                        {
                            if(duty_set_val <= PWM_DUTY_MIN)
                            {
                                duty_set_val++;
                                PWM_Duty_Set(duty_set_val);
                            }
                        }
                    }
                    else
                    {
                        if(duty_set_val <= PWM_DUTY_MIN)
                        {
                            duty_set_val++;
                            PWM_Duty_Set(duty_set_val);
                        }
                        else
                        {
                            if(uln_level != 0)
                            {
                                uln_level = uln_level - 1;
                                VT_value_set();
                            }
                        }
                    }
                }
                else
                {
                    if(pwm_adc_evg <= CURRENT_TARGET - 2)
                    {
                        if(pwm_adc_evg <= CURRENT_TARGET - 6)
                        {
                            if(uln_level < 15)
                            {
                                uln_level++;
                                VT_value_set();
                            }
                            else
                            {
                                if(duty_set_val != 0)
                                {   
                                    duty_set_val -= 1;
                                    PWM_Duty_Set(duty_set_val);
                                }
                            }  
                        }
                        else
                        {
                            if(duty_set_val != 0)
                            {   
                                duty_set_val -= 1;
                                PWM_Duty_Set(duty_set_val);
                            }
                            else
                            {
                                uln_level++;
                                if(uln_level >= 15)
                                {
                                    uln_level = 15;
                                }
                                VT_value_set();                        
                            }
                        }
                    }
                }                         
            }                        
#endif
#if CURRENT_PROTECTION_EN == 1
            if(pwm_adc_val >= CURRENT_THR2)
            {
                cnt_alarm2++;
                if(cnt_alarm2 >= ALARM2_COUNT )
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
                    break;
                }
            }
            else
            {
                cnt_alarm2 = 0;
            }
            if(pwm_adc_val >= bk_adc_max + CURT_OFFSET)
            {
                cnt_alarm3++;
                if(cnt_alarm3 >= ALARM3_COUNT)
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
                    break;
                }
            }
            else
            {
                cnt_alarm3 = 0;
            }         
#endif
       	} 
       	else
        {
       		s_spray_state = STATE_DOSE_COMPLETE;
       		spray_intv = 2;
            ADI_OE_OFF();
            DC8V_EN_OFF();
            DC24V_EN_OFF();
            //HAL_UART_Send( (uint8_t *)pwm_tx_adc, 200 );
       	}
        break;

        case STATE_DOSE_COMPLETE:
            if (!is_mem_full()) save_shot();
             HalSdp_Stop_Measure();
             if (spray_triger == TRIGER_AUTO && OFF_TIME > 20) spray_intv = OFF_TIME;
             else spray_intv = 20;
             spray_triger = 0; 
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

void measure_pressure()
{
	if (curPress < PRESSURE_SIZE){
		HalSdp_GetValue( &pressBuf[curPress], SDP_STOP_THR );
		curPress++;
	}
    return;
}

void save_pressure()
{
	// save pressure buffer to memory
 	write_pressure(pressBuf);
 	// reset pressure buffer index
 	curPress = 0;
 	// reset pressure buffer
 	for (int i = 0; i < PRESSURE_SIZE; i++) pressBuf[i] = 0;
    return;
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
    currentIndex = 0;
    shot_info.spray_time = 0;
    memset(shot_info.pressure, 0, sizeof(shot_info.pressure));
    memset(shot_info.current, 0, sizeof(shot_info.current));
    memset(shot_info.autotune, 0, sizeof(shot_info.autotune));
}


static uint8_t state_spray_ready_init( void )
{
    uint16_t value;
    uint8_t rst = false;
    
    if ( bat_vol <= BAT_VOL_OFF ){
        return rst;
    }

    value = HalSdp_GetThr();
#if PROG_TYPE != PRESSURE_CHECK
#if PROG_TYPE != BUTTON && PROG_TYPE != MULTI_SHOT
    if ( value > SDP_START_THR ){
        spray_triger = TRIGER_SDP;
        rst = true;
        sdp_value_max = value;
    }
#endif
#if PROG_TYPE != PRESS && PROG_TYPE != SMOKE_MC
    //if ( sw_status == KEY_SPRESS ){
    if ( spray_key_en ){
        spray_key_en = false;
        sdp_thr = 2;
        spray_triger = TRIGER_KEY;
        rst = true;
    }
#endif
#if PROG_TYPE == MULTI_SHOT
    if (queued_shots > 1){
        spray_key_en = false;
        sdp_thr = 2;
        spray_triger = TRIGER_AUTO;
        rst = true;
        queued_shots--;
    }
#endif
#else
    if ( sw_status == KEY_HOLD ){
        sdp_thr = 2;
    	rst = true;
        spray_triger = TRIGER_HOLD;
    }
#endif
    return rst;
}

/***********************************************************************************
*  @fn     
*  @brief  Update system ticket every for all tasks
*  @param      
*            
*  @return   none
*/
//*** 1 Task_ticket_update = 500us * TICKET_RELOAD
void task_ticket_update( void )
{
    uint8_t eventFlag = false;
    
    sys_ticket++;                               
    wdt_ticket = 0;
    if ( lt_en_time )                                           
    {         
        spray_time++;
        if ( sys_ticket >= lt_en_time && (spray_triger == TRIGER_KEY || spray_triger == TRIGER_AUTO))            
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
			LedTask_Process();
			eventFlag = true;
		}
	}
    if ( pApp_Task->task_spray.interval > 0 ){
        if ( --pApp_Task->task_spray.interval ==0 ){
            pApp_Task->task_spray.event |=  TASK_ACTIVE_EVT;
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

	if ( pApp_Task->task_sdp3x.interval > 0 ){
		if ( --pApp_Task->task_sdp3x.interval == 0 ){
			pApp_Task->task_sdp3x.event |= TASK_ACTIVE_EVT;
			eventFlag = true;
		}
	}
}

/***********************************************************************************
*  @fn     
*  @brief  powers device up from low power state
*  @param  type    
*            
*  @return   none
*/
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

/***********************************************************************************
*  @fn     
*  @brief  entering low power/off state
*  @param      
*            
*  @return   none
*/
static void power_off( void )
{    
    if ( bat_flag != BAT_CHARGE_OFF ){
        return;
    }
    HalLed_AllOnOff( HAL_LED_MODE_OFF );
    VBAT_CN_OFF();
    while(1);
}

/***********************************************************************************
*  @fn     
*  @brief  Check voltage of the battery
*  @param      
*            
*  @return   Battery state
*/
uint8_t adc_bat_check()
{
    uint8_t bat_vol_bk;
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
    return(bat_vol_bk);
}
/***********************************************************************************
*  @fn     
*  @brief  blinks LEDs
*  @param  param1, blink time in ms
*            
*  @return   none
*/
void blink_setup(uint8_t param1,uint8_t param2)
{
     pApp_Task->task_led.interval = 100;                           // ####
     pApp_Task->task_led.event = LED_STATE_BLINK_EVT;            // #### ?????????LED 2?
     pApp_Task->task_led.param1 = param1;                        // ####
     pApp_Task->task_led.param2 = param2;                        // #### 50*2ms
}
/***********************************************************************************
*  @fn     
*  @brief  Updates state of LEDs
*  @param      
*            
*  @return   none
*/
void led_bat_update()
{
#if LED_METHOD
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
#else
        // All off
    led_state.led_array = led_state.led_array & 0xf0;
    if ( bat_vol == BAT_VOL_FULL ){
        led_state.led_array = led_state.led_array | LED_BAT_FULL;
        led_state.led1_color = LED1_GREEN;
    }
    else if ( bat_vol == BAT_VOL_NORMAL ){
        led_state.led_array = led_state.led_array | LED_BAT_NORMAL;
        led_state.led1_color = LED1_GREEN;
    }
    else if ( bat_vol == BAT_VOL_LOW ){
        led_state.led_array = led_state.led_array | LED_BAT_LOW;
        led_state.led1_color = LED1_GREEN;
    }
    else if ( bat_vol == BAT_VOL_BAD ){
        led_state.led_array = led_state.led_array | LED_BAT_BAD;
        led_state.led1_color = LED1_YELLOW;
    }
    else if ( bat_vol == BAT_VOL_OFF ){
        led_state.led_array = led_state.led_array | LED_BAT_BAD;
        led_state.led1_color = LED1_RED;
    }
#endif
}
