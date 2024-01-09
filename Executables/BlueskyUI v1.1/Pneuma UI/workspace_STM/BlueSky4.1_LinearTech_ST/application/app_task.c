/***********************************************************************************
  Filename:     task.c

  Description:  
                
***********************************************************************************/
/******************************************************
BlueSky pushmode 20210820(no RTOS),without IDchip
change data:20210820
907 row  SCAN_ROW HAVE CHANGED SCAN_COLUMN
lt_en_time =0 have moved 1004 from 994 row
LT_SHNDA_OFF() change ,have change PWM on/off control spray
REMOVE ROW 449  PWM SETUP() ； key Spray time =1.5s
change data: 20210821
added Wachdog Reset time =1S, Wachdog Enable hal_board_cfg.h (ROW 59) ;
opened Inc stm32l4xx_hal_conf.h(#define HAL_IWDG_MODULE_ENABLED)
change data: 20210824
1:change PCB Version  LED i/o PORT have be changed
2:change led i/o port define program(hal_board_cfg.h),LED3 have changed as PB9
3:Hide Main.c Row 127,128 ; Disable PWM start
4:1:change {PWM start(add initial pwm) ; PWM_STOP} (Src/tim.c have been changed )
    PWM STOP(initial PWM port as I/O, OUTPUT=0)
change data: 20210825
change row 1030 ,1031

PCB Version: BlueSky4.0 Pushmode(ST) RevA1 20210813

change data: 2021-12-29
pressure sensor test : reverse direction

Change Data: BlueSky4.1_Linear Tech 20220624
Pcb Version: BlueSky_xx Rev A 20220616
********************************************************/

/***********************************************************************************
* INCLUDES
*/

#include "string.h"
#include "stdlib.h"

#include "main.h"
#include "stm32l4xx_it.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h"
#include "portmacro.h"
#include "usart.h"

#include "stm32l4xx_hal_iwdg.h"
#include "stm32l4xx_ll_utils.h"

#include "hal_board_cfg.h"
#include "atca_basic.h"
#include "hal_i2c.h"
#include "atca_main.h"

#include "app_task.h"

extern volatile TickType_t xTickCount;

PRIVILEGED_DATA volatile static uint32_t app_timer = 0;

PRIVILEGED_DATA volatile uint32_t app_task_enable = pdFALSE;

extern osThreadId fingerprintHandle;
extern osThreadId periHandle;
PRIVILEGED_DATA volatile static fp_task_suspend = false;

//extern IWDG_HandleTypeDef IwdgHandle;

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

#define SCAN_DLY_1              100       // 50x2ms
#define SCAN_DLY_2              1         // 25x2ms
#define SDP_STOP_PERCENT        40        // stop spray pressure value percent = pressure_value_max * 40%
#define SDP_OFF_CNT             1         // 2 times
#define SPRAY_PWM_DUTY          20        // Spray percent
#define SPRAY_DATE              20

#define TEST                    0
#define UNUSED                  76
     
#define PWM_AB_SWITCH_CNT       0         // 2,4,6,8.....
#define PWM_A_SWITCH_TIME       0        // 50 * 2ms
#define PWM_B_SWITCH_TIME       0

/***********************************************************************************
 * MACROS
 */
#define VOICE_OFF_IMME()        st( Voice_Command( VOICE_AMP_OFF, volume );  cur_voice = 0; )

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
uint16_t queued_shots;									// amount of shots queued to spray when using automated shots program

uint8_t sdp_thr = 0;
cartidge_attr_t cart_attr, *p_cart_attr = &cart_attr, *p_bt_state = NULL;


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
volatile uint8_t sys_ticket_bool = pdFALSE;
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

#define PWM_SCAN_FOSC_INI      436             //183.49K
#define SCAN_ROW               1
#define SCAN_COLUMN            23
#define SCAN_CNT               SCAN_COLUMN * SCAN_ROW

//static uint16_t pwm_freq_list[SCAN_COLUMN] = { 615, 616, 617, 618, 619, 620, 621, 622, 623, 624, 625, 626, 627, 628, 629, 630, 631, 632, 633, 634, 635, 636, 637 };
// 130.08KHZ -- 125.5KHZ (SCAN Frequency)

static uint16_t pwm_freq_list[SCAN_COLUMN];
// 127.79KHZ -- 123.45KHZ (SCAN Frequency)

//static uint16_t pwm_freq_list[SCAN_COLUMN] = { 626, 626, 626, 626, 626, 626, 626, 626, 626, 626, 626, 626, 626, 626, 626, 626, 626, 626, 626 };
//1--127.79KHZ

//static uint16_t pwm_freq_list[SCAN_COLUMN] = { 627, 627, 627, 627, 627, 627, 627, 627, 627, 627, 627, 627, 627, 627, 627, 627, 627, 627, 627 };
//2--127.59KHZ

//static uint16_t pwm_freq_list[SCAN_COLUMN] = { 628, 628, 628, 628, 628, 628, 628, 628, 628, 628, 628, 628, 628, 628, 628, 628, 628, 628, 628 };
//3--127.39KHZ

//static uint16_t pwm_freq_list[SCAN_COLUMN] = { 629, 629, 629, 629, 629, 629, 629, 629, 629, 629, 629, 629, 629, 629, 629, 629, 629, 629, 629 };
//4--127.18KHZ

//static uint16_t pwm_freq_list[SCAN_COLUMN] = { 630, 630, 630, 630, 630, 630, 630, 630, 630, 630, 630, 630, 630, 630, 630, 630, 630, 630, 630 };
//5--126.98KHZ

//static uint16_t pwm_freq_list[SCAN_COLUMN] = { 631, 631, 631, 631, 631, 631, 631, 631, 631, 631, 631, 631, 631, 631, 631, 631, 631, 631, 631 };
//6--126.78KHZ

//static uint16_t pwm_freq_list[SCAN_COLUMN] = { 632, 632, 632, 632, 632, 632, 632, 632, 632, 632, 632, 632, 632, 632, 632, 632, 632, 632, 632 };
//7--126.58KHZ

//static uint16_t pwm_freq_list[SCAN_COLUMN] = { 633, 633, 633, 633, 633, 633, 633, 633, 633, 633, 633, 633, 633, 633, 633, 633, 633, 633, 633 };
//8--126.38KHZ

//static uint16_t pwm_freq_list[SCAN_COLUMN] = { 634, 634, 634, 634, 634, 634, 634, 634, 634, 634, 634, 634, 634, 634, 634, 634, 634, 634, 634 };
//9--126.18KHZ

//static uint16_t pwm_freq_list[SCAN_COLUMN] = { 635, 635, 635, 635, 635, 635, 635, 635, 635, 635, 635, 635, 635, 635, 635, 635, 635, 635, 635 };
//10--125.98KHZ

//static uint16_t pwm_freq_list[SCAN_COLUMN] = { 636, 636, 636, 636, 636, 636, 636, 636, 636, 636, 636, 636, 636, 636, 636, 636, 636, 636, 636 };
//11--125.786KHZ

//static uint16_t pwm_freq_list[SCAN_COLUMN] = { 637, 637, 637, 637, 637, 637, 637, 637, 637, 637, 637, 637, 637, 637, 637, 637, 637, 637, 637 };
//12--125.588KHZ

//static uint16_t pwm_freq_list[SCAN_COLUMN] = { 638, 638, 638, 638, 638, 638, 638, 638, 638, 638, 638, 638, 638, 638, 638, 638, 638, 638, 638 };
//13--125.39KHZ

//static uint16_t pwm_freq_list[SCAN_COLUMN] = { 639, 639, 639, 639, 639, 639, 639, 639, 639, 639, 639, 639, 639, 639, 639, 639, 639, 639, 639 };
//14--125.195KHZ

//static uint16_t pwm_freq_list[SCAN_COLUMN] = { 640, 640, 640, 640, 640, 640, 640, 640, 640, 640, 640, 640, 640, 640, 640, 640, 640, 640, 640 };
//15--125KHZ

//static uint16_t pwm_freq_list[SCAN_COLUMN] = { 641, 641, 641, 641, 641, 641, 641, 641, 641, 641, 641, 641, 641, 641, 641, 641, 641, 641, 641 };
//16--124.8KHZ

//static uint16_t pwm_freq_list[SCAN_COLUMN] = { 642, 642, 642, 642, 642, 642, 642, 642, 642, 642, 642, 642, 642, 642, 642, 642, 642, 642, 642 };
//17--124.6KHZ

//static uint16_t pwm_freq_list[SCAN_COLUMN] = { 643, 643, 643, 643, 643, 643, 643, 643, 643, 643, 643, 643, 643, 643, 643, 643, 643, 643, 643 };
//18--124.4KHZ

//static uint16_t pwm_freq_list[SCAN_COLUMN] = { 644, 644, 644, 644, 644, 644, 644, 644, 644, 644, 644, 644, 644, 644, 644, 644, 644, 644, 644 };
//19--124.2KHZ

//static uint16_t pwm_freq_list[SCAN_COLUMN] = { 645, 645, 645, 645, 645, 645, 645, 645, 645, 645, 645, 645, 645, 645, 645, 645, 645, 645, 645 };
//20--124.03KHZ

//static uint16_t pwm_freq_list[SCAN_COLUMN] = { 646, 646, 646, 646, 646, 646, 646, 646, 646, 646, 646, 646, 646, 646, 646, 646, 646, 646, 646 };
//21--123.83KHZ

//static uint16_t pwm_freq_list[SCAN_COLUMN] = { 647, 647, 647, 647, 647, 647, 647, 647, 647, 647, 647, 647, 647, 647, 647, 647, 647, 647, 647 };
//22--123.65KHZ

//static uint16_t pwm_freq_list[SCAN_COLUMN] = { 648, 648, 648, 648, 648, 648, 648, 648, 648, 648, 648, 648, 648, 648, 648, 648, 648, 648, 648 };
//23--123.45KHZ

//static uint16_t pwm_freq_list[SCAN_COLUMN] = { 667, 667, 667, 667, 667, 667, 667, 667, 667, 667, 667, 667, 667, 667, 667, 667, 667, 667, 667 };
//23--120KHZ

//static uint16_t pwm_freq_list[SCAN_COLUMN] = { 436, 436, 436, 436, 436, 436, 436, 436, 436, 436, 436, 436, 436, 436, 436, 436, 436, 436, 436 };
// 183.49KHZ


//static uint16_t pwm_freq_list[SCAN_COLUMN] = { 442, 443, 444, 445, 446, 447, 448, 449 };  // 180.2KHZ (181K--178.2K)
//static uint16_t pwm_freq_list[SCAN_COLUMN] = { 439, 440, 441, 442, 443, 444, 445, 446 };  // 181.5KHZ (182.2K--179.4K)
//static uint16_t pwm_freq_list[SCAN_COLUMN] = { 437, 438, 439, 440, 441, 442, 443, 444 };  // 182.5KHZ (183K--180.18K)

//static uint16_t pwm_freq_list[SCAN_COLUMN] = { 440, 441, 442, 443, 444, 445, 446, 447 };  // 181KHZ (181.4K--178.6K) TAIWAN+OMRON

//static uint16_t pwm_freq_list[SCAN_COLUMN] = { 431, 432, 433, 434, 435, 436, 437, 438 };  // 184.75KHZ (185.6K--182.6K)   // H4-T3-6,7,8 - 5°
//static uint16_t pwm_freq_list[SCAN_COLUMN] = { 434, 435, 436, 437, 438, 439, 440, 441 };  // 183.5KHZ (184.3K--181.4K)  // H4-T3-6,7,8 - 25°
//static uint16_t pwm_freq_list[SCAN_COLUMN] = { 435, 436, 437, 438, 439, 440, 441, 442 };  // 183 KHZ (183.9K--180.99K)  // H4-T3-6,7,8 - 40°
//static uint16_t pwm_freq_list[SCAN_COLUMN] = { 433, 434, 435, 436, 437, 438, 439, 440 };  // 184KHZ (184.75K--181.81K)
//static uint16_t pwm_freq_list[SCAN_COLUMN] = { 431, 432, 433, 434, 435, 436, 437, 438 };  // 185KHZ (185.6K--183.48K)
//static uint16_t pwm_freq_list[SCAN_COLUMN] = { 426, 427, 428, 429, 430, 431, 432, 433 };  // 187KHZ (187.8K--184.75K)
//static uint16_t pwm_freq_list[SCAN_COLUMN] = { 419, 420, 421, 422, 423, 424, 425, 426 };  // 190KHZ (190.9K--187.79K)
//static uint16_t pwm_freq_list[SCAN_COLUMN] = { 437, 438, 439, 440, 441, 442, 443, 444 };  // 183K--180.18KHZ SCAN FREQUENCY(TAIWAN PIEZO)
//static uint16_t pwm_freq_list[SCAN_COLUMN] = { 441, 442, 443, 444, 445, 446, 447, 448 };  // 180.6KHZ (181.4K--178.6K) TAIWAN+OMRON
//static uint16_t pwm_freq_list[SCAN_COLUMN] = { 407, 407, 407, 407, 407, 407, 407, 407 };  // 196.56KHZ
//static uint16_t pwm_freq_list[SCAN_COLUMN] = { 359, 360, 361, 362, 363, 364, 365, 366 };  // H1(222.8K--218.5K)

//static uint16_t pwm_freq_list[SCAN_COLUMN] = { 362, 363, 364, 365, 366, 367, 368, 369 };  // H2(221K--216.8K)

//static uint16_t pwm_freq_list[SCAN_COLUMN] = { 370, 371, 372, 373, 374, 375, 376, 377 };  // H3(216.2K--212.2K)

//static uint16_t pwm_freq_list[SCAN_COLUMN] = { 432, 433, 434, 435, 436, 437, 438, 439 };  // H4(185.2K--182.2K)

//static uint16_t pwm_freq_list[SCAN_COLUMN] = { 406, 407, 408, 409, 410, 411, 412, 413 };  // H5-H6(197.0K--193.7K)

//static uint16_t pwm_freq_list[SCAN_COLUMN] = { 405, 406, 407, 408, 409, 410, 411, 412 };    // H7-H8-197K (198K  --194.6K)

//static uint16_t pwm_freq_list[SCAN_COLUMN] = { 410, 411, 412, 413, 414, 415, 416, 417 };  // 194K (195K -- 191.8K)

//static uint16_t pwm_freq_list[SCAN_COLUMN] = { 361, 362, 363, 364, 365, 366, 367, 368 };  // H1-221K     (220.6K--217.4K)
//static uint16_t pwm_freq_list[SCAN_COLUMN] = { 359, 360, 361, 362, 363, 364, 365, 366 };  // H1-222.2K   (222.8K--218.5K)
//static uint16_t pwm_freq_list[SCAN_COLUMN] = { 364, 365, 366, 367, 368, 369, 370, 371 };  // H2-219K     (219.8K--215.6K)
//static uint16_t pwm_freq_list[SCAN_COLUMN] = { 362, 363, 364, 365, 366, 367, 368, 369 };  // H2-220K     (221K--216.8K)
//static uint16_t pwm_freq_list[SCAN_COLUMN] = { 371, 372, 373, 374, 375, 376, 377, 378 };  // H3-214.75K  (215.6K--211.6K)
//static uint16_t pwm_freq_list[SCAN_COLUMN] = { 370, 371, 372, 373, 374, 375, 376, 377 };  // H3-215.5K   (216.2K--212.2K)
//static uint16_t pwm_freq_list[SCAN_COLUMN] = { 435, 436, 437, 438, 439, 440, 441, 442 };  // H4-183.35K  (183.9K--181K)
//static uint16_t pwm_freq_list[SCAN_COLUMN] = { 432, 433, 434, 435, 436, 437, 438, 439 };  // H4-184.5K   (185.2K--182.2K)
//static uint16_t pwm_freq_list[SCAN_COLUMN] = { 408, 409, 410, 411, 412, 413, 414, 415 };  // H5-195.5K   (196.1K--192.8K)
//static uint16_t pwm_freq_list[SCAN_COLUMN] = { 406, 407, 408, 409, 410, 411, 412, 413 };  // H5-196.5K   (197.0K--193.7K)
//static uint16_t pwm_freq_list[SCAN_COLUMN] = { 408, 409, 410, 411, 412, 413, 414, 415 };  // H6-195.4K   (196.1K--192.8K)
//static uint16_t pwm_freq_list[SCAN_COLUMN] = { 404, 405, 406, 407, 408, 409, 410, 411 };  // H7-H8-197K  (198K  --194.6K)

static uint8_t pwm_mfreq_idx = 0;
static uint16_t pwm_adc_max;
static uint16_t pwm_adc_value;
static uint8_t pwm_list_offset = 0;
static uint8_t spray_cnt = 0;
static uint8_t flag_start_scan = 0;       //已经开始扫频标志
static uint32_t sdp_value_max;            //喷雾过程中 最大压力值
static uint16_t sdp_value;
static uint16_t bat_adc = 0;
static uint8_t bat_vol = 0;
static uint8_t bat_flag = BAT_CHARGE_OFF;
static uint8_t bat_cnt = 0;
static uint8_t bat_led_flash = 0;

static uint8_t sw_status = 0;
static uint8_t fg_stop_spray;
uint8_t spray_pwm_cnt;             //喷雾占空比计数值

static uint8_t eventActive = pdFALSE;

#define QUEUE_SIZE            (uint32_t)5
osMessageQId app_Queue;

/***********************************************************************************
* EXTERNAL VARIABLES
*/
extern volatile uint8_t adc_state;

extern IWDG_HandleTypeDef IwdgHandle;
extern UART_HandleTypeDef Uart1Handle;
extern ITStatus UartReady;
extern uint8_t aTxBuffer[];
extern uint8_t aRxBuffer[];
/***********************************************************************************
* GLOBAL FUNCTIONS
*/
extern uint8_t uartTest( void );

/***********************************************************************************
* LOCAL FUNCTIONS
*/
static void KeyTask_Process( void );

static uint8_t state_spray_ready_init( void );
static uint8_t state_spray_active_init( void );

static void power_off( void );
static void power_on( uint8_t type );
void LedTask_Process( void );
void ChargeTask_Process( void );
void BatTask_Process( void );
void SprayTask_Process( void );
void cart_Detect( void );
static void CartTask_Process( void );

uint8_t uartTask_Process( void );

void app_task_Init( void );
void app_task_Callback( void );

/***********************************************************************************
*  OS Queue and Task
*/
#if 0
void app_Queue_Init(void)
{
	osMessageQDef(app_Queue, QUEUE_SIZE, uint16_t);
	app_Queue = osMessageCreate (osMessageQ(app_Queue), NULL);
}

void app_Queue_Send(uint32_t d)
{
	osMessagePut ( app_Queue, d, 0 );
}

int app_Queue_Receive(void)
{
	osEvent event;
	int state;
	event = osMessageGet( app_Queue, osWaitForever );
	if( event.status == osEventMessage )
	{
		state = event.value.v;
		return state;
	}
	return 0;
}
#endif


void appThread(void * argument)
{

	HalSdp_Init();
//	app_Queue_Init();
	app_task_Init();

	app_task_enable = pdTRUE;      // 20210129  pdFALSE;

	//while(1)  delay_ms(10);

	while(1)
	{
		if ( eventActive == pdTRUE ){
			eventActive = pdFALSE;
			app_task_Callback();
		}
#if 0
		if ( app_task_enable == pdTRUE ){
			if ( bat_flag == BAT_CHARGE_OFF ){
				if ( fp_task_suspend == true ){
					fp_task_suspend = false;
					vTaskResume( fingerprintHandle );
					vTaskResume( periHandle );
				}
			}
			else {
				if ( fp_task_suspend == false ){
					fp_task_suspend = true;
					vTaskSuspend( fingerprintHandle );
					vTaskSuspend( periHandle );
				}
			}
		}
#endif
//		LL_mDelay(2);
	}
}

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

/***********************************************************************************
*  @fn     
*  @brief  Set the environment variables, check the cartridge information
*  @param      
*            
*  @return   none
*/
void app_task_Init( void )
{   
	int i = 0;
	int j = -6;
	for( i = 0; i < SCAN_COLUMN; i++ )
	{
		pwm_freq_list[i] = RES_FREQ + j;
		j++;
	}
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
#if TEST
    eventActive = pdTRUE;
#else
    eventActive = pdFALSE;
#endif
    wakeup_type |= WAKEUP_ON_KEY;

    memset( (uint8_t *)led_state, 0x00, sizeof(led_state) );

    power_on(wakeup_type);
    spray_ready_time_total = IDLE_TIMEOUT + 20;
    
    s_spray_state = STATE_SPRAY_READY;
    sw_status = KEY_POWER_ON;     // KEY_SPRESS;
    
    // Read flash, Init parameters
    p_user_info = &user_info;
    p_user_info->total_cnt = 100;       //强制将剩余次数置为最大
	p_user_info->used_cnt = 0;
	p_user_info->unused_pct = 100;
    
	cart_Detect();

//    BT_AB_OFF();

	HalSdp_Init();
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
		pApp_Task->task_charge.interval = 75;
		ChargeTask_Process();
	}

	if ( pApp_Task->task_battery.event & TASK_ACTIVE_EVT ){
		pApp_Task->task_battery.event ^= TASK_ACTIVE_EVT;
		pApp_Task->task_battery.interval = 1000;
		BatTask_Process();
	}

	if ( pApp_Task->task_led.event & TASK_ACTIVE_EVT ){
		pApp_Task->task_led.event ^= TASK_ACTIVE_EVT;
		LedTask_Process();
	}

	if ( pApp_Task->task_key.event & TASK_ACTIVE_EVT ){
		pApp_Task->task_key.event ^= TASK_ACTIVE_EVT;
		pApp_Task->task_key.interval = 10;
		KeyTask_Process();
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
		//uartTest();
		//uartTask_Process();
	}
	#if WTD_ENABLE
    // Refresh IWDG: reload counter
    if(HAL_IWDG_Refresh(&IwdgHandle) != HAL_OK){
    	//Error_Handler();
    }
#endif

}

void KeyTask_Process( void )
{
    uint8_t key;
    
     if ( lt_en_time ) {
      return;                  //喷雾期间不响应按键功能
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
#if PROG_TYPE == 3
    	queued_shots = AMOUNT_OF_SHOTS;
#else
        power_off();
#endif
    }
}

void ChargeTask_Process( void )
{
    uint8_t vbus_pin, charge_pin;
    uint8_t bat_flag_bk;
    
    bat_flag_bk = bat_flag;
    bat_flag = BAT_CHARGE_OFF;
    
    if ( HAL_GPIO_ReadPin(VBUS_PORT, VBUS_PIN) == GPIO_PIN_SET ){
        LL_mDelay(5);
        vbus_pin = HAL_GPIO_ReadPin( VBUS_PORT, VBUS_PIN );
//		LT_SHNDA_OFF();
//        VBAT_CN_OFF();
        charge_pin = HAL_GPIO_ReadPin( CHARGE_PORT, CHARGE_PIN );
        if ( vbus_pin == GPIO_PIN_SET ){
            pApp_Task->task_power_off.interval = 0;
            pApp_Task->task_power_off.event = 0;
            if ( charge_pin == GPIO_PIN_RESET ){
                bat_flag = BAT_CHARGE_ON;
            }
            else{
                bat_flag = BAT_CHARGE_FULL;
            }
            
//            lt_en_time = 0;
//            PWM_Stop( TIM_CHANNEL_1 );
//            s_spray_state = STATE_SYSTEM_WAIT;
//            GPIO_setOutputHighOnPin( LED5_PORT, LED5_PIN );
//            GPIO_setOutputHighOnPin( LED6_PORT, LED6_PIN );
//            GPIO_setOutputHighOnPin( LED7_PORT, LED7_PIN );
//            GPIO_setOutputHighOnPin( LED8_PORT, LED8_PIN );
        }
        else {
            //s_spray_state = STATE_SPRAY_READY;
        }
    }
    
    if ( bat_flag != bat_flag_bk ){
        if ( bat_flag != BAT_CHARGE_OFF ){
            pApp_Task->task_led.interval = 5;
            pApp_Task->task_led.event |= LED_STATE_UPDATE_EVT;
        }
        else{
            power_off();
        }
    }
    
    if ( bat_flag == BAT_CHARGE_ON ){
        pApp_Task->task_charge.interval = 250;
        
        if ( ++bat_led_flash > 5 ){
            bat_led_flash = 1;
        }
        switch ( bat_led_flash ){
            case 1:
                 GPIO_setOutputHighOnPin( LED1R_PORT, LED1R_PIN );
                 GPIO_setOutputHighOnPin( LED1G_PORT, LED1G_PIN );
                 GPIO_setOutputHighOnPin( LED1B_PORT, LED1B_PIN );
                 GPIO_setOutputHighOnPin( LED2_PORT, LED2_PIN );
                 GPIO_setOutputHighOnPin( LED3_PORT, LED3_PIN );
                 GPIO_setOutputHighOnPin( LED4_PORT, LED4_PIN );
                 
                 GPIO_setOutputLowOnPin( LED1G_PORT, LED1G_PIN );
                 break;
            case 2:
                 GPIO_setOutputLowOnPin( LED2_PORT, LED2_PIN );
                 break;
            case 3:
                 GPIO_setOutputLowOnPin( LED3_PORT, LED3_PIN );
                 break;
            case 4:
                 GPIO_setOutputLowOnPin( LED4_PORT, LED4_PIN );
                 break;
            default:
                 break;
        }
    }
    else if ( bat_flag == BAT_CHARGE_FULL ){
        bat_led_flash = 0;
        GPIO_setOutputHighOnPin( LED1R_PORT, LED1R_PIN );
        GPIO_setOutputHighOnPin( LED1B_PORT, LED1B_PIN );
        GPIO_setOutputLowOnPin( LED1G_PORT, LED1G_PIN );
        GPIO_setOutputLowOnPin( LED2_PORT, LED2_PIN );
        GPIO_setOutputLowOnPin( LED3_PORT, LED3_PIN );
        GPIO_setOutputLowOnPin( LED4_PORT, LED4_PIN );
    }

}

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
    // Test
	//bat_vol = BAT_VOL_FULL;
    if ( bat_vol_bk == bat_vol ){
        bat_cnt = 0;
    }
    else{
        if ( (++bat_cnt < 3) && (bat_vol_bk != 0) ){
            bat_vol = bat_vol_bk;
            return;
        }
        bat_cnt = 0;
        
        if ( bat_flag != BAT_CHARGE_OFF ){
            pApp_Task->task_battery.interval = 2500;
            return;
        }
        
        // All off
        GPIO_setOutputHighOnPin( LED1R_PORT, LED1R_PIN );
        GPIO_setOutputHighOnPin( LED1G_PORT, LED1G_PIN );
        GPIO_setOutputHighOnPin( LED1B_PORT, LED1B_PIN );
        GPIO_setOutputHighOnPin( LED2_PORT, LED2_PIN );
        GPIO_setOutputHighOnPin( LED3_PORT, LED3_PIN );
        GPIO_setOutputHighOnPin( LED4_PORT, LED4_PIN );
        
        switch( bat_vol ){
            case BAT_VOL_FULL:
                 // Green-1/2/3/4
                 GPIO_setOutputLowOnPin( LED1G_PORT, LED1G_PIN );
                 GPIO_setOutputLowOnPin( LED2_PORT, LED2_PIN );
                 GPIO_setOutputLowOnPin( LED3_PORT, LED3_PIN );
                 GPIO_setOutputLowOnPin( LED4_PORT, LED4_PIN );
                 break;
            case BAT_VOL_NORMAL:
                 // Green-1/2/3
                 GPIO_setOutputLowOnPin( LED1G_PORT, LED1G_PIN );
                 GPIO_setOutputLowOnPin( LED2_PORT, LED2_PIN );
                 GPIO_setOutputLowOnPin( LED3_PORT, LED3_PIN );
                 break;
            case BAT_VOL_LOW:
                 // Green-1/2
                 GPIO_setOutputLowOnPin( LED1G_PORT, LED1G_PIN );
                 GPIO_setOutputLowOnPin( LED2_PORT, LED2_PIN );
                 break;
            case BAT_VOL_BAD:
                 // Yellow-1
                 GPIO_setOutputLowOnPin( LED1R_PORT, LED1R_PIN );
                 GPIO_setOutputLowOnPin( LED1G_PORT, LED1G_PIN );
                 break;
            case BAT_VOL_OFF:
                 // Red-1
                 GPIO_setOutputLowOnPin( LED1R_PORT, LED1R_PIN );
                 break;
            default:
                 break;
        }
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
    percent = p_user_info->unused_pct;
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
    
    if ( p_bt_state == NULL ){
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
    switch( s_spray_state )
    {
        case STATE_SYSTEM_WAIT:
             //if ( (bat_flag == BAT_CHARGE_OFF) && (bat_vol != BAT_VOL_BAD) ){
        	if ( bat_flag == BAT_CHARGE_OFF ){
                 s_spray_state = STATE_SPRAY_READY;
             }
             break;
             
        case STATE_SYSTEM_CHECK:
             break;
             
        case STATE_SPRAY_READY:
             if ( state_spray_ready_init() ){
                 if ( (p_bt_state == NULL) || (p_user_info->unused_pct == 0) ){
                     if ( pApp_Task->task_led.param1 == 0 ){
                         pApp_Task->task_led.interval = 2;
                         pApp_Task->task_led.event = LED_STATE_BLINK_EVT;
                         pApp_Task->task_led.param1 = 5;
                         pApp_Task->task_led.param2 = 100;        // 100*2ms
                     }
                 }
                 else {
                     if ( sdp_thr++ > 0 ){
                         sdp_thr = 0;
                         pApp_Task->task_led.interval = 2;
                         pApp_Task->task_led.event = LED_STATE_BLINK_EVT;
                         
                         //if ( p_user_info->used_cnt < p_user_info->total_cnt ){                  // 喷雾次数暂时不用
                         if ( 1 ){
                             pApp_Task->task_led.param1 = 2;
                             pApp_Task->task_led.param2 = 150;        // 100*2ms

                             pApp_Task->task_power_off.interval = 0;
                             pApp_Task->task_power_off.event = 0;
                             s_spray_state = STATE_SPRAY_ACTIVE_DLY;
                             spray_intv = SPRAY_DELAY;
                             flag_start_scan=0;
                             lt_en_time = 0;
                             // if ( spray_triger == TRIGER_SDP ){
                             HalSdp_Start_Measure();
                             //}
                         }
                         else{
                             pApp_Task->task_led.param1 = 7;
                             pApp_Task->task_led.param2 = 300;

                             pApp_Task->task_power_off.interval = 0;
                             pApp_Task->task_power_off.event = 0;
                             s_spray_state = STATE_SPRAY_READY;
                             spray_intv = 1000;
                             p_user_info->unused_pct = 0;
#if TEST
                             p_user_info->unused_pct = UNUSED;
#endif
                         }
                     }
                     else{
                         spray_intv = 50;
                     }
                 }
             }
             else{
                 if ( ++spray_ready_time > spray_ready_time_total ){
                     s_spray_state = STATE_SPRAY_READY_TIMEOUT;
                     spray_intv = 5;
                 }
                 else{
                     spray_intv = 50;                           //如果无压力触发，则0.1计数一次待机时间
                 }
             }
             break;

        case STATE_SPRAY_ACTIVE_DLY:
             spray_cnt = 0;
             if ( spray_cnt == 0 ){
                 if(flag_start_scan == 0)
                 {
                    flag_start_scan = 1;
                    PWM_Stop( TIM_CHANNEL_1 );
                    PWM_Setup( PWM_SCAN_FOSC_INI - 1 );
                    PWM_Start( TIM_CHANNEL_1 );
                    LT_SHNDA_ON();
                    spray_intv = SCAN_DLY_1;
                 }                                    
                 else                                
                 {
                     flag_start_scan=0;
                     pwm_list_offset = 0;
                     pwm_adc_max = 0;
                     pwm_mfreq_idx = 0;
                     PWM_Stop( TIM_CHANNEL_1 );
                     PWM_Setup( pwm_freq_list[0] - 1 );                          //输入首个扫频频率值
					 PWM_Start( TIM_CHANNEL_1 );
                     s_spray_state = STATE_SPRAY_SCAN;
                     spray_pwm_cnt = 0;                                          //clear pwm duty count

                     lt_en_time = sys_ticket + ON_TIME;
                     if( spray_triger == TRIGER_SDP )
                     {
                         lt_en_time = sys_ticket + BREATH_ON_TIME;              //呼吸触发时间限制
                     }
                     fg_stop_spray = 0;                                         //#### clear fg_stop_spray
                     spray_intv = SCAN_DLY_2;
                 }
             }
             else{
             	 PWM_Stop( TIM_CHANNEL_1 );
                 PWM_Setup( pwm_freq_list[pwm_mfreq_idx] - 1 );
				 PWM_Start( TIM_CHANNEL_1 );
            	 LT_SHNDA_ON();                                         //开启工作
				 s_spray_state = STATE_SPRAY_ACTIVE;                          //后续再次触发,直接按扫频结果喷雾
				 spray_intv = 50;
				 if( spray_triger == TRIGER_SDP ){
					 lt_en_time = sys_ticket + p_user_info->per_sdp_time;
				  }
				  else if ( spray_triger == TRIGER_KEY ){
					  lt_en_time = sys_ticket + p_user_info->per_key_time;
				  }
             }
             sdp_thr = 0;
             break;

        case STATE_SPRAY_SCAN:
        	 spray_pwm_cnt = 0;
             spray_intv = SCAN_DLY_2;
             ADC_Start( ADC_CUT_CH );
             while( adc_state == ADC_BUSY );
             pwm_adc_value = ADC_Average( ADC_CUT_CH );
        	 if(pwm_adc_value > pwm_adc_max)
        	 {
        		pwm_adc_max = pwm_adc_value;
        		pwm_mfreq_idx = pwm_list_offset;
        	 }
             pwm_list_offset++;
             if ( pwm_list_offset < SCAN_CNT )
             {
                 PWM_Stop( TIM_CHANNEL_1 );
				 PWM_Setup( pwm_freq_list[pwm_list_offset] - 1 );
				 PWM_Start( TIM_CHANNEL_1 );
             }
             else
             {
                 PWM_Stop( TIM_CHANNEL_1 );
				 PWM_Setup( pwm_freq_list[pwm_mfreq_idx] - 1 );
				 PWM_Start( TIM_CHANNEL_1 );

                 if ( spray_triger == TRIGER_SDP ){
                     spray_intv = 25;
                     s_spray_state = STATE_SPRAY_INHALATION;
                 }
                 else{
                	 spray_intv = abs(lt_en_time - sys_ticket) + 10;
                	 s_spray_state = STATE_SPRAY_ACTIVE;
                 }
             }
             break;
             
        case STATE_SPRAY_READY_TIMEOUT:
             if ( bat_flag != BAT_CHARGE_OFF ){
                 spray_ready_time = 0;
                 s_spray_state = STATE_SPRAY_READY;
             }
             power_off();
             break;

        case STATE_SPRAY_INHALATION:
             spray_intv = 50;                                    //0.1S检测一次气压值
             sdp_value = 0;
             HalSdp_GetValue( &sdp_value, SDP_STOP_THR );
             if( sdp_value > sdp_value_max){
                 sdp_value_max = sdp_value;
             }
             if( sdp_value < sdp_value_max*STOP_PCT/100 )
             {
                     if(++sdp_thr >= SDP_OFF_CNT)
                     {
                           sdp_thr = 0;
                           LT_SHNDA_OFF();
                           lt_en_time = 0;
                           s_spray_state = STATE_SPRAY_ACTIVE;
                           if(fg_stop_spray == 0)                                          //####
                           {
                               fg_stop_spray = 3;                                          // #### spray is complete
                               pApp_Task->task_led.interval = 2;                           // ####
                               pApp_Task->task_led.event = LED_STATE_BLINK_EVT;            // #### 新增结束喷雾是闪烁LED 2次
                               pApp_Task->task_led.param1 = 4;                             // ####
                               pApp_Task->task_led.param2 = 70;                            // #### 50*2ms
                           }
                     }
             }
             break;

        case STATE_SPRAY_ACTIVE:
             s_spray_state = STATE_DOSE_COMPLETE;
             spray_intv = OFF_TIME;
             break;
             
        case STATE_DOSE_COMPLETE:

             HalSdp_Stop_Measure();
             spray_triger = 0;
             spray_intv = 50;
             s_spray_state = STATE_SPRAY_READY; 
             spray_ready_time = 0;
             pApp_Task->task_led.interval = 10;
             pApp_Task->task_led.event |= LED_STATE_UPDATE_EVT;
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

    value = HalSdp_GetThr();
#if PROG_TYPE != 1 && PROG_TYPE != 3
    if ( value > PRESSURE_THRESHOLD ){
        spray_triger = TRIGER_SDP;
        rst = true;
        sdp_value_max = value;
    }
#endif
#if PROG_TYPE != 2
    //if ( sw_status == KEY_SPRESS ){
    if ( spray_key_en ){
        spray_key_en = false;
        sdp_thr = 2;
        spray_triger = TRIGER_KEY;
        rst = true;
    }
#endif
#if PROG_TYPE == 3
    if (queued_shots > 1){
        spray_key_en = false;
        sdp_thr = 2;
        spray_triger = TRIGER_KEY;
        rst = true;
        queued_shots--;
    }
#endif
    return rst;
}

static uint8_t state_spray_active_init( void )
{
    int8_t rst = false;
    uint16_t value;
    
    if ( spray_triger == TRIGER_SDP ){
        HalSdp_GetValue( &value, SDP_STOP_THR );
        if ( value <= SDP_STOP_THR ){
            rst = true;
        }
    }
    else{
        sdp_thr = 2;
        rst = true;
    }
    
    return rst;
}

static void CartTask_Process( void )
{
    cartidge_attr_t *state_bk = p_bt_state;
#if 1        // Test,默认为一直有药瓶模式
    return;
#endif
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
                    p_user_info->unused_pct = UNUSED;
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
        //if ( (p_bt_state != NULL) && (p_user_info->used_time < p_user_info->total_time) ){   // 会导致最后一次喷雾时间无法写入芯片
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
}

void cart_Detect( void )
{
#if 1    // Test  屏蔽药瓶检测，默认为一直有药瓶
    p_bt_state = p_cart_attr;
	p_user_info->total_cnt = p_cart_attr->total_eje_dose = 200;
	p_user_info->used_cnt = p_cart_attr->used_eje_dose = 0;
	p_user_info->total_time = p_cart_attr->total_eje_time = 3000000;
	p_user_info->used_time = p_cart_attr->used_eje_time = 0;
	p_user_info->per_sdp_time = p_cart_attr->per_sdp_time = 10000;
	p_user_info->per_key_time = p_cart_attr->per_key_time = ON_TIME;
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
            p_user_info->unused_pct = UNUSED;
#endif
        }
        Hal_I2C_Release( SHA204A );
    }
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
}

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

/******************************************************************************/


/***********************************************************************************/


/***********************************************************************************
*  @fn     
*  @brief  Update system ticket every for all tasks
*  @param      
*            
*  @return   none
*/
void task_ticket_update( void )
{    
    uint32_t time;
    uint8_t eventFlag = false;

	sys_ticket_bool ^= 1;
    if ( (sys_ticket_bool) || (app_task_enable == pdFALSE) ){
        return;
    }
	
    sys_ticket++;
    wdt_ticket = 0;
    if ( lt_en_time )
     {
         if ( sys_ticket >= lt_en_time )
         {
             LT_SHNDA_OFF();
             if(fg_stop_spray == 0)
             {
                 fg_stop_spray = 3;
                 pApp_Task->task_led.interval = 2;
                 pApp_Task->task_led.event = LED_STATE_BLINK_EVT;
                 pApp_Task->task_led.param1 = 4;
                 pApp_Task->task_led.param2 = 70;
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
        	pApp_Task->task_key.interval = 10;
			if ( lt_en_time == 0 ) {
				if ( key_value == KEY_RLS ){
					key_value = halKeyScan();
				}
			}
        }
    }
    
    if ( pApp_Task->task_led.interval > 0 ){
        pApp_Task->task_led.interval--;
        if ( pApp_Task->task_led.interval == 1 ){
             pApp_Task->task_led.event |= TASK_ACTIVE_EVT;
             eventFlag = true;
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

    if ( eventFlag ){
    	//app_Queue_Send( EVENT_APP_TASK_ACTIVE );
    	eventActive = pdTRUE;
    }

//    app_timer = portCONVERT_TICKS_2_MS(xTickCount);
}
