
#include "portmacro.h"
#include "platform_api.h"
#include "led_thread.h"
#include "hal_board_cfg.h"

#define ENROLL_BLINK_NUM  3
#define ENROLL_DELAY 100

// external variables
extern PRIVILEGED_DATA volatile uint32_t app_task_enable;

GPIO_TypeDef* GPIO_PORT[10] = {LED1R_PORT, LED1G_PORT , LED1B_PORT,
		LED2_PORT,LED3_PORT,LED4_PORT,LED5_PORT,LED6_PORT,LED7_PORT,LED8_PORT};

const uint16_t GPIO_PIN[10] = { LED1R_PIN , LED1G_PIN , LED1B_PIN,
		LED2_PIN, LED3_PIN, LED4_PIN, LED5_PIN, LED6_PIN, LED7_PIN, LED8_PIN};


static int prv_LED_standby(void);
static int prv_LED_enroll_start(void);
static int prv_LED_enroll_success(void);
static int prv_LED_enroll_complete(void);
static int prv_LED_verify_success(void);
static int prv_LED_all_off(void);
static int prv_LED_reset_complete(void);
static void prv_LED_5vin(void);


static int enroll_cnt = 0;


/**
 * @brief       Run the thread that control Peripheral.
 */
void periThread(void * argument)
{

	int state = 0;
	int prev_state= 0;

	while(1)
	{

		state = API_Queue_Receive();

		if ( state != 0 ){
			prev_state = 0;
		}else{
			state = prev_state;
		}

		switch(state)
		{
		case EVENT_STANDBY:
		case EVENT_SENSOR_CAPTURE_COMPLETE:
		case EVENT_VERIFY_FAIL:
		case EVENT_ETC_SHORTTOUCH:
			prv_LED_standby();
			prev_state = EVENT_STANDBY;
			break;
		case EVENT_ETC_LONGTOUCH_SECTION_1:
			prv_LED_enroll_start();
			break;
		case EVENT_ENROL_SUCCESS:
			prv_LED_enroll_success();
			break;
		case EVENT_ENROL_COMPLETE:
			prv_LED_enroll_complete();
			break;
		case EVENT_VERIFY_SUCCESS:
			prv_LED_verify_success();
			app_task_enable = pdTRUE;           // MY
			break;
		case EVENT_TPL_RESET_COMPLETE:
			prv_LED_reset_complete();
			break;
		case EVENT_ETC_5VIN:
			prv_LED_5vin();
			break;
		case EVENT_ETC_POWER_OFF:
			prv_LED_all_off();
			VBAT_CN_OFF();
			while(1);
			break;
		case EVENT_TIME_OUT:
		case EVENT_UNKNOWN_ERROR:
		case EVENT_USER_CANCEL:
		case EVENT_ENROL_FAIL:
		case EVENT_NONE:
		default:
			API_Delay(1);
			break;
		}
	}
}

static void prv_LED_5vin(void){
	prv_LED_all_off();
	LED_On(LED1G);
	for(int i = LED2 ; i < LED5 ; i++){
			LED_On(i);
	}
}


/**
 * @brief       This function represent a state of LED when reset is complete.
 * @return
 */
static int prv_LED_reset_complete(){
	int ret = 0;

	LED_Off(LED1R);
	for(int i = LED2 ; i < LED_MAX ; i++){
		LED_Off(i);
	}

	for (int i = 0 ; i < (ENROLL_BLINK_NUM*2); i ++){
		LED_Toggle(LED1R);
		for(int i = LED2 ; i < LED_MAX ; i++){
			LED_Toggle(i);
		}

		vTaskDelay(ENROLL_DELAY);
	}
	return ret;
}


/**
 * @brief       This function represent a state of LED when enrollment mode start.
 * @return
 */
static int prv_LED_standby(void)
{
	int ret = 0;
	// 10% brightness.
	LED_On(LED1G);
	for( int i =LED2 ; i < LED_MAX ; i++){
		LED_On(i);
	}
	vTaskDelay(1);
	LED_Off(LED1G);
	for( int i =LED2 ; i < LED_MAX ; i++){
		LED_Off(i);
	}
	vTaskDelay(9);

	return ret;
}


/**
 * @brief       This function represent a state of LED when enrollment mode start.
 * @return
 */
static int prv_LED_enroll_start(void)
{
	int ret = 0;
	enroll_cnt=0;
	LED_On(LED1R);
	LED_On(LED1G);
	LED_Off(LED1B);
	for( int i =LED2 ; i < LED_MAX ; i++){
		LED_On(i);
	}

	return ret;
}


/**
 * @brief       This function represent a state of LED when enrollment is success.
 * @return
 */
static int prv_LED_enroll_success(void)
{
	int ret = 0;
	enroll_cnt++;
	LED_Off(LED_MAX-enroll_cnt);


	return ret;
}

/**
 * @brief       This function represent a state of LED when enrollment is complete.
 * @return
 */
static int prv_LED_enroll_complete(void)
{
	int ret = 0;

	LED_Off(LED1G);
	LED_Off(LED1R);
	LED_Off(LED1B);
	for(int i = LED2 ; i < LED_MAX ; i++){
		LED_Off(i);
	}

	for (int i = 0 ; i < (ENROLL_BLINK_NUM*2); i ++){
		LED_Toggle(LED1G);
		for(int i = LED2 ; i < LED_MAX ; i++){
			LED_Toggle(i);
		}

		API_Delay(ENROLL_DELAY);
	}


	return ret;
}

/**
 * @brief       This function represent a state of LED when verification is success.
 * @return
 */
static int prv_LED_verify_success(void)
{

	int ret = 0;

	LED_On(LED1G);
	LED_Off(LED1R);
	LED_Off(LED1B);
	for(int i = LED2 ; i < LED_MAX ; i++){
		LED_On(i);
	}


	return ret;
}

/**
 * @brief       This function represent a state of LED when enrollment or verification is fail.
 * @return
 */
static int prv_LED_all_off(void)
{

	int ret = 0;
	if ( app_task_enable == pdFALSE ){
		for( int i =0 ; i < LED_MAX ; i++){
			LED_Off(i);
		}
	}
	return ret;
}

void LED_On(Led_Def Led){
	HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_RESET);
}
void LED_Off(Led_Def Led){
	HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_SET);

}
void LED_Toggle(Led_Def Led){
	HAL_GPIO_TogglePin(GPIO_PORT[Led], GPIO_PIN[Led]);
}




