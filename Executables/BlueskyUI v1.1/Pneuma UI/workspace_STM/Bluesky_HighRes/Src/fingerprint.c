/**
  ******************************************************************************
  * @file           : fingerprint.c
  * @brief          : Related to fingerprint function
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 Crucialtec.
  * All rights reserved.</center></h2>
  *
  *
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "fingerprint.h"
#include "libbarley.h"
#include "main.h"

#include "hal_board_cfg.h"


// external variables
extern PRIVILEGED_DATA volatile uint32_t app_task_enable;

/* Private variables ---------------------------------------------------------*/
static BARLEY_Mode g_cur_mode ; /// Mode used by fingerprint thread

/* Private function prototypes -----------------------------------------------*/

static void prv_send_event(int event);
static void prv_process_result(int event);
static int prv_enable_machine(void);
static int prv_power_down(void);
static int prv_standby(BARLEY_CONTEXT* context);
static int prv_charge_mode(BARLEY_CONTEXT* context);
void prv_set_next_mode(int mode);
/**
 * @brief  This is fingerprint thread.
 * @param  argument: pointer that is passed to the thread function as start argument.(unused)
 */
void fingerprintThread(void  * argument)
{
	int result = -1;
	BARLEY_CONTEXT *context = NULL;
	GPIO_PinState pin_vin1, pin_vin2;

	context = lib_fp_init();
	if(context == NULL ){
		g_cur_mode = MODE_ERROR;
	}else{
		g_cur_mode = MODE_STANDBY;
	}

	while(1)
	{
#ifdef IMG_CAP_MODE
		g_cur_mode = MODE_CAPTURE_IMG;
#endif

		result = -1;
		switch(g_cur_mode)
		{
			case MODE_STANDBY:
				 result = prv_standby(context);
				 break;
			case MODE_ENROL:
				 result = lib_fp_enroll(context);
				 break;
			case MODE_VERIFY:
				 result = lib_fp_verify(context);
				 break;
			case MODE_RESET:
				 result = lib_fp_reset(context);
				 break;
			case MODE_CAPTURE_IMG:
				 result = lib_wait_fp_and_capture(context);
				 break;
			case MODE_PWR_ON_STATE:
				 result = prv_enable_machine();
				 break;
			case MODE_POWER_OFF:
				 result = prv_power_down();
				 break;
			case MODE_WAIT_FP_UP:
				 result = lib_wait_finger(context);
				 break;
			case MODE_5VIN:
				 result = prv_charge_mode(context);
				 app_task_enable = pdTRUE;                   // MY
				 break;

			default:
				 break;
		}
		prv_process_result(result);
		pin_vin1 = HAL_GPIO_ReadPin(VBUS_PORT, VBUS_PIN);
		API_Delay(10);
		pin_vin2 = HAL_GPIO_ReadPin(VBUS_PORT, VBUS_PIN);
		// MY
		if ( (pin_vin1 && pin_vin2) && (app_task_enable != pdTRUE) ){
			g_cur_mode = MODE_5VIN;
			BARLEY_UserCancel();
		}
	}
}



/**
 * @brief Standby mode.
 * @param context library context.
 * @return
 */
static int prv_standby(BARLEY_CONTEXT* context){

	if ( HAL_GPIO_ReadPin(VBUS_PORT , VBUS_PIN) == GPIO_PIN_SET){
		return EVENT_ETC_5VIN;
	}else{
		return EVENT_STANDBY;
	}
	return EVENT_STANDBY;
}

/**
 * @brief If USB power is applied.
 * @param context library context.
 * @return
 */
static int prv_charge_mode(BARLEY_CONTEXT* context){
	BARLEY_deinit(context);
	return EVENT_ETC_5VIN;
}


/**
 * @brief  Power down mode. Set BAT_CN pin low
 * @return EVENT_NONE
 */
static int prv_power_down(void){

	if ( app_task_enable == pdFALSE ){
		HAL_GPIO_WritePin(VBAT_CN_PORT, VBAT_CN_PIN, GPIO_PIN_RESET);
	}
	return EVENT_NONE;
}


/**
 *	@brief If fingerprint regcognition is success,
 * @return
 */
static int prv_enable_machine(void){
	int result = EVENT_ETC_POWER_OFF;

	API_Delay(10000);

	return result;
}
/**
 * @brief Determine the next mode according to the event that occurred.
 * @param result
 */
static void prv_determine_next_mode(int event){

	switch(event){
	case EVENT_ENROL_FULL:
	case EVENT_ETC_SHORTTOUCH:
	case EVENT_TPL_RESET_COMPLETE:
		prv_set_next_mode(MODE_STANDBY);
		break;
	case EVENT_ENROL_COMPLETE:
		prv_set_next_mode(MODE_WAIT_FP_UP);
		break;
	case EVENT_STANDBY:
		prv_set_next_mode(MODE_CAPTURE_IMG);
		break;
	case EVENT_ENROL_FIRST:
		prv_set_next_mode(MODE_WAIT_FP_UP);
		break;
	case EVENT_SENSOR_CAPTURE_COMPLETE:
		prv_set_next_mode(MODE_VERIFY);
		break;
	case EVENT_ETC_LONGTOUCH_SECTION_1:
	case EVENT_ENROL_SUCCESS:
		prv_set_next_mode(MODE_ENROL);
		break;
	case EVENT_VERIFY_SUCCESS:
		prv_set_next_mode(MODE_PWR_ON_STATE);
		break;
	case EVENT_ETC_LONGTOUCH_SECTION_2:
		prv_set_next_mode(MODE_RESET);
		break;
	case EVENT_VERIFY_FAIL:
		prv_set_next_mode(MODE_WAIT_FP_UP);
		break;
	case EVENT_VERIFY_FAIL_TOO_MANY_ATTEMPTS:
	case EVENT_TIME_OUT:
	case EVENT_ETC_5VIN:
	case EVENT_ETC_POWER_OFF:
		prv_set_next_mode(MODE_POWER_OFF);
		break;
	default:
		break;
	}
	return;
}

/**
 * @brief       Process the event. Decide the next mode and send the event.
 * @param[in]   event  The event.
 */
static void prv_process_result(int event){
	if ( event < 0 ) {
		return;
	}
	prv_send_event(event);
	prv_determine_next_mode(event);

	return;
}
/**
 * @brief Send a event to the led thread using a queue.
 * @param event Event to be sent
 */
static void prv_send_event(int proc)
{
	API_Queue_Send( proc );
}

/**
 * @brief Sets the next mode of the fingerprint thread.
 * @param mode Next mode to be set
 */
void prv_set_next_mode(int mode){
	g_cur_mode = mode;
}



/**
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == VBUS_PIN)
  {
	  prv_set_next_mode(MODE_5VIN);
	  BARLEY_UserCancel();
  }
}




/******************************** END OF FILE *********************************/
