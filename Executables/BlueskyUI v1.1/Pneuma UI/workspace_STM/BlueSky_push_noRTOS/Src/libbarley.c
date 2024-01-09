
#include "libbarley.h"

#define TIME_5SEC 5000
#define TIME_10SEC 10000

static PLATFORM_API platform_api =
{
		.malloc     = API_Malloc,
		.calloc     = API_Calloc,
		.free       = API_Free,
		.memset     = API_Memset,
		.memcpy     = API_Memcpy,
		.delay      = API_Delay,
		.get_tick		=API_GetTick,

};

BARLEY_CONTEXT* lib_fp_init(void){

	API_Queue_Init();

	return BARLEY_init(platform_api);

}

int lib_fp_enroll(BARLEY_CONTEXT* context){

	int index = 0 ;
	int barley_ret = -1;
	int result = 0;

	index = BARLEY_get_next_finger_index();// valid index:  0~N ,  If all indices are already enrolled, return BARLEY_ENROLLABLE_INDEX_EXCEEDED(-1)

	if ( index < 0){
		// All indices are already enrolled.
		result = EVENT_ENROL_FULL;
		goto FINISH;
	}

	barley_ret = BARLEY_wait_fingerup(context, 0);

	if( barley_ret != BARLEY_OK){
		// Handle Error.
		result = EVENT_UNKNOWN_ERROR;
		goto FINISH;
	}

	barley_ret = BARLEY_start_capture(context , 0);

	if( barley_ret != BARLEY_OK){
		// Handle Error.
		result = EVENT_UNKNOWN_ERROR;
		goto FINISH;
	}

	barley_ret = BARLEY_enroll(context,index);

	switch(barley_ret){
	case BARLEY_ENROLL_COUNT_INCREASED:
		result = EVENT_ENROL_SUCCESS;
		break;
	case BARLEY_ENROLL_FINISHED:
		result = EVENT_ENROL_COMPLETE;
		break;
	case BARLEY_USER_CANCEL:
		result = EVENT_USER_CANCEL;
		break;
	case BARLEY_TIMEOUT:
		result = EVENT_SENSOR_TIME_OUT;
		break;
	case BARLEY_ALL_SLOTS_IS_FULL:
		result = EVENT_ENROL_FULL;
		break;
	default:
		result = EVENT_UNKNOWN_ERROR;
		break;
	}

FINISH:
	return result;
}


int lib_fp_verify(BARLEY_CONTEXT* context){
	int barley_ret = 0 ;
	int result = 0;
	static int verify_fail_count = 0 ;

	barley_ret = BARLEY_verify(context , -1);

	switch (barley_ret){

	case BARLEY_VERIFY_FAIL:
		result = EVENT_VERIFY_FAIL;
		verify_fail_count++;
		break;
	case BARLEY_VERIFY_SUCCESS:{
		result = EVENT_VERIFY_SUCCESS;
		verify_fail_count = 0 ;
		break;
	}
	case BARLEY_USER_CANCEL:
		result = EVENT_USER_CANCEL;
		break;
	default:
		result = EVENT_UNKNOWN_ERROR;
		break;
	}

	if(  verify_fail_count > MAX_OF_ATTEMPTS){
		result  = EVENT_VERIFY_FAIL_TOO_MANY_ATTEMPTS;
		verify_fail_count =0;
	}


	return result;
}

int lib_fp_reset(BARLEY_CONTEXT* context){
	BARLEY_reset();
	return EVENT_TPL_RESET_COMPLETE;
}

int lib_wait_finger(BARLEY_CONTEXT* context){

	int start_time = API_GetTick();
	int duration =0 ;
	int result =0 ;
	int barley_ret = 0 ;
	barley_ret = BARLEY_wait_fingerup(context, 0);

	if ( barley_ret == BARLEY_OK ){
		duration = API_GetTick() - start_time;

		if( duration > TIME_5SEC && duration < TIME_10SEC){
			if(BARLEY_get_next_finger_index() < 0 ){
				result = EVENT_ETC_SHORTTOUCH; // Enroll is full.
			}else{
				result = EVENT_ETC_LONGTOUCH_SECTION_1;//EVENT_ENROL_START;
			}
		}else if( duration > TIME_10SEC){
			result = EVENT_ETC_LONGTOUCH_SECTION_2; // Delete mode
		}else{
			result = EVENT_ETC_SHORTTOUCH; // No event.
		}
	}else if (barley_ret == BARLEY_TIMEOUT){
		result = EVENT_SENSOR_TIME_OUT;
	}else{
		result = EVENT_USER_CANCEL;
	}

	return result;
}

short prv_reverse_byte_order_short(short n){
	short tmp =0 ;
	char* p= (char*)&tmp;
	p[0] =(n >> 8) & 0xFF;
	p[1] = n & 0xFF;
	return tmp;
}
int lib_wait_fp_and_capture(BARLEY_CONTEXT* context){
	int result = 0;
	int barley_ret = 0;

	barley_ret= BARLEY_start_capture(context , 10);

	if(barley_ret == BARLEY_OK){
		result = EVENT_SENSOR_CAPTURE_COMPLETE;
	}else if( barley_ret == BARLEY_USER_CANCEL){
		result = EVENT_USER_CANCEL;
	}else if(barley_ret == BARLEY_ERR_INIT_FAIL ){
		result = EVENT_UNKNOWN_ERROR;
	}else if( barley_ret== BARLEY_ERR_INTERNAL_ERROR){
		result = EVENT_UNKNOWN_ERROR;
	}else if (barley_ret == BARLEY_TIMEOUT){
		result = EVENT_TIME_OUT;
	}else{
		result = EVENT_UNKNOWN_ERROR;
	}
	return result;
}


