/**
 * @file    barley_sdk.h
 * @version V1.0.1
 * @brief   This file provides the function to operate barley_sdk.
 ******************************************************************************
 * @attention
 * 
 * Copyright (c) 2016 Canvasbio <www.canvasbio.com>
 *
 */

#ifndef __BARLEY_SDK_H_
#define __BARLEY_SDK_H_
#pragma pack(1)
#include "barley_retno.h"
#include <stddef.h>
#include <stdint.h>



/**
 * @brief Barley SDK (Fingerprint Library) context.
 */
typedef struct st_barley_context{
	const char* sdk_ver;	/* SDK Version */
	const char* algo_ver;	/* Algorithm Version */
	char chip_id;			/* Sensor id */
	void* fp_image;			/* Fingerprint image */
}BARLEY_CONTEXT;

/**
 * @brief Structure of platform APIs. This apis will be used inside the SDK
 */
typedef struct _PLATFORM_API
{

	void* (*malloc) (size_t size);								/* Memory allocation function */
	void* (*calloc) (size_t nmemb, size_t sizeb);				/* Memory c-allocation function */
	void  (*free)   (void* ptr);								/* Memory free function */
	void* (*memset) (void* s, int c, size_t n);					/* Memory set function */
	void* (*memcpy) (void *dest, const void *src, size_t n);	/* Memory copy function */
	void   (*delay)  (uint32_t milliseconds);					/* Delay function */
	uint32_t  (*get_tick)(void); 								/* Get tick value */


} PLATFORM_API;


/**
 *	@brief	Initialize SDK. <p>This function must be called befor using any other SDK apis.
 *	@return Library context.
 */
BARLEY_CONTEXT* BARLEY_init( PLATFORM_API api);


/**
 *	@brief De-Initialize Barley SDK.
 *	@param context Library context.
 *  @return BARLEY_OK.
 *
 */
int BARLEY_deinit(BARLEY_CONTEXT* context);


/**
 *	@brief Enroll Fingerprint.
 *	@param idx To be stored index of fingerprint.
 *	@return
 *	- BARLEY_ENROLL_COUNT_INCREASED The enroll operation has been successfully executed, but not has been finished enroll operation.aaaa If the number of successes exceeds MAX_ENROLL_COUNT(in this case '8'), return BARLEY_ENROLL_FINISHED.
 *	- BARLEY_ENROLL_FINISHED This operation has been successfully executed.
 *	- BARLEY_ENROLL_FAIL The enroll operation is failed.
 *	- BARLEY_ERR_INIT_FAIL SDK initialization was failed.
 *	- BARLEY_INVALID_PARAM Invaid parameter.
 *	- BARLEY_USER_CANCEL This operaion was aborted by a cancel request.
 *	- BARLEY_BAD_IMAGE Because bad image is captured, this operation is failed.
 */
int BARLEY_enroll(BARLEY_CONTEXT* context , int idx);


/**
 * @brief Verify Fingerprint.
 * @param idx Finger index to match.<p>If idx is '-1', all enrolled finger indices will be matched against.
 * @return
 * - BARLEY_VERIFY_SUCCESS if this operation has been successfully executed
 * - BARLEY_VERIFY_FAIL Verify is fail.
 * - BARLEY_BAD_IMAGE Because bad image is captured, this operation is failed.
 * - BARLEY_ERR_INIT_FAIL SDK initialization was failed.
 * - BARLEY_INVALID_PARAM Invaid parameter.
 * - BARLEY_USER_CANCEL This operaion was aborted by a cancel request.
 * - BARLEY_BAD_IMAGE Because bad image is captured, this operation is failed.
 */
int BARLEY_verify(BARLEY_CONTEXT* context , int idx);


/**
 *	@brief This API will wait until the moment when the finger is detached from the sensor.
 *  @param context Library context.
 *  @param timeout The amount of time to keep this operation.(Second)<p>If you set this value to '-1', wait forever.(until user_cancel)
 *	@return
 *	- BARLEY_OK If this operation has been successfully executed.
 *	- BARLEY_USER_CANCEL This operation was aborted by a cancel request from the user.
 * 	- BARLEY_TIMEOUT Timeout.
 *
 */
int BARLEY_wait_fingerup(BARLEY_CONTEXT* context , int timeout);


/**
 *	@brief This API will wait until the moment when the finger is attached on the sensor.
 *  @param context Library context.
 *  @param timeout The amount of time to keep this operation.(Second)<p>If you set this value to '-1', wait forever.(until user_cancel)
 *	@return
 *	- BARLEY_OK If this operation has been successfully executed.
 *	- BARLEY_USER_CANCEL This operation was aborted by a cancel request from the user.
 * 	- BARLEY_TIMEOUT Timeout.
 *
 */
int BARLEY_wait_fingerdown(BARLEY_CONTEXT* context , int timeout);


/**
 * @brief Erase a fingerprint template slot with given index(idx)
 * @param context Library context.
 * @param idx Index to be deleted fingerprint.
 * @return
 *  - BARLEY_OK if this operation has been successfully executed, error code otherwise.
 *	- BARLEY_ERR_INIT_FAIL SDK initialization was failed.
 *  - BARLEY_INVALID_PARAM Invaid parameter.
 *  - BARLEY_ERR_INTERNAL_ERROR Flash api error.
 */
int BARLEY_delete_template(BARLEY_CONTEXT* context , int idx);


/**
 * @brief This API call will wait for finger attach and then will capture image after attach is detected.
 * @param context Library context.
 * @param timeout The amount of time to keep this operation.(Second)<p>If you set this value to '-1', wait forever.(until user_cancel)
 * @return
 *  - BARLEY_OK if this operation has been successfully executed, error code otherwise.
 *  - BARLEY_INVALID_PARAM Invaid parameter.
 * 	- BARLEY_TIMEOUT Timeout.
 *  - BARLEY_ERR_INIT_FAIL SDK initialization was failed.
  */
int BARLEY_start_capture(BARLEY_CONTEXT* context , int timeout);

/**
 *  @brief Cancel on-going operation(Capture, Enroll or Verify).
 *	@return BARLEY_OK if this operation has been successfully executed
 */
int BARLEY_UserCancel(void);

/**
 * @breif Get next empty finger index that is available to enroll a new fingerprint.
 * @return An empty finger index for enrollment. If there is no finger index that can enroll, return '-1'.(all slots are full already.)
 */
int BARLEY_get_next_finger_index();


/**
 * @breif Erase all fingerprint templates
 * @return BARLEY_OK
 */
int BARLEY_reset(void);



#endif//__BARLEY_SDK_H_
/******************************** END OF FILE *********************************/
