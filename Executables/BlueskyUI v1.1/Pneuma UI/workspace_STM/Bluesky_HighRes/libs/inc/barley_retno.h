/**
 * @file    barley_retno.h
 * @version V1.0.1
 * @date    24-Oct-2019
 * @brief   This file defines return-value for using BARLEY SDK.
 ******************************************************************************
 */

#ifndef SRC_BARLEY_RETNO_H_
#define SRC_BARLEY_RETNO_H_

/**
 * Barley SDK retrun codes.
 */
typedef enum{

	/**
	 * OK.
	 */
	BARLEY_OK = 0,                

	/**
	 * This value will be returned during enrollment session until enrollment for a finger is completed.
	 */
	BARLEY_ENROLL_COUNT_INCREASED,

	/**
	 * The enroll for a finger was successfully finished
	 */
	BARLEY_ENROLL_FINISHED ,      

	/**
	 * Failed to enroll.
	 */
	BARLEY_ENROLL_FAIL,           

	/**
	 * Invalid parameters.
	 */
	BARLEY_INVALID_PARAM,         

	/**
	 * Succeeded to verify(authenticate) fingerprint.
	 */
	BARLEY_VERIFY_SUCCESS ,       

	/**
	 * Failed to verify(authenticate) fingerprint
	 */
	BARLEY_VERIFY_FAIL,           

	/**
	 * Cancellation by user request.
	 */
	BARLEY_USER_CANCEL ,          

	/**
	 * Reset complete.
	 */
	BARLEY_RESET_COMPLETE,        

	/**
	 * Internal error.
	 */
	BARLEY_ERR_INTERNAL_ERROR ,   



	/**
	 * Sensor fail.
	 */
	BARLEY_SENSOR_ERROR,          

	/**
	 * SDK fail.
	 */
	BARLEY_ERR_INIT_FAIL,         
	/**
	 * Fingerprint capture complete.
	 */
	BARLEY_IMG_CAPTURE_COMPLETE,  

	/**
	 * Not supported function.
	 */
	BARLEY_NOT_SURPPORTED,        

	/**
	 * Processing timeout.
	 */
	BARLEY_TIMEOUT,               
    
	/**
	 * All slots are full.
	 */
	BARLEY_ALL_SLOTS_IS_FULL,     

	/**
	 * The fingerprint slot with given index is empty.
	 */
	BARLEY_SLOT_IS_EMPTY,         

	/**
	 * The fingerprint image is too bad.
	 */
	BARLEY_BAD_IMAGE,             


}BARLEY_RETURN_CODE;

#endif /* SRC_BARLEY_RETNO_H_ */
/******************************** END OF FILE *********************************/
