/*
 * libbarley.h
 *
 *  Created on: Sep 29, 2019
 *      Author: hdkim
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef INC_LIBBARLEY_H_
#define INC_LIBBARLEY_H_

/* Includes ------------------------------------------------------------------*/
#include "barley_sdk.h"
#include "platform_api.h"

/* Exported Functions --------------------------------------------------------*/
BARLEY_CONTEXT* lib_fp_init(void);
int lib_fp_enroll(BARLEY_CONTEXT* context);
int lib_fp_verify(BARLEY_CONTEXT* context);
int lib_fp_reset(BARLEY_CONTEXT* context);
int lib_wait_finger(BARLEY_CONTEXT* context);
int lib_wait_fp_and_capture(BARLEY_CONTEXT* context);


#endif /* INC_LIBBARLEY_H_ */
