/*
 * stm32_hal.h
 *
 *  Created on: 2017. 10. 23.
 *      Author: hdkim
 */

#ifndef STM32_API_H_
#define STM32_API_H_

#include "cmsis_os.h"
#include "stm32l4xx.h"
#include "stm32l4xx_hal.h"
#include "barley_def.h"
#include "main.h"

void API_Queue_Init(void);
void API_Queue_Send(uint32_t d);
int API_Queue_Receive(void);

uint32_t API_GetTick(void);
void API_Delay(uint32_t ms);
void* API_Malloc(size_t s);
void API_Free(void* p);
void* API_Realloc(void *ptr, size_t size);
void* API_Calloc(size_t nmemb, size_t size);
void* API_Memset(void* s, int c, size_t n);
void* API_Memcpy(void *dest, const void *src, size_t n);




#endif /* STM32_API_H_ */
