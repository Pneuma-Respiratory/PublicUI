/*
 * stm32_api.c
 *
 *  Created on: 2017. 10. 23.
 *  Author: hdkim
 */
#include "platform_api.h"
#include "cmsis_os.h"
#include "task.h"
#include <string.h>

//#include "portmacro.h"
//#include "FreeRTOSConfig.h"

#define QUEUE_SIZE    (uint32_t) 7
osMessageQId osQueue;

void API_Queue_Init(void){
	osMessageQDef(osqueue, QUEUE_SIZE, uint16_t);
	osQueue = osMessageCreate (osMessageQ(osqueue), NULL);
}

void API_Queue_Send(uint32_t d){
	osMessagePut ( osQueue, d, 0);
}

int API_Queue_Receive(void){
	osEvent event;
	int state;
	event = osMessageGet( osQueue, 1 );
	if( event.status == osEventMessage )
	{
		state = event.value.v;
		return state;
	}
	return 0;
}


void* API_Malloc(size_t s){
	return (void*)pvPortMalloc(s);
}

void API_Free(void* p){
	vPortFree(p);
}

void* API_Realloc(void *ptr, size_t size){
	void *p = NULL;
	API_Free(ptr);
	p = (void*)API_Malloc(size);
	if( p != NULL )
	{
		return p;
	}
	return NULL;
}

void* API_Memset(void* s, int c, size_t n){
	void* _s = s;
	memset(_s, c, n);
	return _s;
}

void* API_Calloc(size_t nmemb, size_t size){
	size_t tsize = nmemb * size;
	void *p;
	p = API_Malloc(tsize);
	if( p != NULL )
	{
		API_Memset(p, 0, tsize);
		return p;
	}
	return NULL;
}


void* API_Memcpy(void *dest, const void *src, size_t n){
	void* _dest = dest;
	memcpy(_dest, src, n);
	return _dest;
}
uint32_t API_GetTick(){
	return HAL_GetTick();
}

void API_Delay(uint32_t ms){
	vTaskDelay(ms);
	// MY vTaskDelay(pdMS_TO_TICKS(ms));
}


