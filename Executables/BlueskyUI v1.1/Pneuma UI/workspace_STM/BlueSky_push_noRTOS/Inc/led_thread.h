/*
 * led_thread.h
 *
 *  Created on: Sep 29, 2019
 *      Author: hdkim
 */

#ifndef INC_LED_THREAD_H_
#define INC_LED_THREAD_H_

typedef enum
{
  LED1R = 0,
  LED1G,
  LED1B,
  LED2,
  LED3,
  LED4,
  LED5,
  LED6,
  LED7,
  LED8,
  LED_MAX
}Led_Def;

void LED_On(Led_Def n);
void LED_Off(Led_Def n);
void LED_Toggle(Led_Def n);

#endif /* INC_LED_THREAD_H_ */
