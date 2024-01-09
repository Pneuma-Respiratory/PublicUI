/* Microchip Technology Inc. and its subsidiaries.  You may use this software 
 * and any derivatives exclusively with Microchip products. 
 * 
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES, WHETHER 
 * EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED 
 * WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A 
 * PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION 
 * WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION. 
 *
 * IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
 * INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
 * WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS 
 * BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE 
 * FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS 
 * IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF 
 * ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE 
 * TERMS. 
 */

/* 
 * File:   
 * Author: 
 * Comments:
 * Revision history: 
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef HAL_FLASH_H
#define	HAL_FLASH_H

#include <xc.h> // include processor files - each processor file is guarded.  
#define PRESSURE_SIZE   50
#define AUTOTUNE_SIZE   64
#define CURRENT_SIZE    50
typedef struct{
       uint16_t shot_num;
       uint32_t init_freq;
       uint32_t used_freq;
       uint32_t sys_ticket;
       uint16_t spray_time;
       uint16_t batt_lvl;
       uint16_t pressure[PRESSURE_SIZE];
       uint16_t autotune[AUTOTUNE_SIZE];
       uint16_t current[CURRENT_SIZE];
}info_shot_t;

uint8_t is_mem_full();
uint16_t set_flash_address();

uint16_t flash_init(void);

uint8_t is_mem_full(void);

void write_shot(info_shot_t *shot);

void write_pressure(uint16_t press_buff[]);

#endif	/* XC_HEADER_TEMPLATE_H */

