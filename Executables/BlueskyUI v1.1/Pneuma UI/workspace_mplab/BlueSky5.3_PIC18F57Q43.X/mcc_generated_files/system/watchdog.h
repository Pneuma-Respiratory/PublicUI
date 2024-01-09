
/**
 * WWDT Generated Driver API Header File.
 * 
 * @file wwdt.h
 * 
 * @defgroup  wwdt WWDT
 * 
 * @brief This file contains the API prototypes for the WWDT driver.
 *
 * @version WWDT Driver Version 2.0.0
*/

/*
? [2023] Microchip Technology Inc. and its subsidiaries.

    Subject to your compliance with these terms, you may use Microchip 
    software and any derivatives exclusively with Microchip products. 
    You are responsible for complying with 3rd party license terms  
    applicable to your use of 3rd party software (including open source  
    software) that may accompany Microchip software. SOFTWARE IS ?AS IS.? 
    NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS 
    SOFTWARE, INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT,  
    MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT 
    WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY 
    KIND WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF 
    MICROCHIP HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE 
    FORESEEABLE. TO THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP?S 
    TOTAL LIABILITY ON ALL CLAIMS RELATED TO THE SOFTWARE WILL NOT 
    EXCEED AMOUNT OF FEES, IF ANY, YOU PAID DIRECTLY TO MICROCHIP FOR 
    THIS SOFTWARE.
*/

#ifndef WATCHDOG_H
#define WATCHDOG_H

 /**
   Section: Included Files
 */

#include <xc.h>
#include <stdbool.h>
#include <stdint.h>

#define WDTCWS  7
#define WDTCCS  0
#define WDTCPS  0x16   /// 38



/**
 * @ingroup wwdt
 * @brief Initializes the Windowed Watchdog Timer (WWDT) driver.
 * @param None.
 * @return None.
 * 
 */
void WWDT_Initialize(void);

/**
 * @ingroup wwdt
 * @brief Enables the WWDT.
 * @param None.
 * @return None.
 * 
 */
void WWDT_SoftEnable(void);

/**
 * @ingroup wwdt
 * @brief Disables the WWDT.
 * @param None.
 * @return None.
 * 
 */
void WWDT_SoftDisable(void);

/**
 * @ingroup wwdt
 * @brief Clears the WWDT counter.
 * @param None.
 * @return None.
 */
void WWDT_TimerClear(void);

/**
 * @ingroup wwdt
 * @brief Checks if a reset has occurred or not.
 * @param None.
 * @retval 0 - WWDT reset has occurred.
 * @retval 1 - WWDT reset has not occurred. 
 */
bool WWDT_TimeOutStatusGet(void);

/**
 * @ingroup wwdt
 * @brief Checks if a window violation reset has occurred or not.
 * @param None.
 * @retval 0 - WWDT window violation reset has occurred.
 * @retval 1 - WWDT window violation reset has not occurred.
 */
bool WWDT_WindowViolationStatusGet(void);  

#endif	/* WATCHDOG_H */
/**
 End of File
*/