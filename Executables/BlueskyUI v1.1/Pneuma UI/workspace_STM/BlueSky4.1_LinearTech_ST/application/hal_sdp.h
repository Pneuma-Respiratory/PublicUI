
/**************************************************************************************************
 *                                        - hal_sdp.h -
 *
 * Special header for the Texas Instruments MSP430 System on Chip.
 *
 **************************************************************************************************
 */

#ifndef HAL_SDP_H
#define HAL_SDP_H


/***********************************************************************************
* INCLUDES
*/
#include "stdint.h"

#include "hal_sdp.h"

/*********************************************************************
 * MACROS
 */

    
/*********************************************************************
 * CONSTANTS
 */
#define SDP_DIFF_AVERAGE              0x3615        // Differential pressure average
#define SDP_STOP_MEASURE              0x3FF9
#define SDP_PROD_NUMBER               0x367C        // Read product number, 0x03010185 or 0x03010281
#define SDP_PROD_ID                   0xE102
     
     
/*********************************************************************
 * TYPEDEFS
 */



/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * FUNCTIONS
 */

/*
 * @fn      
 *
 * @brief   
 *
 * @param   None.
 *
 * @return  None.
 */
uint8_t HalSdp_Init( void );
uint16_t HalSdp_GetThr( void );
//int8_t HalSdp_GetValue( uint8_t *value );
int8_t HalSdp_GetValue( uint16_t *value, uint16_t thr );

uint8_t HalSdp_GetID( void );
uint8_t HalSdp_Test( void );

uint8_t HalSdp_Start_Measure( void );
uint8_t HalSdp_Stop_Measure( void );

#endif
