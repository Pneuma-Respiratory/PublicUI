/***********************************************************************************
  Filename:     ADC_App.c

  Description:  
                

***********************************************************************************/

/***********************************************************************************
* INCLUDES
*/
#include "cmsis_os.h"
#include "stm32l4xx.h"
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_gpio.h"

#include "main.h"
#include "adc.h"
#include "hal_board_cfg.h"
#include "hal_adc.h"

/***********************************************************************************
 * CONSTANTS
 */

/***********************************************************************************
 * MACROS
 */
#define ADC_INTERNAL_REF        1

/***********************************************************************************
* GLOBAL VARIABLES
*/
 extern ADC_HandleTypeDef Adc1Handle;
 extern DMA_HandleTypeDef hdma_adc1;

volatile uint8_t adc_state = ADC_IDLE;     // ADC_IDLE or ADC_BUSY

/***********************************************************************************
* LOCAL VARIABLES
*/
#define ADC_CONVERTED_DATA_BUFFER_SIZE    10

volatile uint8_t adc_buf_index = 0;
volatile uint16_t adc_results[ADC_CONVERTED_DATA_BUFFER_SIZE];

uint32_t adc_current_ch = ADC_CHANNEL_16;


/***********************************************************************************
* LOCAL FUNCTIONS
*/

/***********************************************************************************
* GLOBAL FUNCTIONS
*/


/***********************************************************************************
* LOCAL FUNCTIONS
*/


/***********************************************************************************
*  @fn   
*  @brief    reference static void MX_ADC1_Init(void)
*/
void ADC_Init( uint32_t adc_ch )
{
}

/***********************************************************************************
 *  @fn       ADC_Start
 *  @brief    
 *  @param      
 *            
 *  @return   none
 */
uint8_t ADC_Start( uint32_t adc_ch )
{
	uint8_t rslt = true;

	adc_current_ch = adc_ch;
	MX_ADC1_Init();
#if ADC_NO_DMA
	// Run the ADC calibration in single-ended mode
	if (HAL_ADCEx_Calibration_Start(&Adc1Handle, ADC_SINGLE_ENDED) != HAL_OK){
	    // Calibration Error
		rslt = false;
	}
	// Start the conversion process
   if (HAL_ADC_Start( &Adc1Handle ) != HAL_OK){
     // Start Conversation Error
	 Error_Handler();
   }


   for ( i = 0; i < sizeof(adc_results); i++ ){
	   adc_results[i] = 0;
	   if (HAL_ADC_PollForConversion( &Adc1Handle, 2 ) != HAL_OK){
		   rslt = false;
		   i = sizeof(adc_results);
	   }
	   else{
		   // ADC conversion completed, Get the converted value of regular channel
		   adc_results[i] = HAL_ADC_GetValue( &Adc1Handle );
	   }
   }
#else
   adc_state = ADC_BUSY;
   memset( (uint8_t *)adc_results, 0x00, sizeof(adc_results) );
   if (HAL_ADC_Start_DMA(&Adc1Handle, (uint32_t *)adc_results, ADC_CONVERTED_DATA_BUFFER_SIZE ) != HAL_OK){
	   Error_Handler();
   }
   // while( adc_state == ADC_BUSY );        // 0814
   // while ( hdma_adc1.Instance->CNDTR != 0 );
   while( DMA1_Channel1->CNDTR != 0 );
   adc_state = ADC_IDLE;

   HAL_ADC_Stop_DMA( &Adc1Handle );

#endif

    return rslt;
}

uint16_t ADC_Average( uint32_t adc_ch )
{
    uint8_t i;
    uint16_t adc_max, adc_min, adc_sum = 0;
    
    if ( adc_ch != adc_current_ch ){
        return 0;
    }
    adc_current_ch = 0;
    adc_max = adc_min = adc_results[0];
    for ( i = 0; i < 10; i++ ){
        if ( adc_results[i] > adc_max ){
            adc_max = adc_results[i];
        }
        else if( adc_results[i] < adc_min ){
            adc_min = adc_results[i];
        }
        adc_sum += adc_results[i];
    }
    // adc_result[] detect the max and min value then get the average value
    adc_sum = adc_sum - adc_max - adc_min;
    
    return ( adc_sum >> 3 );
}

/***********************************************************************************
 *  @fn       ADC_Stop
 *  @brief    
 *  @param    
 *            
 *  @return   
 */
void ADC_Stop( void )
{
	HAL_ADC_Stop( &Adc1Handle );
}
