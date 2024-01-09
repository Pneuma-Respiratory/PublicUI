/***********************************************************************************
  Filename:     ADC_App.c

  Description:  
                

***********************************************************************************/

/***********************************************************************************
* INCLUDES
*/

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


volatile uint8_t adc_state = ADC_IDLE;     // ADC_IDLE or ADC_BUSY

/***********************************************************************************
* LOCAL VARIABLES
*/
#define ADC_CONVERTED_DATA_BUFFER_SIZE    10

volatile uint8_t adc_buf_index = 0;
volatile uint16_t adc_results[ADC_CONVERTED_DATA_BUFFER_SIZE];

uint32_t adc_current_ch = 0;


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
#if 1
uint8_t ADC_Start( adcc_channel_t adc_ch )
{
    return 0;
}

uint16_t ADC_Average( adcc_channel_t adc_ch )
{
    uint8_t i;
    uint16_t adc_max, adc_min, adc_sum = 0, adc_single;

    ADCC_GetSingleConversion(adc_ch);
    adc_max = 0;
    adc_min = 4096;
    for ( i = 0; i < 10; i++ ){
        adc_single = ADCC_GetSingleConversion(adc_ch);
        if ( adc_single > adc_max ){
            adc_max = adc_single;
        }
        else if( adc_single < adc_min ){
            adc_min = adc_single;
        }
        adc_sum += adc_single;
    }
    
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
inline void ADC_Stop( void )
{
	ADCON0bits.ADGO = 0;
}
#endif
