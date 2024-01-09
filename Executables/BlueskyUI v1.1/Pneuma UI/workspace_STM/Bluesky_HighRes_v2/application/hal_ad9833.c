/***********************************************************************************
  Filename:     hal_AD9833.c

  Description:  
                

***********************************************************************************/

/***********************************************************************************
* INCLUDES
*/
#include "cmsis_os.h"
#include "stm32l4xx.h"
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_gpio.h"
//#include "stm32l4xx_hal_i2c.h"

#include "main.h"
#include "hal_board_cfg.h"
#include "hal_AD9833.h"

//#include "hal_virtual_i2c.h"

/***********************************************************************************
 * CONSTANTS
 */

/***********************************************************************************
 * MACROS
 */


/***********************************************************************************
* GLOBAL VARIABLES
*/
uint16_t ctrl_reg_value = 0;

/***********************************************************************************
* LOCAL VARIABLES
*/

/***********************************************************************************
* LOCAL FUNCTIONS
*/

/***********************************************************************************
* GLOBAL FUNCTIONS
*/


/***********************************************************************************
* LOCAL FUNCTIONS
*/

void SPI_DELAY( uint32_t delay_us )
{
	uint32_t wait_loop_index;
	wait_loop_index = delay_us * (SystemCoreClock / 1000000U)/10;
	while ( --wait_loop_index != 0U );
}
void ADI_FSYNC_AS_OUTPUT( void )
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = ADI_FSYNC_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(ADI_FSYNC_PORT, &GPIO_InitStruct);
}
void ADI_SDATA_AS_OUTPUT( void )
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = ADI_SDATA_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(ADI_SDATA_PORT, &GPIO_InitStruct);
}
void ADI_SCLK_AS_OUTPUT( void )
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = ADI_SCLK_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(ADI_SCLK_PORT, &GPIO_InitStruct);
}
void ADI_OE_AS_OUTPUT( void )
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = ADI_OE_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(ADI_OE_PORT, &GPIO_InitStruct);
}
void ad9833_IO_ini(void)
{
	ADI_OE_OUT_H();
	ADI_FSYNC_H();
	ADI_SCLK_H();

	ADI_SCLK_AS_OUTPUT();
	ADI_SDATA_AS_OUTPUT();
	ADI_FSYNC_AS_OUTPUT();
	ADI_OE_AS_OUTPUT();

	ADI_OE_OUT_H();
	ADI_FSYNC_H();
	ADI_SCLK_H();
}
/***********************************************************************************
*  @fn
*  @param
*  @return
*/
void VSPI_WRITE( uint16_t value )
{
	uint8_t i;
	uint16_t data;

	data = value;

	ADI_SCLK_H();
	ADI_SDATA_H();
	ADI_FSYNC_L();
	SPI_DELAY(1);

    for ( i = 0; i < 16; i++ ){
    	ADI_SCLK_H();
        if ( data & 0x8000 ){
        	ADI_SDATA_H();
        }
        else{
        	ADI_SDATA_L();
        }
        __NOP();
        ADI_SCLK_L();
        __NOP();
        __NOP();
        data <<= 1;
    }
    SPI_DELAY(1);
    ADI_SCLK_H();
    ADI_FSYNC_H();
}
void ad9833_reset(void)
{
	uint16_t spi_data = 0;
	ADI_OE_OUT_H();

	ctrl_reg_value = 0;
	spi_data = ctrl_reg_value | AD9833_CTRLRESET | AD9833_CTRLSLEEP12;
	VSPI_WRITE( spi_data );			//RESET = 1
}
void ad9833_set_frq( uint32_t frq_value )
{
	uint16_t spi_data = 0;
	uint32_t ul_freq_register;
	uint16_t i_freq_lsb, i_freq_msb;
	ctrl_reg_value = 0;
	spi_data = ctrl_reg_value | AD9833_CTRLB28 | AD9833_CTRLSLEEP12 | AD9833_CTRLOPBITEN | AD9833_CTRLDIV2;
//	spi_data = ctrl_reg_value | AD9833_CTRLB28;
	VSPI_WRITE( spi_data );
	SPI_DELAY(2);
	// Set frequency
	ul_freq_register = frq_value;     // read 28bit frequency value
	i_freq_lsb = (ul_freq_register & 0x0003FFF);
	i_freq_msb = ((ul_freq_register & 0xFFFC000) >> 14);
	VSPI_WRITE( BIT_F0ADDRESS + i_freq_lsb );
	VSPI_WRITE( BIT_F0ADDRESS + i_freq_msb );
}
