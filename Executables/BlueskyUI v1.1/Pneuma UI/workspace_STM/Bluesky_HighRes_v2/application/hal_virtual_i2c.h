
/**************************************************************************************************
 *                                        - hal_virtual_i2c.h -
 *
 * Special header for the Texas Instruments MSP430G2x System on Chip.
 *
 **************************************************************************************************
 */

#ifndef HAL_VIRTUAL_I2C_H
#define HAL_VIRTUAL_I2C_H


/***********************************************************************************
* INCLUDES
*/
#include <stdbool.h>
#include "stm32l4xx_hal_gpio.h"
#include "hal_board_cfg.h"

#include "hal_virtual_i2c.h"

/***********************************************************************************
 * CONSTANTS
 */
#define LIS3DH_ADDR           0x18
#define HT16C21_ADDR          0x38
#define SDP3X_ADDR            0x21
#define SHA204A_ADDR          (0xC8 >> 1)
     
#define HT16C21               0x01
#define LIS3DH                0x02
#define SDP3X                 0x03
#define SDP3X_RESET           0x04
#define SHA204A               0x05
#define SHA204A_RESET         0x06

#define I2C_ACK               0x00
#define I2C_NACK              0x01
     
#if 0
/***********************************************************************************
 * MACROS
 */
//#define GPIO_setOutputHighOnPin( port, pin )   HAL_GPIO_WritePin( port, pin, GPIO_PIN_SET )
//#define GPIO_setOutputLowOnPin( port, pin )    HAL_GPIO_WritePin( port, pin, GPIO_PIN_RESET )

//#define SDA_PIN_AS_INPUT()        GPIO_setAsInputPin( I2C_SDA_PORT, SDA_PIN )
//#define SDA_PIN_AS_OUTPUT()       GPIO_setAsOutputPin( I2C_SDA_PORT, SDA_PIN )
//#define SCL_PIN_AS_INPUT()        GPIO_setAsInputPin( I2C_SCL_PORT, SCL_PIN )
//#define SCL_PIN_AS_OUTPUT()       GPIO_setAsOutputPin( I2C_SCL_PORT, SCL_PIN )

__STATIC_INLINE void SDA_PIN_AS_INPUT( void )
{
	  GPIO_InitTypeDef GPIO_InitStruct = {0};

	  GPIO_InitStruct.Pin = SDA_PIN;
	  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init( I2C_SDA_PORT, &GPIO_InitStruct);
}

__STATIC_INLINE void SDA_PIN_AS_OUTPUT( void )
{
	  GPIO_InitTypeDef GPIO_InitStruct = {0};

	  GPIO_InitStruct.Pin = SDA_PIN;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init( I2C_SDA_PORT, &GPIO_InitStruct);
}

__STATIC_INLINE void SCL_PIN_AS_INPUT( void )
{
	  GPIO_InitTypeDef GPIO_InitStruct = {0};

	  GPIO_InitStruct.Pin = SCL_PIN;
	  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init( I2C_SCL_PORT, &GPIO_InitStruct);
}

__STATIC_INLINE void SCL_PIN_AS_OUTPUT( void )
{
	  GPIO_InitTypeDef GPIO_InitStruct = {0};

	  GPIO_InitStruct.Pin = SCL_PIN;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init( I2C_SCL_PORT, &GPIO_InitStruct);
}
#if 0
#define SDA_PIN_OUT_H()           st ( GPIO_setOutputHighOnPin( I2C_SDA_PORT, SDA_PIN );   \
                                       GPIO_setAsOutputPin( I2C_SDA_PORT, SDA_PIN );  )
#define SDA_PIN_OUT_L()           st ( GPIO_setOutputLowOnPin( I2C_SDA_PORT, SDA_PIN );    \
                                       GPIO_setAsOutputPin( I2C_SDA_PORT, SDA_PIN );  )
#define SCL_PIN_OUT_H()           st ( GPIO_setOutputHighOnPin( I2C_SCL_PORT, SCL_PIN );   \
                                       GPIO_setAsOutputPin( I2C_SCL_PORT, SCL_PIN );  )
#define SCL_PIN_OUT_L()           st ( GPIO_setOutputLowOnPin( I2C_SCL_PORT, SCL_PIN );    \
                                       GPIO_setAsOutputPin( I2C_SCL_PORT, SCL_PIN );  )
#endif
__STATIC_INLINE void SDA_PIN_OUT_H( void )
{
	HAL_GPIO_WritePin( I2C_SDA_PORT, SDA_PIN, GPIO_PIN_SET);
	SDA_PIN_AS_OUTPUT();
}
__STATIC_INLINE void SDA_PIN_OUT_L( void )
{
	HAL_GPIO_WritePin( I2C_SDA_PORT, SDA_PIN, GPIO_PIN_RESET);
	SDA_PIN_AS_OUTPUT();
}
__STATIC_INLINE void SCL_PIN_OUT_H( void )
{
	HAL_GPIO_WritePin( I2C_SCL_PORT, SCL_PIN, GPIO_PIN_SET);
	SCL_PIN_AS_OUTPUT();
}
__STATIC_INLINE void SCL_PIN_OUT_L( void )
{
	HAL_GPIO_WritePin( I2C_SCL_PORT, SCL_PIN, GPIO_PIN_RESET);
	SCL_PIN_AS_OUTPUT();
}

//#define SDA_PIN_GET()             GPIO_getInputPinValue( I2C_SDA_PORT, SDA_PIN )
//#define SCL_PIN_GET()             GPIO_getInputPinValue( I2C_SCL_PORT, SCL_PIN )

__STATIC_INLINE GPIO_PinState SDA_PIN_GET( void )
{
	return HAL_GPIO_ReadPin( I2C_SDA_PORT, SDA_PIN );
}

__STATIC_INLINE GPIO_PinState SCL_PIN_GET( void )
{
	return HAL_GPIO_ReadPin( I2C_SCL_PORT, SCL_PIN );
}


#define I2C_DELAY()               __delay_cycles(100);     // MCLK = 20MHz(20 000KHz) --> 50KHz
#endif

/***********************************************************************************
* TYPEDEFS
*/

/***********************************************************************************
* Global Variable
*/

/*******************************************************************************
*  Global Function
*/
#if 0
bool HalSensorReadReg(uint8_t addr, uint8_t *pBuf, uint8_t nBytes);
uint8_t HalSensorWriteReg( uint8_t addr, uint8_t *pBuf, uint8_t nBytes );

bool HalSensorRead(uint8_t *pBuf, uint8_t nBytes);
uint8_t HalSensorWrite( uint8_t *pBuf, uint8_t nBytes );

uint8_t HalI2CWrite( uint8_t len, uint8_t *pBuf );
uint8_t HalI2CRead(  uint8_t len, uint8_t *pBuf  );

uint8_t VI2C_Select( uint8_t device );
uint8_t VI2C_Release( uint8_t device );

uint8_t VI2C_Probe( void );
uint8_t I2C_Device_Reset( void );
#endif
void VI2C_PIN_Cfg( void );

uint8_t HalI2CWrite( uint8_t len, uint8_t *pBuf );
uint8_t HalI2CRead(  uint8_t len, uint8_t *pBuf  );

void VI2C_DeInit( void );
void VI2C_Configure( void );

void VI2C_WR_AckBit( void );
void VI2C_WR_NAckBit( void );
uint8_t VI2C_RD_AckBit( void );

uint8_t atcab_wakeup( void );
#endif
