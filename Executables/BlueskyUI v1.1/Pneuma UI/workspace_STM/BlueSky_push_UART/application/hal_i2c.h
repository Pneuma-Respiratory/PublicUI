
/**************************************************************************************************
 *                                        - hal_virtual_i2c.h -
 *
 * Special header for the Texas Instruments MSP430G2x System on Chip.
 *
 **************************************************************************************************
 */

#ifndef HAL_I2C_H
#define HAL_I2C_H


/***********************************************************************************
* INCLUDES
*/
#include <stdbool.h>


/***********************************************************************************
 * CONSTANTS
 */
#define LIS3DH_ADDR           0x18
#define HT16C21_ADDR          0x38
#define SDP3X_ADDR            0x21
#define SHA204A_ADDR          (0xC8 >> 1)   // 0x07???    hal_i2c_discover_devices()
     
#define HT16C21               0x01
#define LIS3DH                0x02
#define SDP3X                 0x03
#define SDP3X_RESET           0x04
#define SHA204A               0x05
#define SHA204A_RESET         0x06

#define I2C_ACK               0x00
#define I2C_NACK              0x01
     
/***********************************************************************************
 * MACROS
 */
#define SDA_PIN_OUT_H()             GPIO_setOutputHighOnPin( I2C_PORT, I2C_SDA_PIN )
#define SDA_PIN_OUT_L()             GPIO_setOutputLowOnPin( I2C_PORT, I2C_SDA_PIN )
#define SCL_PIN_OUT_H()             GPIO_setOutputHighOnPin( I2C_PORT, I2C_SCL_PIN )
#define SCL_PIN_OUT_L()             GPIO_setOutputLowOnPin( I2C_PORT, I2C_SCL_PIN )

#define SDA_PIN_GET()               HAL_GPIO_ReadPin( I2C_PORT, I2C_SDA_PIN )
#define SCL_PIN_GET()               HAL_GPIO_ReadPin( I2C_PORT, I2C_SCL_PIN )

/***********************************************************************************
* TYPEDEFS
*/

/***********************************************************************************
* Global Variable
*/

/*******************************************************************************
*  Global Function
*/

uint8_t Hal_I2C_Select( uint8_t device );
uint8_t Hal_I2C_Release( uint8_t device );

uint8_t HalSensorReadReg( uint8_t addr, uint8_t *pBuf, uint8_t nBytes );
uint8_t HalSensorWriteReg( uint8_t addr, uint8_t *pBuf, uint8_t nBytes );

uint8_t HalSensorRead( uint8_t *pBuf, uint8_t nBytes );
uint8_t HalSensorWrite( uint8_t *pBuf, uint8_t nBytes );

uint8_t atcab_wakeup( void );

void SDA_PIN_AS_INPUT( void );
void SDA_PIN_AS_OUTPUT( void );
void SCL_PIN_AS_INPUT( void );
void SCL_PIN_AS_OUTPUT( void );
void I2C_DELAY( uint32_t delay_us );

void VI2C_StartBit( void );
void VI2C_StopBit( void );

#endif
