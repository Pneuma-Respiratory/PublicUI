
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
     

/***********************************************************************************
* TYPEDEFS
*/

/***********************************************************************************
* Global Variable
*/

/*******************************************************************************
*  Global Function
*/

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
