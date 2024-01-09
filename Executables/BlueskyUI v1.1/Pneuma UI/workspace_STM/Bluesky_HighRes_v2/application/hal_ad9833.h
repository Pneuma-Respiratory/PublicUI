
/**************************************************************************************************
 *                                        - hal_virtual_i2c.h -
 *
 * Special header for the Texas Instruments MSP430G2x System on Chip.
 *
 **************************************************************************************************
 */

#ifndef HAL_AD9833_H
#define HAL_AD9833_H


/***********************************************************************************
* INCLUDES
*/
#include <stdbool.h>


/***********************************************************************************
 * CONSTANTS
 */
#define BIT_F0ADDRESS           0x4000      // Frequency Register 0 address.
#define BIT_F1ADDRESS           0x8000      // Frequency Register 1 address.
#define BIT_P0ADDRESS           0xC000      // Phase Register 0 address.
#define BIT_P1ADDRESS           0xE000      // Phase Register 1 address.

#define AD9833_CTRLB28          (1 << 13)
#define AD9833_CTRLHLB          (1 << 12)
#define AD9833_CTRLFSEL         (1 << 11)
#define AD9833_CTRLPSEL         (1 << 10)
#define AD9834_CTRLPINSW        (1 << 9)
#define AD9833_CTRLRESET        (1 << 8)
#define AD9833_CTRLSLEEP1       (1 << 7)
#define AD9833_CTRLSLEEP12      (1 << 6)
#define AD9833_CTRLOPBITEN      (1 << 5)
#define AD9834_CTRLSIGNPIB      (1 << 4)
#define AD9833_CTRLDIV2         (1 << 3)
#define AD9833_CTRLMODE         (1 << 1)
     

#define BIT_F0ADDRESS           0x4000      // Frequency Register 0 address.
#define BIT_F1ADDRESS           0x8000      // Frequency Register 1 address.
#define BIT_P0ADDRESS           0xC000      // Phase Register 0 address.
#define BIT_P1ADDRESS           0xE000      // Phase Register 1 address.

/***********************************************************************************
 * MACROS
 */
#define ADI_OE_OUT_H()                GPIO_setOutputHighOnPin( ADI_OE_PORT, ADI_OE_PIN )
#define ADI_OE_OUT_L()                GPIO_setOutputLowOnPin( ADI_OE_PORT, ADI_OE_PIN )

#define ADI_FSYNC_H()                 GPIO_setOutputHighOnPin( ADI_FSYNC_PORT, ADI_FSYNC_PIN )
#define ADI_FSYNC_L()                 GPIO_setOutputLowOnPin( ADI_FSYNC_PORT, ADI_FSYNC_PIN )

#define ADI_SDATA_H()                 GPIO_setOutputHighOnPin( ADI_SDATA_PORT, ADI_SDATA_PIN )
#define ADI_SDATA_L()                 GPIO_setOutputLowOnPin( ADI_SDATA_PORT, ADI_SDATA_PIN )

#define ADI_SCLK_H()                  GPIO_setOutputHighOnPin( ADI_SCLK_PORT, ADI_SCLK_PIN )
#define ADI_SCLK_L()                  GPIO_setOutputLowOnPin( ADI_SCLK_PORT, ADI_SCLK_PIN )

/***********************************************************************************
* TYPEDEFS
*/

/***********************************************************************************
* Global Variable
*/

/*******************************************************************************
*  Global Function
*/
void ad9833_IO_ini(void);
void ad9833_reset(void);
void ad9833_set_frq( uint32_t frq_value );

#endif
