/***********************************************************************************
  Filename:     hal_virtual_i2c.c

  Description:  

***********************************************************************************/

/***********************************************************************************
* INCLUDES
*/
#include "../mcc_generated_files/pin_manager.h"
#include "hal_board_cfg.h"
#include "hal_i2c.h"
#include "hal_virtual_i2c.h"

/***********************************************************************************
 * CONSTANTS
 */

/***********************************************************************************
 * MACROS
 */


/***********************************************************************************
* GLOBAL VARIABLES
*/
extern uint8_t i2c_device;
extern uint8_t Device_Address;

/***********************************************************************************
* LOCAL VARIABLES
*/

uint8_t I2C_Device;
static uint8_t Device_Address_Write;
static uint8_t Device_Address_Read;
static uint8_t buffer[60];

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
*  @fn         VI2C_Select              
*  @param      i2c_device = 1/2/3               
*  @return   
*/                                      
uint8_t Hal_I2C_Select( uint8_t device )
{

    if ( device == HT16C21 ){
    	Device_Address_Write = HT16C21_ADDR;
    }
    else if ( device == SDP3X ){  
    	Device_Address = SDP3X_ADDR;
    	Device_Address_Write = SDP3X_ADDR << 1;
    	Device_Address_Read = (SDP3X_ADDR << 1) + 1;
    }
    else if ( device == SDP3X_RESET ){
    	Device_Address = 0x00;
    	Device_Address_Write = 0x00;
    	Device_Address_Read = 0x00;
    }
    else if ( device == SHA204A ){
    	Device_Address = SHA204A_ADDR;
		Device_Address_Write = SHA204A_ADDR << 1;
		Device_Address_Read = (SHA204A_ADDR << 1) + 1;
    }
    else{
        return I2C_ERROR_DEVICE;
    }
    I2C_Device = device;
    
    return SUCCESS;
}

uint8_t Hal_I2C_Release( uint8_t device )
{
    if ( device == I2C_Device ){
    	Device_Address = 0xFF;
    	Device_Address_Write = 0xFF;
    	Device_Address_Read = 0xFF;
    }
    else{
        return I2C_ERROR_DEVICE;
    }
    return SUCCESS;
}

/**************************************************************************************************
 * @fn          halSensorReadReg
 *
 * @brief       This function implements the I2C protocol to read from a sensor. The sensor must
 *              be selected before this routine is called.
 *
 * @param       addr - which register to read
 * @param       pBuf - pointer to buffer to place data
 * @param       nBytes - numbver of bytes to read
 *
 * @return      pdTRUE if the required number of bytes are reveived
 **************************************************************************************************/
uint8_t HalSensorReadReg(uint8_t addr, uint8_t *pBuf, uint8_t nBytes)
{
  uint8_t i = 0;
  // Send address we're reading from
  if ( HalI2CWrite( 1, &addr ) == 1 ){
    // Now read data
    i = HalI2CRead( nBytes,pBuf );
  }
  return i == nBytes;
}

uint8_t HalSensorRead(uint8_t *pBuf, uint8_t nBytes)
{
	uint8_t i = HalI2CRead( nBytes,pBuf );
	return i == nBytes;
}



/***********************************************************************************
 *  @fn       HalSensorWriteReg
 *  @brief    Write data to I2C.
 *  @param    addr - which register to write
 *            pBuf - pointer to buffer containing data to be written
 *            nBytes - number of bytes to write
 *
 *  @return   pdTRUE if successful write
 */
uint8_t HalSensorWriteReg( uint8_t addr, uint8_t *pBuf, uint8_t nBytes )
{
    uint8_t i;
    uint8_t *p = buffer;
    // Copy address and data to local buffer for burst write
    *p++ = addr;
    for (i = 0; i < nBytes; i++)
    {
        *p++ = *pBuf++;
    }
    nBytes++;
    // Send address and data
    i = HalI2CWrite( nBytes, buffer );
    return (i == nBytes);
}

uint8_t HalSensorWrite( uint8_t *pBuf, uint8_t nBytes )
{
    uint8_t i = HalI2CWrite( nBytes, pBuf );
    return (i == nBytes);
}


//#define I2C_DELAY()             __delay_cycles(100);

void SDA_PIN_AS_INPUT( void )
{
     IO_RD0_SetDigitalInput();
}

void SDA_PIN_AS_OUTPUT( void )
{
    IO_RD0_SetDigitalOutput();
}

void SCL_PIN_AS_INPUT( void )
{
    IO_RD1_SetDigitalInput();
}

void SCL_PIN_AS_OUTPUT( void )
{
    IO_RD1_SetDigitalOutput();
}

void I2C_DELAY( uint8_t delay_us )
{
//	uint32_t wait_loop_index;
//
//	wait_loop_index = delay_us * 100;
//	while ( --wait_loop_index != 0U );
    while ( --delay_us != 0U );
}

void I2C_CLK_DELAY( void )
{
    uint16_t wait_loop_index = 60;
    while( --wait_loop_index != 0U );
}


// 9 clk reset I2C device
uint8_t I2C_Device_Reset( void )
{
    uint8_t i;

    SDA_PIN_AS_OUTPUT();
    SCL_PIN_AS_INPUT();
    I2C_DELAY(10);
    if ( SCL_PIN_GET() == GPIO_PIN_SET ){
        SDA_PIN_OUT_H();
        I2C_DELAY(10);
        SCL_PIN_AS_OUTPUT();
        for ( i = 0; i < 9; i++ ){
            SCL_PIN_OUT_L();                   // SCL output Low
            I2C_DELAY(10);                    // Delay cycle
            SCL_PIN_OUT_H();                   // SCL output High
            I2C_DELAY(10);                    // Delay cycle
        }
    }
    return true;
}

void VI2C_StartBit( void )
{
    
    SDA_PIN_AS_OUTPUT();
    SCL_PIN_AS_OUTPUT();

    SDA_PIN_OUT_H();                  // SDA output High
    I2C_DELAY(10);                   // Delay cycle
    SCL_PIN_OUT_H();                  // SCL output High
    I2C_DELAY(10);                   // Delay cycle
    SDA_PIN_OUT_L();                  // SDA output Low
    I2C_DELAY(10);                   // Delay cycle
    SCL_PIN_OUT_L();                  // SCL output Low
    I2C_DELAY(10);

}

void VI2C_StopBit( void )
{
    SDA_PIN_AS_OUTPUT();
    SCL_PIN_AS_OUTPUT();

    SDA_PIN_OUT_L();                  // SDA output Low
    I2C_DELAY(10);                      // Delay cycle
    SCL_PIN_OUT_H();                  // SCL output High
    I2C_DELAY(10);                      // Delay cycle
    SDA_PIN_OUT_H();                  // SDA output High
    I2C_DELAY(10);                      // Delay cycle
}

#if 0

void i2c_send_wake_token(void)
{
	SDA_PIN_AS_OUTPUT();
	SCL_PIN_AS_OUTPUT();

	SDA_PIN_OUT_L();
	// __delay_cycles(2000);            // 100us
	I2C_DELAY(100);
	SDA_PIN_OUT_H();

}

uint8_t atcab_wakeup( void )
{
    uint8_t buf[4] = { 0x00, 0x00, 0x00, 0x00 };    // 04, 11, 33, 43


    VI2C_StartBit();
    i2c_send_wake_token();
    I2C_DELAY(4000);             // 3ms
    VI2C_StopBit();
    // Synchronization, chapter 6.5
    VI2C_StartBit();
    I2C_Device_Reset();
    VI2C_StartBit();
    VI2C_StopBit();

    if ( HalI2CRead( 4, buf ) != 4 ){
        buf[0] = 0;
    }

    return buf[0];
}
#endif

// 1: 6.1us,  3: 7.9us, 5: 11.35us, 7: 14.4us, 10:22us, 15: 28us
void delay_us( uint8_t delay_us )
{
    while ( --delay_us != 0U );
}

// 960ms
void delay_ms( uint16_t delay_ms )
{
	uint16_t wait_loop_index;
	wait_loop_index = delay_ms << 8;
	while ( --wait_loop_index != 0U ){
         NOP();    NOP();    NOP();    NOP();
     }
}