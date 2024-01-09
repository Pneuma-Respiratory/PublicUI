/***********************************************************************************
  Filename:     hal_virtual_i2c.c

  Description:  
                

***********************************************************************************/

/***********************************************************************************
* INCLUDES
*/

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

/***********************************************************************************
* LOCAL VARIABLES
*/

//(GPIO_TypeDef *) I2C_SDA_PORT, I2C_SCL_PORT;
//uint16_t SDA_PIN, SCL_PIN;
//static uint8_t I2C_Device;
uint8_t Device_Address;
//static uint8_t buffer[20];

/***********************************************************************************
* LOCAL FUNCTIONS
*/

/***********************************************************************************
* GLOBAL FUNCTIONS
*/
extern void I2C_DELAY( uint8_t delay_us );

/***********************************************************************************
* LOCAL FUNCTIONS
*/

/***********************************************************************************
*  @fn       VI2C_PIN_Cfg
*  @brief    Setup all of the 3ch I2C GPIOs as GPIO output
*/
void VI2C_PIN_Cfg( void )
{
    // SDA, SCL as GPIO OUTPUT
    
    IO_RA4_SetDigitalOutput();
    IO_RA4_SetOpenDrain();
    IO_RA5_SetDigitalOutput();
    IO_RA5_SetOpenDrain();

}


/***********************************************************************************
*  @fn        VI2C_DeInit
*  @brief    Set I2C as GPIO pin to low
*  @param                
*  @return   none
*/
void VI2C_DeInit( void )
{
    
}

/***********************************************************************************
*  @fn       VI2C_RD_AckBit
*  @brief    Read I2C ACK
*/
uint8_t VI2C_RD_AckBit( void )
{
    uint8_t bit = 0;
    uint16_t wait = 0;
    
    SCL_PIN_AS_OUTPUT();              //$$
    SCL_PIN_OUT_L();
    SDA_PIN_AS_INPUT();               // SDA as Input
    I2C_DELAY(10);                    // Delay cycle
    SCL_PIN_OUT_H();                  // SCL output High
    I2C_DELAY(10);                    // Delay cycle
    do {
        bit = SDA_PIN_GET();          // bit = Read SDA
        if ( ++wait > 8000 )  break;
    }while( bit != I2C_ACK );
    I2C_DELAY(10);                     // Delay cycle
    SCL_PIN_OUT_L();                  // SCL output Low
    I2C_DELAY(10);                     // Delay cycle
    SDA_PIN_AS_OUTPUT();              // SDA as output
    
    return bit;
}

/***********************************************************************************
*  @fn       VI2C_WR_AckBit
*  @brief    Write I2C ACK
*/
void VI2C_WR_AckBit( void )
{
    SDA_PIN_AS_OUTPUT();              // SDA as output
    SCL_PIN_AS_OUTPUT();              //$$ 
    I2C_DELAY(10);
    SCL_PIN_OUT_L();                  // SCL output Low
    I2C_DELAY(10);
    SDA_PIN_OUT_L();                  // SDA output Low
    I2C_DELAY(10);                      // Delay cycle
    SCL_PIN_OUT_H();                  // SCL output High
    I2C_DELAY(10);                      // Delay cycle
    SCL_PIN_OUT_L();                  // SCL output Low  ***********
    I2C_DELAY(10);                      // Delay cycle  *************
//    SDA_PIN_OUT_H();                  // SDA output High
}

/***********************************************************************************
*  @fn       VI2C_WR_NAckBit
*  @brief    Write I2C NACK
*/
void VI2C_WR_NAckBit( void )
{
    SDA_PIN_AS_OUTPUT();              // SDA as Output
    SCL_PIN_AS_OUTPUT();              //$$ 
    I2C_DELAY(10);
#if 0
    SDA_PIN_OUT_H();                  // SDA output High
    I2C_DELAY(5);                      // Delay cycle
    SCL_PIN_OUT_H();                  // SCL output High
    I2C_DELAY(5);                      // Delay cycle
    SCL_PIN_OUT_L();                  // SCL output Low
    I2C_DELAY(5);                      // Delay cycle
#else
    SCL_PIN_OUT_L();                  // SCL output Low
    I2C_DELAY(10);                      // Delay cycle
    SDA_PIN_OUT_H();                  // SDA output High
    I2C_DELAY(10);                      // Delay cycle
    SCL_PIN_OUT_H();                  // SCL output High
    I2C_DELAY(10);                      // Delay cycle
    SCL_PIN_OUT_L();                  // SCL output High
    I2C_DELAY(10);                      // Delay cycle
#endif
}

/***********************************************************************************
 *  @fn       VI2C_WRITE
 *  @brief    Write signle data from I2C.
 *  @param    address   -   
 *            data      -   
 *            
 *  @return   read data length
 */
void VI2C_WRITE( uint8_t data )
{
    uint8_t i = 0;
    
     SDA_PIN_AS_OUTPUT();              // SDA as output
     SCL_PIN_OUT_L();                    //ss
    for ( i = 0; i < 8; i++ ){
        if ( data & 0x80 ){
            SDA_PIN_OUT_H();              // SDA = 1
        }
        else{
            SDA_PIN_OUT_L();              // SDA = 0
        }
        I2C_DELAY(5);
        SCL_PIN_OUT_H();                  // SCL output High
        I2C_DELAY(10);                      // Delay cycle
        SCL_PIN_OUT_L();                  // SCL output Low
        I2C_DELAY(10);                      // Delay cycle
        data <<= 1;
    }
    //SDA_PIN_OUT_H();
    I2C_DELAY(10);
    //SCL_PIN_OUT_H();
    I2C_DELAY(10);
}

/***********************************************************************************
 *  @fn       VI2C_WRITE
 *  @brief    Write signle data from I2C.
 *  @param    address   -   
 *            data      -   
 *            
 *  @return   read data length
 */
#if 0
uint8_t VI2C_READ( void )
{
    uint8_t i, data = 0;
    
    SCL_PIN_OUT_L();
    SCL_PIN_AS_OUTPUT();              //$$ 
    SDA_PIN_AS_INPUT();                   // SDA as Input
    I2C_DELAY(5);
    for ( i = 0; i < 8; i++ ){
        SCL_PIN_OUT_H();                  // SCL output High
        I2C_DELAY(5);                      // Delay cycle
        data <<= 1;
        if ( SDA_PIN_GET() == 1 ){
            data |= 0x01;
        }
        else{
//            data &= 0xFE;
        }
        I2C_DELAY(5);                      // Delay cycle
        SCL_PIN_OUT_L();                  // SCL output Low
        I2C_DELAY(5);                      // Delay cycle
    }
    return data;
}
#else
uint8_t VI2C_READ( void )
{
    uint8_t i, data = 0;
    
    SCL_PIN_OUT_L();
    SDA_PIN_AS_INPUT();                   // SDA as Input
    I2C_DELAY(10);
    for ( i = 0; i < 8; i++ ){
        SCL_PIN_OUT_H();                  // SCL output High
        I2C_DELAY(10);                      // Delay cycle
        data <<= 1;
        if ( SDA_PIN_GET() == 1 ){
            data |= 0x01;
        }
        I2C_DELAY(10);                      // Delay cycle
        SCL_PIN_OUT_L();                  // SCL output Low
        I2C_DELAY(10);                      // Delay cycle
    }
    return data;
}
#endif


#if 1

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
 * @return      TRUE if the required number of bytes are reveived
 **************************************************************************************************/
/*
bool HalSensorReadReg(uint8_t addr, uint8_t *pBuf, uint8_t nBytes)
{
  uint8_t i = 0;

  // Send address we're reading from
  if ( HalI2CWrite( 1, &addr ) == 1 ){
    // Now read data
    i = HalI2CRead( nBytes,pBuf );
  }

  return i == nBytes;
}

bool HalSensorRead(uint8_t *pBuf, uint8_t nBytes)
{
  uint8_t i = HalI2CRead( nBytes,pBuf );
  
  return i == nBytes;
}
*/

/***********************************************************************************
 *  @fn       HalSensorWriteReg
 *  @brief    Write data to I2C.
 *  @param    addr - which register to write
 *            pBuf - pointer to buffer containing data to be written
 *            nBytes - number of bytes to write
 *            
 *  @return   TRUE if successful write
 */
/*
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
*/
/***********************************************************************************
 *  @fn       HalI2CWrite
 *  @brief    Write data to I2C.
 *  @param    address   -   write address
 *            data      -   
 *            length    -   write length pointer
 *            
 *  @return   The number of bytes successfully written.
 */
uint8_t HalI2CWrite( uint8_t len, uint8_t *pBuf )
{
    uint8_t cnt, tmpAddr;
    
    tmpAddr = (uint8_t)(Device_Address << 1);
    
    VI2C_StartBit();  
    VI2C_WRITE( tmpAddr | 0x00 );
    if ( VI2C_RD_AckBit() != I2C_ACK ){
        return 0;
    }
    
    for ( cnt = 0; cnt < len; cnt++ ){
        VI2C_WRITE( *pBuf++ );
        if ( VI2C_RD_AckBit() != I2C_ACK ){
            break;
        }
    }
    
    VI2C_StopBit();
    
    return cnt;
}

/***********************************************************************************
 *  @fn   
 *  @brief    Read data from I2C.
 *  @param    address   -   read address
 *            data      -   save the read data
 *            length    -   read length
 *            
 *  @return   read data length
 */
uint8_t HalI2CRead(  uint8_t len, uint8_t *pBuf  )
{
    uint8_t cnt = 0, tmpAddr;
    
    tmpAddr = (uint8_t)(Device_Address << 1);
    
    VI2C_StartBit();  
    VI2C_WRITE( tmpAddr | 0x01 );
    if ( VI2C_RD_AckBit() != I2C_ACK ){
        len = 0;
    }
    if ( len > 1 ){
//        VI2C_WR_AckBit();
    }
    
    while( len > 0 ){
        *pBuf++ = VI2C_READ();
        if ( len == 1 ){
            VI2C_WR_NAckBit();
        }
        else{
            VI2C_WR_AckBit();
        }
        cnt++;
        len--;
    }
    
    VI2C_StopBit();
    
    return cnt;
}
#endif

uint8_t sensor_write( uint8_t *pBuf , uint8_t len )
{
    uint8_t cnt, tmpAddr;
    
    tmpAddr = (uint8_t)(*pBuf << 1);
    
    VI2C_StartBit();  
    VI2C_WRITE( tmpAddr | 0x00 );
    if ( VI2C_RD_AckBit() != I2C_ACK ){
        return 0;
    }
    
    for ( cnt = 0; cnt < len; cnt++ ){
        VI2C_WRITE( *++pBuf );
        if ( VI2C_RD_AckBit() != I2C_ACK ){
            break;
        }
    }
    
    VI2C_StopBit();
    
    return cnt;
}

uint8_t sensor_read( uint8_t address, uint8_t *pBuf , uint8_t len )
{
    uint8_t cnt = 0, tmpAddr;
    
    tmpAddr = (uint8_t)(address << 1);
    
    VI2C_StartBit();  
    VI2C_WRITE( tmpAddr | 0x01 );
    if ( VI2C_RD_AckBit() != I2C_ACK ){
        len = 0;
    }
    
    while( len > 0 ){
        *pBuf++ = VI2C_READ();
        if ( len == 1 ){
            VI2C_WR_NAckBit();
        }
        else{
            VI2C_WR_AckBit();
        }
        cnt++;
        len--;
    }
    
    VI2C_StopBit();
    
    return cnt;
}
#if 0
void i2c_send_wake_token(void)
{
	SDA_PIN_OUT_L();
	I2C_DELAY(2000);            // 100us
	SDA_PIN_OUT_H();
        
}

uint8_t atcab_wakeup( void )
{
    uint8_t i, buf[4] = { 0x00, 0x00, 0x00, 0x00};    // 04, 11, 33, 43

    VI2C_StartBit();
    i2c_send_wake_token();
    I2C_DELAY(60000);
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
