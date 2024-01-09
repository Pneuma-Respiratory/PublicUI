/***********************************************************************************
  Filename:     hal_sdp.c

  Description:  
                

***********************************************************************************/

/***********************************************************************************
* INCLUDES
*/

#include "cmsis_os.h"
#include "i2c.h"
//#include "stm32l4xx_hal_i2c.h"
#include "stm32l4xx_ll_utils.h"

#include "hal_i2c.h"
#include "hal_sdp.h"

/***********************************************************************************
 * CONSTANTS
 */

/***********************************************************************************
 * MACROS
 */                                                                            

/***********************************************************************************
* GLOBAL VARIABLES
*/
uint8_t pressureBuf[3];

/***********************************************************************************
* LOCAL VARIABLES
*/
extern I2C_HandleTypeDef I2cHandle;

/***********************************************************************************
* GLOBAL FUNCTIONS
*/


/***********************************************************************************
* LOCAL FUNCTIONS
*/
static uint8_t HalSdp_Read( void );
static uint8_t xCal_crc( uint8_t *ptr, uint8_t len );


/***********************************************************************************
*  @fn   
*  @brief   Write 0x0006 RESET command, 
*           after the reset command the sensor will take maximum 1ms to reset
*  @param   
*                   
*  @return   true if successful write
*/
static uint8_t HalSdp_Reset( void )
{
    uint8_t result;
    
    Hal_I2C_Select( SDP3X_RESET );
    result = HalSensorWriteReg( 0x06, NULL, 0 );  
    Hal_I2C_Release( SDP3X_RESET );
    LL_mDelay(25);
    
    return result;
}

uint8_t HalSdp_Init( void )
{
    uint8_t result;
    
    result = HalSdp_Reset();
    
    if ( result ){
        return HalSdp_GetID();
    }
    else{
        return result;
    }
}

/***********************************************************************************
*  @fn   
*  @brief   Write commands to sensor
*           The sensor takes 33ms to measure, no command can be sent to the snesor
*  @param   
*                   
*  @return   true if successful write
*/
static uint8_t HalSdp_Command( uint16_t cmd )
{
    uint8_t data, result;
    
    Hal_I2C_Select( SDP3X );
    data  = (uint8_t)cmd;
    result = HalSensorWriteReg( (uint8_t)(cmd >> 8), &data, 1 );     // Delay>5ms then read result
    Hal_I2C_Release( SDP3X );
    
    return result;
}

uint8_t HalSdp_Start_Measure( void )
{
    return HalSdp_Command( SDP_DIFF_AVERAGE );
}

uint8_t HalSdp_Stop_Measure( void )
{
    uint8_t result;
    
    HalSdp_Command( SDP_STOP_MEASURE );
    LL_mDelay(2);
    //if ( HalSdp_Command( SDP_STOP_MEASURE ) ){
    if ( 1 ){
        Hal_I2C_Select( SDP3X_RESET );
        result = HalSensorWriteReg( 0x06, NULL, 0 );  
        Hal_I2C_Release( SDP3X_RESET );
    }
    
    return result;
}

static uint8_t HalSdp_Read( void )
{
    uint8_t crc_value, result = 0xFF;
    
    pressureBuf[0] = pressureBuf[1] = pressureBuf[2] = 0;
    Hal_I2C_Select( SDP3X );
    result = HalSensorRead( pressureBuf, 3 );
    Hal_I2C_Release( SDP3X );
    
    if ( result == true ){
        crc_value = xCal_crc( pressureBuf, 2 );
    }
    else{  
        // HalSdp_Stop_Measure();   LL_mDelay(25);   HalSdp_Start_Measure();
        // uart printf
//         HalSdp_Reset();   HalSdp_Start_Measure();
        return false;
    }
    return ( crc_value == pressureBuf[2] );
}

// Sensor threshold value
uint16_t HalSdp_GetThr( void )
{
    uint16_t data = 0;
    
    HalSdp_Command( SDP_DIFF_AVERAGE );
    // Delay 50ms
    LL_mDelay(30);
    if ( true == HalSdp_Read() ){
        data = pressureBuf[0];
        data <<= 8;
        data |= pressureBuf[1];
    }
    else{
        LL_mDelay(10);
    }

    HalSdp_Command( SDP_STOP_MEASURE );
        
    if ( data > 32768 ){
        return 0;
    }
    else{
        //return data/130;
        return data;
    }
}

int8_t HalSdp_GetValue( uint16_t *value, uint16_t thr )
{
    uint16_t data = 0;
    
    if ( true == HalSdp_Read() ){
        data = pressureBuf[0];
        data <<= 8;
        data |= pressureBuf[1];
    }
    else{
        return false;
    }

    if ( data >= 32768 ){
        *value = 0;
    }
    else{
        *value = data;
    }
    return true;
}

// 0x03 01 9D 01 88 B6, 00 00 81 31 32 60, 35 4A A2 57 54 72
uint8_t HalSdp_GetID( void )
{
    uint8_t i, buffer[18];
    
    for ( i = 0; i < sizeof(buffer); i++ ){
        buffer[i] = 0;
    }
    
    if ( HalSdp_Command( SDP_PROD_NUMBER ) ){
         if ( HalSdp_Command( SDP_PROD_ID ) ){
        //if ( 1 ){
            Hal_I2C_Select( SDP3X );
            HalSensorRead( buffer, 18 );
            Hal_I2C_Release( SDP3X );
            
            if ( (buffer[2] == xCal_crc(&buffer[0], 2)) && (buffer[5] == xCal_crc(&buffer[3], 2)) ){
                return true;
            }
        }
    }
    
    return false;
}

uint8_t HalSdp_Test( void )
{
    uint8_t result = 0, i = 0;
    
    HalSdp_Init();
    //HalSdp_GetID();
    // Delay 5ms, @20MHz
    //I2C_Device_Reset();
    LL_mDelay(5);
    result = HalSdp_Command( SDP_DIFF_AVERAGE );
    // Delay 40ms
    LL_mDelay(50);
    while( result ){
        LL_mDelay(50);
        i++;
        result = HalSdp_Read();
    }
    
    return result | ( i << 5);
}

/**************************
CRC8 У�����     
**************************/	
static uint8_t xCal_crc( uint8_t *ptr, uint8_t len )		
{
    uint8_t i, crc = 0xFF;
    
    while( len-- ){
        crc ^= *ptr++;
        for( i = 0; i < 8; i++ ){
            if( crc & 0x80 ){
                crc = (crc << 1) ^ 0x31;
            }else{
                crc <<= 1;
            }
        }
    }
    
    return crc;
}



