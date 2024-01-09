/*
 * File:   mac.c
 * Author: A19582
 *
 * Created on August 14, 2018, 4:31 PM
 */
#include "cmsis_os.h"

#include "hal_board_cfg.h"
#include "atca_basic.h"
#include "i2c.h"

#include "hal_i2c.h"
#include "hal_sdp.h"

//#define DEBUG_MESSAGE

ATCAPacket packet;

ATCA_STATUS atcab_idle( void );
uint16_t swap_endian( uint16_t data );

/** \brief Calculates CRC over the given raw data and returns the CRC in
 *         little-endian byte order.
 *
 * \param[in]  length  Size of data not including the CRC byte positions
 * \param[in]  data    Pointer to the data over which to compute the CRC
 * \param[out] crc_le  Pointer to the place where the two-bytes of CRC will be
 *                     returned in little-endian byte order.
 */
void atCRC(size_t length, const uint8_t *data, uint8_t *crc_le)
{
    size_t counter;
    uint16_t crc_register = 0;
    uint16_t polynom = 0x8005;
    uint8_t shift_register;
    uint8_t data_bit, crc_bit;

    for(counter = 0; counter < length; counter++)
    {
        for(shift_register = 0x01; shift_register > 0x00; shift_register <<= 1)
        {
            data_bit = (data[counter] & shift_register) ? 1 : 0;
            crc_bit = crc_register >> 15;
            crc_register <<= 1;
            if(data_bit != crc_bit)
                crc_register ^= polynom;
        }
    }
    crc_le[0] = (uint8_t)(crc_register & 0x00FF);
    crc_le[1] = (uint8_t)(crc_register >> 8);
}


/** \brief This function calculates CRC and adds it to the correct offset in the packet data
 * \param[in] packet Packet to calculate CRC data for
 */

void atCalcCrc()
{
    uint8_t length, *crc;

    length = packet.txsize - ATCA_CRC_SIZE;
    // computer pointer to CRC in the packet
    crc = &(packet.txsize) + length;

    // stuff CRC into packet
    atCRC(length, &(packet.txsize), crc);
}


/** \brief This function checks the consistency of a response.
 * \param[in] response pointer to response
 * \return status of the consistency check
 */

ATCA_STATUS atCheckCrc( const uint8_t *response )
{
    uint8_t crc[ATCA_CRC_SIZE];
    uint8_t count = response[ATCA_COUNT_IDX];

    if(count < ATCA_CRC_SIZE)
        return ATCA_BAD_PARAM;

    count -= ATCA_CRC_SIZE;
    atCRC(count, response, crc);

    return (crc[0] == response[count] && crc[1] == response[count + 1]) ? ATCA_SUCCESS : ATCA_RX_CRC_ERROR;
}
#if 0
uint8_t atcab_wakeup_sda( void )
{    
    GPIO_setOutputLowOnPin( GPIO_PORT_P6, GPIO_PIN4 );  // SDA
    GPIO_setOutputLowOnPin( GPIO_PORT_P6, GPIO_PIN5 );  // SCL
    __delay_cycles(1000);

    GPIO_setOutputHighOnPin( GPIO_PORT_P6, GPIO_PIN4 );  // SDA
    __delay_cycles(25);
    GPIO_setOutputHighOnPin( GPIO_PORT_P6, GPIO_PIN4 );
        
    return true;
}
#endif
ATCA_STATUS atcab_read_serial_number( uint8_t* serial_number )
{
    ATCA_STATUS status = ATCA_GEN_FAIL;
    uint8_t rst = false, i;
    
    for ( i = 0; i < 3; i++ ){
        // Awake
        if ( atcab_wakeup() == 0x04 ) {  
            rst = true;  
            break;  
        }
        //LL_mDelay(50);
    }
    if ( rst == false ){
        return status;
    }
    rst = false;
    packet.txsize = 0x07;
    packet.opcode = ATCA_READ;
    packet.param1 = 0x80;            // 0x80: Read CONFIG zone for 32 bytes, 0x00: 4 bytes
    packet.param2 = 0x0000;
    atCalcCrc();
   for ( i = 0; i < 4; i++ ){
       at_I2CWrite( 0x03, (uint8_t*)&packet, packet.txsize );
       LL_mDelay(2);           // delay 1ms, 0.4~4ms  Table 8-4
       rst = at_I2CRead( packet.data, 35 );   // LEN(1) + 32 + CRC(2)
       if ( rst == true ){
           //return status;
           break;
       }
   }
   if ( rst == false ){
        return status;
    }
    
    status = atCheckCrc( packet.data );
    if ( status == ATCA_SUCCESS ){
        memcpy(&serial_number[0], &packet.data[1], 4);
        memcpy(&serial_number[4], &packet.data[9], 5);
    }
    atcab_idle();            //'''
    return status;
}

ATCA_STATUS atcab_read_slot_data( uint8_t slot_seq, uint8_t *data )
{
    ATCA_STATUS status = ATCA_GEN_FAIL;
    uint8_t i, rst;
    
    memset( (uint8_t *)&packet, 0xFF, 20 );

    for ( i = 0; i < 3; i++ ){
        // Awake
        atcab_wakeup();
        //I2C_Device_Reset();
        LL_mDelay(5);             // 3ms < n < 96ms�ڶ�доƬ������96msоƬ���Զ�����
        rst = at_I2CRead( packet.data, 4 );
        if ( rst )   break;
    }
    if ( rst == false ){
        return status;
    }
    
    // Build the write command
    packet.txsize = 0x07;
    packet.opcode = ATCA_READ;
    packet.param1 = 0x82;            // BIT7=1: 32 bytes mode, BIT7=0: 4 bytes mode, BIT[1,0]=0x02 Data zone
    packet.param2 = slot_seq * 8;
    atCalcCrc();
#if WDT_EN
    WDT_A_resetTimer( WDT_A_BASE );
#endif
    at_I2CWrite( 0x03, (uint8_t*)&packet, packet.txsize );
    LL_mDelay(1);           // delay 4ms, 0.4~4ms  Table 8-4
    rst = at_I2CRead( packet.data, 35 );
    if ( rst == false ){
        return status;
    }
    
    status = atCheckCrc(packet.data);
    if ( status == ATCA_SUCCESS ){
        for ( i = 1; i <= 32; i++ ){
            *data++ = packet.data[i];
        }
    }
    atcab_idle();      //..
#if WDT_EN
    WDT_A_resetTimer( WDT_A_BASE );
#endif
    return status;
}

ATCA_STATUS atcab_write_slot_data( uint8_t slot_seq, uint8_t *data )
{
    ATCA_STATUS status = ATCA_GEN_FAIL;
    uint8_t i, rst;
    
    for ( i = 0; i < 3; i++ ){
        if ( atcab_wakeup() == 0x04 ) {  
            rst = true;  
            break;  
        }
    }
    //__delay_cycles(200000); 
    // Build the write command
    packet.txsize = 0x27;
    packet.opcode = 0x12;
    packet.param1 = 0x82;            // BIT7=1: Read for 32 bytes, BIT7=0: 4 bytes, BIT[1,0]=0x02 Data zone
    packet.param2 = slot_seq * 8;
#if 0
    for ( i = 0; i < 30; i++ ){
        packet.data[i+1] = i;
    }
    packet.data[31] = 0x03;
    packet.data[32] = 0x37;
    packet.data[33] = 0xF6;
#else
    for ( i = 0; i < 32; i++ ){
        packet.data[i] = *data++;
    }
    atCalcCrc();
#endif
#if WDT_EN
    WDT_A_resetTimer( WDT_A_BASE );
#endif
    at_I2CWrite( 0x03, (uint8_t*)&packet, packet.txsize );
    for ( i = 0; i < 4; i++ ){
    	LL_mDelay(5);           // delay 4~42ms  Table 8-4
        rst = at_I2CRead( packet.data, 4 );
        if ( rst )  break;
    }
    if ( rst == false ){
        return status;
    }
    
    status = atCheckCrc( packet.data );
    if ( status != ATCA_SUCCESS ){
        status = ATCA_CMD_EXE_ERROR;
    }
    atcab_idle();
#if WDT_EN
    WDT_A_resetTimer( WDT_A_BASE );
#endif    
    return status;
}

ATCA_STATUS atcab_idle( void )
{
    ATCA_STATUS status = ATCA_TX_TIMEOUT;
    
    if ( at_I2CWrite(0x02, NULL, 0) ){
        status = ATCA_SUCCESS;
    }
        
    return status;
}

uint16_t swap_endian( uint16_t data )
{
    uint16_t tmp;
    
    tmp = (data << 8) & 0xFF00;
    data = (data >> 8) & 0xFF;
    
    return (data | tmp);
}

// Read Slot 0~3
ATCA_STATUS atcab_read_all( uint8_t *read_buf, uint8_t num )
{
    ATCA_STATUS status = ATCA_GEN_FAIL;
    uint8_t i, j, *data, rst, m = 0;
   
    repeat:
    status = ATCA_GEN_FAIL;
    rst = false;
    memset( (uint8_t *)&packet, 0xFF, 20 );
    for ( i = 0; i < 3; i++ ){
        if ( atcab_wakeup() == 0x04 ) {
            rst = true;
            break;
        }
    }
    if ( rst == false ){
        return status;
    }
    //__delay_cycles(70000); 
    
    // Build the write command
    packet.txsize = 0x07;
    packet.opcode = ATCA_READ;
    packet.param1 = 0x82;            // BIT7=1: Read for 32 bytes, BIT7=0: 4 bytes, BIT[1,0]=0x02 Data zone
        
    data = read_buf;
    for ( i = 0; i < num; i++ ){
        //data = read_buf + i * 35;
        packet.param2 = i * 8;
        packet.data[0] = 0x00;
        packet.data[1] = 0x00;
        packet.data[2] = 0x00;
        atCalcCrc();
#if WDT_EN
    WDT_A_resetTimer( WDT_A_BASE );
#endif        
        at_I2CWrite( 0x03, (uint8_t*)&packet, packet.txsize );
        LL_mDelay(2);           // delay 1ms, 0.4~4ms  Table 8-4
        rst = at_I2CRead( packet.data, 35 );
        if ( rst == false ){
            //return status;
            break;
        }
        
        status = atCheckCrc( packet.data );
        if ( (status == ATCA_SUCCESS) && (packet.data[0] == 0x23) ){
            for ( j = 1; j <= 32; j++ ){
                *data++ = packet.data[j];
            }
        }
        else {
            rst = false;
            break;
        }
    }
    atcab_idle();                //..
        
    if ( rst == false ){
        if ( ++m < 3 ){
        	LL_mDelay(1);
            goto repeat;
        }
    }
        
    return status;
}

/******************************************************************************/
/*
uint8_t sha_detect( void )
{
    ATCA_STATUS status = ATCA_GEN_FAIL;
        
    VI2C_Select( SHA204A );
    status = atcab_read_serial_number( atcab_sn );
    VI2C_Release( SHA204A );
    
    if ( status == ATCA_SUCCESS ){
        return true;
    }
    else{
        return false;
    }
}

// Read All information

uint8_t sha_read_slot( uint8_t slot_seq, uint8_t *data )
{
    ATCA_STATUS status = ATCA_GEN_FAIL;
    
    VI2C_Select( SHA204A );
    status = atcab_read_slot_data( slot_seq, data );
    VI2C_Release( SHA204A );
    
    if ( status == ATCA_SUCCESS ){
        return true;
    }
    else{
        return false;
    }
}

// Write Slot2/3
uint8_t sha_write_slot( uint8_t slot_seq, uint8_t *data )
{
    ATCA_STATUS status = ATCA_GEN_FAIL;
    
    if ( p_bt_state == NULL ){
        return status;
    }
    
    VI2C_Select( SHA204A );
    status = atcab_write_slot_data( slot_seq, data );
    VI2C_Release( SHA204A );
    
    if ( status == ATCA_SUCCESS ){
        return true;
    }
    else{
        return false;
    }
}

uint8_t sha_update_slot3( void )
{
    ATCA_STATUS status = ATCA_GEN_FAIL;
    
    if ( p_bt_state == NULL ){
        return status;
    }
    
    // utc_clock = UTC_getClock();
    if ( p_new_cart_attr->cart_state & BOTTLE_EXTRA_CNT ){
        p_new_cart_attr->remain_cnt = 0;
        p_new_cart_attr->rescue_cnt = p_bt_state->count;
    }
    else{
        p_new_cart_attr->remain_cnt = p_bt_state->count;
        p_new_cart_attr->rescue_cnt = p_new_cart_attr->total_rescue_cnt;
    }
    p_new_cart_attr->volTime = p_bt_state->volTime;
    p_new_cart_attr->updateClock = utc_clock;
    p_new_cart_attr->day_cnt = p_bt_state->day_cnt;
    p_new_cart_attr->cart_state = p_bt_state->state;

    status = ATCA_GEN_FAIL; 
    VI2C_Select( SHA204A );
    //status = atcab_write_slot_data( 3, (uint8_t *)(&(p_new_cart_attr->remain_cnt)) );
    memcpy( slot_data, (uint8_t *)p_new_cart_attr, sizeof(slot_data) );
    status = atcab_write_slot_data( 3, slot_data + 3*32 );
    VI2C_Release( SHA204A );
    
    return status;
}
*/
