
#include "string.h"
#include "stdlib.h"

#include "atca_basic.h"
#include "atca_status.h"
#include "hal_board_cfg.h"
#include "i2c.h"
#include "hal_i2c.h"

#include "atca_main.h"


//ATCAPacket packet;
uint8_t sha_detect( void )
{
	uint8_t atcab_sn[9];
    ATCA_STATUS status = ATCA_GEN_FAIL;

    Hal_I2C_Select( SHA204A );
    status = atcab_read_serial_number( atcab_sn );
    Hal_I2C_Release( SHA204A );

    if ( (status == ATCA_SUCCESS) && atcab_sn[0]){
        return true;
    }
    else{
        return false;
    }
}

ATCA_STATUS sha_update_slot2( uint16_t used_dose, uint32_t used_time )
{
    ATCA_STATUS status = ATCA_GEN_FAIL;
    cartidge_slot2_t slot_data;
    
    memset( (uint8_t *)(&slot_data), 0xFF, sizeof(cartidge_slot2_t) );
    slot_data.used_eje_dose = used_dose;
    slot_data.used_eje_time = used_time;
    
    status = ATCA_GEN_FAIL; 
    Hal_I2C_Select( SHA204A );
    status = atcab_write_slot_data( 2, (uint8_t *)(&slot_data) );
    Hal_I2C_Release( SHA204A );
    
    return status;
}
